//! The step loop: the clock, the deadline policy, and the thread wiring.
//!
//! Everything else in the kernel is a component this file arranges. It owns the
//! one thing that cannot be delegated -- when a step happens -- and the policy
//! for what to do when one does not happen on time.
//!
//! ```text
//!   HidThread                     StepThread (pinned, SCHED_FIFO)
//!   ---------                     --------------------------------
//!   poll device                   1. read the newest DriverInput
//!     -> input seqlock            2. shape the steering position
//!                                 3. plant.step(dt)          <- PlantKernel
//!   ffb seqlock                   4. integrate pose from body velocities
//!     -> apply torque             5. condition the feedback torque
//!     + staleness watchdog        6. publish state and torque
//!                                 7. push a telemetry frame
//!                                       |
//!                                 TelemetryThread -> disk
//! ```

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

use crate::config::{KernelConfig, OverrunPolicy};
use crate::generated::frames::{fault_flags, DriverInput, FfbCommand, VehicleState, LAYOUT_HASH};
use crate::generated::frames::{TraceDevice, TraceStep};
use crate::io::device::{DeviceCaps, HapticSink, InputSource};
use crate::io::ffb::{FfbChain, FfbConfig};
use crate::io::hid::{HidConfig, HidLoop, HidStats};
use crate::io::input_shaper::{ShaperConfig, SteerShaper};
use crate::metrics::{RealtimeFactor, StepTimeHistogram};
use crate::plant::integrator::Method;
use crate::plant::{ladder, InitialConditions};
use crate::pose::Pose;
use crate::sys::clock::{monotonic_ns, sleep_until_precise};
use crate::sys::sched::{harden_current_thread, RtStatus};
use crate::sys::{shm, signals};
use crate::telemetry::trace::TraceMeta;
use crate::telemetry::{self, SessionMeta, TelemetryStats, TraceStats};
use crate::transport::{ring_channel, SeqlockReader, SeqlockWriter};

/// One span stamp, taken only when tracing is on.
///
/// This is the whole cost of the trace on the critical path: with `--trace`
/// absent, `enabled` is false for the entire session, so the branch predicts
/// perfectly and no clock is read at all. With it on, a vDSO CLOCK_MONOTONIC
/// read is ~20-25 ns and there are five of these, against a 1 ms budget.
#[inline(always)]
fn stamp(enabled: bool) -> u64 {
    if enabled {
        monotonic_ns()
    } else {
        0
    }
}

/// What a session did, reported honestly whether or not it went well.
pub struct SessionReport {
    pub steps: u64,
    pub sim_time: f64,
    pub wall_time_s: f64,
    pub rt_status: RtStatus,
    pub device: DeviceCaps,
    pub histogram: StepTimeHistogram,
    pub deadline_misses: u64,
    /// Steps where the loop had fallen so far behind that it re-anchored the
    /// clock rather than bursting to catch up.
    pub reanchors: u64,
    pub fault_flags: u64,
    pub ffb_clamps: u64,
    pub watchdog_trips: u64,
    pub telemetry_frames: u64,
    pub telemetry_dropped: u64,
    /// Spans recorded. Zero unless `--trace` was given.
    pub trace_steps: u64,
    pub trace_dropped: u64,
    pub stale_commands: u64,
    pub ladder_report: String,
    pub plant_error: Option<String>,
}

impl SessionReport {
    pub fn print(&self) {
        println!("\n--- session report ---------------------------------------------");
        println!("{}", self.ladder_report);
        println!("device      {}", self.device.describe());
        let missing = self.device.missing_cues();
        if !missing.is_empty() {
            println!(
                "            NOT available: {}. Do not attribute these to the car.",
                missing.join(", ")
            );
        }
        println!("scheduling  {}", self.rt_status.describe());
        for line in self.rt_status.advice() {
            println!("            {line}");
        }
        println!(
            "steps       {} over {:.2} s sim / {:.2} s wall (rtf {:.4})",
            self.steps,
            self.sim_time,
            self.wall_time_s,
            if self.wall_time_s > 0.0 {
                self.sim_time / self.wall_time_s
            } else {
                0.0
            }
        );
        println!("step time   {}", self.histogram.summary_us());
        println!(
            "deadlines   {} missed ({:.4}%), {} re-anchors",
            self.deadline_misses,
            100.0 * self.deadline_misses as f64 / self.steps.max(1) as f64,
            self.reanchors
        );
        println!(
            "feedback    {} clamped steps, {} watchdog trips, {} stale commands at the device",
            self.ffb_clamps, self.watchdog_trips, self.stale_commands
        );
        if self.telemetry_frames > 0 || self.telemetry_dropped > 0 {
            println!(
                "telemetry   {} frames written, {} dropped",
                self.telemetry_frames, self.telemetry_dropped
            );
        }
        if self.fault_flags != 0 {
            println!("FAULTS      {}", describe_faults(self.fault_flags));
        }
        if let Some(error) = &self.plant_error {
            println!("PLANT ERROR {error}");
        }
        println!("----------------------------------------------------------------");
    }
}

pub fn describe_faults(flags: u64) -> String {
    let mut names = Vec::new();
    for (bit, name) in [
        (
            fault_flags::PLANT_NONFINITE,
            "plant produced a non-finite value",
        ),
        (fault_flags::DEADLINE_MISSED, "deadlines missed"),
        (fault_flags::FFB_CLAMPED, "feedback torque clamped"),
        (fault_flags::TELEMETRY_OVERFLOW, "telemetry dropped samples"),
        (fault_flags::DEVICE_LOST, "device lost"),
        (fault_flags::PLANT_STEP_FAILED, "plant step failed"),
        (fault_flags::INPUT_STALE, "driver input went stale"),
    ] {
        if flags & bit != 0 {
            names.push(name);
        }
    }
    if names.is_empty() {
        "none".to_string()
    } else {
        names.join("; ")
    }
}

/// Remove segments left behind by a previous run.
///
/// A crashed session leaves its shared memory in place, and attaching to a
/// stale segment would resurrect its contents -- including, in the worst case, a
/// non-zero force-feedback command.
pub fn clear_stale_segments() {
    shm::unlink(DriverInput::SHM_NAME);
    shm::unlink(VehicleState::SHM_NAME);
    shm::unlink(FfbCommand::SHM_NAME);
}

/// Run a full session. Blocks until the duration elapses or the loop is interrupted.
pub fn run<D>(config: KernelConfig, device: D) -> Result<SessionReport, String>
where
    D: InputSource + HapticSink + Send + 'static,
{
    signals::install();
    signals::reset();
    clear_stale_segments();

    // --- transports -----------------------------------------------------
    // The feedback segment is created before the device thread starts, because
    // that thread attaches to it.
    let mut ffb_writer = SeqlockWriter::<FfbCommand>::create(FfbCommand::SHM_NAME, LAYOUT_HASH)
        .map_err(|e| format!("feedback segment: {e}"))?;
    let mut state_writer =
        SeqlockWriter::<VehicleState>::create(VehicleState::SHM_NAME, LAYOUT_HASH)
            .map_err(|e| format!("state segment: {e}"))?;

    let stop = Arc::new(AtomicBool::new(false));
    let hid_stats = Arc::new(HidStats::default());
    let hid_config = HidConfig {
        spin_ns: config.spin_ns / 2,
        ..Default::default()
    };
    // The trace rings are created here, before the device thread, because the
    // device thread owns one end of the second one. The drain thread that
    // consumes them is spawned further down, once the plant has been chosen and
    // there is a kernel id worth recording.
    let (trace_step_tx, trace_step_rx, trace_device_tx, trace_device_rx) = match &config.trace_path
    {
        Some(_) => {
            let (step_tx, step_rx) = ring_channel::<TraceStep>(config.trace_capacity);
            let (device_tx, device_rx) = ring_channel::<TraceDevice>(config.trace_capacity);
            (
                Some(step_tx),
                Some(step_rx),
                Some(device_tx),
                Some(device_rx),
            )
        }
        None => (None, None, None, None),
    };

    let hid = HidLoop::new(device, hid_config, Arc::clone(&hid_stats))
        .map_err(|e| format!("device thread: {e}"))?
        .with_trace(trace_device_tx);
    let device_caps = hid.caps();

    let hid_stop = Arc::clone(&stop);
    let hid_thread = std::thread::Builder::new()
        .name("bobdil-hid".to_string())
        .spawn(move || hid.run(hid_stop))
        .map_err(|e| format!("cannot spawn device thread: {e}"))?;

    let input_reader = SeqlockReader::<DriverInput>::attach(DriverInput::SHM_NAME, LAYOUT_HASH)
        .map_err(|e| format!("input segment: {e}"))?;

    // --- plant ------------------------------------------------------------
    let selection = ladder::select(
        config.plant_dir.as_deref(),
        config.step_dt,
        config.method,
        config.substeps,
        2_000,
        ladder::DEFAULT_GATE_P999_NS,
    );
    let ladder_report = selection.report(ladder::DEFAULT_GATE_P999_NS);
    let mut plant = selection.plant;
    let caps = plant.capabilities();

    if config.step_dt > caps.max_stable_dt + 1e-12 {
        stop.store(true, Ordering::Relaxed);
        let _ = hid_thread.join();
        return Err(format!(
            "configured step of {:.3} ms exceeds what {} is stable at ({:.3} ms). \
             A plant integrated past its stability limit produces a plausible-looking \
             divergence the driver would feel as vagueness, which is worse than refusing to run.",
            config.step_dt * 1e3,
            caps.name,
            caps.max_stable_dt * 1e3
        ));
    }

    plant
        .reset(&InitialConditions {
            speed: config.initial_speed,
            steering_angle: 0.0,
        })
        .map_err(|e| format!("plant reset: {e}"))?;

    // --- telemetry ---------------------------------------------------------
    let telemetry_stats = Arc::new(TelemetryStats::default());
    let (telemetry_tx, telemetry_thread) = match &config.telemetry_path {
        Some(path) => {
            let (tx, rx) = ring_channel::<VehicleState>(config.telemetry_capacity);
            let meta = SessionMeta {
                step_dt: config.step_dt,
                kernel_id: caps.id,
                evaluations_per_step: (config.method.evaluations() * config.substeps) as u64,
                vehicle_hash: 0,
                build_hash: 0,
            };
            let stop_telemetry = Arc::clone(&stop);
            let stats = Arc::clone(&telemetry_stats);
            let path = path.clone();
            let handle = std::thread::Builder::new()
                .name("bobdil-telemetry".to_string())
                .spawn(move || telemetry::run(rx, path, meta, stop_telemetry, stats))
                .map_err(|e| format!("cannot spawn telemetry thread: {e}"))?;
            (Some(tx), Some(handle))
        }
        None => (None, None),
    };

    // --- the trace ----------------------------------------------------------
    // Two rings because `spsc_ring` is single-producer and the device thread is
    // the second producer. Both are allocated here, before the thread hardens,
    // for the same reason everything else on the step path is.
    let trace_stats = Arc::new(TraceStats::default());
    let trace_thread = match (&config.trace_path, trace_step_rx, trace_device_rx) {
        (Some(path), Some(step_rx), Some(device_rx)) => {
            let meta = TraceMeta {
                step_dt: config.step_dt,
                kernel_id: caps.id,
                build_hash: 0,
            };
            let stop_trace = Arc::clone(&stop);
            let stats = Arc::clone(&trace_stats);
            let path = path.clone();
            Some(
                std::thread::Builder::new()
                    .name("bobdil-trace".to_string())
                    .spawn(move || {
                        telemetry::run_trace(step_rx, device_rx, path, meta, stop_trace, stats)
                    })
                    .map_err(|e| format!("cannot spawn trace thread: {e}"))?,
            )
        }
        _ => None,
    };

    // --- the loop -----------------------------------------------------------
    // Everything the step path touches is allocated here, before the thread is
    // hardened, so that mlock(current) covers all of it and the loop itself
    // never reaches the allocator.
    let mut shaper = SteerShaper::new(ShaperConfig::default());
    let mut chain = FfbChain::new(FfbConfig {
        gain: config.ffb_gain,
        torque_limit_nm: config.ffb_torque_limit,
        ..Default::default()
    });
    let mut histogram = StepTimeHistogram::new(20_000_000, 1_000);
    let mut rtf = RealtimeFactor::new(500_000_000);

    // --- real-time hardening, after preallocation ----------------------------
    let rt_status = if config.request_realtime {
        harden_current_thread(config.rt_cpu)
    } else {
        RtStatus::default()
    };

    let step_ns = config.step_ns();
    let dt = config.step_dt;
    let mut sim_time = 0.0f64;
    let mut step_index = 0u64;
    let mut pose = Pose::default();
    let mut deadline_misses = 0u64;
    let mut reanchors = 0u64;
    let mut faults = 0u64;
    let mut plant_error = None;
    let tracing = trace_step_tx.is_some();

    let wall_start = monotonic_ns();
    rtf.start(wall_start, 0.0);
    let mut deadline = wall_start + step_ns;
    let total_steps = config
        .duration_s
        .map(|seconds| (seconds / dt).round() as u64);

    loop {
        if signals::interrupted() || stop.load(Ordering::Relaxed) {
            break;
        }
        if let Some(limit) = total_steps {
            if step_index >= limit {
                break;
            }
        }

        let step_start = monotonic_ns();

        // 1. The newest driver input. A seqlock read, so this never waits on
        //    the device thread and never sees a half-written sample.
        let mut input = match input_reader.read_latest() {
            Some((input, _)) => input,
            None => {
                faults |= fault_flags::INPUT_STALE;
                DriverInput::default()
            }
        };
        let t_after_read = stamp(tracing);

        // 2. Shape the steering position so the plant sees an angle, a rate and
        //    an acceleration that are actually derivatives of one another.
        let shaped = shaper.update(input.steering_angle_command, dt);
        input.steering_angle_command = shaped.angle;
        let t_after_shape = stamp(tracing);

        // 3. Advance the plant by exactly dt.
        let mut state = match plant.step(&input, dt) {
            Ok(state) => state,
            Err(error) => {
                faults |= fault_flags::PLANT_STEP_FAILED;
                if matches!(error, crate::plant::PlantError::NonFinite { .. }) {
                    faults |= fault_flags::PLANT_NONFINITE;
                }
                plant_error = Some(error.to_string());
                // Publish a silent command before leaving, so the wheel is
                // slack the instant the plant is known to be bad.
                ffb_writer.publish(&chain.silent(monotonic_ns()));
                break;
            }
        };
        let t_after_plant = stamp(tracing);
        sim_time += dt;
        step_index += 1;

        // 4. Pose. VehicleFMI reports body-frame velocities but no global
        //    position, and adding one would mean editing BobLib. Integrating it
        //    here keeps BobLib untouched and gives the view something to draw.
        //    Replay uses the same `Pose`, so a paired A/B can never show a
        //    trajectory difference that came from the integrator.
        pose.advance(&state, dt);

        // 5. Did this step meet its deadline? The answer feeds the feedback
        //    watchdog: a loop that is not keeping up must not keep pushing
        //    torque as though nothing is wrong.
        let now = monotonic_ns();
        let deadline_met = now <= deadline;
        if !deadline_met {
            deadline_misses += 1;
            faults |= fault_flags::DEADLINE_MISSED;
        }

        let outcome = chain.condition(&state, dt, now, deadline_met);
        if outcome.clamped {
            faults |= fault_flags::FFB_CLAMPED;
        }
        if outcome.nonfinite {
            faults |= fault_flags::PLANT_NONFINITE;
        }
        let t_after_ffb = stamp(tracing);

        // 6. Fill in the health fields and publish.
        state.host_time_ns = now;
        pose.apply(&mut state);
        state.ffb_torque_nm = outcome.command.torque_nm;
        state.kernel_id = caps.id;
        state.deadline_misses = deadline_misses;
        state.fault_flags = faults;
        state.rtf = rtf.update(now, sim_time);
        state.step_time_us = (now.saturating_sub(step_start)) as f64 / 1000.0;
        state.step_time_p99_us = histogram.percentile_ns(0.99) as f64 / 1000.0;

        state_writer.publish(&state);
        ffb_writer.publish(&outcome.command);

        // The trace, if it is on. Pushed after everything the driver can feel
        // has already been published, so a full ring can never delay a torque.
        // `now` is the stamp the command itself carries, which is what the
        // device thread's record joins back to.
        if let Some(tx) = &trace_step_tx {
            let _ = tx.push(TraceStep {
                step_index,
                input_host_time_ns: input.host_time_ns,
                input_sample_index: input.sample_index,
                t_step_start: step_start,
                t_after_read,
                t_after_shape,
                t_after_plant,
                t_command_stamp: now,
                t_after_ffb,
                t_after_publish: stamp(tracing),
            });
        }

        // 7. Telemetry. A rejected push is a recorded fault, never a silent drop.
        if let Some(tx) = &telemetry_tx {
            if tx.push(state).is_err() {
                faults |= fault_flags::TELEMETRY_OVERFLOW;
            }
        }

        histogram.record(monotonic_ns().saturating_sub(step_start));

        // --- pacing ---------------------------------------------------------
        // `deadline` is the end of *this* step's slot, so the loop waits for it
        // and only then advances. Advancing first and then sleeping would make
        // every step be judged against a slot it had already spent, which reads
        // as a 100% miss rate on a loop that is in fact comfortably on time.
        let now = monotonic_ns();
        if now > deadline {
            // Behind by a whole step or more. Do NOT burst to catch up: running
            // several steps back to back spikes the feedback and diverges from
            // wall clock. Re-anchor and report instead -- a tool that silently
            // varies its own timescale is worse than useless, because the
            // driver's verdict then encodes the stutter rather than the setup.
            match config.overrun_policy {
                OverrunPolicy::DegradeAndReport => {
                    reanchors += 1;
                    deadline = now;
                }
                OverrunPolicy::Absorb => {}
            }
        } else {
            let _ = sleep_until_precise(deadline, config.spin_ns);
        }
        deadline += step_ns;
    }

    // --- shutdown ------------------------------------------------------------
    // Publish a silent command and give the device thread a moment to deliver
    // it before it is asked to stop. It zeroes the device on its way out as
    // well; this is belt and braces, and this is the one place that deserves it.
    ffb_writer.publish(&chain.silent(monotonic_ns()));
    std::thread::sleep(std::time::Duration::from_millis(20));
    stop.store(true, Ordering::Relaxed);
    let _ = hid_thread.join();
    if let Some(handle) = telemetry_thread {
        let _ = handle.join();
    }
    // Drop the producers before joining, so the drain sees empty rings and a
    // set stop flag rather than waiting out its poll interval on every exit.
    drop(trace_step_tx);
    if let Some(handle) = trace_thread {
        let _ = handle.join();
    }

    let wall_time_s = (monotonic_ns() - wall_start) as f64 * 1e-9;
    let telemetry_dropped = telemetry_stats.dropped.load(Ordering::Relaxed);
    if telemetry_dropped > 0 {
        faults |= fault_flags::TELEMETRY_OVERFLOW;
    }

    Ok(SessionReport {
        steps: step_index,
        sim_time,
        wall_time_s,
        rt_status,
        device: device_caps,
        histogram,
        deadline_misses,
        reanchors,
        fault_flags: faults,
        ffb_clamps: chain.clamp_events(),
        watchdog_trips: chain.watchdog().trips(),
        telemetry_frames: telemetry_stats.frames_written.load(Ordering::Relaxed),
        trace_steps: trace_stats.steps_written.load(Ordering::Relaxed),
        trace_dropped: trace_stats.dropped.load(Ordering::Relaxed),
        telemetry_dropped,
        stale_commands: hid_stats.stale_commands.load(Ordering::Relaxed),
        ladder_report,
        plant_error,
    })
}

/// Convenience for tests and benchmarks: a session with no hardware.
pub fn run_headless(config: KernelConfig) -> Result<SessionReport, String> {
    use crate::io::device::{NullDevice, RawInput};
    let device = NullDevice::scripted(|index| {
        let t = index as f64 * 1e-3;
        let command = ladder::maneuver(t);
        RawInput {
            steer: (command.steering_angle_command / 2.0).clamp(-1.0, 1.0),
            accelerator: command.accelerator_pedal_command,
            brake: command.brake_pedal_command,
            buttons: 0,
            host_time_ns: monotonic_ns(),
        }
    });
    run(config, device)
}

/// The default method used when nothing overrides it.
pub const DEFAULT_METHOD: Method = Method::Rk4;
