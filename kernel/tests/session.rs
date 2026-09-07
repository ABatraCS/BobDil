//! End-to-end session tests.
//!
//! These run the real loop -- real threads, real shared memory, real pacing --
//! against a scripted null device. They are the tests that would have caught a
//! loop which reports itself healthy while missing every deadline, or one whose
//! telemetry quietly loses samples.
//!
//! They share fixed shared-memory segment names, and every session clears
//! stale segments as it starts, so two running at once would tear down each
//! other's transport. `SESSION_LOCK` serialises them: without it the failure is
//! confusing rather than obvious, because the victim is whichever session was
//! mid-flight, not the one that did the clearing.

use std::path::PathBuf;
use std::sync::Mutex;

use bobdil_kernel::config::KernelConfig;
use bobdil_kernel::generated::frames::fault_flags;
use bobdil_kernel::loop_runner;
use bobdil_kernel::telemetry::TelemetryReader;

static SESSION_LOCK: Mutex<()> = Mutex::new(());

/// Take the session lock, ignoring poisoning: a panic in one session test must
/// surface as that test's own failure, not as a poison error in every other.
fn session_guard() -> std::sync::MutexGuard<'static, ()> {
    SESSION_LOCK.lock().unwrap_or_else(|e| e.into_inner())
}

fn scratch(name: &str) -> PathBuf {
    std::env::temp_dir().join(format!("bobdil_session_{name}_{}.bdt", std::process::id()))
}

#[test]
fn a_headless_session_holds_its_deadline_and_records_every_step() {
    let _guard = session_guard();
    let telemetry = scratch("headless");
    let config = KernelConfig {
        duration_s: Some(2.0),
        telemetry_path: Some(telemetry.clone()),
        // CI runners are shared and unprivileged; asking for SCHED_FIFO there
        // just adds noise to the report.
        request_realtime: false,
        ..Default::default()
    };

    let report = loop_runner::run(
        config,
        bobdil_kernel::io::device::NullDevice::scripted(|index| {
            let t = index as f64 * 1e-3;
            let command = bobdil_kernel::plant::ladder::maneuver(t);
            bobdil_kernel::io::device::RawInput {
                steer: (command.steering_angle_command / 2.0).clamp(-1.0, 1.0),
                accelerator: command.accelerator_pedal_command,
                brake: command.brake_pedal_command,
                buttons: 0,
                host_time_ns: bobdil_kernel::sys::clock::monotonic_ns(),
            }
        }),
    )
    .expect("a headless session must run");

    // --- the loop actually ran in real time -----------------------------
    assert_eq!(report.steps, 2000, "2 s at 1 kHz is 2000 steps");
    assert!(report.plant_error.is_none(), "{:?}", report.plant_error);
    let rtf = report.sim_time / report.wall_time_s;
    assert!(
        (0.95..=1.05).contains(&rtf),
        "the loop must track wall clock; realtime factor was {rtf:.4}"
    );

    // --- and it knows whether it did ------------------------------------
    // A shared CI machine will drop the occasional step. What must not happen
    // is a loop that misses deadlines wholesale, or one that reports success
    // while doing so.
    let miss_rate = report.deadline_misses as f64 / report.steps as f64;
    assert!(
        miss_rate < 0.05,
        "missed {:.2}% of deadlines ({}), step time {}",
        miss_rate * 100.0,
        report.deadline_misses,
        report.histogram.summary_us()
    );
    assert_eq!(
        report.fault_flags & fault_flags::PLANT_NONFINITE,
        0,
        "the plant must stay finite for a whole session"
    );

    // --- telemetry is complete ---------------------------------------------
    assert_eq!(
        report.telemetry_dropped, 0,
        "telemetry must be lossless below capacity"
    );
    let recording = TelemetryReader::open(&telemetry).expect("recording must be readable");
    assert_eq!(
        recording.frames.len() as u64,
        report.telemetry_frames,
        "every frame the recorder counted must be in the file"
    );
    assert!(
        recording.frames.len() >= 1900,
        "almost every step should be recorded"
    );

    // Step indices must be contiguous: a gap means a lost sample, which would
    // silently invalidate any A/B comparison made from this file.
    for pair in recording.frames.windows(2) {
        assert_eq!(
            pair[1].step_index,
            pair[0].step_index + 1,
            "telemetry has a hole at step {}",
            pair[0].step_index
        );
    }

    // --- the car did something ---------------------------------------------
    let fastest = recording
        .frames
        .iter()
        .map(|frame| frame.vehicle_speed)
        .fold(0.0f64, f64::max);
    assert!(
        fastest > 5.0,
        "the scripted driver should have got moving, peaked at {fastest:.1} m/s"
    );
    assert!(
        recording
            .frames
            .iter()
            .any(|frame| frame.ffb_torque_nm.abs() > 0.1),
        "the session must have produced real force feedback"
    );

    // --- pose was integrated -------------------------------------------------
    let travelled = recording
        .frames
        .last()
        .map(|frame| (frame.pos_x * frame.pos_x + frame.pos_y * frame.pos_y).sqrt())
        .unwrap_or(0.0);
    assert!(
        travelled > 5.0,
        "the car should have covered ground, got {travelled:.1} m"
    );

    let _ = std::fs::remove_file(&telemetry);
}

// --- the round-trip span trace ------------------------------------------
//
// The two promises the trace makes: it costs nothing when it is off, and when
// it is on its spans genuinely bound the phases they claim to bound. Neither is
// a timing assertion -- a trace measures *this box* and its numbers do not
// travel (AGENTS.md), so these test the plumbing and the ordering only.

fn trace_scratch(name: &str) -> PathBuf {
    std::env::temp_dir().join(format!(
        "bobdil_trace_{name}_{}.bdtrace",
        std::process::id()
    ))
}

#[test]
fn a_session_without_trace_writes_no_trace_at_all() {
    let _guard = session_guard();
    let path = trace_scratch("off");
    std::fs::remove_file(&path).ok();

    let config = KernelConfig {
        duration_s: Some(0.2),
        request_realtime: false,
        trace_path: None,
        ..Default::default()
    };
    let report = loop_runner::run_headless(config).expect("session must run");

    assert!(report.steps > 0);
    assert!(
        !path.exists(),
        "tracing is off by default and must leave nothing behind"
    );
    assert_eq!(
        report.trace_steps, 0,
        "no span may be recorded when tracing is off"
    );
}

#[test]
fn a_traced_session_writes_spans_that_bound_their_phases() {
    let _guard = session_guard();
    let path = trace_scratch("on");
    std::fs::remove_file(&path).ok();

    let config = KernelConfig {
        duration_s: Some(0.2),
        request_realtime: false,
        trace_path: Some(path.clone()),
        ..Default::default()
    };
    let report = loop_runner::run_headless(config).expect("session must run");

    assert!(
        report.trace_steps > 0,
        "a traced session must record spans; got {}",
        report.trace_steps
    );

    let trace = bobdil_kernel::telemetry::trace::TraceReader::open(&path)
        .expect("the trace this build wrote must be readable by this build");
    std::fs::remove_file(&path).ok();

    assert_eq!(
        trace.steps.len() as u64,
        report.trace_steps,
        "every recorded span must reach the file"
    );

    for span in &trace.steps {
        // The stamps are taken in order, so they must come back in order. A
        // regression that stamps a phase in the wrong place shows up here as a
        // span that ends before it starts.
        let stamps = [
            span.t_step_start,
            span.t_after_read,
            span.t_after_shape,
            span.t_after_plant,
            span.t_command_stamp,
            span.t_after_ffb,
            span.t_after_publish,
        ];
        for pair in stamps.windows(2) {
            assert!(
                pair[1] >= pair[0],
                "step {} has a span that ends before it starts: {stamps:?}",
                span.step_index
            );
        }
        assert!(
            span.input_host_time_ns <= span.t_step_start,
            "a step cannot consume an input sampled after it began"
        );
        // Ordering alone would be satisfied by stamps that are all zero, which
        // is exactly what a tracing flag wired up wrong would produce.
        assert!(
            span.t_after_publish > span.t_step_start,
            "step {} recorded no elapsed time at all: {stamps:?}",
            span.step_index
        );
        assert_eq!(
            span.t_command_stamp, stamps[4],
            "the join key must be one of the recorded stamps"
        );
    }
}

#[test]
fn a_traced_session_records_the_device_leg_and_flags_what_it_delivered() {
    let _guard = session_guard();
    let path = trace_scratch("device");
    std::fs::remove_file(&path).ok();

    let config = KernelConfig {
        duration_s: Some(0.2),
        request_realtime: false,
        trace_path: Some(path.clone()),
        ..Default::default()
    };
    loop_runner::run_headless(config).expect("session must run");

    let trace = bobdil_kernel::telemetry::trace::TraceReader::open(&path).expect("readable trace");
    std::fs::remove_file(&path).ok();

    assert!(
        !trace.devices.is_empty(),
        "the device leg is half the round trip and must be recorded"
    );

    use bobdil_kernel::generated::frames::trace_flags;
    let mut joined = 0usize;
    let stamps: std::collections::HashSet<u64> =
        trace.steps.iter().map(|s| s.t_command_stamp).collect();

    for record in &trace.devices {
        assert!(
            record.t_after_apply >= record.t_pickup,
            "a delivery cannot finish before it was picked up"
        );
        // The flag is what keeps a broken leg out of a latency percentile, so
        // every record must say which it was rather than leaving it to be
        // inferred from a suspicious number.
        assert_ne!(
            record.flags,
            trace_flags::NONE,
            "every device record must state what it did with the command"
        );
        if record.flags & trace_flags::COMMAND_FRESH != 0 {
            assert!(
                record.command_host_time_ns > 0,
                "a fresh command carries the stamp its step wrote"
            );
            if stamps.contains(&record.command_host_time_ns) {
                joined += 1;
            }
        }
    }

    assert!(
        joined > 0,
        "no device record joined back to a step: the round trip is not connected. \
         {} devices, {} steps",
        trace.devices.len(),
        trace.steps.len()
    );
}
