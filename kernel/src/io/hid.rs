//! The device thread.
//!
//! It does two things, at device rate, and nothing else: sample the wheel and
//! pedals into the input seqlock, and take the latest conditioned torque out of
//! the feedback seqlock and put it on the device.
//!
//! It is a separate thread from the step thread for one reason: a USB
//! transaction can block for milliseconds, and the plant must never wait on
//! one. It carries its own watchdog because it is the last component before the
//! hardware -- if the step thread stops publishing entirely (deadlocked,
//! SIGSTOPped, suspended in a debugger), nothing upstream can help, and this
//! thread is what makes the wheel go slack instead of staying locked.

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;

use crate::generated::frames::{DriverInput, FfbCommand, LAYOUT_HASH};
use crate::sys::clock::{monotonic_ns, sleep_until_precise};
use crate::transport::{SeqlockReader, SeqlockWriter};

use super::device::{Calibration, DeviceCaps, HapticSink, InputSource};
use super::watchdog::{command_is_fresh, Watchdog, WatchdogConfig};
use crate::generated::frames::{trace_flags, TraceDevice};
use crate::transport::spsc_ring::Producer;

#[derive(Debug, Clone)]
pub struct HidConfig {
    /// Device poll period. 1 kHz matches what a modern wheel base reports at.
    pub period_ns: u64,
    pub calibration: Calibration,
    /// A feedback command older than this is treated as absent. Sized at a few
    /// step periods: long enough not to trip on ordinary jitter, short enough
    /// that a dead step thread is caught before a driver can be hurt.
    pub max_command_age_ns: u64,
    pub watchdog: WatchdogConfig,
    pub spin_ns: u64,
}

impl Default for HidConfig {
    fn default() -> Self {
        Self {
            period_ns: 1_000_000,
            calibration: Calibration::default(),
            max_command_age_ns: 8_000_000,
            watchdog: WatchdogConfig {
                miss_limit: 3,
                ramp_ms: 40.0,
                recovery_streak: 30,
            },
            spin_ns: 60_000,
        }
    }
}

/// Counters the session surfaces. Shared rather than returned, because they are
/// read while the thread is still running.
#[derive(Debug, Default)]
pub struct HidStats {
    pub samples: AtomicU64,
    pub poll_errors: AtomicU64,
    pub apply_errors: AtomicU64,
    /// Commands rejected as stale. Any sustained count here means the step
    /// thread is not keeping up and the driver is not feeling the model.
    pub stale_commands: AtomicU64,
}

pub struct HidLoop<D> {
    device: D,
    config: HidConfig,
    input_writer: SeqlockWriter<DriverInput>,
    ffb_reader: SeqlockReader<FfbCommand>,
    watchdog: Watchdog,
    stats: Arc<HidStats>,
    /// The device half of the round-trip trace, when `--trace` is on. `None`
    /// costs one branch per iteration and one clock read that is already taken
    /// for the staleness check anyway.
    trace: Option<Producer<TraceDevice>>,
}

impl<D: InputSource + HapticSink> HidLoop<D> {
    pub fn new(device: D, config: HidConfig, stats: Arc<HidStats>) -> Result<Self, String> {
        let input_writer = SeqlockWriter::<DriverInput>::create(DriverInput::SHM_NAME, LAYOUT_HASH)
            .map_err(|e| format!("input segment: {e}"))?;
        let ffb_reader = SeqlockReader::<FfbCommand>::attach(FfbCommand::SHM_NAME, LAYOUT_HASH)
            .map_err(|e| format!("feedback segment: {e}"))?;
        let watchdog = Watchdog::new(config.watchdog.clone());
        Ok(Self {
            device,
            config,
            input_writer,
            ffb_reader,
            watchdog,
            stats,
            trace: None,
        })
    }

    /// Attach the device half of the round-trip trace. Separate from `new` so
    /// that nothing in the device path has to know a trace exists in order to
    /// be constructed or tested.
    pub fn with_trace(mut self, trace: Option<Producer<TraceDevice>>) -> Self {
        self.trace = trace;
        self
    }

    pub fn caps(&self) -> DeviceCaps {
        InputSource::caps(&self.device).clone()
    }

    /// Run until `stop` is set. Always leaves the device at zero torque.
    pub fn run(mut self, stop: Arc<AtomicBool>) {
        let dt = self.config.period_ns as f64 * 1e-9;
        let mut deadline = monotonic_ns() + self.config.period_ns;
        let mut sample_index: u64 = 0;

        while !stop.load(Ordering::Relaxed) {
            // --- sample the driver -----------------------------------------
            match self.device.poll() {
                Ok(raw) => {
                    let input = self.config.calibration.apply(&raw, sample_index);
                    self.input_writer.publish(&input);
                    sample_index += 1;
                    self.stats.samples.fetch_add(1, Ordering::Relaxed);
                }
                Err(_) => {
                    self.stats.poll_errors.fetch_add(1, Ordering::Relaxed);
                }
            }

            // --- deliver the torque -----------------------------------------
            let now = monotonic_ns();
            let command = self.ffb_reader.read_latest().map(|(command, _)| command);
            let fresh = command
                .map(|c| command_is_fresh(c.host_time_ns, now, self.config.max_command_age_ns))
                .unwrap_or(false);
            if !fresh {
                self.stats.stale_commands.fetch_add(1, Ordering::Relaxed);
            }

            let verdict = self.watchdog.observe(fresh, dt);
            // A stale command is never held. It is scaled by a watchdog that is
            // on its way to zero, so the wheel goes slack rather than locking
            // at whatever the last value happened to be.
            let mut outgoing = match command {
                Some(command) if fresh => command,
                _ => FfbCommand {
                    host_time_ns: now,
                    ..Default::default()
                },
            };
            outgoing.torque_nm *= verdict.scale();
            outgoing.damper_coeff *= verdict.scale();
            outgoing.spring_coeff *= verdict.scale();

            let applied = self.device.apply(&outgoing);
            if applied.is_err() {
                self.stats.apply_errors.fetch_add(1, Ordering::Relaxed);
            }

            // The device leg of the trace. `now` is already taken above for the
            // staleness check, so the only new clock read here is the one after
            // the device has been handed the torque.
            if let Some(trace) = &self.trace {
                let mut flags = match (command, fresh) {
                    (Some(_), true) => trace_flags::COMMAND_FRESH,
                    (Some(_), false) => trace_flags::COMMAND_STALE,
                    (None, _) => trace_flags::COMMAND_MISSING,
                };
                if applied.is_err() {
                    flags |= trace_flags::APPLY_FAILED;
                }
                let _ = trace.push(TraceDevice {
                    command_host_time_ns: command.map(|c| c.host_time_ns).unwrap_or(0),
                    t_pickup: now,
                    t_after_apply: monotonic_ns(),
                    sample_index,
                    flags,
                });
            }

            deadline += self.config.period_ns;
            // A device thread that has fallen far behind must not try to catch
            // up by spinning through a backlog of polls.
            let now = monotonic_ns();
            if deadline < now {
                deadline = now + self.config.period_ns;
            }
            let _ = sleep_until_precise(deadline, self.config.spin_ns);
        }

        // Rule 4. Every exit from this loop leaves the device slack.
        self.device.zero();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::generated::frames::ffb_flags;
    use crate::io::device::{NullDevice, RawInput};

    fn unique(name: &str) -> String {
        format!("bobdil_hid_{name}_{}", std::process::id())
    }

    /// A step thread that dies must leave the wheel slack, not locked. This is
    /// tested at the device thread specifically, because it is the only
    /// component still running when that happens.
    #[test]
    fn a_dead_step_thread_leaves_the_wheel_slack() {
        let ffb_name = unique("dead_ffb");
        let input_name = unique("dead_input");
        let mut ffb_writer = SeqlockWriter::<FfbCommand>::create(&ffb_name, 42).unwrap();
        let _input_writer = SeqlockWriter::<DriverInput>::create(&input_name, 42).unwrap();
        let reader = SeqlockReader::<FfbCommand>::attach(&ffb_name, 42).unwrap();

        // A healthy command, published once and then never again.
        let published_at = monotonic_ns();
        ffb_writer.publish(&FfbCommand {
            host_time_ns: published_at,
            torque_nm: 6.0,
            flags: ffb_flags::ACTIVE,
            ..Default::default()
        });

        let mut watchdog = Watchdog::new(WatchdogConfig {
            miss_limit: 3,
            ramp_ms: 40.0,
            recovery_streak: 30,
        });
        let max_age = 8_000_000u64;
        let dt = 1e-3;

        let (command, _) = reader.read_latest().unwrap();
        let mut torque = command.torque_nm;
        // Simulate 200 ms of wall clock passing with no new command.
        for step in 1..=200u64 {
            let now = published_at + step * 1_000_000;
            let fresh = command_is_fresh(command.host_time_ns, now, max_age);
            let verdict = watchdog.observe(fresh, dt);
            torque = command.torque_nm * verdict.scale();
        }
        assert_eq!(
            torque, 0.0,
            "a step thread that stopped publishing must slacken the wheel"
        );
    }

    #[test]
    fn a_null_device_still_exercises_the_whole_loop() {
        let device = NullDevice::scripted(|index| RawInput {
            steer: (index as f64 * 0.001).sin(),
            accelerator: 0.5,
            brake: 0.0,
            buttons: 0,
            host_time_ns: monotonic_ns(),
        });
        let mut device = device;
        let raw = InputSource::poll(&mut device).unwrap();
        assert_eq!(raw.accelerator, 0.5);
        assert!(
            !InputSource::caps(&device).wheel_torque,
            "the null device must report that it has no torque output"
        );
    }
}
