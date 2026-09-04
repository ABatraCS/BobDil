//! Session configuration.
//!
//! Everything that changes how the loop behaves lives in one place so a session
//! can be described, recorded alongside its telemetry, and reproduced exactly.

use std::path::PathBuf;

use crate::plant::integrator::Method;

/// How the loop should behave when it cannot keep up (architecture.md 1.8).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OverrunPolicy {
    /// Absorb a single late step and carry on. Correct for the ordinary case:
    /// one late step is invisible to a driver.
    Absorb,
    /// Under sustained overrun, stretch the step rather than bursting to catch
    /// up, and tell the driver. Bursting four steps back to back spikes the
    /// force feedback and diverges from wall clock -- a tool that silently
    /// varies its own timescale is worse than useless, because the driver's
    /// verdict then encodes the stutter rather than the setup.
    DegradeAndReport,
}

#[derive(Debug, Clone)]
pub struct KernelConfig {
    /// Fixed step. 1 ms is the baseline; the ladder may select a slower one.
    pub step_dt: f64,
    /// Integration method for the host-owned integrator.
    pub method: Method,
    pub substeps: usize,
    /// Directory holding an unpacked FMI 2.0 Model Exchange FMU. `None` selects
    /// the built-in reduced kernel.
    pub plant_dir: Option<PathBuf>,
    pub initial_speed: f64,
    /// CPU to pin the step thread to. `None` lets the OS decide.
    pub rt_cpu: Option<usize>,
    /// Ask for SCHED_FIFO. Best-effort: an unprivileged session still runs.
    pub request_realtime: bool,
    /// How long to busy-wait before a deadline, trading a core for jitter.
    pub spin_ns: u64,
    pub overrun_policy: OverrunPolicy,
    /// Consecutive missed deadlines before the force-feedback watchdog trips.
    pub watchdog_miss_limit: u32,
    /// Telemetry ring capacity in steps. 60 s at 1 kHz by default, so a
    /// momentarily blocked disk cannot lose samples.
    pub telemetry_capacity: usize,
    pub telemetry_path: Option<PathBuf>,
    /// Absolute force-feedback torque clamp [N.m]. Must be set below the
    /// device's capability before anyone drives.
    pub ffb_torque_limit: f64,
    /// Overall force-feedback gain, 0 disables feedback entirely.
    pub ffb_gain: f64,
    pub duration_s: Option<f64>,
}

impl Default for KernelConfig {
    fn default() -> Self {
        Self {
            step_dt: 1e-3,
            method: Method::Rk4,
            substeps: 1,
            plant_dir: None,
            initial_speed: 0.0,
            rt_cpu: crate::sys::sched::suggested_rt_cpu(),
            request_realtime: true,
            spin_ns: 120_000,
            overrun_policy: OverrunPolicy::DegradeAndReport,
            watchdog_miss_limit: 5,
            telemetry_capacity: 65_536,
            telemetry_path: None,
            // 8 N.m is a firm but survivable default. A direct-drive wheel can
            // deliver 20+ N.m, which is enough to injure a wrist, so the limit
            // is opt-in-raised rather than opt-out-lowered.
            ffb_torque_limit: 8.0,
            ffb_gain: 1.0,
            duration_s: None,
        }
    }
}

impl KernelConfig {
    pub fn step_ns(&self) -> u64 {
        (self.step_dt * 1e9).round() as u64
    }

    pub fn rate_hz(&self) -> f64 {
        1.0 / self.step_dt
    }
}
