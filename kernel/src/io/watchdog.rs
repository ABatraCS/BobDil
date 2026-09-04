//! SAFETY. Read this before changing anything in it.
//!
//! A direct-drive wheel delivers more than 20 N.m. That is enough to break a
//! wrist. A torque that is stale, non-finite, or stuck at maximum is a physical
//! hazard, not a rendering artifact, and the spec does not mention it -- so this
//! module is where BobDil takes the position that the driver's safety is a
//! property of the kernel and not of the driver's reflexes.
//!
//! Four rules, none of them optional:
//!
//! 1. Non-finite torque means zero, immediately, and the session is faulted.
//! 2. Torque is clamped below the device's capability, configured before first
//!    launch.
//! 3. If the step thread misses `miss_limit` consecutive deadlines, torque
//!    ramps to zero within `ramp_ms`. It is never held at its last value.
//! 4. Torque is zeroed on every exit path: clean shutdown, panic, signal, and
//!    the parent process dying.
//!
//! Two independent watchdogs use this type. One lives on the step thread and
//! watches deadlines. The other lives on the device thread and watches the age
//! of the last command, so that a step thread which stops publishing entirely --
//! deadlocked, suspended in a debugger, or SIGSTOPped -- still results in the
//! wheel going slack rather than locking solid.

/// What the watchdog says the torque should be multiplied by, right now.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Verdict {
    /// Healthy. Pass torque through unchanged.
    Pass,
    /// Failing. Multiply by `scale`, which is on its way to zero.
    RampingDown { scale: f64 },
    /// Fully tripped. Torque is zero and stays zero until `recover` is called.
    Tripped,
}

impl Verdict {
    pub fn scale(&self) -> f64 {
        match self {
            Self::Pass => 1.0,
            Self::RampingDown { scale } => *scale,
            Self::Tripped => 0.0,
        }
    }

    pub fn is_healthy(&self) -> bool {
        matches!(self, Self::Pass)
    }
}

#[derive(Debug, Clone)]
pub struct WatchdogConfig {
    /// Consecutive failures tolerated before the ramp starts.
    pub miss_limit: u32,
    /// Time from trip to zero torque. Short enough that a driver cannot fight
    /// a stuck torque for long; long enough that the wheel does not snap out of
    /// their hands.
    pub ramp_ms: f64,
    /// Consecutive healthy observations needed to leave the ramp. Deliberately
    /// larger than one, so a loop that is flapping in and out of its deadline
    /// does not produce a torque that pulses.
    pub recovery_streak: u32,
}

impl Default for WatchdogConfig {
    fn default() -> Self {
        Self {
            miss_limit: 5,
            ramp_ms: 50.0,
            recovery_streak: 50,
        }
    }
}

/// Tracks consecutive failures and produces a torque scale.
#[derive(Debug, Clone)]
pub struct Watchdog {
    config: WatchdogConfig,
    consecutive_misses: u32,
    healthy_streak: u32,
    scale: f64,
    tripped: bool,
    trips: u64,
}

impl Watchdog {
    pub fn new(config: WatchdogConfig) -> Self {
        Self {
            config,
            consecutive_misses: 0,
            healthy_streak: 0,
            scale: 1.0,
            tripped: false,
            trips: 0,
        }
    }

    /// Record one observation and advance the ramp by `dt` seconds.
    ///
    /// `healthy` is "the step met its deadline" on the step thread, and "the
    /// last command is fresh" on the device thread.
    pub fn observe(&mut self, healthy: bool, dt: f64) -> Verdict {
        if healthy {
            self.consecutive_misses = 0;
            self.healthy_streak = self.healthy_streak.saturating_add(1);
        } else {
            self.healthy_streak = 0;
            self.consecutive_misses = self.consecutive_misses.saturating_add(1);
            if self.consecutive_misses >= self.config.miss_limit && !self.tripped {
                self.tripped = true;
                self.trips += 1;
            }
        }

        if self.tripped {
            // Ramp down while tripped. Recovery needs a sustained healthy
            // streak, so a loop that is flapping does not produce pulsing torque.
            if self.healthy_streak >= self.config.recovery_streak {
                self.tripped = false;
                self.scale = (self.scale + dt / (self.config.ramp_ms * 1e-3)).min(1.0);
            } else {
                let step = dt / (self.config.ramp_ms * 1e-3).max(1e-9);
                self.scale = (self.scale - step).max(0.0);
            }
        } else if self.scale < 1.0 {
            // Ramp back up at the same rate, never instantly: a torque that
            // jumps from zero to full is exactly the impulse this protects against.
            let step = dt / (self.config.ramp_ms * 1e-3).max(1e-9);
            self.scale = (self.scale + step).min(1.0);
        }

        if self.tripped && self.scale <= 0.0 {
            Verdict::Tripped
        } else if self.scale < 1.0 {
            Verdict::RampingDown { scale: self.scale }
        } else {
            Verdict::Pass
        }
    }

    /// Force the watchdog fully open with zero torque. Used on a fault that has
    /// no recovery path, such as a non-finite plant output.
    pub fn trip_now(&mut self) {
        if !self.tripped {
            self.trips += 1;
        }
        self.tripped = true;
        self.scale = 0.0;
        self.healthy_streak = 0;
    }

    /// Clear a trip explicitly, after the operator has acknowledged it.
    pub fn recover(&mut self) {
        self.tripped = false;
        self.consecutive_misses = 0;
        self.healthy_streak = 0;
        self.scale = 0.0; // Still ramps up; never snaps back to full torque.
    }

    pub fn is_tripped(&self) -> bool {
        self.tripped
    }

    pub fn trips(&self) -> u64 {
        self.trips
    }

    pub fn scale(&self) -> f64 {
        self.scale
    }
}

/// Staleness check for the device thread.
///
/// Deliberately a free function on an age rather than a timer the step thread
/// resets, so that a step thread which stops running entirely cannot keep this
/// watchdog satisfied by accident.
pub fn command_is_fresh(command_time_ns: u64, now_ns: u64, max_age_ns: u64) -> bool {
    if command_time_ns == 0 {
        return false;
    }
    now_ns.saturating_sub(command_time_ns) <= max_age_ns
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f64 = 1e-3;

    #[test]
    fn healthy_operation_passes_torque_through_untouched() {
        let mut watchdog = Watchdog::new(WatchdogConfig::default());
        for _ in 0..10_000 {
            assert_eq!(watchdog.observe(true, DT), Verdict::Pass);
        }
        assert_eq!(watchdog.trips(), 0);
    }

    #[test]
    fn a_few_misses_do_not_trip_it() {
        let mut watchdog = Watchdog::new(WatchdogConfig {
            miss_limit: 5,
            ..Default::default()
        });
        for _ in 0..4 {
            assert!(watchdog.observe(false, DT).is_healthy());
        }
        assert!(
            !watchdog.is_tripped(),
            "4 misses under a limit of 5 must not trip"
        );
    }

    /// Rule 3, stated as a test: a stalled step thread must leave the wheel
    /// slack within the ramp time, and must never hold its last torque.
    #[test]
    fn a_stalled_loop_reaches_zero_torque_within_the_ramp_time() {
        let mut watchdog = Watchdog::new(WatchdogConfig {
            miss_limit: 5,
            ramp_ms: 50.0,
            recovery_streak: 50,
        });
        for _ in 0..5 {
            watchdog.observe(false, DT);
        }
        assert!(watchdog.is_tripped());

        // 50 ms of ramp at 1 kHz, plus the step that reaches exactly zero.
        for _ in 0..51 {
            watchdog.observe(false, DT);
        }
        assert_eq!(watchdog.observe(false, DT), Verdict::Tripped);
        assert_eq!(
            watchdog.scale(),
            0.0,
            "torque must reach exactly zero, not merely become small"
        );
    }

    #[test]
    fn the_ramp_is_monotonic_so_the_wheel_never_snaps() {
        let mut watchdog = Watchdog::new(WatchdogConfig::default());
        for _ in 0..5 {
            watchdog.observe(false, DT);
        }
        let mut previous = watchdog.scale();
        for _ in 0..100 {
            watchdog.observe(false, DT);
            let current = watchdog.scale();
            assert!(
                current <= previous + 1e-12,
                "scale rose during a trip: {previous} -> {current}"
            );
            previous = current;
        }
    }

    #[test]
    fn recovery_is_gradual_and_needs_a_sustained_healthy_streak() {
        let mut watchdog = Watchdog::new(WatchdogConfig {
            miss_limit: 3,
            ramp_ms: 50.0,
            recovery_streak: 50,
        });
        for _ in 0..60 {
            watchdog.observe(false, DT);
        }
        assert_eq!(watchdog.scale(), 0.0);

        // A short healthy burst must not restore torque.
        for _ in 0..10 {
            watchdog.observe(true, DT);
        }
        assert_eq!(
            watchdog.scale(),
            0.0,
            "a brief recovery must not restore torque"
        );

        // A sustained one does, and even then it ramps.
        for _ in 0..60 {
            watchdog.observe(true, DT);
        }
        assert!(
            watchdog.scale() > 0.0 && watchdog.scale() < 1.0,
            "recovery must ramp, not snap"
        );

        for _ in 0..200 {
            watchdog.observe(true, DT);
        }
        assert_eq!(watchdog.scale(), 1.0);
    }

    #[test]
    fn an_unrecoverable_fault_trips_instantly() {
        let mut watchdog = Watchdog::new(WatchdogConfig::default());
        watchdog.trip_now();
        assert_eq!(watchdog.scale(), 0.0);
        assert!(watchdog.is_tripped());
        assert_eq!(watchdog.trips(), 1);
    }

    #[test]
    fn a_command_that_never_arrived_is_never_fresh() {
        assert!(
            !command_is_fresh(0, 1_000_000, 5_000_000),
            "a zero stamp means no command yet"
        );
    }

    #[test]
    fn staleness_is_measured_against_the_clock_not_a_flag() {
        let max_age = 5_000_000; // 5 ms
        assert!(command_is_fresh(1_000_000, 4_000_000, max_age));
        assert!(!command_is_fresh(1_000_000, 7_000_000, max_age));
    }
}
