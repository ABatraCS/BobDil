//! The force-feedback conditioning chain.
//!
//! The plant reports a reaction torque. Between that number and the driver's
//! hands sit six stages, always in this order (architecture.md 1.6):
//!
//! ```text
//!   NaN/inf guard -> gain -> slew limit -> soft end stop -> clamp -> watchdog
//! ```
//!
//! Every stage is parameterised and individually bypassable, because "did the
//! force-feedback tuning or the vehicle change explain what the driver just
//! felt?" is a question this rig exists to answer, and it cannot be answered if
//! the chain is a black box.
//!
//! The chain deliberately does *not* add spring, inertia or canned effects.
//! Anything the driver feels must be derived from the model, which is the whole
//! premise of the product; the one exception is the end stop, which represents
//! the rack running out of travel and is a real feature of the car.

use crate::generated::frames::{ffb_flags, FfbCommand, VehicleState};

use super::watchdog::{Verdict, Watchdog, WatchdogConfig};

#[derive(Debug, Clone)]
pub struct FfbConfig {
    /// Overall gain. 0 disables feedback without changing anything else, which
    /// is the control condition for "is it the tune or the car?".
    pub gain: f64,
    /// Absolute torque limit [N.m]. Set below the device's capability before
    /// anyone drives. This is the last line between a solver divergence and a
    /// driver's wrist.
    pub torque_limit_nm: f64,
    /// Maximum rate of change [N.m/s]. Bounds the impulse a single bad step can
    /// deliver even if it somehow passes every other stage.
    pub slew_limit_nm_per_s: f64,
    /// Handwheel angle at which the soft end stop begins [rad].
    pub end_stop_start_rad: f64,
    /// Stiffness of the end stop past that point [N.m/rad].
    pub end_stop_rate: f64,
    /// Viscous damping against handwheel rate [N.m.s/rad], passed to the device
    /// as a condition effect rather than computed from a differentiated
    /// position, which would chatter.
    pub damper_coeff: f64,
    pub enable_slew_limit: bool,
    pub enable_end_stop: bool,
    pub watchdog: WatchdogConfig,
}

impl Default for FfbConfig {
    fn default() -> Self {
        Self {
            gain: 1.0,
            torque_limit_nm: 8.0,
            slew_limit_nm_per_s: 600.0,
            end_stop_start_rad: 2.2,
            end_stop_rate: 12.0,
            damper_coeff: 0.02,
            enable_slew_limit: true,
            enable_end_stop: true,
            watchdog: WatchdogConfig::default(),
        }
    }
}

/// What the chain produced, and what it had to do to get there.
#[derive(Debug, Clone, Copy)]
pub struct FfbOutcome {
    pub command: FfbCommand,
    /// The clamp was active. Sustained clamping means the limit is set below
    /// what the car actually generates, and the driver is feeling the limiter
    /// rather than the vehicle -- so it is surfaced, not swallowed.
    pub clamped: bool,
    /// The plant produced a non-finite value. Always a session fault.
    pub nonfinite: bool,
    pub watchdog: Verdict,
}

pub struct FfbChain {
    config: FfbConfig,
    watchdog: Watchdog,
    last_torque: f64,
    clamp_events: u64,
    nonfinite_events: u64,
}

impl FfbChain {
    pub fn new(config: FfbConfig) -> Self {
        let watchdog = Watchdog::new(config.watchdog.clone());
        Self {
            config,
            watchdog,
            last_torque: 0.0,
            clamp_events: 0,
            nonfinite_events: 0,
        }
    }

    pub fn config(&self) -> &FfbConfig {
        &self.config
    }

    pub fn clamp_events(&self) -> u64 {
        self.clamp_events
    }

    pub fn nonfinite_events(&self) -> u64 {
        self.nonfinite_events
    }

    pub fn watchdog(&self) -> &Watchdog {
        &self.watchdog
    }

    /// Run one step of the chain.
    ///
    /// `deadline_met` is the step thread's own verdict on the step that
    /// produced this state, and it feeds the watchdog: a loop that is missing
    /// deadlines must not keep pushing torque as though nothing is wrong.
    pub fn condition(
        &mut self,
        state: &VehicleState,
        dt: f64,
        now_ns: u64,
        deadline_met: bool,
    ) -> FfbOutcome {
        let mut flags = ffb_flags::ACTIVE;

        // --- 1. non-finite guard ------------------------------------------
        // First, unconditionally, and it trips the watchdog outright: there is
        // no recovery from a diverged plant, and no torque value derived from
        // one is meaningful.
        let reaction = state.handwheel_torque;
        if !reaction.is_finite() || !state.handwheel_angle.is_finite() {
            self.nonfinite_events += 1;
            self.watchdog.trip_now();
            self.last_torque = 0.0;
            return FfbOutcome {
                command: FfbCommand {
                    host_time_ns: now_ns,
                    torque_nm: 0.0,
                    spring_coeff: 0.0,
                    damper_coeff: 0.0,
                    flags: ffb_flags::WATCHDOG_TRIPPED | ffb_flags::DISABLED,
                },
                clamped: false,
                nonfinite: true,
                watchdog: Verdict::Tripped,
            };
        }

        // The plant reports a reaction torque, matching VehicleFMI's
        // `handwheelTorque = -tau` convention. The torque *applied* to the
        // driver is its negation.
        let mut torque = -reaction * self.config.gain;

        // --- 2. soft end stop ----------------------------------------------
        // The rack has run out of travel. This is a real feature of the car,
        // not an effect: past the limit the driver is pushing against the rack.
        if self.config.enable_end_stop {
            let over = state.handwheel_angle.abs() - self.config.end_stop_start_rad;
            if over > 0.0 {
                torque -= state.handwheel_angle.signum() * over * self.config.end_stop_rate;
            }
        }

        // --- 3. slew limit ---------------------------------------------------
        if self.config.enable_slew_limit {
            let max_change = self.config.slew_limit_nm_per_s * dt;
            let change = (torque - self.last_torque).clamp(-max_change, max_change);
            torque = self.last_torque + change;
        }

        // --- 4. absolute clamp -----------------------------------------------
        let limit = self.config.torque_limit_nm.abs();
        let clamped = torque.abs() > limit;
        if clamped {
            self.clamp_events += 1;
            torque = torque.clamp(-limit, limit);
        }

        // --- 5. watchdog ------------------------------------------------------
        let verdict = self.watchdog.observe(deadline_met, dt);
        torque *= verdict.scale();
        match verdict {
            Verdict::Pass => {}
            Verdict::RampingDown { .. } => flags |= ffb_flags::RAMPING_DOWN,
            Verdict::Tripped => flags = ffb_flags::WATCHDOG_TRIPPED,
        }

        self.last_torque = torque;

        FfbOutcome {
            command: FfbCommand {
                host_time_ns: now_ns,
                torque_nm: torque,
                spring_coeff: 0.0,
                damper_coeff: self.config.damper_coeff * verdict.scale(),
                flags,
            },
            clamped,
            nonfinite: false,
            watchdog: verdict,
        }
    }

    /// A command that is safe to send when there is nothing to send.
    ///
    /// Used on shutdown, on pause, and by the device thread when the step
    /// thread has gone quiet. Never returns the last value.
    pub fn silent(&self, now_ns: u64) -> FfbCommand {
        FfbCommand {
            host_time_ns: now_ns,
            torque_nm: 0.0,
            spring_coeff: 0.0,
            damper_coeff: 0.0,
            flags: ffb_flags::DISABLED,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f64 = 1e-3;

    fn state(torque: f64, angle: f64) -> VehicleState {
        VehicleState {
            handwheel_torque: torque,
            handwheel_angle: angle,
            ..Default::default()
        }
    }

    fn settled(chain: &mut FfbChain, torque: f64, angle: f64, steps: usize) -> FfbOutcome {
        let mut outcome = chain.condition(&state(torque, angle), DT, 0, true);
        for i in 1..steps {
            outcome = chain.condition(&state(torque, angle), DT, i as u64, true);
        }
        outcome
    }

    #[test]
    fn a_reaction_torque_is_negated_into_a_restoring_torque() {
        let mut chain = FfbChain::new(FfbConfig::default());
        // Steering left produces a positive reaction; the driver must feel a
        // torque pushing the wheel back to the right.
        let outcome = settled(&mut chain, 3.0, 0.4, 2000);
        assert!(
            outcome.command.torque_nm < -2.0,
            "expected a restoring torque, got {}",
            outcome.command.torque_nm
        );
    }

    /// Rule 1. The single most important test in the repository.
    #[test]
    fn a_non_finite_plant_output_produces_zero_torque_in_one_step() {
        for poison in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            let mut chain = FfbChain::new(FfbConfig::default());
            settled(&mut chain, 5.0, 0.5, 500);
            let outcome = chain.condition(&state(poison, 0.5), DT, 0, true);
            assert_eq!(
                outcome.command.torque_nm, 0.0,
                "{poison} must produce exactly zero torque"
            );
            assert!(outcome.nonfinite);
            assert!(
                chain.watchdog().is_tripped(),
                "a diverged plant must trip the watchdog"
            );
        }
    }

    #[test]
    fn a_non_finite_handwheel_angle_is_caught_as_well() {
        let mut chain = FfbChain::new(FfbConfig::default());
        let outcome = chain.condition(&state(2.0, f64::NAN), DT, 0, true);
        assert_eq!(outcome.command.torque_nm, 0.0);
        assert!(outcome.nonfinite);
    }

    /// Rule 2. No input, however extreme, may exceed the configured limit.
    #[test]
    fn torque_never_exceeds_the_configured_limit() {
        let mut chain = FfbChain::new(FfbConfig {
            torque_limit_nm: 6.0,
            slew_limit_nm_per_s: 1e9,
            ..Default::default()
        });
        for magnitude in [10.0, 100.0, 1e6, 1e12] {
            for sign in [1.0, -1.0] {
                let outcome = chain.condition(&state(sign * magnitude, 0.0), DT, 0, true);
                assert!(
                    outcome.command.torque_nm.abs() <= 6.0 + 1e-9,
                    "{} N.m of reaction produced {} N.m at the wheel",
                    sign * magnitude,
                    outcome.command.torque_nm
                );
                assert!(outcome.clamped);
            }
        }
    }

    #[test]
    fn the_slew_limit_bounds_a_single_step_impulse() {
        let mut chain = FfbChain::new(FfbConfig {
            slew_limit_nm_per_s: 400.0,
            torque_limit_nm: 20.0,
            ..Default::default()
        });
        let outcome = chain.condition(&state(20.0, 0.0), DT, 0, true);
        assert!(
            outcome.command.torque_nm.abs() <= 400.0 * DT + 1e-9,
            "one step jumped to {} N.m",
            outcome.command.torque_nm
        );
    }

    #[test]
    fn the_end_stop_pushes_back_only_past_the_rack_limit() {
        let mut chain = FfbChain::new(FfbConfig {
            end_stop_start_rad: 2.0,
            end_stop_rate: 10.0,
            slew_limit_nm_per_s: 1e9,
            torque_limit_nm: 50.0,
            ..Default::default()
        });
        let inside = chain.condition(&state(0.0, 1.9), DT, 0, true);
        assert!(
            inside.command.torque_nm.abs() < 1e-9,
            "no end stop before the rack limit"
        );

        let outside = chain.condition(&state(0.0, 2.5), DT, 0, true);
        assert!(
            outside.command.torque_nm < -4.0,
            "past the rack limit the wheel must push back, got {}",
            outside.command.torque_nm
        );
    }

    /// Rule 3, end to end through the chain rather than the watchdog alone.
    #[test]
    fn a_missed_deadline_streak_ramps_torque_to_zero_and_never_holds_it() {
        let mut chain = FfbChain::new(FfbConfig {
            slew_limit_nm_per_s: 1e9,
            watchdog: WatchdogConfig {
                miss_limit: 5,
                ramp_ms: 50.0,
                recovery_streak: 50,
            },
            ..Default::default()
        });
        settled(&mut chain, 5.0, 0.5, 100);
        let before = chain
            .condition(&state(5.0, 0.5), DT, 0, true)
            .command
            .torque_nm;
        assert!(
            before.abs() > 1.0,
            "precondition: real torque before the stall"
        );

        let mut last = before;
        for step in 0..200 {
            let outcome = chain.condition(&state(5.0, 0.5), DT, step, false);
            assert!(
                outcome.command.torque_nm.abs() <= last.abs() + 1e-9,
                "torque grew during a stall at step {step}"
            );
            last = outcome.command.torque_nm;
        }
        assert_eq!(
            last, 0.0,
            "a sustained stall must reach exactly zero torque"
        );
    }

    #[test]
    fn zero_gain_silences_the_wheel_without_changing_anything_else() {
        let mut chain = FfbChain::new(FfbConfig {
            gain: 0.0,
            ..Default::default()
        });
        let outcome = settled(&mut chain, 9.0, 0.5, 500);
        assert_eq!(outcome.command.torque_nm, 0.0);
        assert!(!outcome.nonfinite, "a silenced wheel is not a fault");
    }

    #[test]
    fn the_silent_command_is_zero_on_every_field_that_can_move_the_wheel() {
        let chain = FfbChain::new(FfbConfig::default());
        let command = chain.silent(1234);
        assert_eq!(command.torque_nm, 0.0);
        assert_eq!(command.spring_coeff, 0.0);
        assert_eq!(command.damper_coeff, 0.0);
        assert_eq!(command.flags, ffb_flags::DISABLED);
    }

    /// Sustained clamping means the driver is feeling the limiter, not the car.
    /// It must be countable so the UI can say so.
    #[test]
    fn clamping_is_counted_so_it_can_be_reported() {
        let mut chain = FfbChain::new(FfbConfig {
            torque_limit_nm: 1.0,
            slew_limit_nm_per_s: 1e9,
            ..Default::default()
        });
        for _ in 0..100 {
            chain.condition(&state(50.0, 0.0), DT, 0, true);
        }
        assert_eq!(chain.clamp_events(), 100);
    }
}
