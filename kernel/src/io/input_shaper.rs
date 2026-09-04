//! Reference shaping for the steering position input.
//!
//! The plant's boundary is position-in / torque-out, and a real steering column
//! has inertia. Feed it a raw measured wheel angle and the reaction torque
//! picks up a numerically differentiated second derivative of a quantised,
//! noisy encoder signal -- and the wheel chatters, violently enough to be
//! unpleasant on a belt wheel and alarming on a direct-drive one.
//!
//! Three mitigations, in the order architecture.md 1.6 puts them:
//!
//! 1. Move the column inertia out of the model. The driver's real wheel *is*
//!    the inertia. That is a `VehicleRT` change, and it is the correct fix.
//! 2. Shape the position input, here, so the plant sees a consistent angle,
//!    rate and acceleration that are actually derivatives of one another.
//! 3. Condition the output, in `ffb.rs`.
//!
//! A critically damped second-order filter is used rather than a low-pass on
//! the raw signal because it produces all three quantities from one state, so
//! they cannot disagree, and because critical damping means no overshoot: the
//! shaped angle never leads the driver's hands.

/// Angle, rate and acceleration, guaranteed consistent with each other.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct ShapedSteer {
    pub angle: f64,
    pub rate: f64,
    pub acceleration: f64,
}

#[derive(Debug, Clone)]
pub struct ShaperConfig {
    /// Corner frequency [Hz]. High enough that a driver cannot feel the delay
    /// (a human steering input has almost no content above 5 Hz), low enough to
    /// remove encoder quantisation noise.
    pub cutoff_hz: f64,
    /// Bypass the shaper. Useful exactly once: to demonstrate the chatter this
    /// exists to prevent, so nobody removes it later thinking it does nothing.
    pub enabled: bool,
}

impl Default for ShaperConfig {
    fn default() -> Self {
        Self {
            cutoff_hz: 40.0,
            enabled: true,
        }
    }
}

pub struct SteerShaper {
    config: ShaperConfig,
    angle: f64,
    rate: f64,
}

impl SteerShaper {
    pub fn new(config: ShaperConfig) -> Self {
        Self {
            config,
            angle: 0.0,
            rate: 0.0,
        }
    }

    /// Jump the filter to an angle without a transient. Used at session start,
    /// so the plant does not see a step from zero to wherever the wheel is
    /// actually sitting.
    pub fn reset(&mut self, angle: f64) {
        self.angle = angle;
        self.rate = 0.0;
    }

    pub fn update(&mut self, target: f64, dt: f64) -> ShapedSteer {
        if !self.config.enabled {
            let rate = if dt > 0.0 {
                (target - self.angle) / dt
            } else {
                0.0
            };
            self.angle = target;
            self.rate = rate;
            return ShapedSteer {
                angle: target,
                rate,
                acceleration: 0.0,
            };
        }

        let omega = 2.0 * std::f64::consts::PI * self.config.cutoff_hz;
        let acceleration = omega * omega * (target - self.angle) - 2.0 * omega * self.rate;
        // Semi-implicit Euler: unconditionally stable for this filter at any dt
        // the loop will ever run at, which explicit Euler is not.
        self.rate += acceleration * dt;
        self.angle += self.rate * dt;
        ShapedSteer {
            angle: self.angle,
            rate: self.rate,
            acceleration,
        }
    }

    pub fn angle(&self) -> f64 {
        self.angle
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f64 = 1e-3;

    #[test]
    fn a_step_input_is_tracked_without_overshoot() {
        let mut shaper = SteerShaper::new(ShaperConfig::default());
        let mut peak: f64 = 0.0;
        for _ in 0..200 {
            peak = peak.max(shaper.update(1.0, DT).angle);
        }
        assert!(
            peak <= 1.0 + 1e-6,
            "critical damping must not overshoot, peaked at {peak}"
        );
        assert!(
            peak > 0.99,
            "the shaper must actually get there, reached {peak}"
        );
    }

    #[test]
    fn it_settles_within_a_few_tens_of_milliseconds() {
        let mut shaper = SteerShaper::new(ShaperConfig::default());
        let mut settled_at = None;
        for step in 0..500 {
            let shaped = shaper.update(1.0, DT);
            if settled_at.is_none() && (shaped.angle - 1.0).abs() < 0.01 {
                settled_at = Some(step);
            }
        }
        let settled = settled_at.expect("the shaper must converge");
        assert!(
            settled < 40,
            "settling took {settled} ms, which a driver would feel as delay"
        );
    }

    /// The property the whole module exists for: a quantised, noisy input must
    /// not produce a rate that jumps around, because that rate is what ends up
    /// in the reaction torque.
    #[test]
    fn quantisation_noise_does_not_reach_the_rate() {
        let mut shaper = SteerShaper::new(ShaperConfig::default());
        let quantum = 0.001; // ~16-bit encoder over a 2-turn wheel
        let mut worst_rate: f64 = 0.0;
        for step in 0..2000 {
            // A slow sweep, quantised: the pathological case is the input
            // sitting between two counts and dithering.
            let smooth = 0.5 * (step as f64 * DT * 2.0).sin();
            let quantised = (smooth / quantum).round() * quantum;
            let shaped = shaper.update(quantised, DT);
            if step > 200 {
                worst_rate = worst_rate.max(shaped.rate.abs());
            }
        }
        // The true peak rate of the sweep is 1.0 rad/s. Raw differencing of the
        // quantised signal would show 0.001/0.001 = 1.0 rad/s of noise on top.
        assert!(
            worst_rate < 1.5,
            "shaped rate reached {worst_rate:.3} rad/s against a true peak of 1.0"
        );
    }

    #[test]
    fn resetting_avoids_a_step_at_session_start() {
        let mut shaper = SteerShaper::new(ShaperConfig::default());
        shaper.reset(0.8);
        let shaped = shaper.update(0.8, DT);
        assert!((shaped.angle - 0.8).abs() < 1e-9);
        assert_eq!(shaped.rate, 0.0, "a reset must not produce a rate");
    }

    #[test]
    fn the_bypass_really_is_a_bypass() {
        let mut shaper = SteerShaper::new(ShaperConfig {
            enabled: false,
            cutoff_hz: 40.0,
        });
        let shaped = shaper.update(0.7, DT);
        assert_eq!(
            shaped.angle, 0.7,
            "a bypassed shaper must pass the input through unchanged"
        );
    }
}
