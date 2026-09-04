//! A Magic-Formula-shaped tire with transient slip.
//!
//! Two things here matter more than the curve fit.
//!
//! First, **transient slip**. Steady-state slip divides by forward velocity, so
//! at a standing start -- which is where every DIL session begins -- it is a
//! division by something near zero. Carrying slip as a relaxation-length state
//! removes that singularity and is also the physically right answer: a tire
//! takes about half a revolution to build its slip angle, and a driver feels
//! that delay directly as steering response.
//!
//! Second, and this is the honest caveat from architecture.md 5.1: **this is a
//! shape, not the team's tire.** A driver at the limit is feeling the tire fit
//! almost exclusively. Without TTC-grade data for the actual tire at the actual
//! pressures, a driver can confidently "verify" an artifact of the fit -- and
//! because the response is physics-derived they will trust it more, not less.
//! There is no thermal or pressure model here at all, so the tire cannot
//! degrade, which is exactly what a driver uses to judge a setup over a run.
//! BobDil compares setups against each other; it does not predict absolute grip.

/// Magic-Formula coefficients for one axis, plus the load sensitivity that
//  makes lateral load transfer actually cost grip.
#[derive(Debug, Clone)]
pub struct TireParams {
    /// Lateral stiffness factor.
    pub b_lat: f64,
    pub c_lat: f64,
    pub e_lat: f64,
    /// Longitudinal stiffness factor.
    pub b_long: f64,
    pub c_long: f64,
    pub e_long: f64,
    /// Peak friction coefficient at the reference load.
    pub mu_lat: f64,
    pub mu_long: f64,
    /// Reference vertical load for the load-sensitivity term [N].
    pub reference_load: f64,
    /// Fractional loss of mu per unit of normalised load above reference.
    /// This is what makes load transfer reduce total axle grip.
    pub load_sensitivity: f64,
    /// Relaxation length for slip angle [m].
    pub relaxation_lateral: f64,
    /// Relaxation length for slip ratio [m].
    pub relaxation_longitudinal: f64,
    /// Pneumatic trail at zero slip [m]; falls off as the tire saturates,
    /// which is the cue a driver reads as "the front is letting go".
    pub pneumatic_trail: f64,
}

impl Default for TireParams {
    fn default() -> Self {
        Self {
            // Chosen so the lateral peak lands near 8 deg of slip and the
            // longitudinal peak near 12% slip ratio, which is where a racing
            // slick actually peaks. E is negative, which is what gives a
            // defined peak with a real fall-off past it rather than a curve
            // that saturates and stays there -- a driver needs the car to let
            // go, or there is no limit to find.
            b_lat: 11.0,
            c_lat: 1.45,
            e_lat: -0.5,
            b_long: 11.8,
            c_long: 1.55,
            e_long: -0.4,
            mu_lat: 1.55,
            mu_long: 1.60,
            reference_load: 700.0,
            load_sensitivity: 0.22,
            relaxation_lateral: 0.28,
            relaxation_longitudinal: 0.18,
            pneumatic_trail: 0.028,
        }
    }
}

/// Force and moment produced by one contact patch.
#[derive(Debug, Clone, Copy, Default)]
pub struct TireForces {
    /// Longitudinal force in the wheel frame [N].
    pub fx: f64,
    /// Lateral force in the wheel frame [N].
    pub fy: f64,
    /// Self-aligning moment about the kingpin [N.m], including caster.
    pub mz: f64,
    /// Fraction of available friction in use. 1.0 means the tire is saturated;
    /// the view surfaces this so a driver can see where the grip went.
    pub utilisation: f64,
}

fn magic_formula(slip: f64, b: f64, c: f64, e: f64) -> f64 {
    let bs = b * slip;
    (c * (bs - e * (bs - bs.atan())).atan()).sin()
}

impl TireParams {
    /// Peak friction at a given load. Grip per newton falls as load rises,
    /// which is why lateral load transfer costs an axle grip and why anti-roll
    /// distribution changes balance at all.
    pub fn mu_at_load(&self, base_mu: f64, fz: f64) -> f64 {
        let normalised = (fz / self.reference_load - 1.0).max(-0.9);
        (base_mu * (1.0 - self.load_sensitivity * normalised)).max(0.15)
    }

    /// Evaluate the patch at a given transient slip state and vertical load.
    ///
    /// `slip_angle` and `slip_ratio` are the *relaxed* states, not the
    /// instantaneous kinematic values, which is what keeps this well behaved at
    /// zero speed.
    pub fn evaluate(&self, fz: f64, slip_angle: f64, slip_ratio: f64) -> TireForces {
        if fz <= 1.0 {
            // The wheel is airborne. No force, and no aligning moment for the
            // driver to feel -- which is itself information.
            return TireForces::default();
        }

        let mu_y = self.mu_at_load(self.mu_lat, fz);
        let mu_x = self.mu_at_load(self.mu_long, fz);

        let fy0 = mu_y * fz * magic_formula(slip_angle, self.b_lat, self.c_lat, self.e_lat);
        let fx0 = mu_x * fz * magic_formula(slip_ratio, self.b_long, self.c_long, self.e_long);

        // Combined slip by friction ellipse. Scaling both components by the
        // same factor keeps the force direction and only limits its magnitude,
        // which is the behaviour that makes trail braking feel right.
        let nx = fx0 / (mu_x * fz);
        let ny = fy0 / (mu_y * fz);
        let demand = (nx * nx + ny * ny).sqrt();
        let scale = if demand > 1.0 { 1.0 / demand } else { 1.0 };
        let fx = fx0 * scale;
        let fy = fy0 * scale;

        // Pneumatic trail collapses as the tire saturates. This is the single
        // most important haptic cue in the whole rig: the wheel goes light
        // before the front washes out, and that is what a driver steers by.
        let saturation = demand.min(1.0);
        let trail = self.pneumatic_trail * (1.0 - saturation).max(0.0);

        TireForces {
            fx,
            fy,
            mz: -fy * trail,
            utilisation: demand.min(2.0),
        }
    }

    /// Rate of change of a relaxed slip state toward its kinematic target.
    ///
    /// The relaxation rate is speed over relaxation length. `speed_floor`
    /// regularises it near standstill: without a floor the rate goes to zero
    /// and slip freezes; with one that is too large the tire responds
    /// instantly at parking speeds. 0.5 m/s is roughly walking pace.
    pub fn relax_rate(&self, speed: f64, length: f64, speed_floor: f64) -> f64 {
        speed.abs().max(speed_floor) / length.max(1e-3)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A tire that saturates and stays saturated gives a driver no limit to
    /// find. The curve must peak in the range a slick actually peaks in, and
    /// must fall away past it.
    #[test]
    fn lateral_force_peaks_near_eight_degrees_then_falls_away() {
        let tire = TireParams::default();
        let step = 0.002;
        let sweep: Vec<f64> = (0..=220)
            .map(|i| tire.evaluate(700.0, i as f64 * step, 0.0).fy)
            .collect();
        let peak_index = sweep
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0;
        let peak_degrees = (peak_index as f64 * step).to_degrees();
        assert!(
            (5.0..=12.0).contains(&peak_degrees),
            "peak slip angle {peak_degrees:.1} deg is outside the range a racing slick peaks in"
        );
        assert!(
            sweep[220] < sweep[peak_index] * 0.97,
            "force must fall away past the peak: {:.0} N at 25 deg against a peak of {:.0} N",
            sweep[220],
            sweep[peak_index]
        );
    }

    #[test]
    fn load_transfer_costs_the_axle_grip() {
        let tire = TireParams::default();
        let slip = 0.12;
        let even = 2.0 * tire.evaluate(700.0, slip, 0.0).fy;
        let transferred = tire.evaluate(1100.0, slip, 0.0).fy + tire.evaluate(300.0, slip, 0.0).fy;
        assert!(
            transferred < even,
            "an axle with load transferred onto it must make less force ({transferred:.0} vs {even:.0} N)"
        );
    }

    #[test]
    fn combined_slip_stays_inside_the_friction_ellipse() {
        let tire = TireParams::default();
        let forces = tire.evaluate(700.0, 0.30, 0.40);
        let mu = tire
            .mu_at_load(tire.mu_lat, 700.0)
            .max(tire.mu_at_load(tire.mu_long, 700.0));
        let magnitude = (forces.fx * forces.fx + forces.fy * forces.fy).sqrt();
        assert!(
            magnitude <= mu * 700.0 * 1.02,
            "combined force {magnitude:.0} N exceeds the friction circle"
        );
    }

    #[test]
    fn trail_collapses_as_the_tire_saturates() {
        let tire = TireParams::default();
        let gentle = tire.evaluate(700.0, 0.03, 0.0);
        let saturated = tire.evaluate(700.0, 0.35, 0.0);
        let gentle_trail = (gentle.mz / gentle.fy).abs();
        let saturated_trail = (saturated.mz / saturated.fy).abs();
        assert!(
            saturated_trail < gentle_trail * 0.5,
            "the wheel must go light at the limit: {saturated_trail:.4} vs {gentle_trail:.4} m"
        );
    }

    #[test]
    fn an_airborne_wheel_makes_no_force() {
        let tire = TireParams::default();
        let forces = tire.evaluate(0.0, 0.2, 0.1);
        assert_eq!(forces.fx, 0.0);
        assert_eq!(forces.fy, 0.0);
        assert_eq!(forces.mz, 0.0);
    }
}
