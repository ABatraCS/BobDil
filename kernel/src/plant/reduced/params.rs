//! Parameters for the reduced-order floor kernel.
//!
//! These are ordinary FSAE numbers, close to the BobSim `vehicle.yml` the rest
//! of the toolchain uses, and they exist so the loop is drivable on any
//! hardware. They are *not* a validated vehicle: the floor kernel's job is to
//! keep a session running when the Modelica plant cannot hold the deadline, and
//! its fidelity is established by comparison against `VehicleFMI`
//! (architecture.md 8), never asserted here.

use super::tire::TireParams;

/// Corner order used throughout: front-left, front-right, rear-left, rear-right.
pub const FL: usize = 0;
pub const FR: usize = 1;
pub const RL: usize = 2;
pub const RR: usize = 3;
pub const CORNERS: usize = 4;

pub const GRAVITY: f64 = 9.80665;

#[derive(Debug, Clone)]
pub struct ReducedParams {
    // --- masses and inertias ------------------------------------------------
    /// Total mass including driver [kg].
    pub mass: f64,
    /// Sprung mass [kg].
    pub sprung_mass: f64,
    /// Unsprung mass per corner [kg].
    pub unsprung_mass: f64,
    pub inertia_roll: f64,
    pub inertia_pitch: f64,
    pub inertia_yaw: f64,

    // --- geometry -----------------------------------------------------------
    /// CG to front axle [m].
    pub a: f64,
    /// CG to rear axle [m].
    pub b: f64,
    pub track_front: f64,
    pub track_rear: f64,
    pub cg_height: f64,
    pub roll_centre_front: f64,
    pub roll_centre_rear: f64,
    /// Height of the pitch centre, which sets how much longitudinal transfer
    /// goes through the links rather than the springs.
    pub pitch_centre: f64,

    // --- suspension ---------------------------------------------------------
    /// Wheel-rate per corner [N/m], i.e. spring rate already through the motion ratio.
    pub wheel_rate_front: f64,
    pub wheel_rate_rear: f64,
    pub damping_front: f64,
    pub damping_rear: f64,
    /// Anti-roll bar rate referred to the wheel [N.m/rad]. Exposed as a tunable.
    pub arb_front: f64,
    pub arb_rear: f64,

    // --- tires and wheels ---------------------------------------------------
    pub tire: TireParams,
    pub wheel_radius: f64,
    pub wheel_inertia: f64,
    /// Vertical tire rate [N/m]. With an unsprung degree of freedom per corner
    /// this is what actually generates Fz, rather than Fz being assumed.
    pub tire_vertical_rate: f64,
    pub tire_vertical_damping: f64,

    // --- steering -----------------------------------------------------------
    /// Handwheel angle per road-wheel angle.
    pub steer_ratio: f64,
    /// Mechanical (caster) trail [m].
    pub mechanical_trail: f64,
    /// Steering-system efficiency from rack to column.
    pub steer_efficiency: f64,
    /// Peak handwheel angle the rack allows [rad].
    pub rack_limit: f64,

    // --- powertrain ---------------------------------------------------------
    /// Peak motor torque [N.m].
    pub motor_peak_torque: f64,
    /// Motor speed at which constant-torque ends [rad/s].
    pub motor_corner_speed: f64,
    pub motor_max_speed: f64,
    pub final_drive: f64,
    /// Locking torque per unit of rear-wheel speed difference [N.m.s/rad].
    pub diff_lock_gain: f64,
    /// Torque-biasing preload [N.m]. Exposed as a tunable.
    pub diff_preload: f64,

    // --- brakes -------------------------------------------------------------
    /// Total brake torque at full pedal [N.m].
    pub brake_torque_max: f64,
    /// Fraction of brake torque at the front axle. Exposed as a tunable.
    pub brake_bias: f64,

    // --- aerodynamics -------------------------------------------------------
    pub air_density: f64,
    pub drag_area: f64,
    pub lift_area: f64,
    /// Fraction of downforce carried by the front axle.
    pub aero_balance: f64,

    // --- regularisation -----------------------------------------------------
    /// Speed floor used wherever a slip quantity divides by velocity [m/s].
    /// Every DIL session starts at rest, so this is load-bearing, not a detail.
    pub speed_floor: f64,
    /// Width of the smooth transition that replaces sign() on brake and
    /// differential friction. State events are what make a Modelica model miss
    /// deadlines (architecture.md 1.5); the floor kernel avoids them by
    /// construction.
    pub omega_epsilon: f64,
}

impl Default for ReducedParams {
    fn default() -> Self {
        Self {
            mass: 280.0,
            sprung_mass: 226.4,
            unsprung_mass: 13.4,
            inertia_roll: 35.0,
            inertia_pitch: 90.0,
            inertia_yaw: 105.0,

            a: 0.78,
            b: 0.77,
            track_front: 1.22,
            track_rear: 1.18,
            cg_height: 0.30,
            roll_centre_front: 0.045,
            roll_centre_rear: 0.075,
            pitch_centre: 0.10,

            wheel_rate_front: 32_000.0,
            wheel_rate_rear: 36_000.0,
            damping_front: 2_600.0,
            damping_rear: 2_900.0,
            arb_front: 14_000.0,
            arb_rear: 9_000.0,

            tire: TireParams::default(),
            wheel_radius: 0.2286,
            wheel_inertia: 0.28,
            tire_vertical_rate: 118_000.0,
            tire_vertical_damping: 550.0,

            steer_ratio: 4.0,
            mechanical_trail: 0.021,
            steer_efficiency: 0.92,
            rack_limit: 2.2,

            motor_peak_torque: 230.0,
            motor_corner_speed: 380.0,
            motor_max_speed: 1_150.0,
            final_drive: 3.4,
            diff_lock_gain: 6.0,
            diff_preload: 25.0,

            brake_torque_max: 2_400.0,
            brake_bias: 0.62,

            air_density: 1.19,
            drag_area: 1.15,
            lift_area: 2.85,
            aero_balance: 0.46,

            speed_floor: 0.5,
            omega_epsilon: 1.5,
        }
    }
}

impl ReducedParams {
    pub fn wheelbase(&self) -> f64 {
        self.a + self.b
    }

    /// Longitudinal position of each corner relative to the CG, +x forward.
    pub fn corner_x(&self, corner: usize) -> f64 {
        if corner < RL {
            self.a
        } else {
            -self.b
        }
    }

    /// Lateral position of each corner relative to the CG, +y to the left.
    pub fn corner_y(&self, corner: usize) -> f64 {
        let half = if corner < RL {
            self.track_front
        } else {
            self.track_rear
        } * 0.5;
        if corner == FL || corner == RL {
            half
        } else {
            -half
        }
    }

    pub fn is_front(&self, corner: usize) -> bool {
        corner < RL
    }

    pub fn wheel_rate(&self, corner: usize) -> f64 {
        if self.is_front(corner) {
            self.wheel_rate_front
        } else {
            self.wheel_rate_rear
        }
    }

    pub fn damping(&self, corner: usize) -> f64 {
        if self.is_front(corner) {
            self.damping_front
        } else {
            self.damping_rear
        }
    }

    /// Static vertical load per corner, before any transfer or downforce.
    pub fn static_load(&self, corner: usize) -> f64 {
        let axle_share = if self.is_front(corner) {
            self.b / self.wheelbase()
        } else {
            self.a / self.wheelbase()
        };
        0.5 * self.mass * GRAVITY * axle_share
    }
}
