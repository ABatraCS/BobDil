//! The floor of the plant ladder: a 14-degree-of-freedom vehicle in Rust.
//!
//! Why this exists at all. The ladder is `VehicleFMI` -> `VehicleRT` ->
//! `Reduced14Dof` (architecture.md 1.3). The top two rungs are Modelica and may
//! or may not hold a 1 ms deadline on a given machine; this one always will, so
//! a session is always drivable and the loop, the buffers and the safety path
//! can be built and proven before any FMU exists.
//!
//! It is deliberately *not* a second physics implementation competing with
//! BobLib. Its role is to guarantee a loop, to be the target the offline
//! fidelity harness compares against BobLib's maneuver suite, and to be the
//! plant the timing benchmark uses as its floor.
//!
//! Fourteen degrees of freedom:
//!   * 3 planar body (longitudinal, lateral, yaw)
//!   * 3 sprung mass (heave, roll, pitch)
//!   * 4 unsprung vertical (one per corner)
//!   * 4 wheel spin
//!
//! plus 8 tire relaxation states, for 29 first-order states in total.

pub mod params;
pub mod tire;

use crate::generated::frames::{kernel_id, DriverInput, VehicleState};
use crate::plant::integrator::{Derivatives, FixedStep, Method};
use crate::plant::{guard_finite, InitialConditions, KernelCaps, PlantError, PlantKernel};

pub use params::{ReducedParams, CORNERS, FL, FR, GRAVITY, RL, RR};
use tire::TireForces;

// --- state vector layout -------------------------------------------------
const IDX_VX: usize = 0;
const IDX_VY: usize = 1;
const IDX_YAW_RATE: usize = 2;
const IDX_HEAVE: usize = 3;
const IDX_HEAVE_RATE: usize = 4;
const IDX_ROLL: usize = 5;
const IDX_ROLL_RATE: usize = 6;
const IDX_PITCH: usize = 7;
const IDX_PITCH_RATE: usize = 8;
const IDX_HOP: usize = 9; // 4 entries
const IDX_HOP_RATE: usize = 13; // 4 entries
const IDX_OMEGA: usize = 17; // 4 entries
const IDX_SLIP_ANGLE: usize = 21; // 4 entries
const IDX_SLIP_RATIO: usize = 25; // 4 entries
pub const STATE_COUNT: usize = 29;

/// Everything the derivative function computes that the output stage also
/// wants, so the outputs never need a second, subtly different evaluation.
#[derive(Debug, Clone, Copy, Default)]
struct Loads {
    fz: [f64; CORNERS],
    forces: [TireForces; CORNERS],
    steer: [f64; CORNERS],
    body_fx: f64,
    body_fy: f64,
    accel_x: f64,
    accel_y: f64,
    front_lateral: f64,
    rear_lateral: f64,
    total_longitudinal: f64,
    kingpin_torque: f64,
}

/// The reduced plant. Owns its state, its integrator, and its parameters.
pub struct Reduced14Dof {
    params: ReducedParams,
    state: Vec<f64>,
    integrator: FixedStep,
    time: f64,
    step_index: u64,
    /// Zero-order held for the duration of a step, which is the correct
    /// treatment for a host-owned fixed-step integrator: the plant sees one
    /// consistent input across all four RK4 stages.
    input: DriverInput,
    /// Axle forces from the previous step, used for the geometric (roll-centre
    /// and pitch-centre) share of load transfer. That term is algebraically
    /// circular -- load transfer depends on lateral force which depends on load
    /// -- so it is taken one step late. At 1 ms the lag is far below anything a
    /// driver can feel, and it keeps the derivative function a pure function of
    /// state.
    previous_front_lateral: f64,
    previous_rear_lateral: f64,
    previous_longitudinal: f64,
    last_loads: Loads,
}

impl Reduced14Dof {
    pub fn new(params: ReducedParams) -> Self {
        Self {
            params,
            state: vec![0.0; STATE_COUNT],
            integrator: FixedStep::new(STATE_COUNT, Method::Rk4, 1),
            time: 0.0,
            step_index: 0,
            input: DriverInput::default(),
            previous_front_lateral: 0.0,
            previous_rear_lateral: 0.0,
            previous_longitudinal: 0.0,
            last_loads: Loads::default(),
        }
    }

    pub fn params(&self) -> &ReducedParams {
        &self.params
    }

    pub fn params_mut(&mut self) -> &mut ReducedParams {
        &mut self.params
    }

    /// Road-wheel angles from the handwheel command, with the rack limit and a
    /// partial-Ackermann split.
    fn steer_angles(&self, handwheel: f64) -> ([f64; CORNERS], f64) {
        let limit = self.params.rack_limit;
        let clamped = handwheel.clamp(-limit, limit);
        let excess = handwheel - clamped;
        let base = clamped / self.params.steer_ratio;

        // Partial Ackermann: the inner wheel turns slightly more. The full
        // geometric value is rarely run on an FSAE car, so this is a fraction.
        let ackermann = 0.35;
        let geometric = if base.abs() > 1e-6 {
            let wheelbase = self.params.wheelbase();
            let turn_radius = wheelbase / base.tan();
            let inner =
                (wheelbase / (turn_radius - self.params.track_front * 0.5 * base.signum())).atan();
            let outer =
                (wheelbase / (turn_radius + self.params.track_front * 0.5 * base.signum())).atan();
            (inner, outer)
        } else {
            (base, base)
        };
        let inner = base + ackermann * (geometric.0 - base);
        let outer = base + ackermann * (geometric.1 - base);

        // Turning left (base > 0) makes the left wheel the inner one.
        let (left, right) = if base >= 0.0 {
            (inner, outer)
        } else {
            (outer, inner)
        };
        ([left, right, 0.0, 0.0], excess)
    }

    /// Motor torque available at the current speed: constant torque to the
    /// corner speed, constant power above it, nothing past the limit.
    fn motor_torque(&self, motor_speed: f64) -> f64 {
        let speed = motor_speed.abs();
        if speed >= self.params.motor_max_speed {
            return 0.0;
        }
        if speed <= self.params.motor_corner_speed {
            self.params.motor_peak_torque
        } else {
            self.params.motor_peak_torque * self.params.motor_corner_speed / speed
        }
    }

    /// The full force calculation. Called by `derivatives` and again by the
    /// output stage, so the two can never disagree.
    fn evaluate_loads(&self, x: &[f64]) -> Loads {
        let p = &self.params;
        let mut loads = Loads::default();

        let (steer, _) = self.steer_angles(self.input.steering_angle_command);
        loads.steer = steer;

        let vx = x[IDX_VX];

        // --- aerodynamics -------------------------------------------------
        let dynamic_pressure = 0.5 * p.air_density * vx * vx;
        let downforce = dynamic_pressure * p.lift_area;
        let drag = dynamic_pressure * p.drag_area * vx.signum();

        // --- geometric load transfer (one step late, see the struct doc) ----
        let geo_lateral_front = self.previous_front_lateral * p.roll_centre_front / p.track_front;
        let geo_lateral_rear = self.previous_rear_lateral * p.roll_centre_rear / p.track_rear;
        let geo_longitudinal = self.previous_longitudinal * p.pitch_centre / p.wheelbase();

        // --- vertical loads -------------------------------------------------
        for corner in 0..CORNERS {
            let hop = x[IDX_HOP + corner];
            let hop_rate = x[IDX_HOP_RATE + corner];
            let aero_share = if p.is_front(corner) {
                downforce * p.aero_balance * 0.5
            } else {
                downforce * (1.0 - p.aero_balance) * 0.5
            };
            let geo_lateral = if p.is_front(corner) {
                geo_lateral_front
            } else {
                geo_lateral_rear
            };
            // +y is left, so a positive lateral force loads the right-hand corners.
            let lateral_sign = -p.corner_y(corner).signum();
            let longitudinal_sign = if p.is_front(corner) { -1.0 } else { 1.0 };

            let load = p.static_load(corner)
                - p.tire_vertical_rate * hop
                - p.tire_vertical_damping * hop_rate
                + aero_share
                + lateral_sign * geo_lateral
                + longitudinal_sign * geo_longitudinal;
            loads.fz[corner] = load.max(0.0);
        }

        // --- tire forces ----------------------------------------------------
        for corner in 0..CORNERS {
            loads.forces[corner] = p.tire.evaluate(
                loads.fz[corner],
                x[IDX_SLIP_ANGLE + corner],
                x[IDX_SLIP_RATIO + corner],
            );
        }

        // --- resolve into the body frame ------------------------------------
        for (corner, &delta) in steer.iter().enumerate() {
            let (sin_d, cos_d) = delta.sin_cos();
            let f = loads.forces[corner];
            let fx_body = f.fx * cos_d - f.fy * sin_d;
            let fy_body = f.fx * sin_d + f.fy * cos_d;
            loads.body_fx += fx_body;
            loads.body_fy += fy_body;
            if p.is_front(corner) {
                loads.front_lateral += fy_body;
            } else {
                loads.rear_lateral += fy_body;
            }
        }
        loads.total_longitudinal = loads.body_fx - drag;

        loads.accel_x = (loads.body_fx - drag) / p.mass;
        loads.accel_y = loads.body_fy / p.mass;

        // --- steering feel ---------------------------------------------------
        // Aligning moment about the kingpin from both front patches, reflected
        // to the column. The pneumatic trail inside Mz collapses as the tire
        // saturates, so the wheel goes light before the front washes out.
        let mut kingpin = 0.0;
        for corner in [FL, FR] {
            let f = loads.forces[corner];
            kingpin += f.mz - f.fy * p.mechanical_trail;
        }
        loads.kingpin_torque = kingpin;
        loads
    }

    fn write_outputs(&self, loads: &Loads, excess: f64) -> VehicleState {
        let p = &self.params;
        let x = &self.state;
        let vx = x[IDX_VX];
        let vy = x[IDX_VY];

        // Reaction torque, matching VehicleFMI's `handwheelTorque = -tau`
        // convention so both kernels feed the force-feedback chain identically.
        let column_torque = loads.kingpin_torque / p.steer_ratio * p.steer_efficiency;

        VehicleState {
            sim_time: self.time,
            step_index: self.step_index,
            host_time_ns: 0,
            steering_angle_command: self.input.steering_angle_command,
            accelerator_pedal_command: self.input.accelerator_pedal_command,
            brake_pedal_command: self.input.brake_pedal_command,
            vehicle_speed: (vx * vx + vy * vy).sqrt(),
            acc_x: loads.accel_x,
            acc_y: loads.accel_y,
            handwheel_angle: self.input.steering_angle_command - excess,
            steer_excess: excess,
            handwheel_torque: -column_torque,
            fz_fl: loads.fz[FL],
            fz_fr: loads.fz[FR],
            fz_rl: loads.fz[RL],
            fz_rr: loads.fz[RR],
            left_steer_angle: loads.steer[FL],
            right_steer_angle: loads.steer[FR],
            roll: x[IDX_ROLL],
            sideslip: if vx.abs() > 0.1 {
                (vy / vx).atan()
            } else {
                0.0
            },
            vel_x: vx,
            vel_y: vy,
            yaw_vel: x[IDX_YAW_RATE],
            ..VehicleState::default()
        }
    }
}

impl Derivatives for Reduced14Dof {
    fn derivatives(&mut self, _t: f64, x: &[f64], dx: &mut [f64]) {
        let p = &self.params;
        let loads = self.evaluate_loads(x);

        let vx = x[IDX_VX];
        let vy = x[IDX_VY];
        let yaw_rate = x[IDX_YAW_RATE];
        let heave = x[IDX_HEAVE];
        let heave_rate = x[IDX_HEAVE_RATE];
        let roll = x[IDX_ROLL];
        let roll_rate = x[IDX_ROLL_RATE];
        let pitch = x[IDX_PITCH];
        let pitch_rate = x[IDX_PITCH_RATE];

        // --- planar body ------------------------------------------------------
        let drag = 0.5 * p.air_density * p.drag_area * vx * vx * vx.signum();
        dx[IDX_VX] = (loads.body_fx - drag) / p.mass + yaw_rate * vy;
        dx[IDX_VY] = loads.body_fy / p.mass - yaw_rate * vx;

        let mut yaw_moment = 0.0;
        for corner in 0..CORNERS {
            let delta = loads.steer[corner];
            let (sin_d, cos_d) = delta.sin_cos();
            let f = loads.forces[corner];
            let fx_body = f.fx * cos_d - f.fy * sin_d;
            let fy_body = f.fx * sin_d + f.fy * cos_d;
            yaw_moment += p.corner_x(corner) * fy_body - p.corner_y(corner) * fx_body;
        }
        dx[IDX_YAW_RATE] = yaw_moment / p.inertia_yaw;

        // --- suspension: spring, damper and anti-roll bar ----------------------
        let mut suspension_force = [0.0f64; CORNERS];
        let mut compression = [0.0f64; CORNERS];
        let mut compression_rate = [0.0f64; CORNERS];
        for corner in 0..CORNERS {
            let cx = p.corner_x(corner);
            let cy = p.corner_y(corner);
            // Vertical motion of the sprung mass at this corner, +up.
            let sprung_z = heave + cx * pitch + cy * roll;
            let sprung_rate = heave_rate + cx * pitch_rate + cy * roll_rate;
            compression[corner] = x[IDX_HOP + corner] - sprung_z;
            compression_rate[corner] = x[IDX_HOP_RATE + corner] - sprung_rate;
            let static_force = p.static_load(corner) - p.unsprung_mass * GRAVITY;
            suspension_force[corner] = static_force
                + p.wheel_rate(corner) * compression[corner]
                + p.damping(corner) * compression_rate[corner];
        }

        // An anti-roll bar is a force couple across an axle: it resists the
        // difference in compression between the two sides and puts the load it
        // takes onto the compressed (outside) wheel. This is the term the
        // driver is judging when they ask for "more front bar".
        let arb_front = p.arb_front * (compression[FL] - compression[FR]) / p.track_front;
        let arb_rear = p.arb_rear * (compression[RL] - compression[RR]) / p.track_rear;
        suspension_force[FL] += arb_front;
        suspension_force[FR] -= arb_front;
        suspension_force[RL] += arb_rear;
        suspension_force[RR] -= arb_rear;

        // --- sprung mass: heave, roll, pitch ------------------------------------
        let dynamic_pressure = 0.5 * p.air_density * vx * vx;
        let downforce = dynamic_pressure * p.lift_area;
        let total_suspension: f64 = suspension_force.iter().sum();
        dx[IDX_HEAVE] = heave_rate;
        dx[IDX_HEAVE_RATE] =
            (total_suspension - p.sprung_mass * GRAVITY - downforce) / p.sprung_mass;

        // Elastic share of load transfer: the sprung mass rolls and pitches
        // about the roll and pitch centres under its own lateral and
        // longitudinal acceleration. The geometric share went straight into Fz.
        let roll_lever = p.cg_height - 0.5 * (p.roll_centre_front + p.roll_centre_rear);
        let pitch_lever = p.cg_height - p.pitch_centre;
        let mut roll_moment = p.sprung_mass * loads.accel_y * roll_lever;
        let mut pitch_moment = p.sprung_mass * loads.accel_x * pitch_lever;
        for (corner, &force) in suspension_force.iter().enumerate() {
            roll_moment += force * p.corner_y(corner);
            pitch_moment += force * p.corner_x(corner);
        }
        dx[IDX_ROLL] = roll_rate;
        dx[IDX_ROLL_RATE] = roll_moment / p.inertia_roll;
        dx[IDX_PITCH] = pitch_rate;
        dx[IDX_PITCH_RATE] = pitch_moment / p.inertia_pitch;

        // --- unsprung vertical --------------------------------------------------
        for corner in 0..CORNERS {
            dx[IDX_HOP + corner] = x[IDX_HOP_RATE + corner];
            dx[IDX_HOP_RATE + corner] =
                (loads.fz[corner] - suspension_force[corner] - p.unsprung_mass * GRAVITY)
                    / p.unsprung_mass;
        }

        // --- powertrain and brakes ------------------------------------------------
        let rear_speed = 0.5 * (x[IDX_OMEGA + RL] + x[IDX_OMEGA + RR]);
        let motor_speed = rear_speed * p.final_drive;
        let demand = self.input.accelerator_pedal_command.clamp(0.0, 1.0);
        let axle_torque = demand * self.motor_torque(motor_speed) * p.final_drive;

        // Limited-slip differential, regularised rather than switched: a hard
        // sign() here would be a state event, and state events are what stop a
        // model holding a deadline (architecture.md 1.5).
        let speed_difference = x[IDX_OMEGA + RL] - x[IDX_OMEGA + RR];
        let lock_torque = p.diff_preload * (speed_difference / p.omega_epsilon).tanh()
            + p.diff_lock_gain * speed_difference;

        let brake_demand = self.input.brake_pedal_command.clamp(0.0, 1.0);
        let brake_total = brake_demand * p.brake_torque_max;

        for corner in 0..CORNERS {
            let omega = x[IDX_OMEGA + corner];
            let brake_share = if p.is_front(corner) {
                brake_total * p.brake_bias * 0.5
            } else {
                brake_total * (1.0 - p.brake_bias) * 0.5
            };
            let brake_torque = -brake_share * (omega / p.omega_epsilon).tanh();
            let drive_torque = match corner {
                RL => 0.5 * axle_torque - lock_torque,
                RR => 0.5 * axle_torque + lock_torque,
                _ => 0.0,
            };
            let traction_torque = -loads.forces[corner].fx * p.wheel_radius;
            dx[IDX_OMEGA + corner] =
                (drive_torque + brake_torque + traction_torque) / p.wheel_inertia;
        }

        // --- tire relaxation ------------------------------------------------------
        for corner in 0..CORNERS {
            let cx = p.corner_x(corner);
            let cy = p.corner_y(corner);
            let contact_vx = vx - yaw_rate * cy;
            let contact_vy = vy + yaw_rate * cx;
            let delta = loads.steer[corner];
            let (sin_d, cos_d) = delta.sin_cos();
            let wheel_vx = contact_vx * cos_d + contact_vy * sin_d;
            let wheel_vy = -contact_vx * sin_d + contact_vy * cos_d;
            let denominator = wheel_vx.abs().max(p.speed_floor);

            let target_slip_angle = -(wheel_vy / denominator).atan();
            let target_slip_ratio =
                ((x[IDX_OMEGA + corner] * p.wheel_radius) - wheel_vx) / denominator;

            let lateral_rate =
                p.tire
                    .relax_rate(wheel_vx, p.tire.relaxation_lateral, p.speed_floor);
            let longitudinal_rate =
                p.tire
                    .relax_rate(wheel_vx, p.tire.relaxation_longitudinal, p.speed_floor);

            dx[IDX_SLIP_ANGLE + corner] =
                lateral_rate * (target_slip_angle - x[IDX_SLIP_ANGLE + corner]);
            dx[IDX_SLIP_RATIO + corner] = longitudinal_rate
                * (target_slip_ratio.clamp(-4.0, 4.0) - x[IDX_SLIP_RATIO + corner]);
        }
    }
}

impl PlantKernel for Reduced14Dof {
    fn reset(&mut self, init: &InitialConditions) -> Result<(), PlantError> {
        self.state.iter_mut().for_each(|value| *value = 0.0);
        self.state[IDX_VX] = init.speed;
        for corner in 0..CORNERS {
            self.state[IDX_OMEGA + corner] = init.speed / self.params.wheel_radius;
            // Start on the static tire deflection so the car does not drop onto
            // its tires at t = 0 and ring for the first second of every session.
            self.state[IDX_HOP + corner] =
                -self.params.static_load(corner) / self.params.tire_vertical_rate;
        }
        self.time = 0.0;
        self.step_index = 0;
        self.input = DriverInput {
            steering_angle_command: init.steering_angle,
            ..Default::default()
        };
        self.previous_front_lateral = 0.0;
        self.previous_rear_lateral = 0.0;
        self.previous_longitudinal = 0.0;
        self.last_loads = Loads::default();
        Ok(())
    }

    fn step(&mut self, input: &DriverInput, dt: f64) -> Result<VehicleState, PlantError> {
        self.input = *input;
        // The plant is its own derivative function, so it must hand itself to
        // the integrator. Both are moved out for the duration of the call and
        // put straight back; neither move allocates.
        let start_time = self.time;
        let mut integrator = std::mem::replace(&mut self.integrator, FixedStep::placeholder());
        let mut state = std::mem::take(&mut self.state);
        let end_time = integrator.advance(self, start_time, &mut state, dt);
        self.state = state;
        self.integrator = integrator;
        self.time = end_time;
        self.step_index += 1;

        let loads = self.evaluate_loads(&self.state);
        self.previous_front_lateral = loads.front_lateral;
        self.previous_rear_lateral = loads.rear_lateral;
        self.previous_longitudinal = loads.total_longitudinal;
        self.last_loads = loads;

        let (_, excess) = self.steer_angles(input.steering_angle_command);
        let output = self.write_outputs(&loads, excess);
        guard_finite(&output, self.step_index)?;
        Ok(output)
    }

    fn set_tunable(&mut self, id: u32, value: f64) -> Result<(), PlantError> {
        let spec = crate::generated::frames::TUNABLES
            .iter()
            .find(|t| t.id == id)
            .ok_or(PlantError::UnknownTunable { id })?;
        if !(spec.min..=spec.max).contains(&value) {
            return Err(PlantError::OutOfRange {
                id,
                value,
                min: spec.min,
                max: spec.max,
            });
        }
        match spec.name {
            "front_arb_rate" => self.params.arb_front = value,
            "rear_arb_rate" => self.params.arb_rear = value,
            "brake_bias" => self.params.brake_bias = value,
            "diff_preload" => self.params.diff_preload = value,
            _ => return Err(PlantError::UnknownTunable { id }),
        }
        Ok(())
    }

    fn capabilities(&self) -> KernelCaps {
        KernelCaps {
            id: kernel_id::REDUCED14DOF,
            name: "Reduced14Dof".to_string(),
            // Set by the 15 Hz wheel-hop mode, the fastest thing in the model.
            max_stable_dt: 4e-3,
            continuous_states: STATE_COUNT,
            supports_tunables: true,
        }
    }

    fn state_vector(&self) -> &[f64] {
        &self.state
    }
}

#[cfg(test)]
mod tests;
