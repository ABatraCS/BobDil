//! Behavioural tests for the floor kernel.
//!
//! These do not assert a validated vehicle -- that is what the offline fidelity
//! harness against BobLib is for. They assert the things that must hold for the
//! rig to be safe and usable at all: it stays finite at the operating points a
//! session actually visits, it obeys the conservation laws a driver would
//! notice violating, and a setup change moves the car the way an engineer
//! expects.

use super::*;
use crate::plant::{InitialConditions, PlantKernel};

fn input(steer: f64, accel: f64, brake: f64) -> DriverInput {
    DriverInput {
        steering_angle_command: steer,
        accelerator_pedal_command: accel,
        brake_pedal_command: brake,
        ..Default::default()
    }
}

/// Run a maneuver at 1 kHz and return the final state.
fn drive(plant: &mut Reduced14Dof, seconds: f64, command: DriverInput) -> VehicleState {
    let steps = (seconds / 1e-3).round() as usize;
    let mut state = VehicleState::default();
    for _ in 0..steps {
        state = plant
            .step(&command, 1e-3)
            .expect("plant must not fail mid-maneuver");
    }
    state
}

fn at_rest() -> Reduced14Dof {
    let mut plant = Reduced14Dof::new(ReducedParams::default());
    plant.reset(&InitialConditions::default()).unwrap();
    plant
}

fn rolling(speed: f64) -> Reduced14Dof {
    let mut plant = Reduced14Dof::new(ReducedParams::default());
    plant
        .reset(&InitialConditions {
            speed,
            steering_angle: 0.0,
        })
        .unwrap();
    plant
}

#[test]
fn static_loads_carry_the_vehicle_weight() {
    let params = ReducedParams::default();
    let total: f64 = (0..CORNERS).map(|c| params.static_load(c)).sum();
    assert!(
        (total - params.mass * GRAVITY).abs() < 1e-6,
        "static corner loads sum to {total:.1} N but the car weighs {:.1} N",
        params.mass * GRAVITY
    );
}

#[test]
fn a_parked_car_stays_parked_and_stays_on_its_tires() {
    let mut plant = at_rest();
    let state = drive(&mut plant, 5.0, input(0.0, 0.0, 0.0));
    assert!(
        state.vehicle_speed.abs() < 1e-3,
        "parked car drifted to {} m/s",
        state.vehicle_speed
    );

    let carried = state.fz_fl + state.fz_fr + state.fz_rl + state.fz_rr;
    let weight = plant.params().mass * GRAVITY;
    assert!(
        (carried / weight - 1.0).abs() < 0.02,
        "corner loads settled at {carried:.0} N against a weight of {weight:.0} N"
    );
    assert!(
        state.roll.abs() < 1e-4,
        "a parked car should not be rolled over"
    );
}

/// The operating point architecture.md 6 calls out specifically: transient slip
/// divides by velocity, and every DIL session starts at rest.
#[test]
fn a_standing_start_stays_finite() {
    let mut plant = at_rest();
    for step in 0..3_000 {
        let state = plant
            .step(&input(0.0, 1.0, 0.0), 1e-3)
            .unwrap_or_else(|e| panic!("standing start failed at step {step}: {e}"));
        assert!(state.is_finite(), "non-finite state at step {step}");
    }
    let state = plant.step(&input(0.0, 1.0, 0.0), 1e-3).unwrap();
    assert!(
        state.vehicle_speed > 8.0,
        "3 s of full throttle only reached {:.1} m/s",
        state.vehicle_speed
    );
}

#[test]
fn full_throttle_then_threshold_braking_returns_to_rest() {
    let mut plant = at_rest();
    let launched = drive(&mut plant, 4.0, input(0.0, 1.0, 0.0));
    assert!(
        launched.vehicle_speed > 10.0,
        "launch reached only {:.1} m/s",
        launched.vehicle_speed
    );
    assert!(launched.acc_x > 0.0, "throttle must accelerate the car");

    let stopped = drive(&mut plant, 4.0, input(0.0, 0.0, 1.0));
    assert!(
        stopped.vehicle_speed < 0.5,
        "4 s of full brake left the car at {:.2} m/s",
        stopped.vehicle_speed
    );
}

/// Compared as a *fraction* of total load, not an absolute one: the car sheds
/// most of its downforce as it slows, so absolute front load can fall even
/// while the balance moves decisively forward.
#[test]
fn braking_moves_load_onto_the_front_axle() {
    fn front_share(state: &VehicleState) -> f64 {
        (state.fz_fl + state.fz_fr) / (state.fz_fl + state.fz_fr + state.fz_rl + state.fz_rr)
    }

    let mut plant = rolling(20.0);
    let coasting = drive(&mut plant, 0.5, input(0.0, 0.0, 0.0));
    let coasting_share = front_share(&coasting);

    let braking = drive(&mut plant, 0.6, input(0.0, 0.0, 0.9));
    let braking_share = front_share(&braking);

    assert!(
        braking.acc_x < -8.0,
        "0.9 brake only produced {:.1} m/s2",
        braking.acc_x
    );
    assert!(
        braking_share > coasting_share + 0.08,
        "front load share barely moved under braking: {:.1}% -> {:.1}%",
        coasting_share * 100.0,
        braking_share * 100.0
    );
    assert!(
        braking.fz_rl + braking.fz_rr > 0.0,
        "the rear axle must stay on the ground under braking"
    );
}

#[test]
fn steering_generates_yaw_and_load_transfer_to_the_outside() {
    // A sub-limit input. Steering past the front tires' peak would test how
    // the model behaves while sliding, which is a different question.
    let mut plant = rolling(18.0);
    let state = drive(&mut plant, 1.5, input(0.12, 0.25, 0.0));

    assert!(
        state.yaw_vel > 0.05,
        "a left steer produced only {:.3} rad/s of yaw",
        state.yaw_vel
    );
    assert!(
        state.acc_y > 1.0,
        "a left steer produced only {:.2} m/s2 lateral",
        state.acc_y
    );

    let left = state.fz_fl + state.fz_rl;
    let right = state.fz_fr + state.fz_rr;
    assert!(
        right > left * 1.05,
        "turning left must load the right-hand side: left {left:.0} N, right {right:.0} N"
    );
    assert!(
        state.roll > 0.0,
        "turning left must roll the car onto its right side"
    );
}

/// The single most important haptic property: the wheel must push back toward
/// centre. If this sign is wrong the rig is unusable and, on a direct-drive
/// wheel, dangerous.
#[test]
fn steering_torque_self_centers() {
    let mut plant = rolling(18.0);
    let left = drive(&mut plant, 1.0, input(0.15, 0.2, 0.0));
    // The chain applies the negation of the reported reaction torque.
    let applied_left = -left.handwheel_torque;
    assert!(
        applied_left < -0.5,
        "steering left must push the wheel back right, got {applied_left:.2} N.m"
    );

    let mut plant = rolling(18.0);
    let right = drive(&mut plant, 1.0, input(-0.15, 0.2, 0.0));
    let applied_right = -right.handwheel_torque;
    assert!(
        applied_right > 0.5,
        "steering right must push the wheel back left, got {applied_right:.2} N.m"
    );
}

#[test]
fn steering_torque_builds_with_speed() {
    let mut slow = rolling(8.0);
    let slow_state = drive(&mut slow, 1.0, input(0.10, 0.15, 0.0));
    let mut fast = rolling(22.0);
    let fast_state = drive(&mut fast, 1.0, input(0.10, 0.15, 0.0));
    assert!(
        fast_state.handwheel_torque.abs() > slow_state.handwheel_torque.abs(),
        "torque at 22 m/s ({:.2}) should exceed torque at 8 m/s ({:.2})",
        fast_state.handwheel_torque,
        slow_state.handwheel_torque
    );
}

#[test]
fn the_rack_limit_is_reported_rather_than_silently_swallowed() {
    let mut plant = rolling(10.0);
    let limit = plant.params().rack_limit;
    let state = drive(&mut plant, 0.2, input(limit + 0.8, 0.0, 0.0));
    assert!(
        (state.steer_excess - 0.8).abs() < 1e-9,
        "steer beyond the rack must appear as excess, got {}",
        state.steer_excess
    );
    assert!((state.handwheel_angle - limit).abs() < 1e-9);
}

/// "More front bar" is the canonical setup change an engineer asks a driver
/// about. Stiffening the front anti-roll bar moves lateral load transfer onto
/// the front axle, which costs the front grip, which is understeer.
#[test]
fn more_front_bar_moves_the_car_toward_understeer() {
    // Held below the front tires' peak, where balance is a real property of
    // the setup. Past the peak both cars are sliding and the comparison says
    // nothing an engineer could act on.
    let command = input(0.12, 0.25, 0.0);

    let mut soft = Reduced14Dof::new(ReducedParams {
        arb_front: 4_000.0,
        ..Default::default()
    });
    soft.reset(&InitialConditions {
        speed: 20.0,
        steering_angle: 0.0,
    })
    .unwrap();
    let soft_state = drive(&mut soft, 2.0, command);
    assert!(
        soft_state.acc_y < 12.0,
        "the reference lap must stay below the limit"
    );

    let mut stiff = Reduced14Dof::new(ReducedParams {
        arb_front: 40_000.0,
        ..Default::default()
    });
    stiff
        .reset(&InitialConditions {
            speed: 20.0,
            steering_angle: 0.0,
        })
        .unwrap();
    let stiff_state = drive(&mut stiff, 2.0, command);

    assert!(
        stiff_state.yaw_vel < soft_state.yaw_vel * 0.995,
        "a stiffer front bar must reduce yaw rate for the same steer: \
         soft {:.4} rad/s, stiff {:.4} rad/s",
        soft_state.yaw_vel,
        stiff_state.yaw_vel
    );
    // ...and it must do it by moving lateral load transfer onto the front axle,
    // which is the mechanism, not a coincidence of the numbers.
    let soft_front_spread = (soft_state.fz_fr - soft_state.fz_fl).abs();
    let stiff_front_spread = (stiff_state.fz_fr - stiff_state.fz_fl).abs();
    assert!(
        stiff_front_spread > soft_front_spread,
        "a stiffer front bar must take more lateral load transfer: {soft_front_spread:.0} -> {stiff_front_spread:.0} N"
    );
}

#[test]
fn tunables_are_range_checked() {
    let mut plant = at_rest();
    let front_arb = crate::generated::frames::TUNABLES
        .iter()
        .find(|t| t.name == "front_arb_rate")
        .unwrap();
    plant
        .set_tunable(front_arb.id, 20_000.0)
        .expect("in-range value must be accepted");
    assert_eq!(plant.params().arb_front, 20_000.0);
    assert!(
        plant.set_tunable(front_arb.id, -1.0).is_err(),
        "a negative bar rate must be rejected"
    );
    assert!(
        plant.set_tunable(999, 1.0).is_err(),
        "an unknown tunable id must be rejected"
    );
}

/// Replay, paired A/B, and every fidelity comparison rest on this: identical
/// inputs must produce bit-identical states.
#[test]
fn identical_inputs_produce_bit_identical_states() {
    let run = || {
        let mut plant = at_rest();
        for step in 0..2_000 {
            let t = step as f64 * 1e-3;
            let command = input(0.6 * (t * 2.0).sin(), 0.7, 0.1 * (t * 3.0).cos().max(0.0));
            plant.step(&command, 1e-3).unwrap();
        }
        plant.state_vector().to_vec()
    };
    let first = run();
    let second = run();
    assert_eq!(
        first, second,
        "the plant must be deterministic or A/B is meaningless"
    );
}

#[test]
fn reports_a_step_size_it_can_actually_hold() {
    let plant = at_rest();
    let caps = plant.capabilities();
    assert_eq!(caps.continuous_states, STATE_COUNT);
    assert!(
        caps.max_stable_dt >= 1e-3,
        "the floor kernel must be stable at the 1 kHz baseline"
    );
    assert!(caps.supports_tunables);
}
