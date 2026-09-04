use bobdil_kernel::generated::frames::DriverInput;
use bobdil_kernel::plant::reduced::{Reduced14Dof, ReducedParams};
use bobdil_kernel::plant::{InitialConditions, PlantKernel};

fn cmd(s: f64, a: f64, b: f64) -> DriverInput {
    DriverInput {
        steering_angle_command: s,
        accelerator_pedal_command: a,
        brake_pedal_command: b,
        ..Default::default()
    }
}

fn run(
    p: ReducedParams,
    speed: f64,
    seconds: f64,
    c: DriverInput,
) -> bobdil_kernel::generated::frames::VehicleState {
    let mut plant = Reduced14Dof::new(p);
    plant
        .reset(&InitialConditions {
            speed,
            steering_angle: 0.0,
        })
        .unwrap();
    let mut st = Default::default();
    for _ in 0..((seconds / 1e-3) as usize) {
        st = plant.step(&c, 1e-3).unwrap();
    }
    st
}

fn main() {
    println!("== braking from 20 m/s ==");
    let coast = run(ReducedParams::default(), 20.0, 0.5, cmd(0.0, 0.0, 0.0));
    println!(
        "coast  ax={:7.2}  Fzf={:7.1} Fzr={:7.1} pitchload={:6.1}",
        coast.acc_x,
        coast.fz_fl + coast.fz_fr,
        coast.fz_rl + coast.fz_rr,
        0.0
    );
    for b in [0.3, 0.6, 0.9] {
        let s = run(ReducedParams::default(), 20.0, 0.8, cmd(0.0, 0.0, b));
        println!(
            "brake {b:.1} ax={:7.2}  Fzf={:7.1} Fzr={:7.1}  v={:5.2}",
            s.acc_x,
            s.fz_fl + s.fz_fr,
            s.fz_rl + s.fz_rr,
            s.vehicle_speed
        );
    }
    println!("\n== steer sweep at 20 m/s, 2 s ==");
    for hw in [0.05, 0.10, 0.15, 0.20, 0.30, 0.50] {
        let s = run(ReducedParams::default(), 20.0, 2.0, cmd(hw, 0.25, 0.0));
        println!("hw={hw:4.2} road={:5.3} r={:6.3} ay={:6.2} beta={:6.3} Fz L={:6.1} R={:6.1} roll={:6.4} tau={:6.2} v={:5.1}",
            s.left_steer_angle, s.yaw_vel, s.acc_y, s.sideslip, s.fz_fl+s.fz_rl, s.fz_fr+s.fz_rr, s.roll, s.handwheel_torque, s.vehicle_speed);
    }
    println!("\n== front bar sweep, hw=0.12, 20 m/s, 2 s ==");
    for arb in [4000.0, 14000.0, 40000.0] {
        let s = run(
            ReducedParams {
                arb_front: arb,
                ..Default::default()
            },
            20.0,
            2.0,
            cmd(0.12, 0.25, 0.0),
        );
        println!("arb_f={arb:7.0} r={:6.4} ay={:6.3} rollgrad={:7.4} FzFL={:6.1} FzFR={:6.1} FzRL={:6.1} FzRR={:6.1}",
            s.yaw_vel, s.acc_y, s.roll, s.fz_fl, s.fz_fr, s.fz_rl, s.fz_rr);
    }
}
