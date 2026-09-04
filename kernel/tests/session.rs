//! End-to-end session tests.
//!
//! These run the real loop -- real threads, real shared memory, real pacing --
//! against a scripted null device. They are the tests that would have caught a
//! loop which reports itself healthy while missing every deadline, or one whose
//! telemetry quietly loses samples.
//!
//! They share fixed shared-memory segment names, so they run as one test.

use std::path::PathBuf;

use bobdil_kernel::config::KernelConfig;
use bobdil_kernel::generated::frames::fault_flags;
use bobdil_kernel::loop_runner;
use bobdil_kernel::telemetry::TelemetryReader;

fn scratch(name: &str) -> PathBuf {
    std::env::temp_dir().join(format!("bobdil_session_{name}_{}.bdt", std::process::id()))
}

#[test]
fn a_headless_session_holds_its_deadline_and_records_every_step() {
    let telemetry = scratch("headless");
    let config = KernelConfig {
        duration_s: Some(2.0),
        telemetry_path: Some(telemetry.clone()),
        // CI runners are shared and unprivileged; asking for SCHED_FIFO there
        // just adds noise to the report.
        request_realtime: false,
        ..Default::default()
    };

    let report = loop_runner::run(
        config,
        bobdil_kernel::io::device::NullDevice::scripted(|index| {
            let t = index as f64 * 1e-3;
            let command = bobdil_kernel::plant::ladder::maneuver(t);
            bobdil_kernel::io::device::RawInput {
                steer: (command.steering_angle_command / 2.0).clamp(-1.0, 1.0),
                accelerator: command.accelerator_pedal_command,
                brake: command.brake_pedal_command,
                buttons: 0,
                host_time_ns: bobdil_kernel::sys::clock::monotonic_ns(),
            }
        }),
    )
    .expect("a headless session must run");

    // --- the loop actually ran in real time -----------------------------
    assert_eq!(report.steps, 2000, "2 s at 1 kHz is 2000 steps");
    assert!(report.plant_error.is_none(), "{:?}", report.plant_error);
    let rtf = report.sim_time / report.wall_time_s;
    assert!(
        (0.95..=1.05).contains(&rtf),
        "the loop must track wall clock; realtime factor was {rtf:.4}"
    );

    // --- and it knows whether it did ------------------------------------
    // A shared CI machine will drop the occasional step. What must not happen
    // is a loop that misses deadlines wholesale, or one that reports success
    // while doing so.
    let miss_rate = report.deadline_misses as f64 / report.steps as f64;
    assert!(
        miss_rate < 0.05,
        "missed {:.2}% of deadlines ({}), step time {}",
        miss_rate * 100.0,
        report.deadline_misses,
        report.histogram.summary_us()
    );
    assert_eq!(
        report.fault_flags & fault_flags::PLANT_NONFINITE,
        0,
        "the plant must stay finite for a whole session"
    );

    // --- telemetry is complete ---------------------------------------------
    assert_eq!(
        report.telemetry_dropped, 0,
        "telemetry must be lossless below capacity"
    );
    let recording = TelemetryReader::open(&telemetry).expect("recording must be readable");
    assert_eq!(
        recording.frames.len() as u64,
        report.telemetry_frames,
        "every frame the recorder counted must be in the file"
    );
    assert!(
        recording.frames.len() >= 1900,
        "almost every step should be recorded"
    );

    // Step indices must be contiguous: a gap means a lost sample, which would
    // silently invalidate any A/B comparison made from this file.
    for pair in recording.frames.windows(2) {
        assert_eq!(
            pair[1].step_index,
            pair[0].step_index + 1,
            "telemetry has a hole at step {}",
            pair[0].step_index
        );
    }

    // --- the car did something ---------------------------------------------
    let fastest = recording
        .frames
        .iter()
        .map(|frame| frame.vehicle_speed)
        .fold(0.0f64, f64::max);
    assert!(
        fastest > 5.0,
        "the scripted driver should have got moving, peaked at {fastest:.1} m/s"
    );
    assert!(
        recording
            .frames
            .iter()
            .any(|frame| frame.ffb_torque_nm.abs() > 0.1),
        "the session must have produced real force feedback"
    );

    // --- pose was integrated -------------------------------------------------
    let travelled = recording
        .frames
        .last()
        .map(|frame| (frame.pos_x * frame.pos_x + frame.pos_y * frame.pos_y).sqrt())
        .unwrap_or(0.0);
    assert!(
        travelled > 5.0,
        "the car should have covered ground, got {travelled:.1} m"
    );

    let _ = std::fs::remove_file(&telemetry);
}
