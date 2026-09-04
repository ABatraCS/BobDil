//! Kernel selection.
//!
//! The ladder is `VehicleFMI` -> `VehicleRT` -> `Reduced14Dof`
//! (architecture.md 1.3). A rung is chosen once, at session start, from
//! *measured* step times -- never guessed from a machine spec, and never
//! changed mid-drive.
//!
//! Not changing it mid-drive is a deliberate constraint, not a limitation.
//! Swapping the plant under a driver at speed would produce a discontinuity in
//! the force feedback, which is unsafe, and it would mean the lap they just
//! drove was two different cars, which makes the resulting opinion worthless.

use std::path::Path;

use crate::generated::frames::{kernel_id, DriverInput};
use crate::metrics::StepTimeHistogram;
use crate::plant::fmu_me::FmuMe;
use crate::plant::integrator::Method;
use crate::plant::reduced::{Reduced14Dof, ReducedParams};
use crate::plant::{InitialConditions, PlantKernel};
use crate::sys::clock::monotonic_ns;

/// The gate from architecture.md 6: half the 1 ms budget, leaving the rest for
/// OS jitter, the device thread, and everything else on the machine.
pub const DEFAULT_GATE_P999_NS: u64 = 500_000;

/// What a probe measured for one candidate kernel.
#[derive(Debug, Clone)]
pub struct Probe {
    pub name: String,
    pub kernel_id: u64,
    pub continuous_states: usize,
    pub steps: u64,
    pub mean_ns: f64,
    pub p50_ns: u64,
    pub p99_ns: u64,
    pub p999_ns: u64,
    pub max_ns: u64,
    /// Set when the plant failed during the probe, which disqualifies it
    /// regardless of how fast it was.
    pub failure: Option<String>,
}

impl Probe {
    pub fn passes(&self, gate_p999_ns: u64) -> bool {
        self.failure.is_none() && self.p999_ns <= gate_p999_ns
    }

    pub fn describe(&self, gate_p999_ns: u64) -> String {
        match &self.failure {
            Some(detail) => format!("{:<24} FAILED: {detail}", self.name),
            None => format!(
                "{:<24} {:>4} states  mean {:6.1}  p50 {:6.1}  p99 {:6.1}  p99.9 {:6.1}  max {:7.1} us  {}",
                self.name,
                self.continuous_states,
                self.mean_ns / 1000.0,
                self.p50_ns as f64 / 1000.0,
                self.p99_ns as f64 / 1000.0,
                self.p999_ns as f64 / 1000.0,
                self.max_ns as f64 / 1000.0,
                if self.passes(gate_p999_ns) { "PASS" } else { "over budget" },
            ),
        }
    }
}

/// Drive a kernel through a representative maneuver and time every step.
///
/// The maneuver is not idle running. It launches from rest, steers, and brakes,
/// because those are the operating points where a vehicle model is slowest: a
/// standing start divides by a near-zero velocity, and a brake or a
/// differential lock-up fires state events.
pub fn probe(plant: &mut dyn PlantKernel, dt: f64, steps: u64) -> Probe {
    let caps = plant.capabilities();
    let mut histogram = StepTimeHistogram::new(20_000_000, 1_000);
    let mut failure = None;

    if let Err(error) = plant.reset(&InitialConditions::default()) {
        failure = Some(format!("reset: {error}"));
    }

    for step in 0..steps {
        let t = step as f64 * dt;
        let input = maneuver(t);
        let started = monotonic_ns();
        let result = plant.step(&input, dt);
        histogram.record(monotonic_ns().saturating_sub(started));
        if let Err(error) = result {
            failure = Some(format!("step {step} at t={t:.3}: {error}"));
            break;
        }
    }

    Probe {
        name: caps.name,
        kernel_id: caps.id,
        continuous_states: caps.continuous_states,
        steps: histogram.count(),
        mean_ns: histogram.mean_ns(),
        p50_ns: histogram.percentile_ns(0.50),
        p99_ns: histogram.percentile_ns(0.99),
        p999_ns: histogram.percentile_ns(0.999),
        max_ns: histogram.max_ns(),
        failure,
    }
}

/// The probe maneuver: standing start, step steer, threshold braking, skidpad.
///
/// Deliberately the same four phases the offline benchmark uses, so a session's
/// own start-up probe and `tools/rt_bench` are measuring the same work.
pub fn maneuver(t: f64) -> DriverInput {
    let phase = t % 8.0;
    let (steer, accelerator, brake) = if phase < 2.0 {
        (0.0, 1.0, 0.0) // standing start
    } else if phase < 3.0 {
        (0.35, 0.4, 0.0) // step steer
    } else if phase < 4.5 {
        (0.0, 0.0, 0.9) // threshold braking
    } else {
        (0.25, 0.35, 0.0) // steady state skidpad
    };
    DriverInput {
        steering_angle_command: steer,
        accelerator_pedal_command: accelerator,
        brake_pedal_command: brake,
        ..Default::default()
    }
}

/// What `select` decided, and why.
pub struct Selection {
    pub plant: Box<dyn PlantKernel>,
    pub probes: Vec<Probe>,
    pub chosen: String,
    /// True when the requested kernel could not hold the deadline and a lower
    /// rung was taken. Surfaced in the UI: the driver is entitled to know they
    /// are not feeling the model they asked for.
    pub degraded: bool,
}

impl Selection {
    pub fn report(&self, gate_p999_ns: u64) -> String {
        let mut lines = vec![format!(
            "plant ladder (gate: p99.9 <= {:.0} us)",
            gate_p999_ns as f64 / 1000.0
        )];
        for probe in &self.probes {
            lines.push(format!("  {}", probe.describe(gate_p999_ns)));
        }
        lines.push(format!(
            "  -> selected {}{}",
            self.chosen,
            if self.degraded {
                "  (DEGRADED: the requested plant missed the deadline)"
            } else {
                ""
            }
        ));
        lines.join("\n")
    }
}

/// Try the requested plant, measure it, and fall back down the ladder if it
/// cannot hold the deadline.
///
/// `probe_steps` of 0 skips measurement and takes the requested plant on trust,
/// which is what a benchmark run wants -- it is doing its own timing.
pub fn select(
    plant_dir: Option<&Path>,
    dt: f64,
    method: Method,
    substeps: usize,
    probe_steps: u64,
    gate_p999_ns: u64,
) -> Selection {
    let mut probes = Vec::new();

    if let Some(dir) = plant_dir {
        match FmuMe::load(dir, kernel_id::VEHICLE_FMI, method, substeps) {
            Ok(mut fmu) => {
                if probe_steps == 0 {
                    let name = fmu.capabilities().name;
                    return Selection {
                        plant: Box::new(fmu),
                        probes,
                        chosen: name,
                        degraded: false,
                    };
                }
                let measurement = probe(&mut fmu, dt, probe_steps);
                let passed = measurement.passes(gate_p999_ns);
                let name = measurement.name.clone();
                probes.push(measurement);
                if passed {
                    // Leave the plant at t=0 for the driver, not wherever the
                    // probe maneuver ended up.
                    let _ = fmu.reset(&InitialConditions::default());
                    return Selection {
                        plant: Box::new(fmu),
                        probes,
                        chosen: name,
                        degraded: false,
                    };
                }
            }
            Err(error) => {
                probes.push(Probe {
                    name: format!("FMU at {}", dir.display()),
                    kernel_id: kernel_id::VEHICLE_FMI,
                    continuous_states: 0,
                    steps: 0,
                    mean_ns: 0.0,
                    p50_ns: 0,
                    p99_ns: 0,
                    p999_ns: 0,
                    max_ns: 0,
                    failure: Some(error.to_string()),
                });
            }
        }
    }

    let mut reduced = Reduced14Dof::new(ReducedParams::default());
    if probe_steps > 0 {
        probes.push(probe(&mut reduced, dt, probe_steps.min(20_000)));
    }
    let _ = reduced.reset(&InitialConditions::default());
    let name = reduced.capabilities().name;
    Selection {
        plant: Box::new(reduced),
        probes,
        chosen: name,
        degraded: plant_dir.is_some(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The floor kernel steps 20 000 times without failing, and the probe
    /// reports a coherent measurement.
    ///
    /// Deliberately **not** a timing assertion. This suite is advertised as
    /// hermetic and safe in CI, and cargo runs it on every core at once: an
    /// earlier version asserted `passes(DEFAULT_GATE_P999_NS)` here and failed
    /// whenever the machine was busy -- it went red mid-session purely because
    /// a Modelica compile was running in a container next to it. A wall-clock
    /// gate measures the machine, not the code, so it belongs in `make bench`
    /// (which reports it, alongside the scheduling policy and memlock limit it
    /// actually obtained) and not in a test whose failure should mean "the code
    /// is wrong".
    #[test]
    fn the_floor_kernel_steps_without_failing() {
        let mut plant = Reduced14Dof::new(ReducedParams::default());
        let measurement = probe(&mut plant, 1e-3, 20_000);
        assert!(measurement.failure.is_none(), "{:?}", measurement.failure);
        assert_eq!(measurement.steps, 20_000);
        assert!(measurement.p50_ns > 0, "probe recorded no time at all");
        assert!(
            measurement.p50_ns <= measurement.p999_ns && measurement.p999_ns <= measurement.max_ns,
            "percentiles are not ordered: {measurement:?}"
        );
    }

    #[test]
    fn a_missing_fmu_falls_back_rather_than_refusing_to_start() {
        let selection = select(
            Some(Path::new("/nonexistent/fmu")),
            1e-3,
            Method::Rk4,
            1,
            2_000,
            DEFAULT_GATE_P999_NS,
        );
        assert_eq!(selection.chosen, "Reduced14Dof");
        assert!(
            selection.degraded,
            "falling back must be reported, not silent"
        );
        assert!(selection.probes.iter().any(|p| p.failure.is_some()));
    }

    #[test]
    fn the_probe_maneuver_visits_every_hard_operating_point() {
        let launch = maneuver(1.0);
        assert_eq!(
            launch.accelerator_pedal_command, 1.0,
            "must include a standing start"
        );
        let steer = maneuver(2.5);
        assert!(
            steer.steering_angle_command.abs() > 0.0,
            "must include a step steer"
        );
        let brake = maneuver(4.0);
        assert!(
            brake.brake_pedal_command > 0.5,
            "must include threshold braking"
        );
    }
}
