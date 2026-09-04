//! Re-run a recording against a plant, deterministically.
//!
//! This is the mechanism paired A/B is built on (architecture.md 1.9). The
//! claim it has to support is narrow and strong: *the same recorded inputs,
//! applied to two setups, produce outputs whose every difference is attributable
//! to the setup.* Nothing else about the second run may vary -- not the step
//! size, not the integrator, not the initial conditions, and not the pose
//! arithmetic, which is why that lives in `crate::pose` and is shared with the
//! live loop rather than written twice.
//!
//! Two things are deliberately absent, and their absence is the guarantee:
//!
//! * **No clock.** Replay runs as fast as the CPU allows and never sleeps. Wall
//!   time is not an input to the physics, so removing it removes the only
//!   source of run-to-run variation.
//! * **No device.** No wheel is opened and no torque is produced. A replay must
//!   never be able to move a wheel nobody is holding.
//!
//! Every replay verifies its own determinism by running twice and comparing.
//! That is cheap next to the replay itself, and a comparison made on a plant
//! that turned out to be non-deterministic is worse than no comparison, because
//! it still looks like a result.

use std::path::Path;

use crate::generated::frames::{kernel_id, DriverInput, TunableSpec, VehicleState, TUNABLES};
use crate::plant::integrator::Method;
use crate::plant::reduced::{Reduced14Dof, ReducedParams};
use crate::plant::{InitialConditions, PlantError, PlantKernel};
use crate::pose::Pose;
use crate::telemetry::{SessionMeta, TelemetryReader, TelemetryRecorder};

/// One `name=value` setup change, resolved against the schema's tunable table.
#[derive(Debug, Clone, Copy)]
pub struct TunableSetting {
    pub spec: TunableSpec,
    pub value: f64,
}

impl TunableSetting {
    /// Parse `name=value`, checking the name and the range against the schema.
    ///
    /// Range-checked here rather than at apply time so a typo in an A/B script
    /// fails before either side runs -- half an A/B is not a result.
    pub fn parse(text: &str) -> Result<Self, String> {
        let (name, raw) = text
            .split_once('=')
            .ok_or_else(|| format!("--tunable expects name=value, got {text:?}"))?;
        let name = name.trim();
        let spec = TUNABLES
            .iter()
            .find(|candidate| candidate.name == name)
            .ok_or_else(|| {
                let known: Vec<&str> = TUNABLES.iter().map(|t| t.name).collect();
                format!("no tunable named {name:?}; the schema defines {known:?}")
            })?;
        let value: f64 = raw
            .trim()
            .parse()
            .map_err(|_| format!("{name}: {raw:?} is not a number"))?;
        if !(spec.min..=spec.max).contains(&value) {
            return Err(format!(
                "{name}={value} is outside the schema's range [{}, {}]",
                spec.min, spec.max
            ));
        }
        Ok(Self { spec: *spec, value })
    }

    pub fn describe(&self) -> String {
        format!("{}={}", self.spec.name, self.value)
    }
}

/// Everything that decides what a replay produces.
pub struct ReplayConfig {
    /// Unpacked FMU directory, or `None` for the built-in reduced kernel.
    pub plant_dir: Option<std::path::PathBuf>,
    pub method: Method,
    pub substeps: usize,
    pub initial: InitialConditions,
    pub tunables: Vec<TunableSetting>,
}

impl Default for ReplayConfig {
    fn default() -> Self {
        Self {
            plant_dir: None,
            method: Method::Rk4,
            substeps: 1,
            initial: InitialConditions::default(),
            tunables: Vec::new(),
        }
    }
}

pub struct ReplayOutcome {
    pub frames: Vec<VehicleState>,
    pub final_state: Vec<f64>,
    pub kernel_name: String,
    pub kernel_id: u64,
    /// True when a second identical run reproduced the first exactly.
    pub deterministic: bool,
}

fn build_plant(config: &ReplayConfig) -> Result<Box<dyn PlantKernel>, String> {
    match &config.plant_dir {
        Some(dir) => {
            let fmu = crate::plant::fmu_me::FmuMe::load(
                dir,
                kernel_id::VEHICLE_FMI,
                config.method,
                config.substeps,
            )
            .map_err(|error| format!("{}: {error}", dir.display()))?;
            Ok(Box::new(fmu))
        }
        None => Ok(Box::new(Reduced14Dof::new(ReducedParams::default()))),
    }
}

fn run_once(config: &ReplayConfig, recording: &TelemetryReader) -> Result<ReplayOutcome, String> {
    let mut plant = build_plant(config)?;
    let caps = plant.capabilities();

    // Order matters: reset first, then apply the setup. An FMU's reset restores
    // the compiled-in parameter values, so tuning before it would be silently
    // undone -- and an A/B whose B side quietly reverted to A is the worst
    // failure available here, because both sides then agree.
    plant.reset(&config.initial).map_err(stringify)?;
    for setting in &config.tunables {
        plant
            .set_tunable(setting.spec.id, setting.value)
            .map_err(|error| format!("{}: {error}", setting.describe()))?;
    }

    let dt = recording.meta.step_dt;
    let mut pose = Pose::default();
    let mut frames = Vec::with_capacity(recording.frames.len());
    let mut sim_time = 0.0f64;

    for (index, recorded) in recording.frames.iter().enumerate() {
        // Only the driver's three commands are taken from the recording. Every
        // other field in a recorded frame is an *output* of the run being
        // replayed, and feeding any of it back would make this a copy rather
        // than a reproduction.
        let input = DriverInput {
            steering_angle_command: recorded.steering_angle_command,
            accelerator_pedal_command: recorded.accelerator_pedal_command,
            brake_pedal_command: recorded.brake_pedal_command,
            ..Default::default()
        };
        let mut state = plant.step(&input, dt).map_err(stringify)?;
        sim_time += dt;
        pose.advance(&state, dt);
        pose.apply(&mut state);
        state.sim_time = sim_time;
        state.step_index = index as u64 + 1;
        state.kernel_id = caps.id;
        // host_time_ns, rtf, step_time_us and deadline_misses stay at zero on
        // purpose. They describe the machine a run happened on, not the run, and
        // writing this machine's values into a replay would make two recordings
        // differ for a reason that has nothing to do with the car.
        frames.push(state);
    }

    Ok(ReplayOutcome {
        final_state: plant.state_vector().to_vec(),
        frames,
        kernel_name: caps.name,
        kernel_id: caps.id,
        deterministic: true,
    })
}

/// Replay `recording` twice and return the second run, having proved they agree.
pub fn replay(config: &ReplayConfig, recording: &TelemetryReader) -> Result<ReplayOutcome, String> {
    let first = run_once(config, recording)?;
    let mut second = run_once(config, recording)?;
    second.deterministic = first.final_state == second.final_state
        && first.frames.len() == second.frames.len()
        && first
            .frames
            .iter()
            .zip(&second.frames)
            .all(|(a, b)| frames_identical(a, b));
    Ok(second)
}

/// Bit-for-bit comparison of two frames, over every field.
///
/// Compares the raw bits rather than the values, so two NaNs count as the same
/// frame. A replay that produced NaN twice is reproducible -- broken, but
/// reproducible -- and calling that non-deterministic points at the wrong bug.
pub fn frames_identical(a: &VehicleState, b: &VehicleState) -> bool {
    (0..VehicleState::FIELD_COUNT).all(|index| a.field(index).to_bits() == b.field(index).to_bits())
}

/// Write a replay out as a telemetry file that replays exactly like a recording.
pub fn write(outcome: &ReplayOutcome, meta: &SessionMeta, path: &Path) -> std::io::Result<u64> {
    let mut recorder = TelemetryRecorder::create(path, meta)?;
    for frame in &outcome.frames {
        recorder.write(frame)?;
    }
    recorder.finish()
}

fn stringify(error: PlantError) -> String {
    error.to_string()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_tunable_is_resolved_by_name_from_the_schema() {
        let setting = TunableSetting::parse("brake_bias=0.6").unwrap();
        assert_eq!(setting.spec.name, "brake_bias");
        assert_eq!(setting.value, 0.6);
    }

    #[test]
    fn an_out_of_range_tunable_is_refused_before_anything_runs() {
        let error = TunableSetting::parse("brake_bias=0.95").unwrap_err();
        assert!(error.contains("outside"), "{error}");
    }

    #[test]
    fn an_unknown_tunable_lists_the_ones_that_exist() {
        let error = TunableSetting::parse("front_arb=1000").unwrap_err();
        assert!(error.contains("front_arb_rate"), "{error}");
    }

    #[test]
    fn a_tunable_without_a_value_is_an_error_not_a_default() {
        assert!(TunableSetting::parse("brake_bias").is_err());
    }

    #[test]
    fn identical_frames_compare_equal_even_when_both_are_nan() {
        let nan = VehicleState {
            acc_y: f64::NAN,
            ..Default::default()
        };
        assert!(frames_identical(&nan, &nan));
    }

    #[test]
    fn frames_that_differ_in_one_field_are_not_identical() {
        let a = VehicleState::default();
        let b = VehicleState {
            acc_y: 1e-300,
            ..Default::default()
        };
        assert!(!frames_identical(&a, &b));
    }
}
