//! The plant boundary.
//!
//! `PlantKernel` is the decoupling the spec asks for: one trait, one
//! integration path, three implementations (architecture.md 1.3). Nothing in
//! this module knows that a steering wheel, a screen, or a driver exists -- it
//! consumes a `DriverInput` and produces a `VehicleState`, and that is the
//! entire contract. Nothing outside this module knows whether the physics came
//! from Modelica or from the built-in reduced model.

pub mod fmi2;
pub mod fmu_me;
pub mod integrator;
pub mod ladder;
pub mod reduced;

use crate::generated::frames::{DriverInput, VehicleState};

/// Why a plant could not do what was asked.
#[derive(Debug)]
pub enum PlantError {
    /// The model produced NaN or infinity. Always fatal for the session: a
    /// non-finite state cannot be recovered from and must never reach the wheel.
    NonFinite {
        field: &'static str,
        step: u64,
    },
    /// The solver could not complete a step within its iteration budget.
    StepFailed {
        detail: String,
    },
    /// The FMU rejected a call, or could not be loaded.
    Fmi {
        call: &'static str,
        status: i32,
    },
    Load {
        detail: String,
    },
    /// A tunable id that this kernel does not expose.
    UnknownTunable {
        id: u32,
    },
    OutOfRange {
        id: u32,
        value: f64,
        min: f64,
        max: f64,
    },
    Unsupported {
        what: &'static str,
    },
}

impl std::fmt::Display for PlantError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NonFinite { field, step } => {
                write!(f, "plant produced a non-finite {field} at step {step}")
            }
            Self::StepFailed { detail } => write!(f, "step failed: {detail}"),
            Self::Fmi { call, status } => write!(f, "FMI call {call} returned status {status}"),
            Self::Load { detail } => write!(f, "could not load plant: {detail}"),
            Self::UnknownTunable { id } => write!(f, "no tunable with id {id}"),
            Self::OutOfRange {
                id,
                value,
                min,
                max,
            } => {
                write!(f, "tunable {id} value {value} is outside [{min}, {max}]")
            }
            Self::Unsupported { what } => write!(f, "this kernel does not support {what}"),
        }
    }
}

impl std::error::Error for PlantError {}

/// How a session starts. A DIL session always begins at rest unless the
/// engineer asks otherwise, which is exactly the operating point where
/// velocity-normalised tire models are most fragile -- so it is the default,
/// and it is what the benchmark exercises first.
#[derive(Debug, Clone, Copy)]
pub struct InitialConditions {
    pub speed: f64,
    pub steering_angle: f64,
}

impl Default for InitialConditions {
    fn default() -> Self {
        Self {
            speed: 0.0,
            steering_angle: 0.0,
        }
    }
}

/// What a kernel can do, so the runner never has to special-case an implementation.
#[derive(Debug, Clone)]
pub struct KernelCaps {
    /// Which rung of the ladder this is (`generated::frames::kernel_id`).
    pub id: u64,
    pub name: String,
    /// Largest fixed step at which this kernel is numerically stable. The
    /// runner refuses a configured dt above this rather than producing a
    /// plausible-looking divergence a driver would feel as "vague".
    pub max_stable_dt: f64,
    pub continuous_states: usize,
    /// Whether parameters can be changed without a recompile (architecture.md 1.9).
    pub supports_tunables: bool,
}

/// The physics contract. Implementations: `reduced::Reduced14Dof`,
/// `fmu_me::FmuMe` over VehicleRT, `fmu_me::FmuMe` over VehicleFMI.
pub trait PlantKernel: Send {
    fn reset(&mut self, init: &InitialConditions) -> Result<(), PlantError>;

    /// Advance the plant by exactly `dt` and return the resulting state.
    ///
    /// Implementations fill the physics and pose fields. The loop fills the
    /// health fields (`rtf`, `step_time_us`, `fault_flags`, ...) afterwards,
    /// because only it knows what the step cost in wall-clock terms.
    fn step(&mut self, input: &DriverInput, dt: f64) -> Result<VehicleState, PlantError>;

    fn set_tunable(&mut self, id: u32, value: f64) -> Result<(), PlantError>;

    fn capabilities(&self) -> KernelCaps;

    /// Continuous state vector, for replay determinism checks and for the
    /// fidelity harness that compares one kernel against another.
    fn state_vector(&self) -> &[f64];
}

/// Shared guard: reject a non-finite plant output at the source.
///
/// Called by every implementation before returning. This is the first of the
/// two independent NaN barriers -- the second is in the force-feedback chain --
/// because a NaN torque reaching a direct-drive wheel is a physical hazard, not
/// a rendering artifact.
pub fn guard_finite(state: &VehicleState, step: u64) -> Result<(), PlantError> {
    if state.is_finite() {
        return Ok(());
    }
    for (index, name) in VehicleState::FIELD_NAMES.iter().enumerate() {
        if !state.field(index).is_finite() {
            return Err(PlantError::NonFinite { field: name, step });
        }
    }
    Err(PlantError::NonFinite {
        field: "unknown",
        step,
    })
}
