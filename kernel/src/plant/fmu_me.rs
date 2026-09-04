//! An FMI 2.0 Model Exchange FMU driven by our own fixed-step integrator.
//!
//! This is the rung of the ladder where the driver is feeling actual Modelica,
//! from actual BobLib source. Everything else in the kernel exists to get a
//! driver's hands onto this.
//!
//! Two design decisions are worth stating plainly.
//!
//! **The kernel never parses XML.** `modelDescription.xml` is read by the
//! session, which resolves every value reference and writes a flat manifest
//! next to the FMU. The real-time process reads a few hundred bytes of
//! `key=value` text at load. That keeps an XML parser -- and its allocations --
//! out of the process that has to hold a 1 ms deadline, and it means the value
//! references are resolved once by the component that is allowed to be slow.
//!
//! **Events are handled, but grudgingly.** A state event costs an unbounded
//! iteration inside `fmi2NewDiscreteStates`. `VehicleRT` is built to have none
//! (architecture.md 1.5), but `VehicleFMI` has plenty, so they are processed
//! with a hard iteration cap: if the FMU cannot settle its discrete states in
//! `MAX_EVENT_ITERATIONS`, the step is failed rather than allowed to run long.
//! A missed deadline the watchdog can see is far better than a stall it cannot.

use std::collections::HashMap;
use std::ffi::CString;
use std::path::{Path, PathBuf};

use crate::generated::frames::{DriverInput, VehicleState};
use crate::plant::fmi2;
use crate::plant::integrator::{Derivatives, FixedStep, Method};
use crate::plant::{guard_finite, InitialConditions, KernelCaps, PlantError, PlantKernel};
use crate::sys::dylib::Dylib;

/// Iterations allowed for the FMU to settle its discrete states after an event.
/// Chosen to be generous for a well-posed model and still bounded: an FMU that
/// needs more than this is chattering, which is a modelling defect that must be
/// fixed in Modelica, not absorbed here.
const MAX_EVENT_ITERATIONS: usize = 16;

/// Everything the session resolved from `modelDescription.xml`.
#[derive(Debug, Clone, Default)]
pub struct PlantManifest {
    pub model_identifier: String,
    pub guid: String,
    /// Path to the platform binary, relative to the manifest.
    pub library: PathBuf,
    /// `resources` directory URI, passed to `fmi2Instantiate`.
    pub resource_uri: String,
    pub continuous_states: usize,
    pub event_indicators: usize,
    /// Signal name -> FMI value reference, for both inputs and outputs.
    pub value_references: HashMap<String, fmi2::ValueReference>,
    /// Tunable id -> FMI value reference.
    pub tunables: HashMap<u32, fmi2::ValueReference>,
    /// Hash of the vehicle this FMU was built from, recorded into telemetry.
    pub vehicle_hash: u64,
}

/// Build the `fmi2Instantiate` resources URI from a manifest value.
///
/// The session writes a path relative to the FMU directory, which is what keeps
/// a cache entry usable after it moves -- a build done in the container and read
/// on the host is the case that forced this. A value that is already a URI is
/// passed through, so an entry written by an older session still loads.
///
/// Percent-encoding is not decoration: a repo under a path containing a space
/// yields a `file://` URI that an FMU is entitled to reject, and the resulting
/// failure appears inside someone else's generated C with no explanation.
fn resource_uri(value: &str, base: &Path) -> String {
    if value.contains("://") {
        return value.to_string();
    }
    // `file://` requires an absolute path: given a relative one, everything up
    // to the first slash is parsed as the *authority*, so `build/x/resources`
    // silently becomes host `build` and path `/x/resources`. canonicalize also
    // resolves the `build/<name>-plant` symlink, which is the normal way this
    // directory is named.
    let joined = base.join(value);
    let absolute = std::fs::canonicalize(&joined).unwrap_or_else(|_| {
        std::env::current_dir()
            .map(|cwd| cwd.join(&joined))
            .unwrap_or(joined)
    });
    let mut uri = String::from("file://");
    for byte in absolute.to_string_lossy().bytes() {
        match byte {
            b'A'..=b'Z' | b'a'..=b'z' | b'0'..=b'9' | b'-' | b'.' | b'_' | b'~' | b'/' => {
                uri.push(byte as char)
            }
            _ => uri.push_str(&format!("%{byte:02X}")),
        }
    }
    uri
}

impl PlantManifest {
    /// Parse the flat manifest the session writes beside the FMU.
    ///
    /// Format is deliberately trivial -- `key=value`, one per line -- so the
    /// real-time process needs no parser library and no allocation beyond the
    /// strings themselves.
    pub fn parse(text: &str, base: &Path) -> Result<Self, String> {
        let mut manifest = Self::default();
        for (number, raw_line) in text.lines().enumerate() {
            let line = raw_line.trim();
            if line.is_empty() || line.starts_with('#') {
                continue;
            }
            let (key, value) = line
                .split_once('=')
                .ok_or_else(|| format!("manifest line {}: expected key=value", number + 1))?;
            let key = key.trim();
            let value = value.trim();
            match key {
                "model_identifier" => manifest.model_identifier = value.to_string(),
                "guid" => manifest.guid = value.to_string(),
                "library" => manifest.library = base.join(value),
                "resources" => manifest.resource_uri = resource_uri(value, base),
                "states" => {
                    manifest.continuous_states =
                        value.parse().map_err(|_| format!("bad states: {value}"))?
                }
                "event_indicators" => {
                    manifest.event_indicators = value
                        .parse()
                        .map_err(|_| format!("bad event_indicators: {value}"))?
                }
                "vehicle_hash" => {
                    manifest.vehicle_hash = u64::from_str_radix(value.trim_start_matches("0x"), 16)
                        .map_err(|_| format!("bad vehicle_hash: {value}"))?
                }
                _ => {
                    if let Some(name) = key.strip_prefix("vref:") {
                        let reference = value
                            .parse()
                            .map_err(|_| format!("bad value reference for {name}: {value}"))?;
                        manifest
                            .value_references
                            .insert(name.to_string(), reference);
                    } else if let Some(id) = key.strip_prefix("tunable:") {
                        let id: u32 = id.parse().map_err(|_| format!("bad tunable id: {id}"))?;
                        let reference = value
                            .parse()
                            .map_err(|_| format!("bad tunable reference: {value}"))?;
                        manifest.tunables.insert(id, reference);
                    }
                }
            }
        }

        if manifest.model_identifier.is_empty() {
            return Err("manifest has no model_identifier".to_string());
        }
        if manifest.continuous_states == 0 {
            return Err("manifest reports zero continuous states; \
                        an FMU with no states cannot be a vehicle"
                .to_string());
        }
        Ok(manifest)
    }

    pub fn load(dir: &Path) -> Result<Self, String> {
        let path = dir.join("bobdil_plant.manifest");
        let text = std::fs::read_to_string(&path)
            .map_err(|e| format!("cannot read {}: {e}", path.display()))?;
        Self::parse(&text, dir)
    }
}

/// A loaded FMU, integrated by us.
pub struct FmuMe {
    api: fmi2::Api,
    /// Kept alive for as long as `component` holds pointers into it. Dropping
    /// this while the component lives would be a use-after-dlclose.
    _library: Dylib,
    component: fmi2::Component,
    manifest: PlantManifest,

    integrator: FixedStep,
    states: Vec<f64>,
    event_indicators: Vec<f64>,

    /// Value references for the three driver inputs, resolved once.
    input_refs: [fmi2::ValueReference; 3],
    /// (value reference, index into VehicleState) for every plant output.
    output_refs: Vec<(fmi2::ValueReference, usize)>,
    output_buffer: Vec<f64>,
    output_ref_buffer: Vec<fmi2::ValueReference>,

    time: f64,
    step_index: u64,
    input: DriverInput,
    which: u64,
}

// SAFETY: an FMI component is owned exclusively by this value and is only
// touched from the thread that owns it. FMI 2.0 explicitly permits an instance
// to be used from one thread at a time.
unsafe impl Send for FmuMe {}

impl FmuMe {
    /// Load, instantiate and initialise an FMU from an unpacked directory.
    pub fn load(
        dir: &Path,
        which: u64,
        method: Method,
        substeps: usize,
    ) -> Result<Self, PlantError> {
        let manifest = PlantManifest::load(dir).map_err(|detail| PlantError::Load { detail })?;
        let library = Dylib::open(&manifest.library).map_err(|e| PlantError::Load {
            detail: e.to_string(),
        })?;
        // SAFETY: the manifest names an FMI 2.0 Model Exchange binary; the
        // signatures in `fmi2` are transcribed from the FMI 2.0.4 headers.
        let api = unsafe { fmi2::Api::resolve(&library) }
            .map_err(|detail| PlantError::Load { detail })?;

        let name =
            CString::new(manifest.model_identifier.clone()).map_err(|_| PlantError::Load {
                detail: "model identifier is not a C string".into(),
            })?;
        let guid = CString::new(manifest.guid.clone()).map_err(|_| PlantError::Load {
            detail: "guid is not a C string".into(),
        })?;
        let resources =
            CString::new(manifest.resource_uri.clone()).map_err(|_| PlantError::Load {
                detail: "resource uri is not a C string".into(),
            })?;

        // Leaked deliberately: FMI does not define how long the FMU may hold
        // the callbacks struct, and it is one allocation for the life of the
        // process. Freeing it and being wrong would be a use-after-free inside
        // someone else's generated C.
        let callbacks: &'static fmi2::CallbackFunctions =
            Box::leak(Box::new(fmi2::default_callbacks()));

        // SAFETY: every pointer below is valid for the duration of the call,
        // and `callbacks` outlives the component.
        let component = unsafe {
            (api.fmi2Instantiate)(
                name.as_ptr(),
                fmi2::TYPE_MODEL_EXCHANGE,
                guid.as_ptr(),
                resources.as_ptr(),
                callbacks,
                0, // not visible
                0, // logging off: an FMU logging per step cannot hold a deadline
            )
        };
        if component.is_null() {
            return Err(PlantError::Load {
                detail: format!(
                    "fmi2Instantiate returned null for {}",
                    manifest.model_identifier
                ),
            });
        }

        let states = manifest.continuous_states;
        let indicators = manifest.event_indicators;

        let mut input_refs = [0u32; 3];
        for (index, name) in [
            "steeringAngleCommand",
            "acceleratorPedalCommand",
            "brakePedalCommand",
        ]
        .iter()
        .enumerate()
        {
            input_refs[index] =
                *manifest
                    .value_references
                    .get(*name)
                    .ok_or_else(|| PlantError::Load {
                        detail: format!("manifest is missing input {name}"),
                    })?;
        }

        // Only the outputs the FMU actually exposes are read. A model that does
        // not publish a signal is a missing signal, not a fatal error: the
        // field stays at zero and the UI shows it as unavailable.
        let mut output_refs = Vec::new();
        for (fmi_name, field_index) in crate::generated::frames::VEHICLE_STATE_FMI_SIGNALS {
            if let Some(reference) = manifest.value_references.get(fmi_name) {
                output_refs.push((*reference, field_index));
            }
        }
        let output_count = output_refs.len();
        let output_ref_buffer = output_refs.iter().map(|(r, _)| *r).collect();

        let mut plant = Self {
            api,
            _library: library,
            component,
            manifest,
            integrator: FixedStep::new(states, method, substeps),
            states: vec![0.0; states],
            event_indicators: vec![0.0; indicators],
            input_refs,
            output_refs,
            output_buffer: vec![0.0; output_count],
            output_ref_buffer,
            time: 0.0,
            step_index: 0,
            input: DriverInput::default(),
            which,
        };
        plant.initialise(&InitialConditions::default())?;
        Ok(plant)
    }

    pub fn manifest(&self) -> &PlantManifest {
        &self.manifest
    }

    fn initialise(&mut self, init: &InitialConditions) -> Result<(), PlantError> {
        // SAFETY: `component` is a live FMI instance for every call in this
        // function; buffer lengths match what the manifest declares.
        unsafe {
            check("fmi2Reset", (self.api.fmi2Reset)(self.component)).ok();
            check(
                "fmi2SetupExperiment",
                (self.api.fmi2SetupExperiment)(self.component, 1, 1e-6, 0.0, 0, 0.0),
            )?;
            check(
                "fmi2EnterInitializationMode",
                (self.api.fmi2EnterInitializationMode)(self.component),
            )?;

            let values = [init.steering_angle, 0.0, 0.0];
            check(
                "fmi2SetReal",
                (self.api.fmi2SetReal)(
                    self.component,
                    self.input_refs.as_ptr(),
                    3,
                    values.as_ptr(),
                ),
            )?;

            check(
                "fmi2ExitInitializationMode",
                (self.api.fmi2ExitInitializationMode)(self.component),
            )?;

            // An FMU always begins in event mode. Settle the discrete states
            // before entering continuous time, or the first derivative
            // evaluation is taken at an inconsistent operating point.
            self.settle_events()?;
            check(
                "fmi2EnterContinuousTimeMode",
                (self.api.fmi2EnterContinuousTimeMode)(self.component),
            )?;
            check(
                "fmi2GetContinuousStates",
                (self.api.fmi2GetContinuousStates)(
                    self.component,
                    self.states.as_mut_ptr(),
                    self.states.len(),
                ),
            )?;
        }
        self.time = 0.0;
        self.step_index = 0;
        Ok(())
    }

    /// Iterate `fmi2NewDiscreteStates` to a fixed point, with a hard cap.
    fn settle_events(&mut self) -> Result<(), PlantError> {
        let mut info = fmi2::EventInfo {
            new_discrete_states_needed: 1,
            ..Default::default()
        };
        let mut iterations = 0;
        while info.new_discrete_states_needed != 0 {
            if iterations >= MAX_EVENT_ITERATIONS {
                return Err(PlantError::StepFailed {
                    detail: format!(
                        "FMU did not settle its discrete states in {MAX_EVENT_ITERATIONS} \
                         iterations at t={:.6}; the model is chattering",
                        self.time
                    ),
                });
            }
            // SAFETY: `component` is live and `info` is a valid EventInfo.
            let status = unsafe { (self.api.fmi2NewDiscreteStates)(self.component, &mut info) };
            check("fmi2NewDiscreteStates", status)?;
            if info.terminate_simulation != 0 {
                return Err(PlantError::StepFailed {
                    detail: "FMU requested termination".to_string(),
                });
            }
            iterations += 1;
        }
        Ok(())
    }

    fn write_inputs(&mut self) -> Result<(), PlantError> {
        let values = [
            self.input.steering_angle_command,
            self.input.accelerator_pedal_command,
            self.input.brake_pedal_command,
        ];
        // SAFETY: three references, three values, both live for the call.
        let status = unsafe {
            (self.api.fmi2SetReal)(self.component, self.input_refs.as_ptr(), 3, values.as_ptr())
        };
        check("fmi2SetReal(inputs)", status)
    }

    fn read_outputs(&mut self) -> Result<VehicleState, PlantError> {
        if !self.output_refs.is_empty() {
            // SAFETY: the reference and value buffers are the same length,
            // allocated once at load.
            let status = unsafe {
                (self.api.fmi2GetReal)(
                    self.component,
                    self.output_ref_buffer.as_ptr(),
                    self.output_ref_buffer.len(),
                    self.output_buffer.as_mut_ptr(),
                )
            };
            check("fmi2GetReal(outputs)", status)?;
        }

        let mut state = VehicleState {
            sim_time: self.time,
            step_index: self.step_index,
            steering_angle_command: self.input.steering_angle_command,
            accelerator_pedal_command: self.input.accelerator_pedal_command,
            brake_pedal_command: self.input.brake_pedal_command,
            kernel_id: self.which,
            ..Default::default()
        };
        for (slot, (_, field_index)) in self.output_refs.iter().enumerate() {
            write_field(&mut state, *field_index, self.output_buffer[slot]);
        }
        Ok(state)
    }
}

/// Turn an FMI status into a `Result`.
///
/// A free function rather than a method so it can be called in the same
/// expression as a mutable borrow of the plant's own buffers, which is the
/// shape every FMI call takes.
fn check(call: &'static str, status: fmi2::Status) -> Result<(), PlantError> {
    if fmi2::status_ok(status) {
        Ok(())
    } else {
        Err(PlantError::Fmi { call, status })
    }
}

/// Assign a `VehicleState` field by schema index.
///
/// The read side of this is generated (`VehicleState::field`); the write side
/// is here because only the FMU loader needs it. Both are driven by the same
/// index, so a schema change moves them together.
fn write_field(state: &mut VehicleState, index: usize, value: f64) {
    match VehicleState::FIELD_NAMES[index] {
        "vehicle_speed" => state.vehicle_speed = value,
        "acc_x" => state.acc_x = value,
        "acc_y" => state.acc_y = value,
        "handwheel_angle" => state.handwheel_angle = value,
        "steer_excess" => state.steer_excess = value,
        "handwheel_torque" => state.handwheel_torque = value,
        "fz_fl" => state.fz_fl = value,
        "fz_fr" => state.fz_fr = value,
        "fz_rl" => state.fz_rl = value,
        "fz_rr" => state.fz_rr = value,
        "left_steer_angle" => state.left_steer_angle = value,
        "right_steer_angle" => state.right_steer_angle = value,
        "roll" => state.roll = value,
        "sideslip" => state.sideslip = value,
        "vel_x" => state.vel_x = value,
        "vel_y" => state.vel_y = value,
        "yaw_vel" => state.yaw_vel = value,
        _ => {}
    }
}

impl Derivatives for FmuMe {
    fn derivatives(&mut self, t: f64, x: &[f64], dx: &mut [f64]) {
        // SAFETY: every buffer below is sized from the manifest's state count
        // and lives for the duration of the calls.
        unsafe {
            (self.api.fmi2SetTime)(self.component, t);
            (self.api.fmi2SetContinuousStates)(self.component, x.as_ptr(), x.len());
            let status = (self.api.fmi2GetDerivatives)(self.component, dx.as_mut_ptr(), dx.len());
            if !fmi2::status_ok(status) {
                // The integrator has no error channel, by design: it must stay
                // branch-free and allocation-free. A failed evaluation is
                // signalled as a non-finite derivative, which the finite guard
                // after the step catches and turns into a session fault.
                dx.iter_mut().for_each(|value| *value = f64::NAN);
            }
        }
    }
}

impl PlantKernel for FmuMe {
    fn reset(&mut self, init: &InitialConditions) -> Result<(), PlantError> {
        self.initialise(init)
    }

    fn step(&mut self, input: &DriverInput, dt: f64) -> Result<VehicleState, PlantError> {
        self.input = *input;
        self.write_inputs()?;

        let start_time = self.time;
        let mut integrator = std::mem::replace(&mut self.integrator, FixedStep::placeholder());
        let mut states = std::mem::take(&mut self.states);
        let end_time = integrator.advance(self, start_time, &mut states, dt);
        self.states = states;
        self.integrator = integrator;
        self.time = end_time;
        self.step_index += 1;

        // SAFETY: `component` is live; the buffers are sized from the manifest.
        unsafe {
            check(
                "fmi2SetTime",
                (self.api.fmi2SetTime)(self.component, self.time),
            )?;
            check(
                "fmi2SetContinuousStates",
                (self.api.fmi2SetContinuousStates)(
                    self.component,
                    self.states.as_ptr(),
                    self.states.len(),
                ),
            )?;

            let mut enter_event_mode: fmi2::Boolean = 0;
            let mut terminate: fmi2::Boolean = 0;
            check(
                "fmi2CompletedIntegratorStep",
                (self.api.fmi2CompletedIntegratorStep)(
                    self.component,
                    1,
                    &mut enter_event_mode,
                    &mut terminate,
                ),
            )?;
            if terminate != 0 {
                return Err(PlantError::StepFailed {
                    detail: "FMU requested termination after a step".to_string(),
                });
            }

            // A fixed-step integrator cannot locate an event in time, so an
            // event is handled at the step boundary. That is a real accuracy
            // cost and it is the price of a bounded step -- which is exactly
            // why VehicleRT is built to have no state events at all.
            if enter_event_mode != 0 || self.crossed_event_indicator()? {
                check(
                    "fmi2EnterEventMode",
                    (self.api.fmi2EnterEventMode)(self.component),
                )?;
                self.settle_events()?;
                check(
                    "fmi2EnterContinuousTimeMode",
                    (self.api.fmi2EnterContinuousTimeMode)(self.component),
                )?;
                check(
                    "fmi2GetContinuousStates",
                    (self.api.fmi2GetContinuousStates)(
                        self.component,
                        self.states.as_mut_ptr(),
                        self.states.len(),
                    ),
                )?;
            }
        }

        let state = self.read_outputs()?;
        guard_finite(&state, self.step_index)?;
        Ok(state)
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
        let reference = *self
            .manifest
            .tunables
            .get(&id)
            .ok_or(PlantError::Unsupported {
                what: "this tunable in this FMU",
            })?;
        // SAFETY: one reference, one value, both live for the call.
        let status = unsafe { (self.api.fmi2SetReal)(self.component, &reference, 1, &value) };
        check("fmi2SetReal(tunable)", status)
    }

    fn capabilities(&self) -> KernelCaps {
        KernelCaps {
            id: self.which,
            name: self.manifest.model_identifier.clone(),
            // Not knowable without measurement. `tools/rt_bench` performs the
            // eigenvalue sweep that establishes it; until it has run, the
            // conservative baseline stands and the loop will refuse a larger dt.
            max_stable_dt: 1e-3,
            continuous_states: self.manifest.continuous_states,
            supports_tunables: !self.manifest.tunables.is_empty(),
        }
    }

    fn state_vector(&self) -> &[f64] {
        &self.states
    }
}

impl FmuMe {
    /// Cheap check for a sign change in any event indicator.
    ///
    /// `fmi2CompletedIntegratorStep` does not reliably report state events on
    /// its own, so the indicators are sampled at the step boundary as well.
    fn crossed_event_indicator(&mut self) -> Result<bool, PlantError> {
        if self.event_indicators.is_empty() {
            return Ok(false);
        }
        let previous_signs: Vec<bool> = self
            .event_indicators
            .iter()
            .map(|value| value.is_sign_positive())
            .collect();
        // SAFETY: the buffer is sized from the manifest's indicator count.
        let status = unsafe {
            (self.api.fmi2GetEventIndicators)(
                self.component,
                self.event_indicators.as_mut_ptr(),
                self.event_indicators.len(),
            )
        };
        check("fmi2GetEventIndicators", status)?;
        Ok(self
            .event_indicators
            .iter()
            .zip(previous_signs)
            .any(|(value, was_positive)| value.is_sign_positive() != was_positive))
    }
}

impl Drop for FmuMe {
    fn drop(&mut self) {
        if !self.component.is_null() {
            // SAFETY: the component was created by `fmi2Instantiate` and is
            // freed exactly once.
            unsafe {
                (self.api.fmi2Terminate)(self.component);
                (self.api.fmi2FreeInstance)(self.component);
            }
            self.component = std::ptr::null_mut();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SAMPLE: &str = "\
# written by session/bobdil/vehicle_build.py
model_identifier=BobLib_Experiments_Standards_VehicleFMI
guid={abc-123}
library=binaries/linux64/VehicleFMI.so
resources=resources
states=412
event_indicators=88
vehicle_hash=0xfeedface
vref:steeringAngleCommand=17
vref:acceleratorPedalCommand=18
vref:brakePedalCommand=19
vref:handwheelTorque=204
vref:accY=207
tunable:0=990
";

    #[test]
    fn a_relative_resources_path_becomes_an_absolute_uri() {
        let manifest = PlantManifest::parse(SAMPLE, Path::new("/tmp/fmu")).unwrap();
        assert_eq!(manifest.resource_uri, "file:///tmp/fmu/resources");
    }

    #[test]
    fn the_uri_follows_the_directory_the_fmu_was_found_in() {
        // The defect this guards: a manifest written in the container recorded
        // /workspace/... and the host loaded it verbatim, pointing fmi2Instantiate
        // at a directory that does not exist.
        let manifest = PlantManifest::parse(SAMPLE, Path::new("/home/a/build/x")).unwrap();
        assert_eq!(manifest.resource_uri, "file:///home/a/build/x/resources");
    }

    #[test]
    fn a_relative_plant_directory_still_yields_an_absolute_uri() {
        // `file://build/x/resources` parses `build` as the authority and hands
        // the FMU `/x/resources`. The real vehicle failed exactly this way.
        let manifest = PlantManifest::parse(SAMPLE, Path::new("build/x")).unwrap();
        assert!(
            manifest.resource_uri.starts_with("file:///"),
            "{}",
            manifest.resource_uri
        );
        assert!(manifest.resource_uri.ends_with("/build/x/resources"));
    }

    #[test]
    fn an_already_absolute_uri_is_left_alone() {
        let text = SAMPLE.replace(
            "resources=resources",
            "resources=file:///elsewhere/resources",
        );
        let manifest = PlantManifest::parse(&text, Path::new("/tmp/fmu")).unwrap();
        assert_eq!(manifest.resource_uri, "file:///elsewhere/resources");
    }

    #[test]
    fn a_path_with_a_space_is_percent_encoded() {
        let manifest = PlantManifest::parse(SAMPLE, Path::new("/home/My Cars/fmu")).unwrap();
        assert_eq!(
            manifest.resource_uri,
            "file:///home/My%20Cars/fmu/resources"
        );
    }

    #[test]
    fn a_manifest_resolves_inputs_outputs_and_tunables() {
        let manifest = PlantManifest::parse(SAMPLE, Path::new("/tmp/fmu")).unwrap();
        assert_eq!(
            manifest.model_identifier,
            "BobLib_Experiments_Standards_VehicleFMI"
        );
        assert_eq!(manifest.continuous_states, 412);
        assert_eq!(manifest.event_indicators, 88);
        assert_eq!(manifest.vehicle_hash, 0xfeed_face);
        assert_eq!(
            manifest.library,
            Path::new("/tmp/fmu/binaries/linux64/VehicleFMI.so")
        );
        assert_eq!(
            manifest.value_references.get("steeringAngleCommand"),
            Some(&17)
        );
        assert_eq!(manifest.value_references.get("handwheelTorque"), Some(&204));
        assert_eq!(manifest.tunables.get(&0), Some(&990));
    }

    #[test]
    fn a_manifest_without_states_is_rejected() {
        let text = SAMPLE.replace("states=412", "states=0");
        let error = PlantManifest::parse(&text, Path::new("/tmp")).unwrap_err();
        assert!(error.contains("zero continuous states"), "{error}");
    }

    #[test]
    fn a_manifest_without_a_model_identifier_is_rejected() {
        let text = SAMPLE.replace(
            "model_identifier=BobLib_Experiments_Standards_VehicleFMI\n",
            "",
        );
        assert!(PlantManifest::parse(&text, Path::new("/tmp")).is_err());
    }

    #[test]
    fn comments_and_blank_lines_are_ignored() {
        let text = format!("\n\n# a comment\n{SAMPLE}\n\n");
        assert!(PlantManifest::parse(&text, Path::new("/tmp")).is_ok());
    }

    /// The field writer and the generated field reader must agree, or an FMU
    /// output would land in the wrong place with no error anywhere.
    #[test]
    fn every_fmi_backed_field_can_be_written_and_read_back() {
        for (name, index) in crate::generated::frames::VEHICLE_STATE_FMI_SIGNALS {
            let mut state = VehicleState::default();
            write_field(&mut state, index, 42.5);
            assert_eq!(
                state.field(index),
                42.5,
                "{name} (field {index}) is not wired into write_field"
            );
        }
    }
}
