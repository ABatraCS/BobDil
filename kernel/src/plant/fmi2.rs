//! FMI 2.0 Model Exchange bindings.
//!
//! Only the Model Exchange half of the standard is bound, deliberately. A
//! Co-Simulation FMU owns its own solver: `fmi2DoStep` runs a variable-step
//! DASSL internally, which is unbounded work per call with no way to impose a
//! deadline. A CS FMU cannot be made soft-real-time safe at all
//! (architecture.md 1.4), so binding `fmi2DoStep` would only make it possible
//! to build something that cannot work.
//!
//! Model Exchange gives us `fmi2GetDerivatives` and we own the integrator,
//! which buys the three things real time requires: a fixed step, a bounded
//! evaluation count, and the ability to abandon a step that runs long.

use std::os::raw::{c_char, c_int, c_uint, c_void};

pub type Component = *mut c_void;
pub type ValueReference = c_uint;
pub type Real = f64;
pub type Boolean = c_int;
pub type Status = c_int;

pub const STATUS_OK: Status = 0;
pub const STATUS_WARNING: Status = 1;
pub const STATUS_DISCARD: Status = 2;
pub const STATUS_ERROR: Status = 3;
pub const STATUS_FATAL: Status = 4;

/// Model Exchange. The other value, 1, is Co-Simulation, which this kernel
/// never instantiates -- see the module docs.
pub const TYPE_MODEL_EXCHANGE: c_int = 0;

pub fn status_ok(status: Status) -> bool {
    status == STATUS_OK || status == STATUS_WARNING
}

pub fn status_name(status: Status) -> &'static str {
    match status {
        STATUS_OK => "ok",
        STATUS_WARNING => "warning",
        STATUS_DISCARD => "discard",
        STATUS_ERROR => "error",
        STATUS_FATAL => "fatal",
        _ => "unknown",
    }
}

#[repr(C)]
pub struct CallbackFunctions {
    pub logger: extern "C" fn(*mut c_void, *const c_char, Status, *const c_char, *const c_char),
    pub allocate_memory: extern "C" fn(usize, usize) -> *mut c_void,
    pub free_memory: extern "C" fn(*mut c_void),
    pub step_finished: Option<extern "C" fn(*mut c_void, Status)>,
    pub component_environment: *mut c_void,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct EventInfo {
    pub new_discrete_states_needed: Boolean,
    pub terminate_simulation: Boolean,
    pub nominals_changed: Boolean,
    pub values_of_continuous_states_changed: Boolean,
    pub next_event_time_defined: Boolean,
    pub next_event_time: Real,
}

extern "C" {
    fn calloc(count: usize, size: usize) -> *mut c_void;
    fn free(ptr: *mut c_void);
}

/// FMI requires the importer to supply an allocator. Handing the FMU `calloc`
/// is the standard choice and, importantly, means the FMU's allocations are
/// visible to `mlockall` -- an FMU that allocated from a pool we could not lock
/// would reintroduce page faults on the hot path.
///
/// A well-behaved FMU allocates only during instantiation and initialisation,
/// never inside `fmi2GetDerivatives`. That assumption is worth checking with a
/// benchmark before trusting a new model, which is what `rt_bench` is for.
pub extern "C" fn allocate_memory(count: usize, size: usize) -> *mut c_void {
    // SAFETY: calloc with a non-negative count and size; a null return is
    // handled by the FMU, which is required to check.
    unsafe { calloc(count.max(1), size.max(1)) }
}

/// The deallocator half of the pair above.
///
/// `clippy::not_unsafe_ptr_arg_deref` wants a function that dereferences a raw
/// pointer to be `unsafe fn`, and it is right in general. It cannot apply here:
/// FMI fixes this callback's type as `extern "C" fn(*mut c_void)`, and marking
/// it `unsafe` would change the function-pointer type so it no longer fits
/// `CallbackFunctions`. The safety contract is instead held by the FMU, which
/// may only pass back a pointer `allocate_memory` returned.
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub extern "C" fn free_memory(ptr: *mut c_void) {
    if !ptr.is_null() {
        // SAFETY: the pointer came from `allocate_memory` above.
        unsafe { free(ptr) };
    }
}

/// The FMU's logger. Writes to stderr, never to the state path.
///
/// This can be called from inside a derivative evaluation, which means it can
/// happen on the real-time thread -- so it must stay cheap, and an FMU that
/// logs per step should have logging switched off rather than being tolerated.
pub extern "C" fn logger(
    _environment: *mut c_void,
    instance: *const c_char,
    status: Status,
    category: *const c_char,
    message: *const c_char,
) {
    let text = |raw: *const c_char| -> String {
        if raw.is_null() {
            String::new()
        } else {
            // SAFETY: FMI requires these to be NUL-terminated strings.
            unsafe { std::ffi::CStr::from_ptr(raw).to_string_lossy().to_string() }
        }
    };
    eprintln!(
        "fmu[{}] {} ({}): {}",
        text(instance),
        status_name(status),
        text(category),
        text(message)
    );
}

pub fn default_callbacks() -> CallbackFunctions {
    CallbackFunctions {
        logger,
        allocate_memory,
        free_memory,
        step_finished: None,
        component_environment: std::ptr::null_mut(),
    }
}

/// The subset of the FMI 2.0 ME entry points this kernel calls.
///
/// Grouped into one struct so the `dlsym` calls happen once, at load, rather
/// than anywhere near a step. Every field is resolved eagerly: an FMU missing
/// one of these is rejected at load time instead of crashing mid-drive.
#[allow(non_snake_case)]
pub struct Api {
    pub fmi2Instantiate: unsafe extern "C" fn(
        *const c_char,
        c_int,
        *const c_char,
        *const c_char,
        *const CallbackFunctions,
        Boolean,
        Boolean,
    ) -> Component,
    pub fmi2FreeInstance: unsafe extern "C" fn(Component),
    pub fmi2SetupExperiment:
        unsafe extern "C" fn(Component, Boolean, Real, Real, Boolean, Real) -> Status,
    pub fmi2EnterInitializationMode: unsafe extern "C" fn(Component) -> Status,
    pub fmi2ExitInitializationMode: unsafe extern "C" fn(Component) -> Status,
    pub fmi2Terminate: unsafe extern "C" fn(Component) -> Status,
    pub fmi2Reset: unsafe extern "C" fn(Component) -> Status,
    pub fmi2GetReal:
        unsafe extern "C" fn(Component, *const ValueReference, usize, *mut Real) -> Status,
    pub fmi2SetReal:
        unsafe extern "C" fn(Component, *const ValueReference, usize, *const Real) -> Status,
    pub fmi2SetTime: unsafe extern "C" fn(Component, Real) -> Status,
    pub fmi2SetContinuousStates: unsafe extern "C" fn(Component, *const Real, usize) -> Status,
    pub fmi2GetContinuousStates: unsafe extern "C" fn(Component, *mut Real, usize) -> Status,
    pub fmi2GetDerivatives: unsafe extern "C" fn(Component, *mut Real, usize) -> Status,
    pub fmi2GetEventIndicators: unsafe extern "C" fn(Component, *mut Real, usize) -> Status,
    pub fmi2EnterEventMode: unsafe extern "C" fn(Component) -> Status,
    pub fmi2NewDiscreteStates: unsafe extern "C" fn(Component, *mut EventInfo) -> Status,
    pub fmi2EnterContinuousTimeMode: unsafe extern "C" fn(Component) -> Status,
    pub fmi2CompletedIntegratorStep:
        unsafe extern "C" fn(Component, Boolean, *mut Boolean, *mut Boolean) -> Status,
}

impl Api {
    /// Resolve every entry point from a loaded FMU binary.
    ///
    /// # Safety
    /// `library` must be an FMI 2.0 Model Exchange shared object. The signatures
    /// above are transcribed from the FMI 2.0.4 specification headers; a library
    /// that is not an FMI 2.0 FMU would be reinterpreted with the wrong ABI.
    pub unsafe fn resolve(library: &crate::sys::dylib::Dylib) -> Result<Self, String> {
        macro_rules! symbol {
            ($name:literal) => {
                library.symbol($name).map_err(|e| e.to_string())?
            };
        }
        Ok(Self {
            fmi2Instantiate: symbol!("fmi2Instantiate"),
            fmi2FreeInstance: symbol!("fmi2FreeInstance"),
            fmi2SetupExperiment: symbol!("fmi2SetupExperiment"),
            fmi2EnterInitializationMode: symbol!("fmi2EnterInitializationMode"),
            fmi2ExitInitializationMode: symbol!("fmi2ExitInitializationMode"),
            fmi2Terminate: symbol!("fmi2Terminate"),
            fmi2Reset: symbol!("fmi2Reset"),
            fmi2GetReal: symbol!("fmi2GetReal"),
            fmi2SetReal: symbol!("fmi2SetReal"),
            fmi2SetTime: symbol!("fmi2SetTime"),
            fmi2SetContinuousStates: symbol!("fmi2SetContinuousStates"),
            fmi2GetContinuousStates: symbol!("fmi2GetContinuousStates"),
            fmi2GetDerivatives: symbol!("fmi2GetDerivatives"),
            fmi2GetEventIndicators: symbol!("fmi2GetEventIndicators"),
            fmi2EnterEventMode: symbol!("fmi2EnterEventMode"),
            fmi2NewDiscreteStates: symbol!("fmi2NewDiscreteStates"),
            fmi2EnterContinuousTimeMode: symbol!("fmi2EnterContinuousTimeMode"),
            fmi2CompletedIntegratorStep: symbol!("fmi2CompletedIntegratorStep"),
        })
    }
}
