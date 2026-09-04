//! Devices, and everything between the model and the driver's hands.
//!
//! Nothing in this module knows what a tire is. It takes a `VehicleState` and
//! turns it into a torque, and takes a device reading and turns it into a
//! `DriverInput`. That separation is what lets the plant be swapped without
//! touching the safety path, and lets the safety path be tested to destruction
//! without a wheel plugged in.
//!
//! `watchdog` is safety code. It is reviewed as safety code, tested
//! adversarially, and run before any human drives the rig.

pub mod device;
pub mod ffb;
pub mod hid;
pub mod input_shaper;
#[cfg(feature = "sdl3")]
pub mod sdl3;

pub use device::{
    Calibration, DeviceCaps, DeviceError, HapticSink, InputSource, NullDevice, RawInput,
};
pub use ffb::{FfbChain, FfbConfig, FfbOutcome};
pub use input_shaper::{ShapedSteer, ShaperConfig, SteerShaper};
pub use watchdog::{Verdict, Watchdog, WatchdogConfig};

pub mod watchdog;
