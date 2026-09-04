//! Device abstraction, and the capability negotiation the spec forces on us.
//!
//! The spec asks for two things that cannot both be true of the same hardware
//! (architecture.md 5.3): support for any off-the-shelf wheel and pedals, *and*
//! pedal stiffness fed back to the driver. Almost no off-the-shelf pedal set
//! can render variable stiffness. A load-cell brake -- the good kind -- measures
//! force; its stiffness is a fixed elastomer stack. Active-force pedals exist
//! but are rare and expensive, and are not what "works in a standard simulator"
//! means.
//!
//! So capability is negotiated and reported, never assumed. Wheel torque is the
//! one mandatory output. Pedal force is optional, and when it is absent the
//! session degrades honestly: the cue is surfaced visually and through a rumble
//! motor if one exists, and the UI does not claim stiffness feedback. Silently
//! pretending would be the worst option, because a driver would attribute the
//! missing cue to the vehicle.

use crate::generated::frames::{DriverInput, FfbCommand};

/// What a connected device can actually do.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct DeviceCaps {
    pub name: String,
    /// Constant-force output on the steering axis. Mandatory for a usable rig.
    pub wheel_torque: bool,
    /// Peak torque the device claims, if it reports one [N.m].
    pub wheel_torque_limit: Option<f64>,
    /// Full range of the wheel [rad].
    pub wheel_range: Option<f64>,
    /// Programmable pedal stiffness. Almost always false. See the module docs.
    pub pedal_force: bool,
    /// A rumble motor, used as the degraded lock-up cue when `pedal_force` is false.
    pub rumble: bool,
    pub axes: usize,
    pub buttons: usize,
}

impl DeviceCaps {
    /// One line, shown in the UI at session start. A driver must be able to see
    /// which cues they are and are not getting before they form an opinion
    /// about the car.
    pub fn describe(&self) -> String {
        let mut cues = Vec::new();
        if self.wheel_torque {
            match self.wheel_torque_limit {
                Some(limit) => cues.push(format!("wheel torque (device peak {limit:.1} N.m)")),
                None => cues.push("wheel torque".to_string()),
            }
        }
        if self.pedal_force {
            cues.push("pedal force".to_string());
        }
        if self.rumble {
            cues.push("rumble".to_string());
        }
        if cues.is_empty() {
            cues.push("no haptic output".to_string());
        }
        format!(
            "{} [{} axes, {} buttons]: {}",
            self.name,
            self.axes,
            self.buttons,
            cues.join(", ")
        )
    }

    /// What the UI must *not* claim. Returned so the message is written once.
    pub fn missing_cues(&self) -> Vec<&'static str> {
        let mut missing = Vec::new();
        if !self.wheel_torque {
            missing.push("steering torque");
        }
        if !self.pedal_force {
            missing.push("pedal stiffness");
        }
        missing
    }
}

#[derive(Debug)]
pub enum DeviceError {
    NotFound,
    Unsupported { what: &'static str },
    Backend { detail: String },
}

impl std::fmt::Display for DeviceError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NotFound => write!(f, "no wheel or pedal device found"),
            Self::Unsupported { what } => write!(f, "device does not support {what}"),
            Self::Backend { detail } => write!(f, "device backend error: {detail}"),
        }
    }
}

impl std::error::Error for DeviceError {}

/// Raw axis and button state, before calibration.
pub trait InputSource: Send {
    /// Poll the device. Returns the raw normalised axes; calibration into
    /// vehicle units happens in `calibration`, not here.
    fn poll(&mut self) -> Result<RawInput, DeviceError>;
    fn caps(&self) -> &DeviceCaps;
}

/// Where conditioned force goes.
///
/// Implementations must treat `zero()` as infallible-in-spirit: it is called on
/// every shutdown path including panics, and leaving a device holding torque is
/// the failure this whole layer exists to prevent.
pub trait HapticSink: Send {
    fn apply(&mut self, command: &FfbCommand) -> Result<(), DeviceError>;
    fn zero(&mut self);
    fn caps(&self) -> &DeviceCaps;
}

/// Device axes as reported, normalised but uncalibrated.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct RawInput {
    /// Steering axis, -1 (full right) to +1 (full left).
    pub steer: f64,
    /// Accelerator, 0 to 1.
    pub accelerator: f64,
    /// Brake, 0 to 1.
    pub brake: f64,
    pub buttons: u64,
    pub host_time_ns: u64,
}

/// Maps raw device axes onto the units the plant expects.
#[derive(Debug, Clone)]
pub struct Calibration {
    /// Handwheel angle at full axis deflection [rad]. Must match what the
    /// device is actually configured to, or the whole rig lies about steering
    /// ratio.
    pub wheel_range: f64,
    /// Deadband around centre, as a fraction of full travel.
    pub steer_deadband: f64,
    pub accelerator_floor: f64,
    pub accelerator_ceiling: f64,
    pub brake_floor: f64,
    pub brake_ceiling: f64,
    /// Exponent applied to the brake axis. A load-cell pedal is roughly linear
    /// in force; a potentiometer pedal is linear in travel, which feels wrong,
    /// and this is the honest place to correct for it.
    pub brake_gamma: f64,
    pub invert_steer: bool,
}

impl Default for Calibration {
    fn default() -> Self {
        Self {
            wheel_range: 4.712_388_980_384_69, // 270 deg each way
            steer_deadband: 0.0,
            accelerator_floor: 0.02,
            accelerator_ceiling: 0.98,
            brake_floor: 0.02,
            brake_ceiling: 0.95,
            brake_gamma: 1.0,
            invert_steer: false,
        }
    }
}

fn rescale(value: f64, floor: f64, ceiling: f64) -> f64 {
    if ceiling <= floor {
        return 0.0;
    }
    ((value - floor) / (ceiling - floor)).clamp(0.0, 1.0)
}

impl Calibration {
    pub fn apply(&self, raw: &RawInput, sample_index: u64) -> DriverInput {
        let mut steer = raw.steer.clamp(-1.0, 1.0);
        if self.invert_steer {
            steer = -steer;
        }
        if steer.abs() < self.steer_deadband {
            steer = 0.0;
        } else if self.steer_deadband > 0.0 {
            let sign = steer.signum();
            steer = sign * (steer.abs() - self.steer_deadband) / (1.0 - self.steer_deadband);
        }

        let accelerator = rescale(
            raw.accelerator,
            self.accelerator_floor,
            self.accelerator_ceiling,
        );
        let brake = rescale(raw.brake, self.brake_floor, self.brake_ceiling).powf(self.brake_gamma);

        DriverInput {
            host_time_ns: raw.host_time_ns,
            sample_index,
            steering_angle_command: steer * self.wheel_range * 0.5,
            accelerator_pedal_command: accelerator,
            brake_pedal_command: brake,
            raw_steer_norm: raw.steer,
            raw_accel_norm: raw.accelerator,
            raw_brake_norm: raw.brake,
            buttons: raw.buttons,
        }
    }
}

/// A device that is not there.
///
/// Used for headless CI, for the timing benchmark, and whenever no wheel is
/// connected. Everything above it -- the loop, the buffers, the watchdog, the
/// telemetry -- runs identically, which is what makes those layers testable
/// without hardware plugged in.
pub struct NullDevice {
    caps: DeviceCaps,
    /// Scripted input, so a benchmark or a regression test can drive.
    script: Option<Box<dyn Fn(u64) -> RawInput + Send>>,
    samples: u64,
    pub last_command: FfbCommand,
    pub zeroed: bool,
}

impl NullDevice {
    pub fn new() -> Self {
        Self {
            caps: DeviceCaps {
                name: "null device (no hardware)".to_string(),
                wheel_torque: false,
                wheel_torque_limit: None,
                wheel_range: None,
                pedal_force: false,
                rumble: false,
                axes: 0,
                buttons: 0,
            },
            script: None,
            samples: 0,
            last_command: FfbCommand::default(),
            zeroed: false,
        }
    }

    /// Drive the loop from a function of sample index instead of hardware.
    pub fn scripted(script: impl Fn(u64) -> RawInput + Send + 'static) -> Self {
        let mut device = Self::new();
        device.caps.name = "scripted input".to_string();
        device.script = Some(Box::new(script));
        device
    }
}

impl Default for NullDevice {
    fn default() -> Self {
        Self::new()
    }
}

impl InputSource for NullDevice {
    fn poll(&mut self) -> Result<RawInput, DeviceError> {
        let index = self.samples;
        self.samples += 1;
        Ok(match &self.script {
            Some(script) => script(index),
            None => RawInput::default(),
        })
    }

    fn caps(&self) -> &DeviceCaps {
        &self.caps
    }
}

impl HapticSink for NullDevice {
    fn apply(&mut self, command: &FfbCommand) -> Result<(), DeviceError> {
        self.last_command = *command;
        self.zeroed = false;
        Ok(())
    }

    fn zero(&mut self) {
        self.last_command = FfbCommand::default();
        self.zeroed = true;
    }

    fn caps(&self) -> &DeviceCaps {
        &self.caps
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn calibration_maps_full_deflection_to_the_configured_range() {
        let calibration = Calibration {
            wheel_range: 4.0,
            ..Default::default()
        };
        let raw = RawInput {
            steer: 1.0,
            ..Default::default()
        };
        let input = calibration.apply(&raw, 0);
        assert!((input.steering_angle_command - 2.0).abs() < 1e-12);
    }

    #[test]
    fn pedal_travel_is_rescaled_past_its_dead_zones() {
        let calibration = Calibration::default();
        let idle = calibration.apply(
            &RawInput {
                brake: 0.01,
                ..Default::default()
            },
            0,
        );
        assert_eq!(
            idle.brake_pedal_command, 0.0,
            "a resting pedal must read exactly zero"
        );
        let pressed = calibration.apply(
            &RawInput {
                brake: 0.99,
                ..Default::default()
            },
            0,
        );
        assert_eq!(
            pressed.brake_pedal_command, 1.0,
            "a fully pressed pedal must reach one"
        );
    }

    #[test]
    fn raw_axes_are_preserved_for_replay_forensics() {
        let calibration = Calibration::default();
        let raw = RawInput {
            steer: 0.37,
            accelerator: 0.4,
            brake: 0.6,
            ..Default::default()
        };
        let input = calibration.apply(&raw, 5);
        assert_eq!(input.raw_steer_norm, 0.37);
        assert_eq!(input.raw_accel_norm, 0.4);
        assert_eq!(input.raw_brake_norm, 0.6);
        assert_eq!(input.sample_index, 5);
    }

    #[test]
    fn a_device_without_force_output_says_so_rather_than_pretending() {
        let device = NullDevice::new();
        let missing = InputSource::caps(&device).missing_cues();
        assert!(missing.contains(&"steering torque"));
        assert!(missing.contains(&"pedal stiffness"));
    }
}
