//! SDL3 wheel and pedal backend.
//!
//! SDL3 is the one API that covers Windows (DirectInput/XInput) and Linux
//! (evdev force feedback) with the same calls, which is precisely the spec's
//! "as long as the wheel and pedals work in a standard simulator, they will
//! work with BobDil".
//!
//! The bindings are hand-written against the SDL3 headers rather than pulled
//! from a crate, for the reason given in `sys/`: this code runs next to the
//! real-time path and it must contain nothing we have not read. Every struct
//! layout below was verified against `sizeof`/`offsetof` on the real headers --
//! `SDL_HapticEffect` is 72 bytes aligned to 8, `SDL_HapticConstant` is 40 --
//! and SDL guarantees ABI stability within a major version.
//!
//! Force feedback is delivered as a single long-running CONSTANT effect whose
//! level is updated each step. That is the standard way to render an arbitrary
//! torque signal through the FFB API: creating an effect per step would
//! allocate in the driver and would not be deadline-safe.

#![cfg(feature = "sdl3")]

use std::ffi::CStr;
use std::os::raw::{c_char, c_int, c_void};

use crate::generated::frames::FfbCommand;
use crate::sys::clock::monotonic_ns;

use super::device::{DeviceCaps, DeviceError, HapticSink, InputSource, RawInput};

const SDL_INIT_JOYSTICK: u32 = 0x0000_0200;
const SDL_INIT_HAPTIC: u32 = 0x0000_1000;
const SDL_HAPTIC_CONSTANT: u32 = 1 << 0;
const SDL_HAPTIC_INFINITY: u32 = 4_294_967_295;
const SDL_HAPTIC_CARTESIAN: u8 = 1;

#[repr(C)]
#[derive(Clone, Copy)]
struct HapticDirection {
    kind: u8,
    dir: [i32; 3],
}

/// Layout verified against SDL_haptic.h: 40 bytes, fields at
/// 0/4/20/24/26/28/30/32/34/36/38.
#[repr(C)]
#[derive(Clone, Copy)]
struct HapticConstant {
    kind: u16,
    direction: HapticDirection,
    length: u32,
    delay: u16,
    button: u16,
    interval: u16,
    level: i16,
    attack_length: u16,
    attack_level: u16,
    fade_length: u16,
    fade_level: u16,
}

/// `SDL_HapticEffect` is a union; SDL only reads the members belonging to the
/// effect type in `kind`, so a correctly sized and aligned buffer with the
/// constant effect at its head is exactly what the API expects.
#[repr(C, align(8))]
#[derive(Clone, Copy)]
struct HapticEffect {
    constant: HapticConstant,
    _union_padding: [u8; 32],
}

const _: () = assert!(std::mem::size_of::<HapticEffect>() == 72);
const _: () = assert!(std::mem::size_of::<HapticConstant>() == 40);
const _: () = assert!(std::mem::size_of::<HapticDirection>() == 16);

#[link(name = "SDL3")]
extern "C" {
    fn SDL_Init(flags: u32) -> bool;
    fn SDL_Quit();
    fn SDL_GetError() -> *const c_char;
    fn SDL_free(mem: *mut c_void);

    fn SDL_GetJoysticks(count: *mut c_int) -> *mut u32;
    fn SDL_OpenJoystick(instance_id: u32) -> *mut c_void;
    fn SDL_CloseJoystick(joystick: *mut c_void);
    fn SDL_GetJoystickName(joystick: *mut c_void) -> *const c_char;
    fn SDL_GetNumJoystickAxes(joystick: *mut c_void) -> c_int;
    fn SDL_GetNumJoystickButtons(joystick: *mut c_void) -> c_int;
    fn SDL_GetJoystickAxis(joystick: *mut c_void, axis: c_int) -> i16;
    fn SDL_GetJoystickButton(joystick: *mut c_void, button: c_int) -> bool;
    fn SDL_UpdateJoysticks();

    fn SDL_IsJoystickHaptic(joystick: *mut c_void) -> bool;
    fn SDL_OpenHapticFromJoystick(joystick: *mut c_void) -> *mut c_void;
    fn SDL_CloseHaptic(haptic: *mut c_void);
    fn SDL_GetHapticName(haptic: *mut c_void) -> *const c_char;
    fn SDL_GetHapticFeatures(haptic: *mut c_void) -> u32;
    fn SDL_CreateHapticEffect(haptic: *mut c_void, effect: *const HapticEffect) -> c_int;
    fn SDL_UpdateHapticEffect(
        haptic: *mut c_void,
        effect_id: c_int,
        effect: *const HapticEffect,
    ) -> bool;
    fn SDL_RunHapticEffect(haptic: *mut c_void, effect_id: c_int, iterations: u32) -> bool;
    fn SDL_StopHapticEffect(haptic: *mut c_void, effect_id: c_int) -> bool;
    fn SDL_DestroyHapticEffect(haptic: *mut c_void, effect_id: c_int);
    fn SDL_SetHapticAutocenter(haptic: *mut c_void, autocenter: c_int) -> bool;
    fn SDL_SetHapticGain(haptic: *mut c_void, gain: c_int) -> bool;
    fn SDL_HapticRumbleSupported(haptic: *mut c_void) -> bool;
}

fn sdl_error() -> String {
    // SAFETY: SDL_GetError always returns a valid NUL-terminated string owned
    // by SDL, valid until the next SDL call on this thread.
    unsafe {
        let raw = SDL_GetError();
        if raw.is_null() {
            String::new()
        } else {
            CStr::from_ptr(raw).to_string_lossy().to_string()
        }
    }
}

fn sdl_string(raw: *const c_char) -> String {
    if raw.is_null() {
        return "unknown".to_string();
    }
    // SAFETY: SDL name getters return a NUL-terminated string or NULL, which
    // was just checked.
    unsafe { CStr::from_ptr(raw).to_string_lossy().to_string() }
}

/// Which physical axis carries which control.
///
/// Wheels disagree wildly about this, and pedals are often a separate device,
/// so it is configuration rather than a guess. The defaults match the most
/// common layout (steer, throttle, brake on axes 0, 1, 2).
#[derive(Debug, Clone)]
pub struct AxisMap {
    pub steer: usize,
    pub accelerator: usize,
    pub brake: usize,
    /// True when a pedal rests at -32768 and is fully pressed at +32767, which
    /// is what most wheels report. False for pedals that rest at 0.
    pub pedals_are_bipolar: bool,
    pub invert_accelerator: bool,
    pub invert_brake: bool,
}

impl Default for AxisMap {
    fn default() -> Self {
        Self {
            steer: 0,
            accelerator: 1,
            brake: 2,
            pedals_are_bipolar: true,
            invert_accelerator: false,
            invert_brake: false,
        }
    }
}

/// One SDL device used for input, optionally with a second one for pedals.
pub struct SdlDevice {
    joystick: *mut c_void,
    pedals: Option<*mut c_void>,
    haptic: Option<*mut c_void>,
    effect_id: c_int,
    caps: DeviceCaps,
    axes: AxisMap,
    /// Torque at which the constant effect reaches full scale [N.m]. The device
    /// takes a -32768..32767 level, so a physical torque has to be referred to
    /// something; this is that reference, and it must be set to the device's
    /// real peak or every torque in the session is wrong by a constant factor.
    full_scale_torque: f64,
    last_level: i16,
}

// SAFETY: SDL joystick and haptic handles are plain pointers into SDL's own
// state. This type is moved to the device thread at construction and only ever
// touched from there.
unsafe impl Send for SdlDevice {}

impl SdlDevice {
    /// Initialise SDL and open the first joystick that reports haptic support,
    /// falling back to the first joystick of any kind.
    pub fn open(axes: AxisMap, full_scale_torque: f64) -> Result<Self, DeviceError> {
        // SAFETY: SDL_Init takes only flags and is safe to call repeatedly.
        if !unsafe { SDL_Init(SDL_INIT_JOYSTICK | SDL_INIT_HAPTIC) } {
            return Err(DeviceError::Backend {
                detail: format!("SDL_Init: {}", sdl_error()),
            });
        }

        let ids = Self::joystick_ids();
        if ids.is_empty() {
            return Err(DeviceError::NotFound);
        }

        // Prefer a device that can actually produce torque.
        let mut chosen = None;
        for id in &ids {
            // SAFETY: `id` came from SDL_GetJoysticks and is still valid.
            let joystick = unsafe { SDL_OpenJoystick(*id) };
            if joystick.is_null() {
                continue;
            }
            // SAFETY: `joystick` is a live handle.
            let haptic_capable = unsafe { SDL_IsJoystickHaptic(joystick) };
            if haptic_capable {
                chosen = Some(joystick);
                break;
            }
            if chosen.is_none() {
                chosen = Some(joystick);
            } else {
                // SAFETY: closing a handle we opened and are discarding.
                unsafe { SDL_CloseJoystick(joystick) };
            }
        }

        let joystick = chosen.ok_or(DeviceError::NotFound)?;
        // SAFETY: `joystick` is a live handle for all of the following.
        let (name, axis_count, button_count) = unsafe {
            (
                sdl_string(SDL_GetJoystickName(joystick)),
                SDL_GetNumJoystickAxes(joystick).max(0) as usize,
                SDL_GetNumJoystickButtons(joystick).max(0) as usize,
            )
        };

        // SAFETY: `joystick` is live; a null return means no haptic support.
        let haptic_ptr = unsafe { SDL_OpenHapticFromJoystick(joystick) };
        let haptic = if haptic_ptr.is_null() {
            None
        } else {
            Some(haptic_ptr)
        };

        let mut caps = DeviceCaps {
            name,
            wheel_torque: false,
            wheel_torque_limit: None,
            wheel_range: None,
            // No mainstream pedal set can render programmable stiffness.
            // Reported honestly rather than assumed; see io::device.
            pedal_force: false,
            rumble: false,
            axes: axis_count,
            buttons: button_count,
        };

        let mut effect_id = -1;
        if let Some(haptic) = haptic {
            // SAFETY: `haptic` is a live handle for all of the following.
            unsafe {
                let features = SDL_GetHapticFeatures(haptic);
                caps.name = format!("{} / {}", caps.name, sdl_string(SDL_GetHapticName(haptic)));
                caps.rumble = SDL_HapticRumbleSupported(haptic);
                if features & SDL_HAPTIC_CONSTANT != 0 {
                    // The device's own autocentre spring would be added to our
                    // torque, and it is not derived from the model. Off.
                    SDL_SetHapticAutocenter(haptic, 0);
                    SDL_SetHapticGain(haptic, 100);

                    let effect = Self::constant_effect(0);
                    effect_id = SDL_CreateHapticEffect(haptic, &effect);
                    if effect_id >= 0 && SDL_RunHapticEffect(haptic, effect_id, SDL_HAPTIC_INFINITY)
                    {
                        caps.wheel_torque = true;
                        caps.wheel_torque_limit = Some(full_scale_torque);
                    }
                }
            }
        }

        Ok(Self {
            joystick,
            pedals: None,
            haptic,
            effect_id,
            caps,
            axes,
            full_scale_torque: full_scale_torque.max(1e-3),
            last_level: 0,
        })
    }

    /// Use a second device for the pedals. Common on real rigs, where the
    /// pedal set enumerates separately from the wheel base.
    pub fn with_pedal_device(mut self, index: usize) -> Result<Self, DeviceError> {
        let ids = Self::joystick_ids();
        let id = ids.get(index).copied().ok_or(DeviceError::NotFound)?;
        // SAFETY: `id` came from SDL_GetJoysticks.
        let handle = unsafe { SDL_OpenJoystick(id) };
        if handle.is_null() {
            return Err(DeviceError::Backend {
                detail: sdl_error(),
            });
        }
        self.pedals = Some(handle);
        Ok(self)
    }

    fn joystick_ids() -> Vec<u32> {
        let mut count: c_int = 0;
        // SAFETY: SDL_GetJoysticks writes the count and returns an
        // SDL-allocated array of that length, or NULL.
        unsafe {
            let raw = SDL_GetJoysticks(&mut count);
            if raw.is_null() || count <= 0 {
                return Vec::new();
            }
            let ids = std::slice::from_raw_parts(raw, count as usize).to_vec();
            SDL_free(raw as *mut c_void);
            ids
        }
    }

    fn constant_effect(level: i16) -> HapticEffect {
        HapticEffect {
            constant: HapticConstant {
                kind: SDL_HAPTIC_CONSTANT as u16,
                // Along the wheel's own axis, which is what a wheel base
                // interprets as steering torque.
                direction: HapticDirection {
                    kind: SDL_HAPTIC_CARTESIAN,
                    dir: [1, 0, 0],
                },
                length: SDL_HAPTIC_INFINITY,
                delay: 0,
                button: 0,
                interval: 0,
                level,
                attack_length: 0,
                attack_level: 0,
                fade_length: 0,
                fade_level: 0,
            },
            _union_padding: [0u8; 32],
        }
    }

    fn axis(&self, handle: *mut c_void, index: usize) -> f64 {
        // SAFETY: `handle` is a live joystick; SDL returns 0 for an out of
        // range axis index rather than faulting.
        let raw = unsafe { SDL_GetJoystickAxis(handle, index as c_int) };
        raw as f64 / 32767.0
    }

    fn pedal(&self, handle: *mut c_void, index: usize, invert: bool) -> f64 {
        let normalised = self.axis(handle, index);
        let value = if self.axes.pedals_are_bipolar {
            (normalised + 1.0) * 0.5
        } else {
            normalised
        };
        let value = value.clamp(0.0, 1.0);
        if invert {
            1.0 - value
        } else {
            value
        }
    }
}

impl InputSource for SdlDevice {
    fn poll(&mut self) -> Result<RawInput, DeviceError> {
        // SAFETY: SDL_UpdateJoysticks takes no arguments and is safe once SDL
        // has been initialised.
        unsafe { SDL_UpdateJoysticks() };
        let pedal_handle = self.pedals.unwrap_or(self.joystick);

        let mut buttons = 0u64;
        for index in 0..self.caps.buttons.min(64) {
            // SAFETY: `joystick` is live and the index is within the reported
            // button count.
            if unsafe { SDL_GetJoystickButton(self.joystick, index as c_int) } {
                buttons |= 1 << index;
            }
        }

        Ok(RawInput {
            steer: self.axis(self.joystick, self.axes.steer).clamp(-1.0, 1.0),
            accelerator: self.pedal(
                pedal_handle,
                self.axes.accelerator,
                self.axes.invert_accelerator,
            ),
            brake: self.pedal(pedal_handle, self.axes.brake, self.axes.invert_brake),
            buttons,
            host_time_ns: monotonic_ns(),
        })
    }

    fn caps(&self) -> &DeviceCaps {
        &self.caps
    }
}

impl HapticSink for SdlDevice {
    fn apply(&mut self, command: &FfbCommand) -> Result<(), DeviceError> {
        let Some(haptic) = self.haptic else {
            return Err(DeviceError::Unsupported {
                what: "constant force output",
            });
        };
        if self.effect_id < 0 {
            return Err(DeviceError::Unsupported {
                what: "constant force output",
            });
        }

        // A non-finite torque must never reach a driver's hands. The chain
        // already guards this; guarding again here costs one comparison and
        // means a future caller that bypasses the chain still cannot hurt
        // anyone. Independent barriers are the point.
        let torque = if command.torque_nm.is_finite() {
            command.torque_nm
        } else {
            0.0
        };
        let normalised = (torque / self.full_scale_torque).clamp(-1.0, 1.0);
        let level = (normalised * 32767.0) as i16;

        if level == self.last_level {
            return Ok(());
        }
        let effect = Self::constant_effect(level);
        // SAFETY: `haptic` is live and `effect_id` refers to an effect created
        // on it; `effect` outlives the call.
        if !unsafe { SDL_UpdateHapticEffect(haptic, self.effect_id, &effect) } {
            return Err(DeviceError::Backend {
                detail: sdl_error(),
            });
        }
        self.last_level = level;
        Ok(())
    }

    fn zero(&mut self) {
        let Some(haptic) = self.haptic else { return };
        if self.effect_id < 0 {
            return;
        }
        let effect = Self::constant_effect(0);
        // SAFETY: `haptic` is live and `effect_id` is valid. Errors are
        // deliberately ignored: this runs on shutdown and panic paths, and
        // there is nothing useful to do with a failure except try the stop.
        unsafe {
            SDL_UpdateHapticEffect(haptic, self.effect_id, &effect);
            SDL_StopHapticEffect(haptic, self.effect_id);
        }
        self.last_level = 0;
    }

    fn caps(&self) -> &DeviceCaps {
        &self.caps
    }
}

impl Drop for SdlDevice {
    fn drop(&mut self) {
        // Rule 4: the device is left at zero torque on every exit path.
        self.zero();
        // SAFETY: every handle below was opened by this value and is dropped
        // exactly once.
        unsafe {
            if let Some(haptic) = self.haptic {
                if self.effect_id >= 0 {
                    SDL_DestroyHapticEffect(haptic, self.effect_id);
                }
                SDL_CloseHaptic(haptic);
            }
            if let Some(pedals) = self.pedals {
                SDL_CloseJoystick(pedals);
            }
            SDL_CloseJoystick(self.joystick);
            SDL_Quit();
        }
    }
}
