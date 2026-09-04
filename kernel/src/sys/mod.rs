//! Thin, audited bindings to the platform primitives the kernel needs.
//!
//! Every symbol here is declared rather than pulled from a crate. The hot path
//! must contain no code we have not read: an unexpected allocation, lock, or
//! syscall inside a dependency is exactly the kind of fault that shows up as an
//! occasional 3 ms spike and takes a week to find.

pub mod clock;
pub mod dylib;
pub mod sched;
pub mod shm;
pub mod signals;

use std::os::raw::c_int;

/// A platform call that failed, carrying the errno the caller needs to report.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SysError {
    pub call: &'static str,
    pub errno: i32,
}

impl std::fmt::Display for SysError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{} failed (errno {})", self.call, self.errno)
    }
}

impl std::error::Error for SysError {}

pub(crate) fn errno() -> i32 {
    std::io::Error::last_os_error().raw_os_error().unwrap_or(0)
}

pub(crate) fn check(call: &'static str, rc: c_int) -> Result<(), SysError> {
    if rc == 0 {
        Ok(())
    } else {
        Err(SysError {
            call,
            errno: errno(),
        })
    }
}
