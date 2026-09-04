//! Interrupt handling, so that Ctrl-C leaves the wheel slack.
//!
//! Rule 4 of the safety contract is that torque is zeroed on *every* exit path,
//! and a driver reaching for Ctrl-C is one of them -- quite possibly the one
//! they reach for precisely because the wheel is doing something they do not
//! like.

use std::os::raw::c_int;
use std::sync::atomic::{AtomicBool, Ordering};

const SIGINT: c_int = 2;
const SIGTERM: c_int = 15;
const SIGHUP: c_int = 1;

static INTERRUPTED: AtomicBool = AtomicBool::new(false);

extern "C" {
    fn signal(signum: c_int, handler: extern "C" fn(c_int)) -> usize;
}

/// Async-signal-safe by construction: a single relaxed atomic store and nothing
/// else. Allocating or locking in a signal handler is undefined behaviour, and
/// the shutdown itself is done by the loop when it next checks this flag.
extern "C" fn handle(_signal: c_int) {
    INTERRUPTED.store(true, Ordering::Relaxed);
}

pub fn install() {
    for signum in [SIGINT, SIGTERM, SIGHUP] {
        // SAFETY: `handle` is a valid `extern "C"` handler and is
        // async-signal-safe.
        unsafe { signal(signum, handle) };
    }
}

pub fn interrupted() -> bool {
    INTERRUPTED.load(Ordering::Relaxed)
}

pub fn reset() {
    INTERRUPTED.store(false, Ordering::Relaxed);
}
