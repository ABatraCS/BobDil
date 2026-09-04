//! Monotonic time and absolute-deadline sleeping.
//!
//! Pacing uses `clock_nanosleep` with `TIMER_ABSTIME` rather than sleeping for a
//! computed remainder. A relative sleep accumulates the wake-up latency of every
//! previous step, so a 1 kHz loop built on it drifts steadily away from wall
//! clock; an absolute deadline does not.

use std::os::raw::{c_int, c_long};

use super::SysError;

pub const CLOCK_MONOTONIC: c_int = 1;
const TIMER_ABSTIME: c_int = 1;

#[repr(C)]
#[derive(Clone, Copy, Default)]
struct Timespec {
    tv_sec: i64,
    tv_nsec: c_long,
}

extern "C" {
    fn clock_gettime(clock_id: c_int, tp: *mut Timespec) -> c_int;
    fn clock_nanosleep(
        clock_id: c_int,
        flags: c_int,
        request: *const Timespec,
        remain: *mut Timespec,
    ) -> c_int;
}

/// Nanoseconds since an unspecified but monotonic epoch. Never goes backwards,
/// and is unaffected by NTP or a user changing the system clock mid-session.
pub fn monotonic_ns() -> u64 {
    let mut ts = Timespec::default();
    // SAFETY: `ts` is a valid, correctly sized, exclusively owned Timespec.
    let rc = unsafe { clock_gettime(CLOCK_MONOTONIC, &mut ts) };
    debug_assert_eq!(rc, 0, "CLOCK_MONOTONIC is always available on Linux");
    (ts.tv_sec as u64)
        .wrapping_mul(1_000_000_000)
        .wrapping_add(ts.tv_nsec as u64)
}

/// Sleep until an absolute monotonic deadline.
///
/// Returns immediately if the deadline has already passed, which is how an
/// overrun is detected: the caller compares the post-sleep clock against the
/// deadline it asked for.
pub fn sleep_until(deadline_ns: u64) -> Result<(), SysError> {
    let request = Timespec {
        tv_sec: (deadline_ns / 1_000_000_000) as i64,
        tv_nsec: (deadline_ns % 1_000_000_000) as c_long,
    };
    loop {
        // SAFETY: `request` is a valid Timespec; a null remainder is allowed
        // with TIMER_ABSTIME because there is nothing to resume from.
        let rc = unsafe {
            clock_nanosleep(
                CLOCK_MONOTONIC,
                TIMER_ABSTIME,
                &request,
                std::ptr::null_mut(),
            )
        };
        match rc {
            0 => return Ok(()),
            // clock_nanosleep returns the error directly rather than via errno.
            4 => continue, // EINTR: a signal arrived; the deadline is unchanged.
            code => {
                return Err(SysError {
                    call: "clock_nanosleep",
                    errno: code,
                })
            }
        }
    }
}

/// Busy-wait for the final stretch before a deadline.
///
/// `clock_nanosleep` typically wakes tens of microseconds late on a general
/// purpose kernel. For the last slice of a 1 ms period that is a large fraction
/// of the budget, so the loop sleeps until `deadline - spin_ns` and spins the
/// remainder. Spinning burns one core; that is an acceptable trade for a rig
/// that exists to be driven, and it is bounded by `spin_ns`.
pub fn sleep_until_precise(deadline_ns: u64, spin_ns: u64) -> Result<(), SysError> {
    let coarse_target = deadline_ns.saturating_sub(spin_ns);
    if monotonic_ns() < coarse_target {
        sleep_until(coarse_target)?;
    }
    while monotonic_ns() < deadline_ns {
        std::hint::spin_loop();
    }
    Ok(())
}

/// Resolution actually delivered by the sleep path, measured rather than assumed.
pub fn measure_sleep_jitter_ns(samples: usize, period_ns: u64) -> Vec<i64> {
    let mut out = Vec::with_capacity(samples);
    let mut deadline = monotonic_ns() + period_ns;
    for _ in 0..samples {
        let _ = sleep_until(deadline);
        out.push(monotonic_ns() as i64 - deadline as i64);
        deadline += period_ns;
    }
    out
}
