//! Core pinning and real-time scheduling.
//!
//! Both of these are best-effort by design. `SCHED_FIFO` needs privileges the
//! app will usually not have on a gaming PC, and refusing to run without it
//! would make BobDil undrivable for most of its users. So the kernel asks,
//! records what it actually got, and reports it: an unprivileged session still
//! runs, it just carries a larger jitter tail, and the driver is told.

use std::os::raw::c_int;

use super::{check, SysError};

const SCHED_FIFO: c_int = 1;
const CPU_SETSIZE_BYTES: usize = 128; // 1024 CPUs, glibc's cpu_set_t.
const MCL_CURRENT: c_int = 1;
const MCL_FUTURE: c_int = 2;
const RLIMIT_MEMLOCK: c_int = 8;

/// Below this, locking future allocations would make ordinary allocations fail
/// rather than making the loop faster. 64 MB is comfortably more than the
/// kernel's whole working set, so a system configured above it can lock
/// everything; a system at the usual 8 MB default cannot.
const MEMLOCK_FUTURE_THRESHOLD: u64 = 64 * 1024 * 1024;

#[repr(C)]
struct SchedParam {
    sched_priority: c_int,
}

#[repr(C)]
struct CpuSet {
    bits: [u8; CPU_SETSIZE_BYTES],
}

#[repr(C)]
#[derive(Clone, Copy, Default)]
struct RLimit {
    soft: u64,
    hard: u64,
}

extern "C" {
    fn sched_setscheduler(pid: c_int, policy: c_int, param: *const SchedParam) -> c_int;
    fn sched_getscheduler(pid: c_int) -> c_int;
    fn sched_setaffinity(pid: c_int, cpusetsize: usize, mask: *const CpuSet) -> c_int;
    fn sched_get_priority_max(policy: c_int) -> c_int;
    fn mlockall(flags: c_int) -> c_int;
    fn sysconf(name: c_int) -> i64;
    fn getrlimit(resource: c_int, limit: *mut RLimit) -> c_int;
    fn setrlimit(resource: c_int, limit: *const RLimit) -> c_int;
}

/// How much of the process image is pinned in RAM.
///
/// This matters more than it looks. A major page fault inside a step is
/// unbounded work -- milliseconds, while a disk is read -- and it is one of the
/// few remaining sources of a multi-millisecond outlier once the thread is
/// pinned and prioritised.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum MemoryLock {
    /// Nothing is locked. Page faults are possible anywhere.
    #[default]
    None,
    /// Everything allocated *before* the lock is pinned. Since the kernel
    /// preallocates all of its hot-path buffers at start-up, this covers the
    /// step path -- it is the realistic best case on a stock desktop.
    Current,
    /// Current and future allocations are pinned. Needs a generous
    /// RLIMIT_MEMLOCK, which stock Linux does not give (the usual default is
    /// 8 MB), so this is normally only reachable in a container started with
    /// `--ulimit memlock=...` or on a tuned machine.
    CurrentAndFuture,
}

impl MemoryLock {
    pub fn describe(&self) -> &'static str {
        match self {
            Self::None => "pageable",
            Self::Current => "mlock(current)",
            Self::CurrentAndFuture => "mlock(current+future)",
        }
    }
}

const SC_NPROCESSORS_ONLN: c_int = 84;

/// What the calling thread actually obtained. Reported to the driver rather
/// than silently assumed.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct RtStatus {
    pub fifo_priority: Option<i32>,
    pub pinned_cpu: Option<usize>,
    pub memory_locked: MemoryLock,
    /// The RLIMIT_MEMLOCK actually in force, so the report can tell an operator
    /// what to change rather than just that something is missing.
    pub memlock_limit_bytes: u64,
}

impl RtStatus {
    /// True when the thread has everything a bounded jitter tail depends on.
    pub fn is_hardened(&self) -> bool {
        self.fifo_priority.is_some()
            && self.pinned_cpu.is_some()
            && self.memory_locked != MemoryLock::None
    }

    /// What an operator would have to do to close the gap. Empty when there is
    /// nothing left to do.
    pub fn advice(&self) -> Vec<String> {
        let mut advice = Vec::new();
        if self.fifo_priority.is_none() {
            advice.push(
                "SCHED_FIFO was refused. Grant it with \
                 `sudo setcap cap_sys_nice=eip <binary>`, or run inside a container started with \
                 `--cap-add=SYS_NICE`."
                    .to_string(),
            );
        }
        match self.memory_locked {
            MemoryLock::CurrentAndFuture => {}
            MemoryLock::Current => advice.push(format!(
                "RLIMIT_MEMLOCK is {:.0} MB, too small to pin future allocations. The hot-path \
                 buffers are preallocated and are pinned, so this costs tail latency on any \
                 allocation made later, not correctness. Raise it with `ulimit -l unlimited` \
                 (needs a limits.conf entry) or `--ulimit memlock=-1` on the container.",
                self.memlock_limit_bytes as f64 / (1024.0 * 1024.0)
            )),
            MemoryLock::None => advice.push(format!(
                "NOTHING is pinned in RAM: RLIMIT_MEMLOCK is {:.0} MB, smaller than this \
                 process's resident set (the telemetry ring alone is tens of MB). A major page \
                 fault inside a step costs milliseconds, so expect outliers. Raise it with \
                 `ulimit -l unlimited` or `--ulimit memlock=-1` on the container.",
                self.memlock_limit_bytes as f64 / (1024.0 * 1024.0)
            )),
        }
        advice
    }

    pub fn describe(&self) -> String {
        let prio = match self.fifo_priority {
            Some(p) => format!("SCHED_FIFO({p})"),
            None => "SCHED_OTHER (unprivileged: expect a longer jitter tail)".to_string(),
        };
        let cpu = match self.pinned_cpu {
            Some(c) => format!("pinned to cpu{c}"),
            None => "unpinned".to_string(),
        };
        format!("{prio}, {cpu}, {}", self.memory_locked.describe())
    }
}

pub fn online_cpus() -> usize {
    // SAFETY: sysconf with a valid name has no preconditions.
    let count = unsafe { sysconf(SC_NPROCESSORS_ONLN) };
    if count > 0 {
        count as usize
    } else {
        1
    }
}

/// Pin the calling thread to one CPU.
///
/// Pinning matters more than priority for jitter: it keeps the step thread's
/// working set in one core's caches and stops the scheduler migrating it
/// mid-step, which costs far more than the migration itself.
pub fn pin_to_cpu(cpu: usize) -> Result<(), SysError> {
    if cpu >= CPU_SETSIZE_BYTES * 8 {
        return Err(SysError {
            call: "sched_setaffinity",
            errno: 22,
        });
    }
    let mut set = CpuSet {
        bits: [0u8; CPU_SETSIZE_BYTES],
    };
    set.bits[cpu / 8] = 1u8 << (cpu % 8);
    // SAFETY: `set` is a correctly sized cpu_set_t; pid 0 means this thread.
    let rc = unsafe { sched_setaffinity(0, CPU_SETSIZE_BYTES, &set) };
    check("sched_setaffinity", rc)
}

/// Request `SCHED_FIFO` at a priority below the maximum.
///
/// Deliberately not the maximum: a runaway thread at the top real-time priority
/// can lock out kernel threads and wedge the machine. Leaving headroom above us
/// means a stuck step thread is still recoverable.
pub fn request_fifo(priority_below_max: c_int) -> Result<i32, SysError> {
    // SAFETY: SCHED_FIFO is a valid policy constant.
    let max = unsafe { sched_get_priority_max(SCHED_FIFO) };
    if max < 0 {
        return Err(SysError {
            call: "sched_get_priority_max",
            errno: super::errno(),
        });
    }
    let priority = (max - priority_below_max).max(1);
    let param = SchedParam {
        sched_priority: priority,
    };
    // SAFETY: `param` is a valid sched_param; pid 0 means this thread.
    let rc = unsafe { sched_setscheduler(0, SCHED_FIFO, &param) };
    check("sched_setscheduler", rc)?;
    Ok(priority)
}

pub fn current_policy_is_fifo() -> bool {
    // SAFETY: pid 0 queries the calling thread.
    unsafe { sched_getscheduler(0) == SCHED_FIFO }
}

/// The RLIMIT_MEMLOCK currently in force, after trying to raise the soft limit
/// to the hard one (which an unprivileged process is allowed to do).
pub fn memlock_limit() -> u64 {
    let mut limit = RLimit::default();
    // SAFETY: `limit` is a valid, exclusively owned rlimit.
    if unsafe { getrlimit(RLIMIT_MEMLOCK, &mut limit) } != 0 {
        return 0;
    }
    if limit.soft < limit.hard {
        let raised = RLimit {
            soft: limit.hard,
            hard: limit.hard,
        };
        // SAFETY: raising the soft limit to the hard limit is always permitted.
        if unsafe { setrlimit(RLIMIT_MEMLOCK, &raised) } == 0 {
            return raised.soft;
        }
    }
    limit.soft
}

/// Pin the process image in RAM, as far as the platform will allow.
///
/// Deliberately *not* unconditionally `MCL_FUTURE`. Stock Linux gives a process
/// an 8 MB memlock allowance, and under `MCL_FUTURE` every subsequent
/// allocation must fit inside it -- so asking for it on a normal desktop does
/// not harden the loop, it makes ordinary allocations fail outright. That is a
/// far worse failure than a page fault, and it is silent until something
/// allocates.
///
/// Call this *after* the hot-path buffers have been allocated, so `MCL_CURRENT`
/// covers them.
pub fn lock_memory() -> (MemoryLock, u64) {
    let limit = memlock_limit();
    if limit >= MEMLOCK_FUTURE_THRESHOLD {
        // SAFETY: mlockall takes only flags.
        if unsafe { mlockall(MCL_CURRENT | MCL_FUTURE) } == 0 {
            return (MemoryLock::CurrentAndFuture, limit);
        }
    }
    // SAFETY: mlockall takes only flags.
    if unsafe { mlockall(MCL_CURRENT) } == 0 {
        (MemoryLock::Current, limit)
    } else {
        (MemoryLock::None, limit)
    }
}

/// Apply the full real-time hardening the platform will allow, and report what
/// was granted. Never fails: a soft-real-time session on a general purpose OS
/// is still useful, and refusing to start would help nobody.
pub fn harden_current_thread(cpu: Option<usize>) -> RtStatus {
    let mut status = RtStatus::default();
    if let Some(cpu) = cpu {
        if pin_to_cpu(cpu).is_ok() {
            status.pinned_cpu = Some(cpu);
        }
    }
    if let Ok(priority) = request_fifo(10) {
        status.fifo_priority = Some(priority);
    }
    let (locked, limit) = lock_memory();
    status.memory_locked = locked;
    status.memlock_limit_bytes = limit;
    status
}

/// Advisory: which CPU the step thread should take.
///
/// The highest-numbered core, because on a hybrid Intel part core 0 carries the
/// interrupt load and the OS is most likely to schedule other work there.
pub fn suggested_rt_cpu() -> Option<usize> {
    let cpus = online_cpus();
    if cpus >= 2 {
        Some(cpus - 1)
    } else {
        None
    }
}
