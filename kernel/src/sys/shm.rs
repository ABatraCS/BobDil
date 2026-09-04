//! POSIX shared memory segments.
//!
//! On Linux these are files under `/dev/shm`, which is deliberately convenient:
//! the Godot view opens the state segment with an ordinary file read and needs
//! no native extension to see physics (architecture.md 1.2). The view maps it
//! read-only, so the "view can never write to physics" rule is enforced by the
//! OS rather than by convention.

use std::ffi::CString;
use std::os::raw::{c_char, c_int, c_void};

use super::{errno, SysError};

const O_RDWR: c_int = 0o2;
const O_CREAT: c_int = 0o100;
const O_RDONLY: c_int = 0;
const PROT_READ: c_int = 1;
const PROT_WRITE: c_int = 2;
const MAP_SHARED: c_int = 1;
const MAP_FAILED: *mut c_void = usize::MAX as *mut c_void;

extern "C" {
    fn shm_open(name: *const c_char, oflag: c_int, mode: u32) -> c_int;
    fn shm_unlink(name: *const c_char) -> c_int;
    fn ftruncate(fd: c_int, length: i64) -> c_int;
    fn close(fd: c_int) -> c_int;
    fn mmap(
        addr: *mut c_void,
        length: usize,
        prot: c_int,
        flags: c_int,
        fd: c_int,
        offset: i64,
    ) -> *mut c_void;
    fn munmap(addr: *mut c_void, length: usize) -> c_int;
}

/// A mapped shared-memory region, unmapped on drop.
///
/// `owner` distinguishes the process that created the segment (and must unlink
/// it) from the ones that merely attached, so a crashed view does not delete
/// the segment the kernel is still publishing into.
pub struct SharedRegion {
    ptr: *mut u8,
    len: usize,
    name: String,
    owner: bool,
}

// SAFETY: the region is a raw mapping with no interior Rust state. Concurrent
// access is mediated by the seqlock built on top of it, which is what actually
// provides the synchronisation.
unsafe impl Send for SharedRegion {}
unsafe impl Sync for SharedRegion {}

impl SharedRegion {
    /// Create (or re-create) a segment and zero it. Called by the kernel only.
    pub fn create(name: &str, len: usize) -> Result<Self, SysError> {
        let c_name = shm_path(name)?;
        // Unlink any stale segment first: a previous run that was SIGKILLed
        // leaves one behind, and silently attaching to it would resurrect its
        // contents -- including a non-zero force-feedback command.
        // SAFETY: `c_name` is a valid NUL-terminated string.
        unsafe { shm_unlink(c_name.as_ptr()) };

        // SAFETY: `c_name` is valid for the duration of the call.
        let fd = unsafe { shm_open(c_name.as_ptr(), O_RDWR | O_CREAT, 0o600) };
        if fd < 0 {
            return Err(SysError {
                call: "shm_open",
                errno: errno(),
            });
        }
        // SAFETY: `fd` is an open shared-memory descriptor.
        if unsafe { ftruncate(fd, len as i64) } != 0 {
            let e = errno();
            // SAFETY: closing a descriptor we own.
            unsafe { close(fd) };
            return Err(SysError {
                call: "ftruncate",
                errno: e,
            });
        }
        let region = Self::map(fd, name, len, true, true)?;
        // SAFETY: the mapping keeps the pages alive; the descriptor is no
        // longer needed once mmap has succeeded.
        unsafe { close(fd) };
        // SAFETY: the region owns `len` bytes it just created.
        unsafe { std::ptr::write_bytes(region.ptr, 0, len) };
        Ok(region)
    }

    /// Attach to an existing segment.
    ///
    /// `writable` is false for every consumer. The view and any external tool
    /// get a read-only mapping, so a bug there faults rather than corrupting
    /// the state the driver is feeling.
    pub fn attach(name: &str, len: usize, writable: bool) -> Result<Self, SysError> {
        let c_name = shm_path(name)?;
        let flags = if writable { O_RDWR } else { O_RDONLY };
        // SAFETY: `c_name` is a valid NUL-terminated string.
        let fd = unsafe { shm_open(c_name.as_ptr(), flags, 0o600) };
        if fd < 0 {
            return Err(SysError {
                call: "shm_open",
                errno: errno(),
            });
        }
        let region = Self::map(fd, name, len, writable, false)?;
        // SAFETY: closing a descriptor we own after a successful mmap.
        unsafe { close(fd) };
        Ok(region)
    }

    fn map(
        fd: c_int,
        name: &str,
        len: usize,
        writable: bool,
        owner: bool,
    ) -> Result<Self, SysError> {
        let prot = if writable {
            PROT_READ | PROT_WRITE
        } else {
            PROT_READ
        };
        // SAFETY: `fd` refers to a segment of at least `len` bytes.
        let ptr = unsafe { mmap(std::ptr::null_mut(), len, prot, MAP_SHARED, fd, 0) };
        if ptr == MAP_FAILED {
            let e = errno();
            // SAFETY: closing a descriptor we own.
            unsafe { close(fd) };
            return Err(SysError {
                call: "mmap",
                errno: e,
            });
        }
        Ok(Self {
            ptr: ptr as *mut u8,
            len,
            name: name.to_string(),
            owner,
        })
    }

    pub fn as_ptr(&self) -> *mut u8 {
        self.ptr
    }

    pub fn len(&self) -> usize {
        self.len
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    pub fn name(&self) -> &str {
        &self.name
    }
}

impl Drop for SharedRegion {
    fn drop(&mut self) {
        // SAFETY: unmapping the region this value exclusively owns.
        unsafe { munmap(self.ptr as *mut c_void, self.len) };
        if self.owner {
            if let Ok(c_name) = shm_path(&self.name) {
                // SAFETY: `c_name` is valid for the call.
                unsafe { shm_unlink(c_name.as_ptr()) };
            }
        }
    }
}

fn shm_path(name: &str) -> Result<CString, SysError> {
    let leading = if name.starts_with('/') { "" } else { "/" };
    CString::new(format!("{leading}{name}")).map_err(|_| SysError {
        call: "shm_open",
        errno: 22,
    })
}

/// Remove a segment left behind by a previous run.
pub fn unlink(name: &str) {
    if let Ok(c_name) = shm_path(name) {
        // SAFETY: `c_name` is valid for the call.
        unsafe { shm_unlink(c_name.as_ptr()) };
    }
}
