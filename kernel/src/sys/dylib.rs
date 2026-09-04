//! `dlopen`/`dlsym`, for loading an FMU's platform binary at run time.
//!
//! An FMI 2.0 FMU ships a shared library whose path is only known once a
//! vehicle has been built, so it cannot be linked. This is the whole of the
//! dynamic-loading surface the kernel needs.

use std::ffi::CString;
use std::os::raw::{c_char, c_int, c_void};
use std::path::Path;

const RTLD_NOW: c_int = 2;
const RTLD_LOCAL: c_int = 0;

extern "C" {
    fn dlopen(filename: *const c_char, flags: c_int) -> *mut c_void;
    fn dlsym(handle: *mut c_void, symbol: *const c_char) -> *mut c_void;
    fn dlclose(handle: *mut c_void) -> c_int;
    fn dlerror() -> *mut c_char;
}

#[derive(Debug)]
pub struct DylibError {
    pub detail: String,
}

impl std::fmt::Display for DylibError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.detail)
    }
}

impl std::error::Error for DylibError {}

/// A loaded shared library. Symbols resolved from it borrow this handle, so the
/// library cannot be unloaded while the plant still holds function pointers
/// into it -- a use-after-dlclose is otherwise silent and instantly fatal.
pub struct Dylib {
    handle: *mut c_void,
    path: String,
}

// SAFETY: a dlopen handle is process-wide and remains valid across threads.
unsafe impl Send for Dylib {}
unsafe impl Sync for Dylib {}

impl Dylib {
    pub fn open(path: &Path) -> Result<Self, DylibError> {
        let text = path.to_string_lossy().to_string();
        let c_path = CString::new(text.clone()).map_err(|_| DylibError {
            detail: format!("path is not a C string: {text}"),
        })?;
        // SAFETY: `c_path` is a valid NUL-terminated path. RTLD_NOW surfaces
        // missing symbols here rather than at the first call, which on the hot
        // path would be a crash mid-drive.
        let handle = unsafe { dlopen(c_path.as_ptr(), RTLD_NOW | RTLD_LOCAL) };
        if handle.is_null() {
            return Err(DylibError {
                detail: format!("dlopen({text}): {}", last_error()),
            });
        }
        Ok(Self { handle, path: text })
    }

    /// Resolve a symbol.
    ///
    /// # Safety
    /// The caller asserts that `T` is the exact ABI type of `name` in this
    /// library. Getting that wrong is undefined behaviour, so every use sits in
    /// one place: the FMI binding table in `plant::fmi2`.
    pub unsafe fn symbol<T: Copy>(&self, name: &str) -> Result<T, DylibError> {
        assert_eq!(
            std::mem::size_of::<T>(),
            std::mem::size_of::<*mut c_void>(),
            "symbol type must be pointer-sized",
        );
        let c_name = CString::new(name).map_err(|_| DylibError {
            detail: format!("bad symbol name: {name}"),
        })?;
        let raw = dlsym(self.handle, c_name.as_ptr());
        if raw.is_null() {
            return Err(DylibError {
                detail: format!("dlsym({name}) in {}: {}", self.path, last_error()),
            });
        }
        Ok(std::mem::transmute_copy::<*mut c_void, T>(&raw))
    }

    pub fn path(&self) -> &str {
        &self.path
    }
}

impl Drop for Dylib {
    fn drop(&mut self) {
        // SAFETY: the handle came from dlopen and is dropped exactly once.
        unsafe { dlclose(self.handle) };
    }
}

fn last_error() -> String {
    // SAFETY: dlerror returns either NULL or a valid NUL-terminated string
    // owned by the loader.
    unsafe {
        let raw = dlerror();
        if raw.is_null() {
            "unknown error".to_string()
        } else {
            std::ffi::CStr::from_ptr(raw).to_string_lossy().to_string()
        }
    }
}
