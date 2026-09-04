//! Wait-free latest-value transport.
//!
//! Three of BobDil's four channels want the *newest* value, not a backlog: the
//! plant wants the driver's current input, the view wants the current state,
//! and the wheel wants the current torque. A queue would be actively wrong for
//! all three -- it converts a dropped sample, which is harmless, into added
//! latency, which is not.
//!
//! A seqlock gives that with no mutex on the real-time path. The writer never
//! blocks and never waits for a reader. A reader that races the writer sees an
//! odd sequence number or a changed one and retries, so it can never observe a
//! half-written frame.
//!
//! The layout is fixed and documented because the Godot view decodes the same
//! bytes from GDScript (architecture.md 1.2):
//!
//! ```text
//!   0..8   magic         u64  identifies a BobDil segment
//!   8..16  layout_hash   u64  schema fingerprint; mismatch means refuse to attach
//!  16..24  payload_size  u64  bytes of payload that follow the header
//!  24..32  sequence      u64  even = stable, odd = write in progress
//!  32..    payload
//! ```

use std::sync::atomic::{fence, AtomicU64, Ordering};

use crate::sys::shm::SharedRegion;
use crate::sys::SysError;

/// "BOBDIL01" little-endian.
pub const SEGMENT_MAGIC: u64 = 0x3130_4c49_4442_4f42;
pub const HEADER_SIZE: usize = 32;

const OFFSET_MAGIC: usize = 0;
const OFFSET_LAYOUT: usize = 8;
const OFFSET_PAYLOAD_SIZE: usize = 16;
const OFFSET_SEQUENCE: usize = 24;

/// A reader refused to attach, or could not obtain a stable frame.
#[derive(Debug)]
pub enum SeqlockError {
    Sys(SysError),
    BadMagic { found: u64 },
    LayoutMismatch { expected: u64, found: u64 },
    SizeMismatch { expected: usize, found: usize },
}

impl std::fmt::Display for SeqlockError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Sys(e) => write!(f, "{e}"),
            Self::BadMagic { found } => {
                write!(f, "not a BobDil segment (magic {found:#018x})")
            }
            Self::LayoutMismatch { expected, found } => write!(
                f,
                "schema mismatch: segment was written by layout {found:#018x}, \
                 this build expects {expected:#018x}. Rebuild both sides from the same schema."
            ),
            Self::SizeMismatch { expected, found } => {
                write!(
                    f,
                    "payload size mismatch: expected {expected}, segment holds {found}"
                )
            }
        }
    }
}

impl std::error::Error for SeqlockError {}

impl From<SysError> for SeqlockError {
    fn from(value: SysError) -> Self {
        Self::Sys(value)
    }
}

fn segment_bytes<T>() -> usize {
    HEADER_SIZE + std::mem::size_of::<T>()
}

/// # Safety
/// `base` must point at a mapped region of at least `HEADER_SIZE` bytes.
unsafe fn header_word(base: *const u8, offset: usize) -> u64 {
    (base.add(offset) as *const u64).read_volatile()
}

/// # Safety
/// `base` must point at a writable mapped region of at least `HEADER_SIZE` bytes.
unsafe fn sequence_cell<'a>(base: *mut u8) -> &'a AtomicU64 {
    &*(base.add(OFFSET_SEQUENCE) as *const AtomicU64)
}

/// The single writer for one segment.
pub struct SeqlockWriter<T: Copy> {
    region: SharedRegion,
    sequence: u64,
    _marker: std::marker::PhantomData<T>,
}

impl<T: Copy> SeqlockWriter<T> {
    /// Create the segment, stamp its header, and publish a zeroed first frame.
    pub fn create(name: &str, layout_hash: u64) -> Result<Self, SeqlockError> {
        let region = SharedRegion::create(name, segment_bytes::<T>())?;
        let base = region.as_ptr();
        // SAFETY: the region was just created with at least HEADER_SIZE bytes
        // and is exclusively owned by this writer until it is published.
        unsafe {
            (base.add(OFFSET_LAYOUT) as *mut u64).write_volatile(layout_hash);
            (base.add(OFFSET_PAYLOAD_SIZE) as *mut u64)
                .write_volatile(std::mem::size_of::<T>() as u64);
            sequence_cell(base).store(0, Ordering::Release);
            // Magic last: a reader that sees the magic is guaranteed to see a
            // fully stamped header behind it.
            fence(Ordering::Release);
            (base.add(OFFSET_MAGIC) as *mut u64).write_volatile(SEGMENT_MAGIC);
        }
        Ok(Self {
            region,
            sequence: 0,
            _marker: std::marker::PhantomData,
        })
    }

    /// Publish a frame. Wait-free: no allocation, no syscall, no blocking.
    pub fn publish(&mut self, value: &T) {
        let base = self.region.as_ptr();
        // SAFETY: the region is sized for a header plus one T, and this writer
        // is the only one permitted to mutate it.
        unsafe {
            let seq = sequence_cell(base);
            self.sequence = self.sequence.wrapping_add(1); // odd: write in progress
            seq.store(self.sequence, Ordering::Release);
            fence(Ordering::Release);

            let payload = base.add(HEADER_SIZE) as *mut T;
            payload.write_volatile(*value);

            fence(Ordering::Release);
            self.sequence = self.sequence.wrapping_add(1); // even: stable again
            seq.store(self.sequence, Ordering::Release);
        }
    }

    pub fn name(&self) -> &str {
        self.region.name()
    }

    pub fn sequence(&self) -> u64 {
        self.sequence
    }
}

/// One of possibly many readers of a segment.
pub struct SeqlockReader<T: Copy> {
    region: SharedRegion,
    _marker: std::marker::PhantomData<T>,
}

impl<T: Copy> SeqlockReader<T> {
    /// Attach read-only, verifying the header before trusting any byte of it.
    pub fn attach(name: &str, layout_hash: u64) -> Result<Self, SeqlockError> {
        let region = SharedRegion::attach(name, segment_bytes::<T>(), false)?;
        let base = region.as_ptr();
        // SAFETY: the mapping is at least HEADER_SIZE bytes by construction.
        let (magic, found_layout, payload_size) = unsafe {
            (
                header_word(base, OFFSET_MAGIC),
                header_word(base, OFFSET_LAYOUT),
                header_word(base, OFFSET_PAYLOAD_SIZE) as usize,
            )
        };
        if magic != SEGMENT_MAGIC {
            return Err(SeqlockError::BadMagic { found: magic });
        }
        if found_layout != layout_hash {
            return Err(SeqlockError::LayoutMismatch {
                expected: layout_hash,
                found: found_layout,
            });
        }
        if payload_size != std::mem::size_of::<T>() {
            return Err(SeqlockError::SizeMismatch {
                expected: std::mem::size_of::<T>(),
                found: payload_size,
            });
        }
        Ok(Self {
            region,
            _marker: std::marker::PhantomData,
        })
    }

    /// Read the newest stable frame, retrying while the writer is mid-update.
    ///
    /// Returns `None` after `max_attempts` -- a reader must never spin forever
    /// on a writer that has died mid-frame.
    pub fn read(&self, max_attempts: u32) -> Option<(T, u64)> {
        let base = self.region.as_ptr();
        for _ in 0..max_attempts {
            // SAFETY: the mapping outlives this borrow and is correctly sized.
            unsafe {
                let seq = sequence_cell(base);
                let before = seq.load(Ordering::Acquire);
                if before & 1 != 0 {
                    std::hint::spin_loop();
                    continue;
                }
                fence(Ordering::Acquire);
                let value = (base.add(HEADER_SIZE) as *const T).read_volatile();
                fence(Ordering::Acquire);
                if seq.load(Ordering::Acquire) == before {
                    return Some((value, before));
                }
            }
        }
        None
    }

    /// Read with a retry budget generous enough for any real writer, but still
    /// bounded. Callers on the hot path use this.
    pub fn read_latest(&self) -> Option<(T, u64)> {
        self.read(64)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicBool, Ordering as O};
    use std::sync::Arc;

    #[repr(C)]
    #[derive(Clone, Copy, Debug, PartialEq, Default)]
    struct Frame {
        a: u64,
        b: u64,
        c: u64,
        d: u64,
    }

    fn unique(name: &str) -> String {
        format!("bobdil_test_{name}_{}", std::process::id())
    }

    #[test]
    fn publishes_and_reads_back() {
        let name = unique("roundtrip");
        let mut writer = SeqlockWriter::<Frame>::create(&name, 0xABCD).unwrap();
        let reader = SeqlockReader::<Frame>::attach(&name, 0xABCD).unwrap();
        writer.publish(&Frame {
            a: 1,
            b: 2,
            c: 3,
            d: 4,
        });
        let (frame, seq) = reader.read_latest().unwrap();
        assert_eq!(
            frame,
            Frame {
                a: 1,
                b: 2,
                c: 3,
                d: 4
            }
        );
        assert_eq!(seq, 2);
    }

    #[test]
    fn refuses_a_segment_written_by_a_different_schema() {
        let name = unique("layout");
        let _writer = SeqlockWriter::<Frame>::create(&name, 0x1111).unwrap();
        match SeqlockReader::<Frame>::attach(&name, 0x2222) {
            Err(err) => assert!(matches!(err, SeqlockError::LayoutMismatch { .. }), "{err}"),
            Ok(_) => panic!("a reader must refuse a segment written by a different schema"),
        }
    }

    /// The property that matters: under a writer hammering the segment, a
    /// reader never observes a frame that mixes two writes. Every field of a
    /// published frame carries the same counter, so any torn read is visible.
    #[test]
    fn reader_never_observes_a_torn_frame() {
        let name = unique("tearing");
        let mut writer = SeqlockWriter::<Frame>::create(&name, 7).unwrap();
        let reader = SeqlockReader::<Frame>::attach(&name, 7).unwrap();
        let stop = Arc::new(AtomicBool::new(false));

        let writer_stop = Arc::clone(&stop);
        let handle = std::thread::spawn(move || {
            let mut counter: u64 = 0;
            while !writer_stop.load(O::Relaxed) {
                counter = counter.wrapping_add(1);
                writer.publish(&Frame {
                    a: counter,
                    b: counter,
                    c: counter,
                    d: counter,
                });
            }
            counter
        });

        let mut observed = 0u64;
        let mut torn = 0u64;
        for _ in 0..200_000 {
            if let Some((frame, _)) = reader.read_latest() {
                observed += 1;
                if frame.a != frame.b || frame.b != frame.c || frame.c != frame.d {
                    torn += 1;
                }
            }
        }
        stop.store(true, O::Relaxed);
        let written = handle.join().unwrap();

        assert!(written > 1000, "writer barely ran ({written} frames)");
        assert!(observed > 1000, "reader barely ran ({observed} frames)");
        assert_eq!(torn, 0, "observed {torn} torn frames out of {observed}");
    }

    #[test]
    fn readers_see_the_newest_frame_not_a_backlog() {
        let name = unique("latest");
        let mut writer = SeqlockWriter::<Frame>::create(&name, 3).unwrap();
        let reader = SeqlockReader::<Frame>::attach(&name, 3).unwrap();
        for i in 0..100 {
            writer.publish(&Frame {
                a: i,
                b: i,
                c: i,
                d: i,
            });
        }
        let (frame, _) = reader.read_latest().unwrap();
        assert_eq!(
            frame.a, 99,
            "a seqlock must yield the newest value, never a queue"
        );
    }
}
