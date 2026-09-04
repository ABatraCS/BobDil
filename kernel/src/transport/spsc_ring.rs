//! Lossless single-producer/single-consumer ring for telemetry.
//!
//! Telemetry is the one channel where dropping a sample is wrong: replay,
//! paired A/B, and every fidelity comparison depend on having *every* step. So
//! this queue is lossless below capacity, and when it does overflow it says so
//! -- a silently truncated recording would invalidate a comparison without
//! anyone noticing, which is worse than losing the session.
//!
//! The producer is the real-time step thread, so `push` allocates nothing,
//! takes no lock, and never blocks. The consumer is an ordinary thread that
//! drains to disk.

use std::cell::UnsafeCell;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

/// Why a push was rejected.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PushError {
    /// The consumer is not keeping up. Recorded as a fault, never ignored.
    Full,
}

struct Shared<T> {
    slots: Box<[UnsafeCell<T>]>,
    mask: usize,
    head: AtomicU64, // next slot to write
    tail: AtomicU64, // next slot to read
    overflows: AtomicU64,
}

// SAFETY: access is disjoint by construction -- the producer only touches slots
// in [tail, head) and the consumer only touches [head, tail). The atomics
// publish the boundary between them.
unsafe impl<T: Send> Send for Shared<T> {}
unsafe impl<T: Send> Sync for Shared<T> {}

/// The producing end. Lives on the real-time thread.
pub struct Producer<T> {
    shared: Arc<Shared<T>>,
}

/// The consuming end. Lives on the telemetry thread.
pub struct Consumer<T> {
    shared: Arc<Shared<T>>,
}

/// Build a ring with `capacity` rounded up to a power of two.
///
/// The whole buffer is allocated here, before the loop starts, because
/// allocating during a step is unbounded work.
pub fn channel<T: Copy + Default + Send + 'static>(capacity: usize) -> (Producer<T>, Consumer<T>) {
    let capacity = capacity.next_power_of_two().max(2);
    let mut slots = Vec::with_capacity(capacity);
    slots.resize_with(capacity, || UnsafeCell::new(T::default()));
    let shared = Arc::new(Shared {
        slots: slots.into_boxed_slice(),
        mask: capacity - 1,
        head: AtomicU64::new(0),
        tail: AtomicU64::new(0),
        overflows: AtomicU64::new(0),
    });
    (
        Producer {
            shared: Arc::clone(&shared),
        },
        Consumer { shared },
    )
}

impl<T: Copy> Producer<T> {
    pub fn push(&self, value: T) -> Result<(), PushError> {
        let head = self.shared.head.load(Ordering::Relaxed);
        let tail = self.shared.tail.load(Ordering::Acquire);
        if (head - tail) as usize >= self.shared.slots.len() {
            self.shared.overflows.fetch_add(1, Ordering::Relaxed);
            return Err(PushError::Full);
        }
        let index = (head as usize) & self.shared.mask;
        // SAFETY: the slot is inside the producer's exclusive range because
        // head - tail is below capacity.
        unsafe { *self.shared.slots[index].get() = value };
        self.shared.head.store(head + 1, Ordering::Release);
        Ok(())
    }

    /// Number of pushes rejected for lack of space. Non-zero invalidates a
    /// recording, and is surfaced as `fault_flags::TELEMETRY_OVERFLOW`.
    pub fn overflows(&self) -> u64 {
        self.shared.overflows.load(Ordering::Relaxed)
    }

    pub fn len(&self) -> usize {
        let head = self.shared.head.load(Ordering::Relaxed);
        let tail = self.shared.tail.load(Ordering::Relaxed);
        (head - tail) as usize
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    pub fn capacity(&self) -> usize {
        self.shared.slots.len()
    }
}

impl<T: Copy> Consumer<T> {
    pub fn pop(&self) -> Option<T> {
        let tail = self.shared.tail.load(Ordering::Relaxed);
        let head = self.shared.head.load(Ordering::Acquire);
        if tail == head {
            return None;
        }
        let index = (tail as usize) & self.shared.mask;
        // SAFETY: the slot is inside the consumer's exclusive range because
        // tail != head.
        let value = unsafe { *self.shared.slots[index].get() };
        self.shared.tail.store(tail + 1, Ordering::Release);
        Some(value)
    }

    /// Drain up to `limit` items into `out`, returning how many moved.
    pub fn drain_into(&self, out: &mut Vec<T>, limit: usize) -> usize {
        let mut moved = 0;
        while moved < limit {
            match self.pop() {
                Some(value) => {
                    out.push(value);
                    moved += 1;
                }
                None => break,
            }
        }
        moved
    }

    pub fn overflows(&self) -> u64 {
        self.shared.overflows.load(Ordering::Relaxed)
    }

    pub fn len(&self) -> usize {
        let head = self.shared.head.load(Ordering::Acquire);
        let tail = self.shared.tail.load(Ordering::Relaxed);
        (head - tail) as usize
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn is_lossless_below_capacity() {
        let (tx, rx) = channel::<u64>(1024);
        for i in 0..1000 {
            tx.push(i).expect("below capacity");
        }
        let mut out = Vec::new();
        rx.drain_into(&mut out, 10_000);
        assert_eq!(out, (0..1000).collect::<Vec<_>>());
        assert_eq!(tx.overflows(), 0);
    }

    #[test]
    fn overflow_is_reported_never_silent() {
        let (tx, _rx) = channel::<u64>(4);
        for i in 0..4 {
            tx.push(i).unwrap();
        }
        assert_eq!(tx.push(99), Err(PushError::Full));
        assert_eq!(tx.push(100), Err(PushError::Full));
        assert_eq!(tx.overflows(), 2, "every rejected push must be counted");
    }

    #[test]
    fn survives_a_producer_and_consumer_racing() {
        const COUNT: u64 = 200_000;
        let (tx, rx) = channel::<u64>(4096);
        let producer = std::thread::spawn(move || {
            let mut sent = 0;
            while sent < COUNT {
                if tx.push(sent).is_ok() {
                    sent += 1;
                } else {
                    std::hint::spin_loop();
                }
            }
            tx.overflows()
        });

        let mut expected = 0u64;
        while expected < COUNT {
            match rx.pop() {
                // Order must be exact: telemetry that reorders is unusable for replay.
                Some(value) => {
                    assert_eq!(value, expected);
                    expected += 1;
                }
                None => std::hint::spin_loop(),
            }
        }
        producer.join().unwrap();
        assert_eq!(expected, COUNT);
    }
}
