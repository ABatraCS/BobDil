//! Every byte that crosses a component boundary goes through this module.
//!
//! Two channel types, picked by what the consumer actually needs
//! (architecture.md 1.7). Using one everywhere would be wrong in both
//! directions: a queue on the input path adds latency to a value that only
//! wants to be current, and a latest-value cell on telemetry loses the samples
//! a comparison depends on.
//!
//! | Channel                      | Type      | Why |
//! |------------------------------|-----------|-----|
//! | driver input -> step thread  | seqlock   | wants the newest input, not a backlog |
//! | vehicle state -> view        | seqlock   | a dropped frame is correct behaviour |
//! | ffb torque -> hid thread     | seqlock   | a stale queue is worse than a dropped sample |
//! | telemetry -> disk            | SPSC ring | every sample matters; overflow is a fault |
//!
//! No mutex ever appears on the real-time path.

pub mod seqlock;
pub mod spsc_ring;

pub use seqlock::{SeqlockError, SeqlockReader, SeqlockWriter};
pub use spsc_ring::{channel as ring_channel, Consumer, Producer, PushError};
