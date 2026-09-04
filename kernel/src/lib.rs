//! BobDil soft-real-time plant kernel.
//!
//! One process, three threads, one job: read what the driver is doing, advance
//! the vehicle model by exactly one fixed step, and give the driver back a
//! force derived from that model -- a thousand times a second, without ever
//! blocking.
//!
//! The layering is deliberate and one-directional:
//!
//! ```text
//!   loop_runner   owns the clock, wires the threads together
//!     |- plant/   physics. Knows nothing about wheels, screens or drivers.
//!     |- io/      devices. Knows nothing about tires.
//!     |- transport/ buffers. Knows nothing about either.
//!     `- sys/     the platform calls the above are built from.
//! ```
//!
//! Nothing in `plant/` may reference `io/`, and nothing in `io/` may reference
//! `plant/`. That is what makes the physics kernel swappable and the device
//! layer testable without a wheel plugged in.

pub mod generated;
pub mod metrics;
pub mod plant;
pub mod pose;
pub mod sys;
pub mod transport;

/// Semantic version of the kernel binary, recorded in every telemetry file so a
/// recording can always be traced back to the code that produced it.
pub const VERSION: &str = env!("CARGO_PKG_VERSION");
