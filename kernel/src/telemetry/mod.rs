//! Recording, and the thread that drains it to disk.
//!
//! Telemetry is the one channel where losing a sample is a defect rather than
//! an optimisation: replay, paired A/B and every fidelity comparison need every
//! step. The step thread pushes into a lossless ring and never touches a file;
//! this thread drains it.

pub mod recorder;

use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;

use crate::generated::frames::VehicleState;
use crate::transport::Consumer;

pub use recorder::{SessionMeta, TelemetryReader, TelemetryRecorder};

#[derive(Debug, Default)]
pub struct TelemetryStats {
    pub frames_written: AtomicU64,
    pub write_errors: AtomicU64,
    /// Non-zero means the recording has holes and cannot be used for a
    /// comparison. Reported, never silently tolerated.
    pub dropped: AtomicU64,
}

/// Drain `consumer` into `path` until `stop` is set, then flush.
pub fn run(
    consumer: Consumer<VehicleState>,
    path: PathBuf,
    meta: SessionMeta,
    stop: Arc<AtomicBool>,
    stats: Arc<TelemetryStats>,
) {
    let mut recorder = match TelemetryRecorder::create(&path, &meta) {
        Ok(recorder) => recorder,
        Err(error) => {
            eprintln!("telemetry: cannot create {}: {error}", path.display());
            stats.write_errors.fetch_add(1, Ordering::Relaxed);
            return;
        }
    };

    let mut batch: Vec<VehicleState> = Vec::with_capacity(4096);
    loop {
        batch.clear();
        let moved = consumer.drain_into(&mut batch, 4096);
        for frame in &batch {
            match recorder.write(frame) {
                Ok(()) => {
                    stats.frames_written.fetch_add(1, Ordering::Relaxed);
                }
                Err(_) => {
                    stats.write_errors.fetch_add(1, Ordering::Relaxed);
                }
            }
        }
        stats.dropped.store(consumer.overflows(), Ordering::Relaxed);

        if moved == 0 {
            if stop.load(Ordering::Relaxed) && consumer.is_empty() {
                break;
            }
            // Nothing to do. Sleeping here is safe: this is not the real-time
            // thread, and waking 200 times a second is far more than enough to
            // keep a 65k-deep ring from filling.
            std::thread::sleep(std::time::Duration::from_millis(5));
        }
    }

    match recorder.finish() {
        Ok(frames) => {
            let dropped = stats.dropped.load(Ordering::Relaxed);
            if dropped > 0 {
                eprintln!(
                    "telemetry: WROTE {frames} frames but DROPPED {dropped}. \
                     This recording has holes and must not be used for a comparison."
                );
            }
        }
        Err(error) => {
            eprintln!("telemetry: flush failed: {error}");
            stats.write_errors.fetch_add(1, Ordering::Relaxed);
        }
    }
}
