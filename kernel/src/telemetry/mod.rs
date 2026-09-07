//! Recording, and the thread that drains it to disk.
//!
//! Telemetry is the one channel where losing a sample is a defect rather than
//! an optimisation: replay, paired A/B and every fidelity comparison need every
//! step. The step thread pushes into a lossless ring and never touches a file;
//! this thread drains it.

pub mod recorder;
pub mod trace;

use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;

use crate::generated::frames::{TraceDevice, TraceStep, VehicleState};
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

#[derive(Debug, Default)]
pub struct TraceStats {
    pub steps_written: AtomicU64,
    pub devices_written: AtomicU64,
    pub write_errors: AtomicU64,
    /// Non-zero means the trace has holes. A trace with holes still says
    /// something useful -- unlike a recording, it is not replayed -- but the
    /// count is reported so a percentile is never quoted off a partial trace.
    pub dropped: AtomicU64,
}

/// Drain both trace rings into one file until `stop` is set, then flush.
///
/// Two consumers rather than one because `spsc_ring` is single-producer and the
/// step thread and the device thread are two threads. Serialising them here is
/// what lets neither of them know the other exists.
pub fn run_trace(
    steps: Consumer<TraceStep>,
    devices: Consumer<TraceDevice>,
    path: PathBuf,
    meta: trace::TraceMeta,
    stop: Arc<AtomicBool>,
    stats: Arc<TraceStats>,
) {
    let mut writer = match trace::TraceWriter::create(&path, &meta) {
        Ok(writer) => writer,
        Err(error) => {
            eprintln!("trace: cannot create {}: {error}", path.display());
            stats.write_errors.fetch_add(1, Ordering::Relaxed);
            return;
        }
    };

    let mut step_batch: Vec<TraceStep> = Vec::with_capacity(4096);
    let mut device_batch: Vec<TraceDevice> = Vec::with_capacity(4096);
    loop {
        step_batch.clear();
        device_batch.clear();
        let moved =
            steps.drain_into(&mut step_batch, 4096) + devices.drain_into(&mut device_batch, 4096);

        for record in &step_batch {
            match writer.write_step(record) {
                Ok(()) => {
                    stats.steps_written.fetch_add(1, Ordering::Relaxed);
                }
                Err(_) => {
                    stats.write_errors.fetch_add(1, Ordering::Relaxed);
                }
            }
        }
        for record in &device_batch {
            match writer.write_device(record) {
                Ok(()) => {
                    stats.devices_written.fetch_add(1, Ordering::Relaxed);
                }
                Err(_) => {
                    stats.write_errors.fetch_add(1, Ordering::Relaxed);
                }
            }
        }
        stats
            .dropped
            .store(steps.overflows() + devices.overflows(), Ordering::Relaxed);

        if moved == 0 {
            if stop.load(Ordering::Relaxed) && steps.is_empty() && devices.is_empty() {
                break;
            }
            std::thread::sleep(std::time::Duration::from_millis(5));
        }
    }

    if let Err(error) = writer.finish() {
        eprintln!("trace: flush failed: {error}");
        stats.write_errors.fetch_add(1, Ordering::Relaxed);
    }
}

#[cfg(test)]
mod trace_thread_tests {
    use super::*;

    #[test]
    fn the_drain_thread_writes_both_producers_and_stops_when_told() {
        let mut path = std::env::temp_dir();
        path.push(format!("bobdil_drain_{}.bdtrace", std::process::id()));

        let (step_tx, step_rx) = crate::transport::spsc_ring::channel::<TraceStep>(64);
        let (device_tx, device_rx) = crate::transport::spsc_ring::channel::<TraceDevice>(64);
        let stop = Arc::new(AtomicBool::new(false));
        let stats = Arc::new(TraceStats::default());

        for index in 0..3 {
            step_tx
                .push(TraceStep {
                    step_index: index,
                    ..Default::default()
                })
                .unwrap();
        }
        device_tx.push(TraceDevice::default()).unwrap();
        stop.store(true, Ordering::Relaxed);

        run_trace(
            step_rx,
            device_rx,
            path.clone(),
            trace::TraceMeta::default(),
            stop,
            stats.clone(),
        );

        assert_eq!(stats.steps_written.load(Ordering::Relaxed), 3);
        assert_eq!(stats.devices_written.load(Ordering::Relaxed), 1);
        assert_eq!(stats.write_errors.load(Ordering::Relaxed), 0);
        assert_eq!(stats.dropped.load(Ordering::Relaxed), 0);

        let size = std::fs::metadata(&path).unwrap().len() as usize;
        std::fs::remove_file(&path).ok();
        let expected = trace::HEADER_SIZE + 3 * (8 + TraceStep::SIZE) + (8 + TraceDevice::SIZE);
        assert_eq!(size, expected);
    }
}
