//! Telemetry recording, and the metadata that makes a recording reproducible.
//!
//! A recording is only useful if it can be replayed *exactly*, so the header
//! carries everything that determines the result: the schema layout, the step
//! size, the integrator, the kernel that produced it, and a hash of the vehicle
//! it was driving. Replay checks all of them before it starts. A recording
//! whose provenance cannot be established is worse than no recording, because
//! it will be compared against something it was never comparable with.
//!
//! The file is packed `VehicleState` frames behind a fixed header. Every field
//! a driver's input contained is echoed in the state frame, so this one file is
//! both the telemetry and the input script that reproduces it.

use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::Path;

use crate::generated::frames::{VehicleState, LAYOUT_HASH, LAYOUT_REVISION};

/// "BDTEL001" little-endian.
pub const TELEMETRY_MAGIC: u64 = 0x3130_304c_4554_4442;
pub const HEADER_SIZE: usize = 128;

/// Everything that determines whether two recordings are comparable.
#[derive(Debug, Clone)]
pub struct SessionMeta {
    pub step_dt: f64,
    /// `generated::frames::kernel_id`.
    pub kernel_id: u64,
    /// Number of derivative evaluations per step, so a timing comparison can be
    /// normalised by the work actually done.
    pub evaluations_per_step: u64,
    /// Hash of the vehicle definition the plant was built from. Two recordings
    /// with different values here are different cars, and diffing them says
    /// nothing about a setup change.
    pub vehicle_hash: u64,
    /// Hash of the kernel build, so a recording can be traced to its code.
    pub build_hash: u64,
}

impl Default for SessionMeta {
    fn default() -> Self {
        Self {
            step_dt: 1e-3,
            kernel_id: 0,
            evaluations_per_step: 4,
            vehicle_hash: 0,
            build_hash: 0,
        }
    }
}

/// Streams frames to disk. Lives on the telemetry thread, never the step thread.
pub struct TelemetryRecorder {
    writer: BufWriter<File>,
    frames: u64,
}

impl TelemetryRecorder {
    pub fn create(path: &Path, meta: &SessionMeta) -> std::io::Result<Self> {
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent)?;
        }
        let file = File::create(path)?;
        // 1 MB buffer: at 264 bytes a frame and 1 kHz that is roughly four
        // seconds between writes, so a momentarily busy disk cannot stall the
        // drain and back the ring up.
        let mut writer = BufWriter::with_capacity(1 << 20, file);

        let mut header = [0u8; HEADER_SIZE];
        let mut cursor = 0usize;
        let mut put = |value: u64, cursor: &mut usize| {
            header[*cursor..*cursor + 8].copy_from_slice(&value.to_le_bytes());
            *cursor += 8;
        };
        put(TELEMETRY_MAGIC, &mut cursor);
        put(LAYOUT_HASH, &mut cursor);
        put(LAYOUT_REVISION, &mut cursor);
        put(VehicleState::SIZE as u64, &mut cursor);
        put(VehicleState::FIELD_COUNT as u64, &mut cursor);
        put(meta.step_dt.to_bits(), &mut cursor);
        put(meta.kernel_id, &mut cursor);
        put(meta.evaluations_per_step, &mut cursor);
        put(meta.vehicle_hash, &mut cursor);
        put(meta.build_hash, &mut cursor);
        writer.write_all(&header)?;

        Ok(Self { writer, frames: 0 })
    }

    pub fn write(&mut self, state: &VehicleState) -> std::io::Result<()> {
        // SAFETY: `VehicleState` is `#[repr(C)]` and contains only 8-byte
        // scalars with no padding, so its bytes are exactly the wire format the
        // schema defines and there is no uninitialised memory to leak.
        let bytes = unsafe {
            std::slice::from_raw_parts(
                (state as *const VehicleState) as *const u8,
                std::mem::size_of::<VehicleState>(),
            )
        };
        self.writer.write_all(bytes)?;
        self.frames += 1;
        Ok(())
    }

    pub fn frames(&self) -> u64 {
        self.frames
    }

    pub fn finish(mut self) -> std::io::Result<u64> {
        self.writer.flush()?;
        Ok(self.frames)
    }
}

/// Read a recording back. Used by replay and by the offline comparison tools.
pub struct TelemetryReader {
    pub meta: SessionMeta,
    pub frames: Vec<VehicleState>,
}

impl TelemetryReader {
    pub fn open(path: &Path) -> std::io::Result<Self> {
        let bytes = std::fs::read(path)?;
        if bytes.len() < HEADER_SIZE {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                "file is shorter than a telemetry header",
            ));
        }
        let word = |index: usize| -> u64 {
            let mut buffer = [0u8; 8];
            buffer.copy_from_slice(&bytes[index * 8..index * 8 + 8]);
            u64::from_le_bytes(buffer)
        };
        if word(0) != TELEMETRY_MAGIC {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                "not a BobDil telemetry file",
            ));
        }
        if word(1) != LAYOUT_HASH {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                format!(
                    "recording was written by schema layout {:#018x}, this build expects {:#018x}",
                    word(1),
                    LAYOUT_HASH
                ),
            ));
        }
        let meta = SessionMeta {
            step_dt: f64::from_bits(word(5)),
            kernel_id: word(6),
            evaluations_per_step: word(7),
            vehicle_hash: word(8),
            build_hash: word(9),
        };

        let payload = &bytes[HEADER_SIZE..];
        let stride = std::mem::size_of::<VehicleState>();
        let mut frames = Vec::with_capacity(payload.len() / stride);
        for chunk in payload.chunks_exact(stride) {
            let mut state = VehicleState::default();
            // SAFETY: `chunk` is exactly `stride` bytes and `VehicleState` is
            // `#[repr(C)]` with no padding, so the copy is well defined.
            unsafe {
                std::ptr::copy_nonoverlapping(
                    chunk.as_ptr(),
                    (&mut state as *mut VehicleState) as *mut u8,
                    stride,
                );
            }
            frames.push(state);
        }
        Ok(Self { meta, frames })
    }

    /// Write the recording out as CSV, for anything that would rather not read
    /// a binary format. Column names come from the schema, so they cannot drift.
    pub fn write_csv(&self, path: &Path) -> std::io::Result<()> {
        let file = File::create(path)?;
        let mut writer = BufWriter::new(file);
        writeln!(writer, "{}", VehicleState::FIELD_NAMES.join(","))?;
        for frame in &self.frames {
            let row: Vec<String> = (0..VehicleState::FIELD_COUNT)
                .map(|index| format!("{:.9}", frame.field(index)))
                .collect();
            writeln!(writer, "{}", row.join(","))?;
        }
        writer.flush()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scratch(name: &str) -> std::path::PathBuf {
        std::env::temp_dir().join(format!("bobdil_tel_{name}_{}.bdt", std::process::id()))
    }

    #[test]
    fn a_recording_round_trips_exactly() {
        let path = scratch("roundtrip");
        let meta = SessionMeta {
            step_dt: 1e-3,
            kernel_id: 1,
            vehicle_hash: 0xDEAD,
            ..Default::default()
        };
        let mut recorder = TelemetryRecorder::create(&path, &meta).unwrap();
        for step in 0..500u64 {
            recorder
                .write(&VehicleState {
                    step_index: step,
                    sim_time: step as f64 * 1e-3,
                    vehicle_speed: step as f64 * 0.1,
                    handwheel_torque: -(step as f64) * 0.01,
                    ..Default::default()
                })
                .unwrap();
        }
        assert_eq!(recorder.finish().unwrap(), 500);

        let read = TelemetryReader::open(&path).unwrap();
        assert_eq!(read.frames.len(), 500);
        assert_eq!(read.meta.vehicle_hash, 0xDEAD);
        assert_eq!(read.meta.step_dt, 1e-3);
        // Bit-exact, not approximately: replay determinism depends on it.
        assert_eq!(read.frames[499].vehicle_speed, 499.0 * 0.1);
        assert_eq!(read.frames[123].step_index, 123);
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn a_foreign_file_is_refused_rather_than_misread() {
        let path = scratch("foreign");
        std::fs::write(&path, vec![0u8; 512]).unwrap();
        assert!(TelemetryReader::open(&path).is_err());
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn csv_columns_come_from_the_schema() {
        let path = scratch("csv");
        let csv_path = path.with_extension("csv");
        let mut recorder = TelemetryRecorder::create(&path, &SessionMeta::default()).unwrap();
        recorder
            .write(&VehicleState {
                vehicle_speed: 12.5,
                ..Default::default()
            })
            .unwrap();
        recorder.finish().unwrap();

        TelemetryReader::open(&path)
            .unwrap()
            .write_csv(&csv_path)
            .unwrap();
        let text = std::fs::read_to_string(&csv_path).unwrap();
        let header = text.lines().next().unwrap();
        assert!(header.starts_with("sim_time,step_index,host_time_ns"));
        assert!(header.contains("handwheel_torque"));
        let _ = std::fs::remove_file(&path);
        let _ = std::fs::remove_file(&csv_path);
    }
}
