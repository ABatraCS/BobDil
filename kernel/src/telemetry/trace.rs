//! The round-trip span trace: what it writes, and what refuses to read it.
//!
//! `metrics.rs` answers "how long did the step take". This answers the question
//! architecture.md 5.6 actually poses -- where the time goes between the
//! driver's hand and the driver's hand -- because a step that costs 200 us
//! inside a loop handing the wheel a 5 ms old torque is a rig that feels
//! rubber-banded while every published number looks healthy.
//!
//! Two producers write here (the step thread and the device thread) and their
//! records interleave in time, so the file is a stream of tagged records rather
//! than two arrays. The tag is what lets one drain thread serialise both
//! without either producer knowing the other exists.

use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::Path;

use crate::generated::frames::{TraceDevice, TraceStep, LAYOUT_HASH, LAYOUT_REVISION};

/// "BDTRC001" little-endian.
pub const TRACE_MAGIC: u64 = 0x3130_3043_5254_4442;
pub const HEADER_SIZE: usize = 128;

/// Record tags. Written as a `u64` so every record stays 8-byte aligned and the
/// Python reader can decode with the same `struct` formats codegen emitted.
pub const KIND_STEP: u64 = 1;
pub const KIND_DEVICE: u64 = 2;

/// What a trace is of. Enough to tell two traces apart, and no more: a trace is
/// a measurement of *this machine* and is never comparable across boxes, so it
/// deliberately does not carry the provenance a recording does.
#[derive(Debug, Clone, Default)]
pub struct TraceMeta {
    pub step_dt: f64,
    pub kernel_id: u64,
    pub build_hash: u64,
}

/// Streams span records to disk. Lives on the drain thread, never on the step
/// thread or the device thread.
pub struct TraceWriter {
    writer: BufWriter<File>,
    steps: u64,
    devices: u64,
}

impl TraceWriter {
    pub fn create(path: &Path, meta: &TraceMeta) -> std::io::Result<Self> {
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent)?;
        }
        let file = File::create(path)?;
        // 1 MB, matching the recorder: at ~120 KB/s of trace that is seconds of
        // slack, so a briefly busy disk cannot stall the drain and back up a
        // ring that a real-time thread is pushing into.
        let mut writer = BufWriter::with_capacity(1 << 20, file);

        let mut header = [0u8; HEADER_SIZE];
        let mut cursor = 0usize;
        let mut put = |value: u64, cursor: &mut usize| {
            header[*cursor..*cursor + 8].copy_from_slice(&value.to_le_bytes());
            *cursor += 8;
        };
        put(TRACE_MAGIC, &mut cursor);
        put(LAYOUT_HASH, &mut cursor);
        put(LAYOUT_REVISION, &mut cursor);
        put(TraceStep::SIZE as u64, &mut cursor);
        put(TraceDevice::SIZE as u64, &mut cursor);
        put(meta.step_dt.to_bits(), &mut cursor);
        put(meta.kernel_id, &mut cursor);
        put(meta.build_hash, &mut cursor);
        writer.write_all(&header)?;

        Ok(Self {
            writer,
            steps: 0,
            devices: 0,
        })
    }

    pub fn write_step(&mut self, record: &TraceStep) -> std::io::Result<()> {
        self.writer.write_all(&KIND_STEP.to_le_bytes())?;
        // SAFETY: `TraceStep` is `#[repr(C)]` and is nothing but 8-byte scalars
        // with no padding, so its bytes are exactly the wire format the schema
        // defines and there is no uninitialised memory to leak.
        let bytes = unsafe {
            std::slice::from_raw_parts(
                (record as *const TraceStep) as *const u8,
                std::mem::size_of::<TraceStep>(),
            )
        };
        self.writer.write_all(bytes)?;
        self.steps += 1;
        Ok(())
    }

    pub fn write_device(&mut self, record: &TraceDevice) -> std::io::Result<()> {
        self.writer.write_all(&KIND_DEVICE.to_le_bytes())?;
        // SAFETY: as above.
        let bytes = unsafe {
            std::slice::from_raw_parts(
                (record as *const TraceDevice) as *const u8,
                std::mem::size_of::<TraceDevice>(),
            )
        };
        self.writer.write_all(bytes)?;
        self.devices += 1;
        Ok(())
    }

    pub fn counts(&self) -> (u64, u64) {
        (self.steps, self.devices)
    }

    pub fn finish(mut self) -> std::io::Result<(u64, u64)> {
        self.writer.flush()?;
        Ok((self.steps, self.devices))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn temp_path(name: &str) -> std::path::PathBuf {
        let mut path = std::env::temp_dir();
        path.push(format!(
            "bobdil_trace_{}_{}.bdtrace",
            name,
            std::process::id()
        ));
        path
    }

    #[test]
    fn a_written_trace_starts_with_its_magic_and_this_builds_layout() {
        let path = temp_path("header");
        let writer = TraceWriter::create(&path, &TraceMeta::default()).unwrap();
        writer.finish().unwrap();

        let bytes = std::fs::read(&path).unwrap();
        std::fs::remove_file(&path).ok();

        assert_eq!(
            bytes.len(),
            HEADER_SIZE,
            "a trace with no records is a header"
        );
        let word = |i: usize| u64::from_le_bytes(bytes[i * 8..i * 8 + 8].try_into().unwrap());
        assert_eq!(word(0), TRACE_MAGIC);
        assert_eq!(word(1), LAYOUT_HASH);
        assert_eq!(word(2), LAYOUT_REVISION);
    }

    #[test]
    fn each_record_is_tagged_so_two_producers_can_interleave() {
        let path = temp_path("interleave");
        let mut writer = TraceWriter::create(&path, &TraceMeta::default()).unwrap();
        writer
            .write_step(&TraceStep {
                step_index: 7,
                ..Default::default()
            })
            .unwrap();
        writer
            .write_device(&TraceDevice {
                sample_index: 9,
                ..Default::default()
            })
            .unwrap();
        let (steps, devices) = writer.finish().unwrap();

        assert_eq!((steps, devices), (1, 1));

        let bytes = std::fs::read(&path).unwrap();
        std::fs::remove_file(&path).ok();

        let word = |i: usize| u64::from_le_bytes(bytes[i * 8..i * 8 + 8].try_into().unwrap());
        let step_tag = HEADER_SIZE / 8;
        assert_eq!(word(step_tag), KIND_STEP);
        assert_eq!(word(step_tag + 1), 7, "step_index is the first field");

        let device_tag = step_tag + 1 + TraceStep::FIELD_COUNT;
        assert_eq!(word(device_tag), KIND_DEVICE);
        assert_eq!(
            word(device_tag + 4),
            9,
            "sample_index is the fourth field of TraceDevice"
        );
    }
}
