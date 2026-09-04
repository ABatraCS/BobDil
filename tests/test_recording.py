"""Reading BobDil telemetry from Python, and refusing to read the wrong thing.

The header exists so that two recordings can be shown to be comparable before
anything is concluded from comparing them. These tests are mostly about the
refusals, because a silent mis-read is the failure that produces a confident
wrong answer.
"""

from __future__ import annotations

import struct
from pathlib import Path

import pytest

from bobdil import recording
from bobdil.generated.frames import LAYOUT_HASH, LAYOUT_REVISION, VehicleState


def write_recording(
    path: Path,
    frames: list[VehicleState],
    *,
    layout_hash: int = LAYOUT_HASH,
    magic: int = recording.TELEMETRY_MAGIC,
    step_dt: float = 1e-3,
    kernel_id: int = 1,
    vehicle_hash: int = 0,
) -> Path:
    header = bytearray(recording.HEADER_SIZE)
    words = [
        magic,
        layout_hash,
        LAYOUT_REVISION,
        VehicleState.SIZE,
        len(VehicleState.FIELDS),
        struct.unpack("<Q", struct.pack("<d", step_dt))[0],
        kernel_id,
        4,
        vehicle_hash,
        0,
    ]
    for index, word in enumerate(words):
        header[index * 8 : index * 8 + 8] = struct.pack("<Q", word)
    path.write_bytes(bytes(header) + b"".join(frame.pack() for frame in frames))
    return path


def frame(step: int, **fields: float) -> VehicleState:
    return VehicleState(step_index=step, sim_time=step * 1e-3, **fields)


def test_a_recording_round_trips_through_the_python_reader(tmp_path):
    written = [frame(step, acc_y=step * 0.25, vehicle_speed=step) for step in range(1, 33)]
    path = write_recording(tmp_path / "r.bdt", written)
    read = recording.read(path)
    assert read.meta.step_dt == pytest.approx(1e-3)
    assert read.meta.kernel_id == 1
    assert len(read.frames) == 32
    assert read.frames[5].acc_y == pytest.approx(written[5].acc_y)
    assert read.duration_s == pytest.approx(32e-3)


def test_a_file_that_is_not_telemetry_is_refused(tmp_path):
    path = tmp_path / "junk.bdt"
    path.write_bytes(b"not telemetry at all" * 20)
    with pytest.raises(recording.RecordingError, match="not a BobDil telemetry file"):
        recording.read(path)


def test_a_recording_from_another_schema_is_refused_rather_than_misread(tmp_path):
    # This is the failure the layout hash exists to prevent: the bytes still
    # parse, they just mean something else, and every number that comes out is
    # plausible and wrong.
    path = write_recording(tmp_path / "old.bdt", [frame(1)], layout_hash=0xDEAD_BEEF)
    with pytest.raises(recording.RecordingError, match="schema layout"):
        recording.read(path)


def test_a_truncated_final_frame_is_reported_not_silently_dropped(tmp_path):
    path = write_recording(tmp_path / "cut.bdt", [frame(1), frame(2)])
    path.write_bytes(path.read_bytes()[:-16])
    with pytest.raises(recording.RecordingError, match="whole frames"):
        recording.read(path)


def test_an_empty_recording_reads_as_zero_frames(tmp_path):
    path = write_recording(tmp_path / "empty.bdt", [])
    assert recording.read(path).frames == []


def test_signal_extracts_one_column_by_schema_name(tmp_path):
    path = write_recording(tmp_path / "r.bdt", [frame(s, acc_y=float(s)) for s in range(1, 6)])
    read = recording.read(path)
    assert list(read.signal("acc_y")) == [1.0, 2.0, 3.0, 4.0, 5.0]


def test_asking_for_a_signal_that_is_not_in_the_schema_names_the_mistake(tmp_path):
    path = write_recording(tmp_path / "r.bdt", [frame(1)])
    with pytest.raises(KeyError, match="lateral_g"):
        recording.read(path).signal("lateral_g")
