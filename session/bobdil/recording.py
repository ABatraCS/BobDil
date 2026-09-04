"""Read BobDil telemetry from Python, or refuse to.

The binary format is written by ``kernel/src/telemetry/recorder.rs`` and the
layout of a frame comes from the schema, so nothing here decides anything about
the bytes -- it reads the same header the kernel writes and the same struct
format ``codegen`` emitted. That is the point: two readers of one schema, and a
``layout_hash`` in the header so a stale one refuses to attach instead of
producing plausible nonsense.

Every failure in this module is loud. A recording is used to make a claim about
a car, and the worst outcome available is a file that parses into numbers that
mean something other than what they are read as.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field
from pathlib import Path

from .generated.frames import LAYOUT_HASH, VehicleState

#: "BDTEL001" little-endian -- must match ``recorder.rs::TELEMETRY_MAGIC``.
TELEMETRY_MAGIC = 0x3130_304C_4554_4442
HEADER_SIZE = 128

#: Header word order, matching the ``put(...)`` sequence in ``recorder.rs``.
_MAGIC, _LAYOUT_HASH, _LAYOUT_REVISION = 0, 1, 2
_FRAME_SIZE, _FIELD_COUNT, _STEP_DT = 3, 4, 5
_KERNEL_ID, _EVALUATIONS, _VEHICLE_HASH, _BUILD_HASH = 6, 7, 8, 9


class RecordingError(ValueError):
    """This file is not a recording this build can read. Always fatal."""


@dataclass(frozen=True)
class RecordingMeta:
    """Everything that decides whether two recordings are comparable."""

    step_dt: float
    kernel_id: int
    evaluations_per_step: int
    vehicle_hash: int
    build_hash: int

    def describe(self) -> str:
        return (
            f"dt={self.step_dt * 1e3:.3f} ms, kernel id {self.kernel_id}, "
            f"vehicle {self.vehicle_hash:#018x}"
        )


@dataclass
class Recording:
    meta: RecordingMeta
    frames: list[VehicleState] = field(default_factory=list)
    path: Path | None = None

    @property
    def duration_s(self) -> float:
        return len(self.frames) * self.meta.step_dt

    def signal(self, name: str) -> list[float]:
        """One column, by its schema field name."""
        if name not in VehicleState.FIELDS:
            raise KeyError(
                f"{name!r} is not a VehicleState field. The schema defines "
                f"{list(VehicleState.FIELDS)}"
            )
        return [getattr(frame, name) for frame in self.frames]

    def describe(self) -> str:
        where = f"{self.path} " if self.path else ""
        return f"{where}{len(self.frames)} frames, {self.duration_s:.3f} s, {self.meta.describe()}"


def _word(data: bytes, index: int) -> int:
    return struct.unpack_from("<Q", data, index * 8)[0]


def read(path: Path) -> Recording:
    """Read a recording, checking its provenance before returning any of it."""
    data = Path(path).read_bytes()
    if len(data) < HEADER_SIZE:
        raise RecordingError(f"{path} is shorter than a telemetry header")
    if _word(data, _MAGIC) != TELEMETRY_MAGIC:
        raise RecordingError(f"{path} is not a BobDil telemetry file")

    layout = _word(data, _LAYOUT_HASH)
    if layout != LAYOUT_HASH:
        raise RecordingError(
            f"{path} was written under schema layout {layout:#018x}, this build "
            f"expects {LAYOUT_HASH:#018x}. The bytes would still parse, and every "
            "number would be wrong, so it is refused rather than read."
        )

    stride = _word(data, _FRAME_SIZE)
    if stride != VehicleState.SIZE:
        raise RecordingError(
            f"{path} has {stride}-byte frames, this build's VehicleState is {VehicleState.SIZE}"
        )

    payload = data[HEADER_SIZE:]
    if payload and len(payload) % stride:
        raise RecordingError(
            f"{path} does not end on a frame boundary: {len(payload)} bytes is not a "
            f"whole number of {stride}-byte whole frames. A recording that was cut "
            "mid-frame lost the end of a run, which is exactly the part a "
            "comparison cares about."
        )

    meta = RecordingMeta(
        step_dt=struct.unpack("<d", struct.pack("<Q", _word(data, _STEP_DT)))[0],
        kernel_id=_word(data, _KERNEL_ID),
        evaluations_per_step=_word(data, _EVALUATIONS),
        vehicle_hash=_word(data, _VEHICLE_HASH),
        build_hash=_word(data, _BUILD_HASH),
    )
    frames = [
        VehicleState.unpack(payload[offset : offset + stride])
        for offset in range(0, len(payload), stride)
    ]
    return Recording(meta=meta, frames=frames, path=Path(path))
