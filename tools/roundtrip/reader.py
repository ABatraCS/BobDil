"""Read a `.bdtrace`, or refuse to.

Written by `kernel/src/telemetry/trace.rs`. Nothing here decides anything about
the bytes: the record layouts come from the same schema that emitted the Rust
structs, and the `layout_hash` in the header is checked before a single record
is decoded.

That check is the whole point of the module. A trace read against the wrong
schema does not fail -- every field decodes, and every number that comes out is
wrong in a way that still looks entirely plausible.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from pathlib import Path

from bobdil.generated.frames import LAYOUT_HASH, TraceDevice, TraceStep

#: "BDTRC001" little-endian -- must match ``trace.rs::TRACE_MAGIC``.
TRACE_MAGIC = 0x3130_3043_5254_4442
HEADER_SIZE = 128

#: Record tags, matching ``trace.rs``.
KIND_STEP = 1
KIND_DEVICE = 2

_MAGIC, _LAYOUT_HASH, _LAYOUT_REVISION = 0, 1, 2
_STEP_SIZE, _DEVICE_SIZE, _STEP_DT, _KERNEL_ID, _BUILD_HASH = 3, 4, 5, 6, 7


class TraceError(ValueError):
    """This file is not a trace this build can read. Always fatal."""


@dataclass(frozen=True)
class TraceMeta:
    """What the trace is of. Deliberately less than a recording carries: a trace
    measures *this machine* and is never comparable across boxes, so there is no
    provenance here to invite a comparison that would not mean anything."""

    step_dt: float
    kernel_id: int
    build_hash: int


@dataclass
class Trace:
    meta: TraceMeta
    steps: list[TraceStep]
    devices: list[TraceDevice]
    path: Path | None = None


def read(path: Path | str) -> Trace:
    path = Path(path)
    raw = path.read_bytes()
    if len(raw) < HEADER_SIZE:
        raise TraceError(f"{path}: ends mid-record at byte {len(raw)} (shorter than a header)")

    words = struct.unpack_from(f"<{HEADER_SIZE // 8}Q", raw, 0)
    if words[_MAGIC] != TRACE_MAGIC:
        raise TraceError(f"{path}: not a BobDil trace (magic {words[_MAGIC]:#018x})")
    if words[_LAYOUT_HASH] != LAYOUT_HASH:
        raise TraceError(
            f"{path}: schema mismatch: trace was written by layout "
            f"{words[_LAYOUT_HASH]:#018x}, this build expects {LAYOUT_HASH:#018x}. "
            "Rebuild both sides from the same schema."
        )

    meta = TraceMeta(
        step_dt=struct.unpack("<d", struct.pack("<Q", words[_STEP_DT]))[0],
        kernel_id=words[_KERNEL_ID],
        build_hash=words[_BUILD_HASH],
    )

    steps: list[TraceStep] = []
    devices: list[TraceDevice] = []
    cursor = HEADER_SIZE
    while cursor < len(raw):
        if cursor + 8 > len(raw):
            raise TraceError(f"{path}: ends mid-record at byte {cursor}")
        (kind,) = struct.unpack_from("<Q", raw, cursor)
        cursor += 8
        if kind == KIND_STEP:
            record_type, target = TraceStep, steps
        elif kind == KIND_DEVICE:
            record_type, target = TraceDevice, devices
        else:
            raise TraceError(f"{path}: unknown record kind {kind} at byte {cursor - 8}")

        end = cursor + record_type.SIZE
        if end > len(raw):
            raise TraceError(f"{path}: ends mid-record at byte {cursor}")
        target.append(record_type.unpack(raw[cursor:end]))
        cursor = end

    return Trace(meta=meta, steps=steps, devices=devices, path=path)
