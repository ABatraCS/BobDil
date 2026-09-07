"""Load and validate schema/bobdil_signals.yaml.

This module owns the *meaning* of the schema. Emitters own the syntax of one
target language each and never re-interpret the YAML themselves.
"""

from __future__ import annotations

import hashlib
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
SCHEMA_PATH = REPO_ROOT / "schema" / "bobdil_signals.yaml"

# Every scalar in a frame is 8 bytes wide. This is a deliberate constraint, not
# an accident: it makes each frame naturally aligned with no padding, so the
# Rust struct, the C struct, the Python struct format and the GDScript byte
# offsets describe the same bytes without any per-language alignment rules.
SCALAR_WIDTH = 8


class SchemaError(ValueError):
    """The schema file is malformed. Always fatal -- never fall back to a guess."""


@dataclass(frozen=True)
class Field:
    name: str
    type: str
    unit: str
    offset: int
    doc: str = ""
    fmi: str | None = None
    range: tuple[float, float] | None = None

    @property
    def is_plant_signal(self) -> bool:
        """True when this field maps onto an FMI variable rather than being kernel-derived."""
        return self.fmi is not None


@dataclass(frozen=True)
class Frame:
    name: str
    doc: str
    #: None for frames that are ring or file records rather than shm segments.
    shm_name: str | None
    fields: tuple[Field, ...]

    @property
    def size_bytes(self) -> int:
        return len(self.fields) * SCALAR_WIDTH

    @property
    def pascal_name(self) -> str:
        return "".join(part.capitalize() for part in self.name.split("_"))

    def plant_signals(self) -> tuple[Field, ...]:
        return tuple(f for f in self.fields if f.is_plant_signal)


@dataclass(frozen=True)
class Enum:
    name: str
    doc: str
    values: Mapping[str, int]

    @property
    def pascal_name(self) -> str:
        return "".join(part.capitalize() for part in self.name.split("_"))


@dataclass(frozen=True)
class Tunable:
    id: int
    name: str
    fmi: str
    unit: str
    min: float
    max: float


@dataclass(frozen=True)
class Schema:
    schema_version: int
    layout_revision: int
    types: Mapping[str, Mapping[str, Any]]
    frames: tuple[Frame, ...]
    enums: tuple[Enum, ...]
    tunables: tuple[Tunable, ...]
    layout_hash: int

    def frame(self, name: str) -> Frame:
        for candidate in self.frames:
            if candidate.name == name:
                return candidate
        raise SchemaError(f"no frame named {name!r}")

    def rust_type(self, field: Field) -> str:
        return str(self.types[field.type]["rust"])

    def c_type(self, field: Field) -> str:
        return str(self.types[field.type]["c"])

    def py_code(self, field: Field) -> str:
        return str(self.types[field.type]["py"])

    def proto_type(self, field: Field) -> str:
        return str(self.types[field.type]["proto"])


def _as_range(value: Any) -> tuple[float, float] | None:
    if value is None:
        return None
    if not isinstance(value, Sequence) or len(value) != 2:
        raise SchemaError(f"range must be [min, max], got {value!r}")
    return (float(value[0]), float(value[1]))


def _build_frame(name: str, spec: Mapping[str, Any], types: Mapping[str, Any]) -> Frame:
    raw_fields = spec.get("fields")
    if not raw_fields:
        raise SchemaError(f"frame {name!r} declares no fields")

    fields: list[Field] = []
    seen: set[str] = set()
    for index, raw in enumerate(raw_fields):
        field_name = raw.get("name")
        if not field_name:
            raise SchemaError(f"frame {name!r} field {index} has no name")
        if field_name in seen:
            raise SchemaError(f"frame {name!r} declares {field_name!r} twice")
        seen.add(field_name)
        field_type = raw.get("type")
        if field_type not in types:
            raise SchemaError(
                f"frame {name!r} field {field_name!r} has unknown type {field_type!r}"
            )
        fields.append(
            Field(
                name=field_name,
                type=field_type,
                unit=str(raw.get("unit", "-")),
                offset=index * SCALAR_WIDTH,
                doc=str(raw.get("doc", "")),
                fmi=raw.get("fmi"),
                range=_as_range(raw.get("range")),
            )
        )

    # A frame is not necessarily a shared-memory segment. The trace records
    # are ring and file records, and giving one a segment name it does not have
    # would be a falsehood in the file everything else treats as the truth.
    # Absent means "no segment"; present-but-empty means someone meant to name
    # one and got it wrong, which is still an error.
    shm_name = spec.get("shm_name")
    if shm_name is not None and not str(shm_name).strip():
        raise SchemaError(f"frame {name!r} has an empty shm_name")

    return Frame(
        name=name,
        doc=" ".join(str(spec.get("doc", "")).split()),
        shm_name=str(shm_name) if shm_name is not None else None,
        fields=tuple(fields),
    )


def _layout_hash(frames: Sequence[Frame], layout_revision: int) -> int:
    """A stable 64-bit fingerprint of the wire layout.

    Written into every shared-memory segment header. A consumer built against a
    different schema sees a mismatch and refuses to attach, instead of reading
    the right bytes with the wrong meaning -- the failure mode that makes
    binary IPC bugs so expensive to find.
    """
    digest = hashlib.blake2b(digest_size=8)
    digest.update(f"bobdil.v{layout_revision}".encode())
    for frame in frames:
        digest.update(f"|{frame.name}".encode())
        for field in frame.fields:
            digest.update(f"|{field.name}:{field.type}:{field.offset}".encode())
    return int.from_bytes(digest.digest(), "little")


def load(path: Path | str = SCHEMA_PATH) -> Schema:
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise SchemaError("schema root must be a mapping")

    types = raw.get("types") or {}
    if not types:
        raise SchemaError("schema declares no types")

    frames = tuple(
        _build_frame(name, spec, types) for name, spec in (raw.get("frames") or {}).items()
    )
    if not frames:
        raise SchemaError("schema declares no frames")

    enums = tuple(
        Enum(name=name, doc=" ".join(str(spec.get("doc", "")).split()), values=dict(spec["values"]))
        for name, spec in (raw.get("enums") or {}).items()
    )

    tunables: list[Tunable] = []
    seen_ids: set[int] = set()
    for spec in raw.get("tunables") or []:
        if spec["id"] in seen_ids:
            raise SchemaError(f"duplicate tunable id {spec['id']}")
        seen_ids.add(spec["id"])
        tunables.append(
            Tunable(
                id=int(spec["id"]),
                name=str(spec["name"]),
                fmi=str(spec["fmi"]),
                unit=str(spec.get("unit", "-")),
                min=float(spec["min"]),
                max=float(spec["max"]),
            )
        )

    layout_revision = int(raw.get("layout_revision", 1))
    return Schema(
        schema_version=int(raw.get("schema_version", 1)),
        layout_revision=layout_revision,
        types=types,
        frames=frames,
        enums=enums,
        tunables=tuple(tunables),
        layout_hash=_layout_hash(frames, layout_revision),
    )
