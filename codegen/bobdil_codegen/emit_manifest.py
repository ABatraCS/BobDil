"""Emit schema/generated/signals.json -- the machine-readable index.

Consumed by tools/rt_bench and by session/ when it resolves FMI value
references, so neither has to parse YAML at runtime.
"""

from __future__ import annotations

import json

from .schema import Schema


def render(schema: Schema) -> str:
    payload = {
        "schema_version": schema.schema_version,
        "layout_revision": schema.layout_revision,
        "layout_hash": f"{schema.layout_hash:#018x}",
        "frames": {
            frame.name: {
                "shm_name": frame.shm_name,
                "size_bytes": frame.size_bytes,
                "fields": [
                    {
                        "name": f.name,
                        "type": f.type,
                        "unit": f.unit,
                        "offset": f.offset,
                        "fmi": f.fmi,
                    }
                    for f in frame.fields
                ],
                "fmi_signals": {f.fmi: f.name for f in frame.plant_signals()},
            }
            for frame in schema.frames
        },
        "enums": {e.name: dict(e.values) for e in schema.enums},
        "tunables": [
            {"id": t.id, "name": t.name, "fmi": t.fmi, "unit": t.unit, "min": t.min, "max": t.max}
            for t in schema.tunables
        ],
    }
    return json.dumps(payload, indent=2) + "\n"
