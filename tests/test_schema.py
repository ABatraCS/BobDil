"""The schema loader's rules, tested at the loader rather than through codegen.

These exist because `shm_name` became optional: a frame that is a ring or file
record has no shared-memory segment, and inventing a segment name for one would
be a falsehood in the file the whole repository treats as the single source of
truth.
"""

from __future__ import annotations

import pytest

from bobdil_codegen import schema as schema_mod
from bobdil_codegen.schema import SchemaError

MINIMAL_TYPES = {
    "f64": {"rust": "f64", "c": "double", "py": "d", "gd": 8, "proto": "double"},
    "u64": {"rust": "u64", "c": "uint64_t", "py": "Q", "gd": 8, "proto": "uint64"},
}


def _frame(spec: dict) -> schema_mod.Frame:
    return schema_mod._build_frame("demo", spec, MINIMAL_TYPES)


def test_a_frame_without_shm_name_loads_and_has_no_segment():
    """A ring/file record is a frame, but it is not a shared-memory segment."""
    frame = _frame({"doc": "a ring record", "fields": [{"name": "a", "type": "u64"}]})

    assert frame.shm_name is None


def test_a_frame_with_shm_name_still_carries_it():
    frame = _frame(
        {
            "doc": "a segment",
            "shm_name": "bobdil_demo",
            "fields": [{"name": "a", "type": "u64"}],
        }
    )

    assert frame.shm_name == "bobdil_demo"


def test_an_empty_shm_name_is_still_refused():
    """Omitting the key means "no segment"; an empty string means someone meant
    to name one and got it wrong, which is a mistake rather than an intent."""
    with pytest.raises(SchemaError, match="shm_name"):
        _frame({"doc": "d", "shm_name": "", "fields": [{"name": "a", "type": "u64"}]})


class TestEmittersOmitTheSegmentConstant:
    """Every emitter writes one line for `shm_name`. A frame with no segment
    must not get that line in any of the five languages -- a constant naming a
    segment that does not exist is worse than no constant, because it compiles.
    """

    @staticmethod
    def _schema_with_a_ringonly_frame() -> schema_mod.Schema:
        segment = _frame(
            {
                "doc": "a segment",
                "shm_name": "bobdil_demo",
                "fields": [{"name": "a", "type": "u64"}],
            }
        )
        ring = schema_mod.Frame(
            name="trace_demo",
            doc="a ring record",
            shm_name=None,
            fields=segment.fields,
        )
        return schema_mod.Schema(
            schema_version=1,
            layout_revision=1,
            types=MINIMAL_TYPES,
            frames=(segment, ring),
            enums=(),
            tunables=(),
            layout_hash=0x1234,
        )

    @pytest.mark.parametrize(
        "module_name",
        ["emit_rust", "emit_c", "emit_python", "emit_gdscript"],
    )
    def test_a_ring_frame_never_renders_a_segment_named_none(self, module_name):
        """The failure this guards against is silent: `str(None)` is a perfectly
        good string, so a missing segment renders as a segment literally named
        "None" and every language compiles it happily."""
        import importlib

        module = importlib.import_module(f"bobdil_codegen.{module_name}")
        rendered = module.render(self._schema_with_a_ringonly_frame())

        assert "bobdil_demo" in rendered, "the real segment must still be named"
        assert '"None"' not in rendered

    def test_the_manifest_records_a_ring_frame_with_no_segment(self):
        from bobdil_codegen import emit_manifest

        rendered = emit_manifest.render(self._schema_with_a_ringonly_frame())

        assert '"shm_name": null' in rendered or "'shm_name': None" in rendered
