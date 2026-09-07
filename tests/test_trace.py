"""The trace reader and the spans it derives.

These are the tests that keep the tool honest about two things: it must refuse a
trace it cannot interpret rather than producing plausible nonsense, and it must
keep a broken leg out of a latency percentile -- a stale command is a fault, and
averaging it into a timing number reports a safety condition as a slow one.
"""

from __future__ import annotations

import struct

import pytest
from roundtrip import reader, spans

from bobdil.generated.frames import LAYOUT_HASH, LAYOUT_REVISION, TraceDevice, TraceFlags, TraceStep

MS = 1_000_000


def build_trace(
    steps: list[TraceStep],
    devices: list[TraceDevice],
    *,
    magic: int = reader.TRACE_MAGIC,
    layout_hash: int = LAYOUT_HASH,
) -> bytes:
    header = bytearray(reader.HEADER_SIZE)
    words = [
        magic,
        layout_hash,
        LAYOUT_REVISION,
        TraceStep.SIZE,
        TraceDevice.SIZE,
        struct.unpack("<Q", struct.pack("<d", 1e-3))[0],
        1,
        0,
    ]
    for index, word in enumerate(words):
        header[index * 8 : index * 8 + 8] = struct.pack("<Q", word)

    body = bytearray()
    for step in steps:
        body += struct.pack("<Q", reader.KIND_STEP) + step.pack()
    for device in devices:
        body += struct.pack("<Q", reader.KIND_DEVICE) + device.pack()
    return bytes(header) + bytes(body)


def a_step(index: int = 0, base: int = 1_000_000_000, input_age_ns: int = 500_000) -> TraceStep:
    """A step whose phases each take a known, distinct time."""
    return TraceStep(
        step_index=index,
        input_host_time_ns=base - input_age_ns,
        input_sample_index=index,
        t_step_start=base,
        t_after_read=base + 10_000,
        t_after_shape=base + 20_000,
        t_after_plant=base + 120_000,
        t_command_stamp=base + 130_000,
        t_after_ffb=base + 150_000,
        t_after_publish=base + 160_000,
    )


class TestTheReaderRefusesWhatItCannotInterpret:
    def test_a_good_trace_round_trips(self, tmp_path):
        path = tmp_path / "good.bdtrace"
        path.write_bytes(build_trace([a_step()], [TraceDevice(flags=TraceFlags.COMMAND_FRESH)]))

        trace = reader.read(path)

        assert len(trace.steps) == 1
        assert len(trace.devices) == 1
        assert trace.steps[0].t_after_plant == 1_000_120_000
        assert trace.meta.step_dt == pytest.approx(1e-3)

    def test_a_file_that_is_not_a_trace_is_refused(self, tmp_path):
        path = tmp_path / "bad.bdtrace"
        path.write_bytes(build_trace([], [], magic=0xDEAD))

        with pytest.raises(reader.TraceError, match="not a BobDil trace"):
            reader.read(path)

    def test_a_trace_from_another_schema_is_refused(self, tmp_path):
        """The failure this prevents is the expensive one: every field still
        decodes, and every number is wrong in a way that looks plausible."""
        path = tmp_path / "stale.bdtrace"
        path.write_bytes(build_trace([a_step()], [], layout_hash=LAYOUT_HASH ^ 0xFF))

        with pytest.raises(reader.TraceError, match="schema mismatch"):
            reader.read(path)

    def test_a_truncated_trace_is_refused_rather_than_silently_short(self, tmp_path):
        path = tmp_path / "cut.bdtrace"
        path.write_bytes(build_trace([a_step(), a_step(1)], [])[:-20])

        with pytest.raises(reader.TraceError, match="ends mid-record"):
            reader.read(path)


class TestSpans:
    def test_each_phase_becomes_a_span_with_its_own_duration(self):
        derived = spans.derive([a_step()], [])

        step = derived.steps[0]
        assert step.phases["read"] == 10_000
        assert step.phases["shape"] == 10_000
        assert step.phases["plant"] == 100_000
        assert step.phases["pose"] == 10_000
        assert step.phases["ffb"] == 20_000
        assert step.phases["publish"] == 10_000
        assert step.total == 160_000

    def test_input_age_is_measured_from_the_sample_the_step_consumed(self):
        derived = spans.derive([a_step(input_age_ns=750_000)], [])

        assert derived.steps[0].input_age == 750_000

    def test_a_reused_sample_is_reported_rather_than_looking_fast(self):
        """Two steps on one sample means the driver's input did not reach the
        second one. Nothing in the step's own timing shows that."""
        first = a_step(0)
        second = a_step(1, base=1_000_500_000)
        second.input_sample_index = first.input_sample_index
        second.input_host_time_ns = first.input_host_time_ns

        derived = spans.derive([first, second], [])

        assert derived.reused_inputs == 1

    def test_the_device_leg_joins_on_the_command_stamp(self):
        step = a_step()
        device = TraceDevice(
            command_host_time_ns=step.t_command_stamp,
            t_pickup=step.t_command_stamp + 200_000,
            t_after_apply=step.t_command_stamp + 260_000,
            sample_index=0,
            flags=TraceFlags.COMMAND_FRESH,
        )

        derived = spans.derive([step], [device])

        assert derived.steps[0].delivery is not None
        assert derived.steps[0].delivery.pickup_delay == 200_000
        assert derived.steps[0].delivery.apply_cost == 60_000

    def test_a_stale_command_is_counted_as_a_fault_not_a_latency(self):
        """The number that would otherwise appear is real and enormous, and it
        would describe a wheel that never got the torque at all."""
        step = a_step()
        stale = TraceDevice(
            command_host_time_ns=step.t_command_stamp,
            t_pickup=step.t_command_stamp + 50 * MS,
            t_after_apply=step.t_command_stamp + 50 * MS + 1000,
            flags=TraceFlags.COMMAND_STALE,
        )

        derived = spans.derive([step], [stale])

        assert derived.stale_commands == 1
        assert derived.steps[0].delivery is None, "a stale command is not a delivery"
        assert derived.percentile("pickup", 0.5) is None, "nothing to take a percentile of"

    def test_round_trip_spans_the_sample_to_the_delivered_torque(self):
        step = a_step(input_age_ns=500_000)
        device = TraceDevice(
            command_host_time_ns=step.t_command_stamp,
            t_pickup=step.t_command_stamp + 200_000,
            t_after_apply=step.t_command_stamp + 260_000,
            flags=TraceFlags.COMMAND_FRESH,
        )

        derived = spans.derive([step], [device])

        # sample -> step start (500 us) + step start -> apply done (390 us)
        assert derived.steps[0].round_trip == 890_000


class TestChromeTrace:
    """The output format is Chrome Trace Event JSON because Perfetto,
    chrome://tracing and speedscope all render it as a zoomable flamegraph with
    no dependency here and no hand-rolled SVG to keep true."""

    @staticmethod
    def _events(derived):
        from roundtrip import chrome

        return chrome.build(derived)["traceEvents"]

    def test_each_phase_is_a_complete_event_nested_in_its_step(self):
        derived = spans.derive([a_step()], [])
        events = self._events(derived)

        step = next(e for e in events if e["name"] == "step")
        plant = next(e for e in events if e["name"] == "plant")

        assert step["ph"] == "X"
        # Chrome nests by time containment on one track, so the phase must sit
        # inside the step on the same tid or it renders as a sibling.
        assert plant["tid"] == step["tid"]
        assert plant["ts"] >= step["ts"]
        assert plant["ts"] + plant["dur"] <= step["ts"] + step["dur"]

    def test_durations_are_microseconds_because_that_is_what_the_format_says(self):
        derived = spans.derive([a_step()], [])
        plant = next(e for e in self._events(derived) if e["name"] == "plant")

        # 100_000 ns of plant
        assert plant["dur"] == pytest.approx(100.0)

    def test_the_device_leg_lands_on_its_own_track(self):
        step = a_step()
        device = TraceDevice(
            command_host_time_ns=step.t_command_stamp,
            t_pickup=step.t_command_stamp + 200_000,
            t_after_apply=step.t_command_stamp + 260_000,
            flags=TraceFlags.COMMAND_FRESH,
        )
        events = self._events(spans.derive([step], [device]))

        step_event = next(e for e in events if e["name"] == "step")
        apply_event = next(e for e in events if e["name"] == "apply")

        assert apply_event["tid"] != step_event["tid"], (
            "the device leg is a different thread and must not nest inside the step"
        )

    def test_an_empty_trace_produces_no_events_rather_than_failing(self):
        assert self._events(spans.derive([], [])) == []


class TestTheTerminalSummary:
    @staticmethod
    def _render(derived):
        from roundtrip import report

        return report.render(derived, meta=None)

    def test_it_names_the_phases_and_their_percentiles(self):
        derived = spans.derive([a_step(i, base=1_000_000_000 + i * MS) for i in range(5)], [])

        text = self._render(derived)

        assert "plant" in text
        assert "p99" in text
        # 100 us of plant, printed in microseconds
        assert "100.0" in text

    def test_a_broken_leg_is_stated_rather_than_left_to_the_numbers(self):
        step = a_step()
        stale = TraceDevice(
            command_host_time_ns=step.t_command_stamp,
            t_pickup=step.t_command_stamp + 50 * MS,
            t_after_apply=step.t_command_stamp + 50 * MS + 1000,
            flags=TraceFlags.COMMAND_STALE,
        )

        text = self._render(spans.derive([step], [stale]))

        assert "stale" in text.lower()
        assert "1" in text

    def test_a_round_trip_that_was_never_completed_says_so(self):
        """Printing a dash rather than 0.0 matters: a zero here reads as an
        instant round trip rather than as one that never happened."""
        text = self._render(spans.derive([a_step()], []))

        row = next(line for line in text.splitlines() if line.startswith("ROUND TRIP"))
        assert "--" in row, row
        assert "0.0" not in row, f"an unmeasured round trip must not print as zero: {row}"
