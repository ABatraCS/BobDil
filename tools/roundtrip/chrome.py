"""Emit the trace as Chrome Trace Event JSON.

Perfetto, `chrome://tracing` and speedscope all read this format and all render
it as a zoomable flamegraph with their own summaries and their own search. That
is the entire reason it was chosen: there is no dependency to add here and no
hand-rolled SVG to keep true against a format that already exists.

Chrome nests spans by time containment on a single track, so the phases of a
step are emitted on the same `tid` as the step itself and nest without being
told to. The device leg goes on its own track, because it is a different thread
and drawing it inside the step would claim a containment that is not real.
"""

from __future__ import annotations

from typing import Any

from .spans import Derived

#: One process, two tracks -- the two threads that actually produce the spans.
PID = 1
TID_STEP = 1
TID_DEVICE = 2


def _ns_to_us(value: int) -> float:
    return value / 1000.0


def build(derived: Derived, *, origin: int | None = None) -> dict[str, Any]:
    """Build the trace document. `origin` is subtracted from every stamp so the
    timeline starts near zero; viewers cope with absolute stamps but they are
    unreadable in a tooltip."""
    events: list[dict[str, Any]] = []
    if not derived.steps:
        return {"traceEvents": events, "displayTimeUnit": "ns"}

    if origin is None:
        origin = min(step.absolute_start - step.input_age for step in derived.steps)

    for step in derived.steps:
        start = step.absolute_start
        # The input's own age, before the step began. Drawn on the step track
        # because it is time the step is accountable for even though it did not
        # spend it: a stale input is latency the driver feels.
        events.append(
            {
                "name": "input age",
                "cat": "input",
                "ph": "X",
                "ts": _ns_to_us(start - step.input_age - origin),
                "dur": _ns_to_us(step.input_age),
                "pid": PID,
                "tid": TID_STEP,
                "args": {"sample_index": step.input_sample_index},
            }
        )
        events.append(
            {
                "name": "step",
                "cat": "step",
                "ph": "X",
                "ts": _ns_to_us(start - origin),
                "dur": _ns_to_us(step.total),
                "pid": PID,
                "tid": TID_STEP,
                "args": {"step_index": step.step_index},
            }
        )
        cursor = start
        for name, duration in step.phases.items():
            events.append(
                {
                    "name": name,
                    "cat": "phase",
                    "ph": "X",
                    "ts": _ns_to_us(cursor - origin),
                    "dur": _ns_to_us(duration),
                    "pid": PID,
                    "tid": TID_STEP,
                    "args": {"step_index": step.step_index},
                }
            )
            cursor += duration

        if step.delivery is not None:
            handoff = start + step.to_command
            events.append(
                {
                    "name": "publish -> pickup",
                    "cat": "device",
                    "ph": "X",
                    "ts": _ns_to_us(handoff - origin),
                    "dur": _ns_to_us(step.delivery.pickup_delay),
                    "pid": PID,
                    "tid": TID_DEVICE,
                    "args": {"step_index": step.step_index},
                }
            )
            events.append(
                {
                    "name": "apply",
                    "cat": "device",
                    "ph": "X",
                    "ts": _ns_to_us(handoff + step.delivery.pickup_delay - origin),
                    "dur": _ns_to_us(step.delivery.apply_cost),
                    "pid": PID,
                    "tid": TID_DEVICE,
                    "args": {"step_index": step.step_index},
                }
            )

    events.extend(
        [
            {
                "name": "thread_name",
                "ph": "M",
                "pid": PID,
                "tid": TID_STEP,
                "args": {"name": "StepThread"},
            },
            {
                "name": "thread_name",
                "ph": "M",
                "pid": PID,
                "tid": TID_DEVICE,
                "args": {"name": "HidThread"},
            },
        ]
    )
    return {"traceEvents": events, "displayTimeUnit": "ns"}
