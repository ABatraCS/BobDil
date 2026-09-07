"""The terminal summary.

Exists so the make target says something useful without opening a browser. It
prints percentiles per span and, separately, the counts that must never be
folded into a percentile: a re-used input and a stale command are faults, and a
fault expressed as a large number reads as slowness.

These numbers measure *this machine*. Per AGENTS.md they are validation-group
and they do not travel: nothing here may be quoted as a property of the model.
"""

from __future__ import annotations

from .reader import TraceMeta
from .spans import PHASES, Derived

#: What to summarise, in the order the round trip happens.
ROWS: tuple[tuple[str, str], ...] = (
    ("input_age", "input age (sample -> step)"),
    *((name, f"  {name}") for name, _, _ in PHASES),
    ("total", "step total"),
    ("pickup", "publish -> device pickup"),
    ("apply", "device apply"),
    ("round_trip", "ROUND TRIP (sample -> torque)"),
)

_QUANTILES = (0.5, 0.99, 0.999)


def _us(value: int | None) -> str:
    """A dash for "never measured". A zero would read as an instant round trip
    rather than as one that never happened."""
    return "     --" if value is None else f"{value / 1000.0:7.1f}"


def render(derived: Derived, meta: TraceMeta | None = None) -> str:
    lines: list[str] = []
    if meta is not None:
        lines.append(f"trace: dt={meta.step_dt * 1e3:.3f} ms, kernel id {meta.kernel_id}")
    lines.append(f"{len(derived.steps)} steps")
    lines.append("")
    lines.append(f"{'span':<32}{'p50':>9}{'p99':>9}{'p99.9':>9}   (us)")
    lines.append("-" * 68)
    for key, label in ROWS:
        cells = "".join(_us(derived.percentile(key, q)).rjust(9) for q in _QUANTILES)
        lines.append(f"{label:<32}{cells}")

    lines.append("")
    lines.append("faults -- counted, never averaged into a span above:")
    lines.append(f"  re-used inputs      {derived.reused_inputs}")
    lines.append(f"  stale commands      {derived.stale_commands}")
    lines.append(f"  missing commands    {derived.missing_commands}")
    lines.append(f"  apply failures      {derived.apply_failures}")
    if derived.unjoined_devices:
        lines.append(
            f"  unjoined deliveries {derived.unjoined_devices}"
            "   (a few at the edges is normal; many means the join is wrong)"
        )

    lines.append("")
    lines.append(
        "These numbers measure this machine and do not travel. The UI leg is "
        "not included -- see the spec."
    )
    return "\n".join(lines)
