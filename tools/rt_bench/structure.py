"""What is inside one step, read out of what omc emitted while compiling.

A fixed-step integrator can only hold a deadline if the work per step is
bounded. Three things in a Modelica model are unbounded, and none of them are
visible from the model source:

* a **non-linear algebraic system** is solved by Newton iteration at runtime, so
  its cost depends on the operating point, not on the model size;
* a **state event** makes the integrator bisect for the crossing time inside
  ``fmi2NewDiscreteStates``, which is an unbounded search;
* **dynamic state selection** re-picks the state set while the model runs, so
  the same step does different work on different laps.

omc already computes all three while it compiles. It writes them into
``<model>_info.json`` and ``<model>_init.xml`` for its transformational
debugger; this module reads them back out. Nothing here runs omc -- see
:mod:`rt_bench.report` for that -- so it is testable against captured output.
"""

from __future__ import annotations

import json
import re
from dataclasses import dataclass, field
from pathlib import Path
from xml.etree import ElementTree

#: omc names the states it invents for dynamic state selection with this prefix.
STATE_SET_PREFIX = "$STATESET"

#: Beyond this, a Newton solve's iteration count stops being predictable enough
#: to sit inside a 1 ms budget. It is a heuristic and is reported as one.
NONLINEAR_SIZE_CONCERN = 8


@dataclass(frozen=True)
class AlgebraicSystem:
    """One algebraic loop omc could not solve by assignment."""

    index: int
    size: int
    linear: bool
    torn: bool
    initial: bool

    @property
    def kind(self) -> str:
        return f"{'torn ' if self.torn else ''}{'linear' if self.linear else 'non-linear'}"


@dataclass(frozen=True)
class EquationCounts:
    """What ``checkModel`` says the flattened model contains."""

    equations: int | None = None
    variables: int | None = None
    trivial: int | None = None


@dataclass(frozen=True)
class EventCounts:
    """Zero crossings, which are the unbounded part of an event-driven model."""

    state_events: int = 0
    time_events: int = 0


@dataclass
class StructuralReport:
    model: str
    continuous_states: int = 0
    systems: list[AlgebraicSystem] = field(default_factory=list)
    state_sets: list[str] = field(default_factory=list)
    counts: EquationCounts = field(default_factory=EquationCounts)
    events: EventCounts = field(default_factory=EventCounts)

    @property
    def dynamic_state_selection(self) -> bool:
        return bool(self.state_sets)

    @property
    def runtime_systems(self) -> list[AlgebraicSystem]:
        """The systems solved every step.

        Initial-section systems are excluded deliberately: they run once, before
        the first step, where an unbounded solve costs startup time and cannot
        miss a deadline.
        """
        return [system for system in self.systems if not system.initial]

    @property
    def initial_systems(self) -> int:
        return len(self.systems) - len(self.runtime_systems)

    @property
    def nonlinear_systems(self) -> int:
        return sum(1 for system in self.runtime_systems if not system.linear)

    @property
    def linear_systems(self) -> int:
        return sum(1 for system in self.runtime_systems if system.linear)

    @property
    def max_nonlinear_size(self) -> int:
        sizes = [system.size for system in self.runtime_systems if not system.linear]
        return max(sizes, default=0)

    @property
    def max_linear_size(self) -> int:
        sizes = [system.size for system in self.runtime_systems if system.linear]
        return max(sizes, default=0)

    def concerns(self) -> list[str]:
        """The real-time reading of these numbers, in plain words.

        Returned rather than printed so the CLI, the report and a test can each
        use it. An empty list means nothing structural stands between this model
        and a fixed step -- which is a claim about *bounded work*, not about
        speed. Only the timing run decides speed.
        """
        found: list[str] = []
        if self.nonlinear_systems:
            found.append(
                f"{self.nonlinear_systems} non-linear algebraic system(s), largest "
                f"{self.max_nonlinear_size} equations. Each is a Newton solve whose "
                "iteration count varies with the operating point, so the step cost "
                "has no upper bound that can be read off the model."
                + (
                    " That is past the size where the iteration count stays predictable; "
                    "this is the first thing VehicleRT should remove (see "
                    "architecture.md 1.5)."
                    if self.max_nonlinear_size > NONLINEAR_SIZE_CONCERN
                    else ""
                )
            )
        if self.events.state_events:
            found.append(
                f"{self.events.state_events} state event indicator(s). Locating a "
                "crossing is a bisection inside fmi2NewDiscreteStates, which is "
                "unbounded work in the middle of a step. Regularise the "
                "discontinuity (tanh, a smooth floor) rather than letting the "
                "solver find it."
            )
        if self.dynamic_state_selection:
            found.append(
                f"dynamic state selection is active ({', '.join(self.state_sets)}). "
                "The state set is re-chosen while the model runs, so two identical "
                "steps can do different work and a recording is no longer "
                "reproducible. Build with --indexReductionMethod=uode."
            )
        return found

    def describe(self) -> str:
        lines = [
            f"structure of {self.model}",
            f"  continuous states     {self.continuous_states}",
        ]
        if self.counts.equations is not None:
            lines.append(
                f"  flattened equations   {self.counts.equations} ({self.counts.trivial} trivial)"
            )
        lines.extend(
            [
                f"  non-linear systems    {self.nonlinear_systems}"
                f" (largest {self.max_nonlinear_size})",
                f"  linear systems        {self.linear_systems} (largest {self.max_linear_size})",
                f"  state events          {self.events.state_events}",
                f"  time events           {self.events.time_events}",
                f"  state selection       {'ACTIVE' if self.dynamic_state_selection else 'static'}",
            ]
        )
        if self.initial_systems:
            lines.append(
                f"  (plus {self.initial_systems} system(s) solved only at "
                "initialisation, which cannot miss a deadline)"
            )
        concerns = self.concerns()
        if concerns:
            lines.append("")
            lines.extend(f"  ! {concern}" for concern in concerns)
        else:
            lines.append("")
            lines.append(
                "  Nothing structural bounds this model away from a fixed step. "
                "Whether it is fast enough is a separate question, answered by "
                "`bobdil-kernel bench`."
            )
        return "\n".join(lines)


_CHECK_MODEL_EQUATIONS = re.compile(r"has (\d+) equation\(s\) and (\d+) variable\(s\)")
_CHECK_MODEL_TRIVIAL = re.compile(r"(\d+) of these are trivial equation\(s\)")


def parse_check_model(output: str) -> EquationCounts:
    """Read the equation and variable counts out of ``checkModel``'s prose.

    A model that failed to check returns all-``None`` rather than zeros: zero
    equations and "we could not tell" are very different claims, and reporting
    the second as the first is how a broken build reads as a trivially fast one.
    """
    sizes = _CHECK_MODEL_EQUATIONS.search(output)
    if sizes is None:
        return EquationCounts()
    trivial = _CHECK_MODEL_TRIVIAL.search(output)
    return EquationCounts(
        equations=int(sizes.group(1)),
        variables=int(sizes.group(2)),
        trivial=int(trivial.group(1)) if trivial else None,
    )


def parse_init_xml(xml_text: str) -> EventCounts:
    """Read the zero-crossing counts out of ``<model>_init.xml``."""
    root = ElementTree.fromstring(xml_text)
    return EventCounts(
        state_events=int(root.get("numberOfEventIndicators", "0")),
        time_events=int(root.get("numberOfTimeEvents", "0")),
    )


def _system_size(equation: dict) -> int:
    """How many equations are solved simultaneously in one system.

    A torn system splits into a solved part and a residual part; omc reports the
    whole thing under ``equation`` and the torn detail under separate keys. The
    size that matters for iteration cost is the residual system, so prefer that
    when it is present.
    """
    for key in ("residualEquations", "equation"):
        members = equation.get(key)
        if isinstance(members, list) and members:
            return len(members)
    return 0


def parse_info_json(payload: dict) -> StructuralReport:
    """Build the report from a parsed ``<model>_info.json``."""
    report = StructuralReport(model=payload.get("info", {}).get("name", "(unnamed)"))

    state_sets: set[str] = set()
    states = 0
    for name, variable in payload.get("variables", {}).items():
        if variable.get("kind") != "state":
            continue
        if name.startswith(STATE_SET_PREFIX):
            # Not a state of the model: a state omc invented so it could pick a
            # different set at runtime. Counting it inflates the model's order
            # and hides the state selection itself, which is the real finding.
            state_sets.add(name.split(".", 1)[0])
        else:
            states += 1
    report.continuous_states = states
    report.state_sets = sorted(state_sets)

    for equation in payload.get("equations", []):
        tag = equation.get("tag")
        if tag not in {"system", "tornsystem"}:
            continue
        display = equation.get("display", "")
        report.systems.append(
            AlgebraicSystem(
                index=equation.get("eqIndex", -1),
                size=_system_size(equation),
                linear="non-linear" not in display,
                torn=tag == "tornsystem" or "torn" in display,
                initial=equation.get("section") == "initial",
            )
        )
    return report


def read(build_dir: Path, model: str, *, check_model_output: str = "") -> StructuralReport:
    """Assemble the report from the artefacts omc left in ``build_dir``.

    Both files are optional in the sense that a partial report is still worth
    printing -- but a missing ``_info.json`` means code generation did not
    finish, which is worth saying rather than reporting an empty model.
    """
    info_path = build_dir / f"{model}_info.json"
    if not info_path.exists():
        raise FileNotFoundError(
            f"{info_path} is missing. omc writes it during code generation, so its "
            "absence means the build stopped in the front end -- read the compiler "
            "log rather than this report."
        )
    report = parse_info_json(json.loads(info_path.read_text(encoding="utf-8")))
    report.model = model

    init_path = build_dir / f"{model}_init.xml"
    if init_path.exists():
        report.events = parse_init_xml(init_path.read_text(encoding="utf-8"))
    if check_model_output:
        report.counts = parse_check_model(check_model_output)
    return report
