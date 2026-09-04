"""The four operating points Phase 0 measures at, and why each one is in the set.

A single linearization is not a viability answer. A vehicle's fastest mode moves
with speed, load and slip: the roll damper dominates at rest, the tyre relaxation
lengths dominate under load, and the slip denominators blow up near zero speed.
Measuring at one point and quoting it as *the* step bound is the mistake this
list exists to prevent.

The four are architecture.md 6's, and the reasoning behind each is recorded here
rather than in the report, because the report prints numbers and this is the part
that says what they mean.
"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class OperatingPoint:
    """One point to linearize at: constant driver inputs, held until settled."""

    name: str
    why: str
    inputs: dict[str, float]
    #: State start values, applied as modifiers on the wrapped model.
    starts: dict[str, float] = field(default_factory=dict)
    #: How long to simulate before linearizing. Long enough to settle, short
    #: enough that a badly-behaved model fails quickly.
    settle_s: float = 6.0


#: The driver inputs every BobDil plant exposes. Named here rather than imported
#: from the schema because these are the *model's* names, and the schema's job is
#: the wire format; the manifest is where the two are reconciled.
STEER = "steeringAngleCommand"
THROTTLE = "acceleratorPedalCommand"
BRAKE = "brakePedalCommand"

#: A *role*, not a parameter name. An operating point says "start this at
#: 18 m/s"; which parameter carries that is the model's business. Writing the
#: fixture's own `vx` into the points below made every sweep of the real vehicle
#: fail with `Modified element vx not found in class VehicleFMI` -- four times,
#: after a twenty-minute translate, with the structural half already correct.
SPEED = "initial_speed"

#: How a model takes a start value. A parameter is bound directly; a state needs
#: a `start` modifier.
PARAMETER = "parameter"
STATE = "state"

#: Role -> how this model takes it. Kept explicit rather than guessed: a wrong
#: guess here does not fail, it linearizes at the wrong operating point and
#: reports a step bound that looks entirely reasonable.
#:
#: The *kind* matters as much as the name. A state is set with a `start`
#: modifier and a parameter with a plain binding; using the wrong one is either
#: a translation error or, worse, accepted and ignored.
START_BINDINGS: dict[str, dict[str, tuple[str, str]]] = {
    # `parameter SI.Velocity initialVel` on Templates.FMI.BaseVehicleFMI feeds
    # chassis.initialLongitudinalVelocity and the wheel spin-up, so setting it
    # starts the whole driveline consistently rather than just the body.
    "BobLib.Experiments.Standards.VehicleFMI": {SPEED: ("initialVel", PARAMETER)},
    "BobDil.Experiments.DilSmokePlant": {SPEED: ("vx", STATE)},
}


class UnboundModel(KeyError):
    """The model has no role->parameter binding, so the sweep must not guess."""


def bind_starts(model: str, starts: dict[str, float]) -> list[str]:
    """Modelica modifier fragments that put ``model`` at this operating point.

    Refuses rather than defaulting. A sweep that silently drops the start value
    still produces eigenvalues -- of the model at rest -- and nothing in the
    output would say the speed never took effect.
    """
    if not starts:
        return []
    binding = START_BINDINGS.get(model)
    if binding is None:
        raise UnboundModel(
            f"{model} has no entry in START_BINDINGS, so rt_bench cannot set an "
            f"operating point on it. Add one mapping {sorted(starts)} to the "
            "parameter(s) this model exposes, and say whether each is a "
            f"{PARAMETER!r} or a {STATE!r}."
        )
    fragments: list[str] = []
    for role, value in starts.items():
        bound = binding.get(role)
        if bound is None:
            raise UnboundModel(f"{model} has no parameter bound to role {role!r}")
        name, kind = bound
        fragments.append(
            f"{name} = {value!r}" if kind == PARAMETER else f"{name}(start = {value!r})"
        )
    return fragments


STANDING_START = OperatingPoint(
    name="standing-start",
    why=(
        "A DIL session always begins at rest, and transient slip divides by "
        "velocity. If the model is going to be stiff anywhere it is here, and it "
        "is the one operating point every single session passes through."
    ),
    inputs={STEER: 0.0, THROTTLE: 1.0, BRAKE: 0.0},
    starts={SPEED: 0.0},
    settle_s=0.5,
)

STEP_STEER = OperatingPoint(
    name="step-steer",
    why=(
        "The transient the yaw and roll modes are excited by, and the maneuver "
        "the handwheel torque cue is judged on. The fastest mode of the "
        "steering and roll subsystem shows up here."
    ),
    inputs={STEER: 0.6, THROTTLE: 0.25, BRAKE: 0.0},
    starts={SPEED: 18.0},
)

THRESHOLD_BRAKING = OperatingPoint(
    name="threshold-braking",
    why=(
        "Peak longitudinal load transfer, so the vertical loads -- and with them "
        "the tyre stiffnesses that set the fast modes -- are at their extreme."
    ),
    inputs={STEER: 0.0, THROTTLE: 0.0, BRAKE: 0.9},
    starts={SPEED: 25.0},
)

SKIDPAD = OperatingPoint(
    name="skidpad",
    why=(
        "Sustained steady-state cornering: the one point where the model is "
        "genuinely at equilibrium, so its eigenvalues are the cleanest reading "
        "of the underlying dynamics rather than of a transient."
    ),
    inputs={STEER: 0.35, THROTTLE: 0.30, BRAKE: 0.0},
    starts={SPEED: 12.0},
)

#: The default sweep. Ordered so the cheapest and most diagnostic runs first.
DEFAULT: tuple[OperatingPoint, ...] = (
    STANDING_START,
    SKIDPAD,
    STEP_STEER,
    THRESHOLD_BRAKING,
)


def by_name(name: str) -> OperatingPoint:
    for point in DEFAULT:
        if point.name == name:
            return point
    raise KeyError(f"unknown operating point {name!r}; have {[p.name for p in DEFAULT]}")
