"""The largest explicit step that is stable, and which state sets it.

Step timing and step *stability* are different failures and are often confused.
A model can evaluate in 40 microseconds and still be undrivable at 1 kHz,
because an explicit integrator diverges once the step exceeds the reciprocal of
the fastest mode. No amount of CPU fixes that: it is a property of the physics,
and the only cures are a smaller step, an implicit method, or removing the fast
mode from the model (architecture.md 1.5, which is what VehicleRT is for).

The bound is computed exactly rather than from a rule of thumb. For an explicit
Runge-Kutta method the numerical solution of ``x' = lambda x`` is multiplied by
``R(h*lambda)`` each step, where ``R`` is the method's stability polynomial, so
the step is stable exactly while ``|R(h*lambda)| <= 1`` for every eigenvalue.
That is a scalar condition per eigenvalue, and bisection on ``h`` solves it
without assuming the eigenvalues are real -- which matters here, because a
vehicle's roll and yaw modes are complex pairs and the real-axis rule of thumb
is wrong for them by up to 40 percent, in the unsafe direction for Euler.

The state matrix itself comes from ``omc``'s ``linearize()``, parsed here.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass, field

import numpy as np

#: Stability polynomial coefficients, ``R(z) = sum(c[k] * z**k)``, matching the
#: methods in ``kernel/src/plant/integrator.rs``. They are the truncated
#: exponential series to the method's order, which is what an explicit RK method
#: of that order reproduces.
STABILITY_POLYNOMIALS: dict[str, tuple[float, ...]] = {
    "euler": (1.0, 1.0),
    "midpoint": (1.0, 1.0, 1.0 / 2.0),
    "rk4": (1.0, 1.0, 1.0 / 2.0, 1.0 / 6.0, 1.0 / 24.0),
}

#: Anything at or below this magnitude is an algebraic or drifting mode, not a
#: dynamic one, and dividing by it produces a meaningless step bound.
NEGLIGIBLE_EIGENVALUE = 1e-9

#: Bisection resolution for the step bound, as a relative tolerance.
BISECTION_TOLERANCE = 1e-9

#: Below this the amplification test stops meaning anything: for a purely
#: imaginary eigenvalue, forward Euler's |1 + i*h*w| is mathematically greater
#: than one for every positive h, but rounds to exactly one once (h*w)^2 falls
#: under the double-precision epsilon. Bisection then "finds" a stable step of a
#: few nanoseconds, which is a floating-point artefact and not a step anybody
#: could take. A bound this small is reported as no bound at all.
MINIMUM_USEFUL_STEP_S = 1e-7


class LinearizationError(ValueError):
    """omc produced no usable linear model. Always reported, never guessed past."""


@dataclass
class LinearModel:
    """``der(x) = A x``, at one operating point, with the states named."""

    name: str
    a: np.ndarray
    state_names: list[str]
    operating_point: dict[str, float] = field(default_factory=dict)

    @property
    def order(self) -> int:
        return int(self.a.shape[0])


@dataclass
class Sweep:
    """What the eigenvalues of one operating point imply for a fixed step."""

    model: str
    method: str
    eigenvalues: np.ndarray
    max_step_s: float
    binding_eigenvalue: complex
    binding_states: list[str]
    unstable_states: list[str]
    stiffness_ratio: float
    operating_point: dict[str, float] = field(default_factory=dict)

    def margin(self, step_s: float) -> float:
        """How many times larger the stable step is than the one we intend to use."""
        return math.inf if step_s <= 0 else self.max_step_s / step_s

    def describe(self, step_s: float) -> str:
        point = ", ".join(f"{k}={v:g}" for k, v in self.operating_point.items())
        lines = [f"  {self.model}{f'  [{point}]' if point else ''}"]
        if self.unstable_states:
            names = ", ".join(self.unstable_states)
            verb = "has" if len(self.unstable_states) == 1 else "have"
            # The step bound is deliberately not printed here. For a divergent
            # mode there is no step that makes it converge, so a number would
            # read as "use a smaller step" -- which is the wrong fix and would
            # send someone tuning the integrator instead of the model.
            lines.append(
                f"    UNSTABLE: {names} {verb} an eigenvalue with a positive real part "
                "at this operating point. No step size makes a divergent model converge; "
                "this is a modelling result, not a timing one, and there is no step "
                "bound to report until it is fixed."
            )
            return "\n".join(lines)
        if math.isinf(self.max_step_s):
            lines.append("    no dynamic modes -- this operating point imposes no step bound")
            return "\n".join(lines)
        margin = self.margin(step_s)
        lines.append(
            f"    max stable step   {self.max_step_s * 1e3:.3f} ms "
            f"({margin:.1f}x the {step_s * 1e3:.1f} ms budget)"
            f"{'' if margin >= 1.0 else '   TOO SMALL'}"
        )
        lines.append(
            f"    set by            {', '.join(self.binding_states) or '(unattributed)'} "
            f"at lambda = {self.binding_eigenvalue.real:.1f}"
            f"{self.binding_eigenvalue.imag:+.1f}j"
        )
        lines.append(f"    stiffness ratio   {self.stiffness_ratio:.0f}")
        return "\n".join(lines)


_MATRIX = re.compile(r"parameter Real A\[n, ?n\]\s*=\s*\[(.*?)\];", re.DOTALL)
_STATE_NAME = re.compile(r"Real '([^']+)'\s*=\s*x\[(\d+)\];")
_MODEL_NAME = re.compile(r'model linearized_model\s+"([^"]*)"')


def parse_linearized_model(text: str) -> LinearModel:
    """Read ``linearized_model.mo``, the Modelica file ``linearize()`` writes.

    Parsed rather than executed. The alternative -- asking omc to dump Python
    and importing it -- would mean running compiler-generated code to read a
    number out of it, which is a much larger trust surface for no gain.
    """
    matrix = _MATRIX.search(text)
    if matrix is None:
        raise LinearizationError(
            "no state matrix in omc's linearized model. That usually means the "
            "simulation did not reach the linearization time -- read the omc log "
            "for the failure rather than treating this as a model with no dynamics."
        )
    rows = [
        [float(value) for value in row.replace("\t", " ").split(",")]
        for row in matrix.group(1).strip().split(";")
        if row.strip()
    ]
    a = np.array(rows, dtype=float)
    if a.ndim != 2 or a.shape[0] != a.shape[1]:
        raise LinearizationError(f"state matrix is {a.shape}, which is not square")

    names = ["" for _ in range(a.shape[0])]
    for match in _STATE_NAME.finditer(text):
        index = int(match.group(2)) - 1
        if 0 <= index < len(names):
            # omc prefixes every state with the component it linearized through
            # ('x_plant.vx'). The prefix is an artefact of the wrapper model this
            # tool generates, and keeping it makes every name in the report
            # unreadable.
            names[index] = match.group(1).removeprefix("x_").split(".", 1)[-1]
    for index, name in enumerate(names):
        if not name:
            names[index] = f"x[{index + 1}]"

    model_name = _MODEL_NAME.search(text)
    return LinearModel(
        name=model_name.group(1) if model_name else "linearized_model",
        a=a,
        state_names=names,
    )


def _amplification(z: complex, coefficients: tuple[float, ...]) -> float:
    total = 0j
    for power, coefficient in enumerate(coefficients):
        total += coefficient * z**power
    return abs(total)


def max_stable_step(eigenvalues: np.ndarray, method: str) -> float:
    """The largest ``h`` for which every eigenvalue stays inside the stability region.

    Returns ``inf`` when there are no dynamic modes (a purely algebraic model
    imposes no bound) and ``0.0`` when some eigenvalue is outside the region for
    every positive step -- which is what forward Euler does to a purely
    imaginary pair, and is a real answer rather than an error.
    """
    try:
        coefficients = STABILITY_POLYNOMIALS[method]
    except KeyError:
        raise ValueError(
            f"unknown method {method!r}; expected one of {sorted(STABILITY_POLYNOMIALS)}"
        ) from None

    dynamic = [complex(value) for value in eigenvalues if abs(value) > NEGLIGIBLE_EIGENVALUE]
    if not dynamic:
        return math.inf

    def stable(step: float) -> bool:
        return all(_amplification(step * value, coefficients) <= 1.0 for value in dynamic)

    # Start from the real-axis rule of thumb and bracket outwards, so the search
    # is bounded even when the true limit is far from it.
    upper = 4.0 / max(abs(value) for value in dynamic)
    while stable(upper):
        upper *= 2.0
        if upper > 1e6:
            return math.inf
    lower = 0.0
    for _ in range(200):
        middle = 0.5 * (lower + upper)
        if stable(middle):
            lower = middle
        else:
            upper = middle
        if upper - lower <= BISECTION_TOLERANCE * max(upper, 1e-12):
            break
    return lower if lower > MINIMUM_USEFUL_STEP_S else 0.0


def modal_participation(a: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Eigenvalues, and how much each state takes part in each mode.

    Participation factors are ``p[i, k] = |V[i, k]| * |W[k, i]|`` for right
    eigenvectors ``V`` and ``W = inv(V)``, normalised per mode. This is what
    turns "the step is 0.3 ms" into "the step is 0.3 ms *because of the roll
    damper*", which is the half of the answer somebody can act on.
    """
    values, vectors = np.linalg.eig(a)
    try:
        left = np.linalg.inv(vectors)
        factors = np.abs(vectors) * np.abs(left.T)
    except np.linalg.LinAlgError:
        # A defective matrix has eigenvectors that do not span, so there is no
        # left inverse. Fall back to right-eigenvector magnitude, which still
        # ranks the states correctly, rather than failing the whole sweep.
        factors = np.abs(vectors)
    columns = factors.sum(axis=0)
    columns[columns == 0] = 1.0
    return factors / columns, values


def analyse(model: LinearModel, method: str, *, participation_floor: float = 0.2) -> Sweep:
    """Eigen-decompose one operating point and attribute the binding mode."""
    factors, values = modal_participation(model.a)

    unstable = []
    for index, value in enumerate(values):
        if value.real > NEGLIGIBLE_EIGENVALUE:
            unstable.extend(
                model.state_names[state]
                for state in np.argsort(factors[:, index])[::-1]
                if factors[state, index] >= participation_floor
            )

    step = max_stable_step(values, method)
    dynamic = [
        (abs(value), index)
        for index, value in enumerate(values)
        if abs(value) > NEGLIGIBLE_EIGENVALUE
    ]
    if dynamic:
        _, fastest = max(dynamic)
        slowest = min(magnitude for magnitude, _ in dynamic)
        binding = complex(values[fastest])
        binding_states = [
            model.state_names[state]
            for state in np.argsort(factors[:, fastest])[::-1]
            if factors[state, fastest] >= participation_floor
        ]
        ratio = abs(binding) / slowest
    else:
        binding, binding_states, ratio = 0j, [], 1.0

    return Sweep(
        model=model.name,
        method=method,
        eigenvalues=values,
        max_step_s=step,
        binding_eigenvalue=binding,
        binding_states=binding_states,
        unstable_states=sorted(set(unstable)),
        stiffness_ratio=ratio,
        operating_point=model.operating_point,
    )
