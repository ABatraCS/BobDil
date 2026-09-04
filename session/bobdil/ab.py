"""Paired and blind A/B -- the two things that turn a rig into an instrument.

**Paired A/B** (architecture.md 1.9) replays one recorded lap against two setups
and diffs the results. Its entire value comes from one property: the driver's
lap-to-lap variance, which is the dominant noise source in any subjective
comparison, is *identically zero* between the two runs because the inputs are
the same bytes. That property is fragile, so most of this module is the set of
checks that refuse to report a difference it cannot attribute to the setup.

**Blind A/B** (architecture.md 8) is the other half, and it is the one that
decides whether the rig is good enough to justify a change to the car. A driver
who cannot pick the setup above chance has told you the rig is not sensitive
enough yet -- regardless of how good the timing numbers look. The scoring here
is deliberately unflattering: it reports a p-value against fair coin-flipping,
and a hit rate that does not clear it is reported as no result rather than as a
weak one.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field

from .generated.frames import VehicleState
from .recording import Recording

#: Fields that describe the machine a run happened on rather than the run.
#: Comparing them makes every A/B report a difference and buries the real one.
MACHINE_FIELDS = frozenset(
    {
        "host_time_ns",
        "rtf",
        "step_time_us",
        "step_time_p99_us",
        "deadline_misses",
    }
)

#: Fields that are bookkeeping: identical by construction in a paired replay,
#: and meaningless as a "difference" if they ever are not.
BOOKKEEPING_FIELDS = frozenset({"sim_time", "step_index", "kernel_id"})

#: The driver's three commands. These must be bit-identical between the two
#: runs, and that is the check the whole method rests on.
INPUT_FIELDS = (
    "steering_angle_command",
    "accelerator_pedal_command",
    "brake_pedal_command",
)

#: Below this a difference is floating-point noise in the last bits, not physics.
NEGLIGIBLE = 1e-12

#: The conventional threshold. Stated as a constant because a blind A/B that
#: moves its own bar afterwards is not a blind A/B.
SIGNIFICANCE = 0.05


class NotComparable(ValueError):
    """These two runs cannot be diffed, and why. Never downgraded to a warning."""


@dataclass(frozen=True)
class SignalDifference:
    signal: str
    unit: str
    max_abs: float
    rms: float
    at_time_s: float
    mean_a: float
    mean_b: float

    def describe(self) -> str:
        return (
            f"{self.signal:<24} max {self.max_abs:>12.6g} {self.unit:<7} "
            f"rms {self.rms:>12.6g}   at t={self.at_time_s:.3f} s   "
            f"mean {self.mean_a:.4g} -> {self.mean_b:.4g}"
        )


@dataclass
class Comparison:
    label_a: str
    label_b: str
    frames: int
    duration_s: float
    ranked: list[SignalDifference] = field(default_factory=list)
    fault_flags_a: int = 0
    fault_flags_b: int = 0

    @property
    def identical(self) -> bool:
        return not self.ranked

    @property
    def worst(self) -> SignalDifference | None:
        return self.ranked[0] if self.ranked else None

    def describe(self) -> str:
        lines = [
            f"paired A/B: {self.label_a} vs {self.label_b}",
            f"  {self.frames} frames, {self.duration_s:.3f} s, identical driver inputs",
            "",
        ]
        if self.identical:
            lines.append(
                "  No signal differs. Either the setup change does nothing to this "
                "model, or it was not applied -- check that the parameter is exported "
                "as an FMI tunable and not compiled in as a constant."
            )
            return "\n".join(lines)
        lines.append(f"  {len(self.ranked)} signal(s) changed, largest effect first:")
        lines.extend(f"    {difference.describe()}" for difference in self.ranked)
        if self.fault_flags_a != self.fault_flags_b:
            lines.append("")
            lines.append(
                f"  fault flags differ: {self.fault_flags_a:#x} vs {self.fault_flags_b:#x}. "
                "One of these runs hit something the other did not; read that before "
                "reading the numbers above."
            )
        lines.append("")
        lines.append(
            "  These are differences between two *simulations*, not between two cars. "
            "Whether a driver can feel any of it is the question blind A/B answers."
        )
        return "\n".join(lines)


def _units() -> dict[str, str]:
    return dict(zip(VehicleState.FIELDS, VehicleState.UNITS, strict=True))


def _check_comparable(a: Recording, b: Recording, label_a: str, label_b: str) -> None:
    if len(a.frames) != len(b.frames):
        raise NotComparable(
            f"{label_a} has {len(a.frames)} frames and {label_b} has {len(b.frames)}: "
            "different length runs cannot be paired frame by frame."
        )
    if a.meta.step_dt != b.meta.step_dt:
        raise NotComparable(
            f"different step sizes ({a.meta.step_dt} vs {b.meta.step_dt}). The "
            "integrator's own error changes with dt, so any difference would be "
            "partly the solver's and there is no way to tell how much."
        )
    if a.meta.kernel_id != b.meta.kernel_id:
        raise NotComparable(
            f"different kernels (id {a.meta.kernel_id} vs {b.meta.kernel_id}). That is "
            "a fidelity comparison, not a setup comparison -- see architecture.md 8."
        )
    if a.meta.vehicle_hash != b.meta.vehicle_hash:
        raise NotComparable(
            f"different vehicles ({a.meta.vehicle_hash:#018x} vs "
            f"{b.meta.vehicle_hash:#018x}). Diffing two cars says nothing about a "
            "setup change."
        )

    for name in INPUT_FIELDS:
        for index, (frame_a, frame_b) in enumerate(zip(a.frames, b.frames, strict=True)):
            if getattr(frame_a, name) != getattr(frame_b, name):
                raise NotComparable(
                    f"the driver input {name} differs at frame {index} "
                    f"({getattr(frame_a, name)} vs {getattr(frame_b, name)}).\n"
                    "Paired A/B is only worth anything because both runs saw the same "
                    "inputs; two runs a driver drove separately are a comparison of the "
                    "driver. Replay one recording against both setups instead."
                )


def compare(a: Recording, b: Recording, *, label_a: str, label_b: str) -> Comparison:
    """Diff two paired runs, or explain why they cannot be diffed."""
    _check_comparable(a, b, label_a, label_b)

    units = _units()
    comparison = Comparison(
        label_a=label_a,
        label_b=label_b,
        frames=len(a.frames),
        duration_s=len(a.frames) * a.meta.step_dt,
        fault_flags_a=a.frames[-1].fault_flags if a.frames else 0,
        fault_flags_b=b.frames[-1].fault_flags if b.frames else 0,
    )

    compared = [
        name
        for name in VehicleState.FIELDS
        if name not in MACHINE_FIELDS
        and name not in BOOKKEEPING_FIELDS
        and name not in INPUT_FIELDS
        and name != "fault_flags"
    ]
    for name in compared:
        left = [float(getattr(frame, name)) for frame in a.frames]
        right = [float(getattr(frame, name)) for frame in b.frames]
        deltas = [x - y for x, y in zip(left, right, strict=True)]
        peak = max((abs(delta), index) for index, delta in enumerate(deltas)) if deltas else (0, 0)
        if peak[0] <= NEGLIGIBLE:
            continue
        comparison.ranked.append(
            SignalDifference(
                signal=name,
                unit=units.get(name, "-"),
                max_abs=peak[0],
                rms=math.sqrt(sum(delta * delta for delta in deltas) / len(deltas)),
                # The frame's own sim_time, not index*dt: a report that points at
                # a moment must point at the moment the plant thinks it is, which
                # is what any other view of the same recording will show.
                at_time_s=a.frames[peak[1]].sim_time,
                mean_a=sum(left) / len(left),
                mean_b=sum(right) / len(right),
            )
        )

    # Ranked by RMS rather than by peak: a single-frame spike is usually an
    # event boundary, while a sustained offset is the setup actually doing
    # something, and it is the second that a driver could feel.
    comparison.ranked.sort(key=lambda difference: difference.rms, reverse=True)
    return comparison


# --- blind A/B -------------------------------------------------------------


@dataclass(frozen=True)
class BlindScore:
    trials: int
    hits: int
    p_value: float

    @property
    def rate(self) -> float:
        return self.hits / self.trials if self.trials else 0.0

    @property
    def above_chance(self) -> bool:
        return self.p_value < SIGNIFICANCE

    def verdict(self) -> str:
        if self.above_chance:
            return (
                f"{self.hits}/{self.trials} correct ({self.rate:.0%}), p={self.p_value:.4f}. "
                "The driver can tell these setups apart, so the rig is sensitive enough "
                "to support a decision between them."
            )
        return (
            f"{self.hits}/{self.trials} correct ({self.rate:.0%}), p={self.p_value:.4f}. "
            "That is not distinguishable from guessing: this driver cannot tell these "
            "two setups apart on this rig. Read it as a statement about the rig and the "
            "size of the change, not about the driver -- and do not use BobDil to "
            "justify this change until it comes back above chance "
            "(architecture.md 8)."
        )


def blind_sequence(trials: int, *, seed: int | None = None) -> list[str]:
    """A balanced, shuffled run order of ``"A"`` and ``"B"``.

    Balanced rather than independently random on purpose. With independent coin
    flips a driver who guesses whichever setup came up more often scores above
    chance without feeling anything, and the binomial test below would believe
    them. ``seed`` makes a session reproducible for an experimenter who needs to
    audit it afterwards -- it must not be shown to the driver.
    """
    if trials <= 0 or trials % 2:
        raise ValueError(
            f"{trials} trials cannot be balanced; use an even, positive number so each "
            "setup is presented the same number of times."
        )
    order = ["A"] * (trials // 2) + ["B"] * (trials // 2)
    random.Random(seed).shuffle(order)
    return order


def _binomial_tail(hits: int, trials: int) -> float:
    """P(at least ``hits`` correct | fair guessing). Exact, no dependency."""
    total = sum(math.comb(trials, k) for k in range(hits, trials + 1))
    return total / (2**trials)


def score_blind(truth: list[str], guesses: list[str]) -> BlindScore:
    """Score a blind session against the null hypothesis that the driver guessed."""
    if len(truth) != len(guesses):
        raise ValueError(
            f"{len(truth)} trials were run but {len(guesses)} guesses were given; "
            "a missing guess is not a wrong one and must not be scored as one."
        )
    hits = sum(1 for actual, guess in zip(truth, guesses, strict=True) if actual == guess)
    # One-sided: the question is whether the driver did *better* than chance.
    # A driver who scores far below chance is also detecting something, but that
    # is a labelling error in the session, not a finding about the car.
    return BlindScore(trials=len(truth), hits=hits, p_value=_binomial_tail(hits, len(truth)))
