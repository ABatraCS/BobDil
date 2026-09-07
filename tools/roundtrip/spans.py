"""Turn trace records into spans, and join the two halves of the round trip.

The step records and the device records arrive as two independent streams --
two threads, two rings -- and the thing worth knowing lives in the join between
them: a step that was fast and a torque that arrived late are the same round
trip, and only one of them is visible in anything the kernel publishes today.

The join key is `t_command_stamp`, which is the exact value the step thread
wrote into the `FfbCommand` the device thread later picked up.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from bobdil.generated.frames import TraceDevice, TraceFlags, TraceStep

#: Phase name -> (start field, end field). The order here is the order they run
#: in, and it is the order the flamegraph draws them in.
PHASES: tuple[tuple[str, str, str], ...] = (
    ("read", "t_step_start", "t_after_read"),
    ("shape", "t_after_read", "t_after_shape"),
    ("plant", "t_after_shape", "t_after_plant"),
    ("pose", "t_after_plant", "t_command_stamp"),
    ("ffb", "t_command_stamp", "t_after_ffb"),
    ("publish", "t_after_ffb", "t_after_publish"),
)


@dataclass(frozen=True)
class Delivery:
    """The device leg: what happened to one step's torque."""

    #: Publish to the device thread picking it up.
    pickup_delay: int
    #: The cost of handing it to the device.
    apply_cost: int


@dataclass
class StepSpans:
    step_index: int
    #: `t_step_start`, kept so a viewer can place the span on a real timeline.
    absolute_start: int
    #: Which sample this step consumed, so a re-used one is identifiable in a
    #: viewer rather than only countable in a summary.
    input_sample_index: int
    #: The consumed sample's age when the step began.
    input_age: int
    phases: dict[str, int]
    total: int
    #: Step start to the stamp the torque command carried. The device leg is
    #: measured from that stamp, not from the end of the step, so this is the
    #: part of the step that precedes the hand-off -- adding `total` instead
    #: would count ffb and publish twice.
    to_command: int = 0
    #: None when no fresh command was delivered for this step. A stale or
    #: missing command is a broken leg, and giving it a duration would put a
    #: safety condition into a latency statistic.
    delivery: Delivery | None = None

    @property
    def round_trip(self) -> int | None:
        """Sample taken, to torque delivered. None when the leg is broken."""
        if self.delivery is None:
            return None
        return (
            self.input_age + self.to_command + self.delivery.pickup_delay + self.delivery.apply_cost
        )


@dataclass
class Derived:
    steps: list[StepSpans] = field(default_factory=list)
    #: Steps that ran on a sample a previous step had already used. The loop was
    #: on time; the driver's input was not there yet.
    reused_inputs: int = 0
    stale_commands: int = 0
    missing_commands: int = 0
    apply_failures: int = 0
    #: Device records that matched no step in this trace. Expected at the edges
    #: (the trace starts and stops mid-flight); a large count means the join key
    #: is wrong and every number here should be distrusted.
    unjoined_devices: int = 0

    def series(self, name: str) -> list[int]:
        """Every value of one measurement, for a percentile."""
        if name == "input_age":
            return [step.input_age for step in self.steps]
        if name == "total":
            return [step.total for step in self.steps]
        if name == "pickup":
            return [s.delivery.pickup_delay for s in self.steps if s.delivery]
        if name == "apply":
            return [s.delivery.apply_cost for s in self.steps if s.delivery]
        if name == "round_trip":
            return [s.round_trip for s in self.steps if s.round_trip is not None]
        return [step.phases[name] for step in self.steps if name in step.phases]

    def percentile(self, name: str, quantile: float) -> int | None:
        """None rather than zero when there is nothing to measure -- a zero here
        would read as a fast round trip rather than as no round trip at all."""
        values = sorted(self.series(name))
        if not values:
            return None
        index = min(len(values) - 1, int(round(quantile * (len(values) - 1))))
        return values[index]


def derive(steps: list[TraceStep], devices: list[TraceDevice]) -> Derived:
    derived = Derived()

    # Only a fresh command is a delivery. The others are counted, and counted
    # separately, because they fail for different reasons and a rig that is
    # dropping commands is a different problem from one that is slow.
    deliveries: dict[int, TraceDevice] = {}
    for record in devices:
        if record.flags & TraceFlags.APPLY_FAILED:
            derived.apply_failures += 1
        if record.flags & TraceFlags.COMMAND_STALE:
            derived.stale_commands += 1
            continue
        if record.flags & TraceFlags.COMMAND_MISSING:
            derived.missing_commands += 1
            continue
        if record.flags & TraceFlags.COMMAND_FRESH:
            # A command is picked up more than once when the device thread polls
            # faster than the step thread publishes. The first pickup is the one
            # the driver felt.
            existing = deliveries.get(record.command_host_time_ns)
            if existing is None or record.t_pickup < existing.t_pickup:
                deliveries[record.command_host_time_ns] = record

    seen_samples: set[int] = set()
    joined: set[int] = set()
    for step in steps:
        if step.input_sample_index in seen_samples:
            derived.reused_inputs += 1
        seen_samples.add(step.input_sample_index)

        phases = {
            name: max(0, getattr(step, end) - getattr(step, start)) for name, start, end in PHASES
        }
        spans = StepSpans(
            step_index=step.step_index,
            absolute_start=step.t_step_start,
            input_sample_index=step.input_sample_index,
            input_age=max(0, step.t_step_start - step.input_host_time_ns),
            phases=phases,
            total=max(0, step.t_after_publish - step.t_step_start),
            to_command=max(0, step.t_command_stamp - step.t_step_start),
        )

        record = deliveries.get(step.t_command_stamp)
        if record is not None:
            joined.add(step.t_command_stamp)
            spans.delivery = Delivery(
                pickup_delay=max(0, record.t_pickup - step.t_command_stamp),
                apply_cost=max(0, record.t_after_apply - record.t_pickup),
            )
        derived.steps.append(spans)

    derived.unjoined_devices = len(deliveries) - len(joined)
    return derived
