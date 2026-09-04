# BobDil

A driver-in-the-loop simulator over [BobLib](../BobLib)'s Modelica vehicle
model. A driver steers, brakes and accelerates; the same Modelica physics BobSim
uses offline is stepped at 1 kHz; and what comes back — the view, and the torque
in the steering wheel — is derived from that model rather than from a feel
curve someone tuned by hand.

It exists to answer one kind of question: *would a driver notice this change?*
Lap-time simulation already tells you whether a setup is faster. It cannot tell
you whether the car is drivable, and that is the question a rig answers.

> **Status.** Phases 1–5 and 8 of [`docs/architecture.md`](docs/architecture.md)
> are built and verified. Phase 0's tooling is complete but its headline
> measurement — the real vehicle's — needs the container. Phase 7 is half done:
> live tunables work in the kernel but are blocked on a BobLib change, and
> `vehicle.yml` → Modelica regeneration is not wired up. The reduced-order plant
> is *plausible, not correlated* against BobLib, and **no torque has ever been
> delivered to real hardware**.
> [`HANDOFF.md`](HANDOFF.md) is the honest list of what is done, what is left,
> and what is known to be missing. Read section 7 before quoting any of this as
> a claim about a car.

---

## Start here

```bash
make venv       # once, on a fresh machine
make doctor     # what is installed, what is missing, what each gap costs
make ci         # the whole gate: hermetic tests, then this machine's validations
make drive      # DRIVE IT (runs the safety self-test first, and refuses if it fails)
```

`make help` is the authoritative target list. Everything goes through the
makefile, which handles `PATH` and the venv itself.

### Two front doors

The **kernel** has its own command line and keeps it, because it is the thing
that must work when everything else is broken:

```bash
bobdil-kernel selftest      # SAFETY. Five checks. Run before a person touches a wheel.
bobdil-kernel bench         # can this machine hold the deadline?
bobdil-kernel run --torque-limit 4
```

The **session layer** is everything the kernel deliberately cannot do:

```bash
python -m bobdil doctor
python -m bobdil build vehicle          # compile BobLib to an FMU the kernel can load
python -m bobdil bench vehicle          # Phase 0: structure, stability, and timing
python -m bobdil drive --torque-limit 4
python -m bobdil replay lap.bdt
python -m bobdil ab paired lap.bdt -b front_arb_rate=45000
```

---

## How it is put together

```
     schema/bobdil_signals.yaml         one file. every signal that crosses a boundary.
                 |
                 |  codegen/  ->  Rust struct, C header, Python dataclass,
                 |                GDScript reader, .proto, FMI vref table
                 v
   +---------------------------+        +--------------------------+
   |  session/  (Python)       |        |  view/  (Godot 4)        |
   |  compile, configure, A/B  |        |  driver POV, read-only   |
   |  never in the loop        |        |  by construction         |
   +-------------+-------------+        +------------+-------------+
                 | manifest (key=value)              | seqlock, /dev/shm
                 v                                   ^
        +--------------------------------------------+-------+
        |  kernel/  (Rust, no external crates)               |
        |    HidThread  -> seqlock ->  StepThread            |
        |                              |  plant/   physics   |
        |                              |  io/      wheel+FFB |
        |                              `-> ring -> Telemetry |
        +----------------------------------------------------+
```

Three ideas carry most of the weight:

**The schema is the single source of truth.** One YAML file emits the packed
struct for Rust, the C header, the Python dataclass, the GDScript byte offsets
and the protobuf definitions. A `layout_hash` sits in every shared-memory
header, so a reader built against a different schema *refuses to attach* rather
than misinterpreting the bytes — a failure that would otherwise surface as
physics that looks subtly wrong.

**The kernel is decoupled from everything.** `plant/` does not know a wheel
exists; `io/` does not know what a tyre is; the view has no write path to
physics. There are three interchangeable plant kernels behind one trait, and
which one runs is decided once at session start, on measured data, never
mid-drive.

**Model Exchange, never Co-Simulation.** A CS FMU owns its solver: `fmi2DoStep`
is unbounded work with no way to impose a deadline, so it cannot be made
real-time safe at all. BobDil imports the FMU's derivatives and owns the
integrator — fixed-step RK4, bounded work per step, by construction.

Read [`docs/architecture.md`](docs/architecture.md) for the reasoning, including
the parts of the original spec that did not survive contact (§5).

---

## Safety

A direct-drive wheel delivers enough torque to break a wrist. This is treated as
safety-critical code, not as a rendering concern:

- Two independent non-finite barriers: the plant guards its own output, and the
  feedback chain guards again before anything reaches a device.
- A watchdog ramps torque to zero when the step thread stops meeting deadlines,
  and the device is left slack on shutdown — including on `SIGKILL`.
- `--torque-limit` defaults to 8 N·m and is opt-in-raised, never opt-out-lowered.
- `make selftest` exercises all of it adversarially. `python -m bobdil drive`
  runs it first and refuses to drive if it fails.

**Set `--torque-limit` below your wheel's capability before anyone drives.**

---

## Phase 0 — the question that comes before the schedule

Nothing in BobLib or BobSim measures whether the vehicle model can be stepped in
real time, and every estimate downstream is unfounded until it is. `rt_bench`
answers it in three separate parts, because they fail for different reasons:

```bash
make rt-bench            # structure + stability, on BobDil's fixture model
make rt-bench-vehicle    # the same, on the real car, in the container
make bench               # step timing, on this machine
```

- **Structure** — non-linear algebraic systems, state events, dynamic state
  selection: the things that make work per step unbounded. A property of the
  model; travels between machines.
- **Stability** — the largest explicit step that is numerically stable, at four
  representative operating points, and *which state sets it*. A property of the
  physics; travels between machines. A model can evaluate in 40 µs and still be
  undrivable at 1 kHz.
- **Timing** — measured by the kernel that will actually run it. A property of
  *this box*, and it does not travel at all.

Gate: p99.9 step time under 500 µs, half the 1 ms budget, leaving room for OS
jitter.

---

## The features that make it a tool

- **Deterministic replay.** Two replays of one recording produce bit-identical
  states, and every replay proves it before reporting anything.
- **Paired A/B.** Replay one recorded lap against two setups and diff them. The
  driver's lap-to-lap variance — the dominant noise source in any subjective
  comparison — is identically zero between the two runs, and the tool refuses to
  report a difference if the inputs were not in fact identical.
- **Blind A/B.** A balanced, sealed run order and a binomial score against fair
  guessing. A driver who cannot pick the setup above chance has told you the rig
  is not sensitive enough to justify the change — regardless of how good the
  timing numbers look.
- **Live tunables.** Kernel-side and schema-side complete; blocked on a BobLib
  change for the parameters currently compiled in as constants. `python -m
  bobdil build` names exactly which ones, every time.

---

## Requirements

| To do this | You need |
| --- | --- |
| Drive the built-in reduced kernel | nothing but the repo |
| Drive a prebuilt FMU | nothing but the repo |
| Build BobDil's fixture FMU | any `omc` |
| Build the real BobLib vehicle | `make omc-image` (MSL 4.1.0 + VehicleInterfaces 2.0.2) |
| Use a wheel and pedals | SDL3, and `make build` rather than `make build-headless` |
| See the driver POV | Godot 4 |

BobLib and BobSim are **read-only inputs**, located by `$BOBDIL_BOBLIB` /
`$BOBDIL_BOBSIM` or as sibling checkouts and mounted `:ro` in the containers.
Nothing here writes to either.

---

## Documentation

| Read | For |
| --- | --- |
| [`docs/architecture.md`](docs/architecture.md) | the design, and why each decision went the way it did |
| [`HANDOFF.md`](HANDOFF.md) | what is built, what is left, what was learned the hard way |
| [`AGENTS.md`](AGENTS.md) | the rules that are load-bearing when changing this repo |
| `make help` | the authoritative list of what you can actually run |
