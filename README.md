# BobDil

A driver-in-the-loop simulator over [BobLib](../BobLib)'s Modelica vehicle
model. A driver steers, brakes and accelerates; the same Modelica physics BobSim
uses offline is stepped at 1 kHz; and what comes back — the view, and the torque
in the steering wheel — is derived from that model rather than from a feel
curve someone tuned by hand.

It exists to answer one kind of question: *would a driver notice this change?*
Lap-time simulation already tells you whether a setup is faster. It cannot tell
you whether the car is drivable, and that is the question a rig answers.

> **Status.** Phases 1, 2, 5 and 8 of
> [`docs/architecture.md`](docs/architecture.md) are built and verified.
> Phases 3 and 4 are built but **not verified against their own acceptance
> criteria**: no torque has ever been delivered to real hardware, and the real
> vehicle model does not currently step at 1 kHz — it compiles and loads, then
> fails on the first step, so a driver has never driven it. Phase 0's tooling
> is complete and its headline measurement is in: it says the real car is
> *not* steppable as it stands (see below). Only Phase 0's stability sweep is
> still missing, and it is blocked inside `omc`, not here. Phase 6
> (`VehicleRT`) and Phase 9 (packaging) are untouched. Phase 7 is half done:
> live tunables work in the kernel but are blocked on a BobLib change, and
> `vehicle.yml` → Modelica regeneration is not wired up. The reduced-order
> plant is *plausible, not correlated* against BobLib.
> [`HANDOFF.md`](HANDOFF.md) is the honest list of what is done, what is left,
> and what is known to be missing. Read section 7 before quoting any of this
> as a claim about a car.

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

New to the repo? [**Where to look for what**](#where-to-look-for-what) below is
the map: what each directory is for, what happens in one millisecond of the
loop, and which file to open for the change you have in mind.

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
physics. The design's plant ladder has three rungs behind one trait; two of
them exist today — an FMU loaded through `fmu_me.rs`, and the built-in
`Reduced14Dof` — and the middle rung (`VehicleRT`) is Phase 6, untouched. Which
rung runs is decided once at session start, on measured data, never mid-drive.
That is not decoration: on this machine the real `VehicleFMI` fails its probe
and the session degrades to `Reduced14Dof` rather than refusing to start.

**Model Exchange, never Co-Simulation.** A CS FMU owns its solver: `fmi2DoStep`
is unbounded work with no way to impose a deadline, so it cannot be made
real-time safe at all. BobDil imports the FMU's derivatives and owns the
integrator — fixed-step RK4, bounded work per step, by construction.

Read [`docs/architecture.md`](docs/architecture.md) for the reasoning, including
the parts of the original spec that did not survive contact (§5).

---

## Where to look for what

Four languages sit in this repo and each one has exactly one job. If you read
nothing else, read this section — it is the map.

```
schema/bobdil_signals.yaml   EVERY signal that crosses a boundary. Start here.
codegen/bobdil_codegen/      turns that file into the Rust, C, Python, GDScript,
                             .proto and JSON bindings. Never hand-edit anything
                             under a generated/ directory -- `make codegen-check`
                             fails the build if you do.

kernel/src/                  Rust. The only thing with a deadline.
  loop_runner.rs               the 1 kHz loop itself: threads, clock, deadlines
  plant/                       physics. ladder.rs picks, reduced/ is the floor,
                               fmu_me.rs runs a Modelica FMU, integrator.rs steps
  io/                          the wheel. watchdog.rs is the safety contract
  transport/                   how frames leave: seqlock.rs (latest wins),
                               spsc_ring.rs (lossless, for telemetry)
  telemetry/                   .bdt recording (replay.rs, alongside, re-runs one)
  sys/                         the OS: clock, scheduling, /dev/shm, dlopen, signals

session/bobdil/              Python. Everything with no deadline.
  cli.py                       `python -m bobdil` -- the six verbs
  fmu_build.py, manifest.py    Modelica -> FMU -> something the kernel can load
  ab.py                        paired and blind A/B, and the refusals
  doctor.py, toolchain.py      what is installed and what each gap costs

tools/rt_bench/              Python. Phase 0: is this model steppable at all?
view/scripts/                GDScript. Driver POV. Reads physics, never writes it.
modelica/BobDil/             the 5-state fixture plant, for testing the FMI path
docker/                      the toolchain this host does not have
tests/                       Python tests. The Rust ones live beside their code.
```

**Every module opens with a comment saying why it exists**, not what it does.
`head -20` on any file in `kernel/src/` or `session/bobdil/` is usually faster
than reading the code.

### One millisecond, end to end

The clearest way to understand the kernel is to follow one step. Three threads:
the step thread owns the deadline, and the other two exist so that it never has
to wait on hardware or on a disk.

1. **`io/hid.rs`** — the device thread — samples the wheel and pedals through
   **`io/sdl3.rs`** and publishes a `DriverInput` into a seqlock. It is a
   separate thread because a USB transaction can block for milliseconds, and
   the plant must never wait on one.
2. **`loop_runner.rs`** — the step thread — wakes on an absolute deadline
   (**`sys/clock.rs`**) and reads the *newest* input: never a queue, never a
   wait, so a slow device costs freshness rather than time. It then shapes the
   steering reference (**`io/input_shaper.rs`**) so the plant is handed an
   angle, a rate and an acceleration that are genuinely derivatives of one
   another.
3. It advances the plant chosen at session start by **`plant/ladder.rs`**,
   using the fixed-step integrator in **`plant/integrator.rs`**. A non-finite
   state ends the session — and publishes a silent feedback command on its way
   out, so the wheel goes slack the instant the physics is known to be bad.
4. The new state is dead-reckoned into a global position (**`pose.rs`**), and
   the step is judged against its deadline. That verdict is an input to the
   feedback watchdog: a loop that is not keeping up must not keep pushing
   torque as though nothing were wrong.
5. The reaction torque is conditioned and clamped by **`io/ffb.rs`** under the
   four rules in **`io/watchdog.rs`**, then published. The device thread picks
   it up and applies it, with its own staleness watchdog as the last thing
   between the kernel and the hardware.
6. The frame is published to `/dev/shm`, where the Godot view reads it
   (**`transport/seqlock.rs`** ↔ **`view/scripts/state_link.gd`**), and pushed
   into a lossless ring (**`transport/spsc_ring.rs`**) that a third thread
   drains to disk (**`telemetry/recorder.rs`**). A rejected push is a recorded
   fault, never a silent drop.
7. Health — step time, realtime factor, faults — is measured rather than
   assumed (**`metrics.rs`**) and rides inside the frame itself. Then the loop
   sleeps until the next millisecond boundary; if it is already late it
   re-anchors and reports, rather than bursting to catch up and quietly
   changing its own timescale.

### If you want to change

| this | open this first |
| --- | --- |
| a signal anything else can see | `schema/bobdil_signals.yaml`, then `make codegen` |
| the loop, threading, or deadline policy | `kernel/src/loop_runner.rs` |
| the built-in vehicle physics | `kernel/src/plant/reduced/` (`tire.rs` is where the grip lives) |
| how a Modelica FMU is stepped | `kernel/src/plant/fmu_me.rs`, over `fmi2.rs` |
| which plant a session runs | `kernel/src/plant/ladder.rs` |
| **anything that reaches the wheel** | `kernel/src/io/watchdog.rs` — read all four rules before touching `ffb.rs` |
| what the driver sees | `view/scripts/driver_view.gd`, `events.gd` for the cone layouts |
| how a vehicle gets compiled | `session/bobdil/fmu_build.py` and `manifest.py` |
| a `python -m bobdil` verb | `session/bobdil/cli.py` |
| the A/B statistics | `session/bobdil/ab.py`, with `tests/test_ab.py` next to it |
| what Phase 0 measures | `tools/rt_bench/structure.py` and `eigen.py` |
| a `make` target | `makefile` — it is the only entry point, and it is commented |

Two rules that will bite you if you skip them, both spelled out in
[`AGENTS.md`](AGENTS.md): generated code is never hand-edited, and BobLib and
BobSim are read-only inputs that nothing here may write to.

## Safety

A direct-drive wheel delivers enough torque to break a wrist. This is treated as
safety-critical code, not as a rendering concern:

- Two independent non-finite barriers: the plant guards its own output, and the
  feedback chain guards again before anything reaches a device.
- Two independent watchdogs: one on the step thread ramps torque to zero when
  deadlines start slipping, and one on the device thread rejects a command that
  has gone stale, so a step thread that stops publishing entirely still leaves
  the wheel slack rather than locked solid.
- The device is zeroed on every exit path the process can run code on: clean
  shutdown, panic, `Drop`, and `SIGINT`/`SIGTERM`/`SIGHUP`, which
  `sys/signals.rs` catches and turns into a clean stop — a driver reaching for
  Ctrl-C is an exit path too. `SIGKILL` cannot be caught by anything, so there
  it falls to the OS dropping the effect when the device fd closes; that has
  never been tested on hardware.
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
                         # (structure passes; the sweep still exits non-zero)
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

### What it has actually said

**The fixture** (`DilSmokePlant`, 5 states) passes all three parts: 0 non-linear
systems, 0 state events, static state selection; the tightest of the four
operating points still admits a 5.52 ms step, 6× the budget.

**The real car** (`BobLib.Experiments.Standards.VehicleFMI`, 45 states) does
not, and this is the measurement Phase 0 existed to get:

```
structure   28 non-linear systems (largest 2), 63 linear, 3 state events,
            24961 flattened equations, static state selection
timing      FAILED: step 1 at t=0.001 --
            fmi2GetEventIndicators returned status 3
            (non-linear system 28062 failed at time=0.002)
stability   NOT MEASURED -- omc's `linearize` fails in symbolic initialisation
            on a MultiBody shape variable, at all four operating points
```

The structural warning and the runtime failure agree: 28 Newton solves whose
iteration counts vary with the operating point, and 3 state events located by
bisection *inside* a step, are exactly the two things that have no upper bound a
deadline can be planned against. **`VehicleFMI` is therefore not currently
drivable at 1 kHz**, which is the argument for `VehicleRT` (Phase 6). The
stability sweep is the one number still owed, and it is blocked in `omc` rather
than here — see [`HANDOFF.md`](HANDOFF.md) §4 item 0 for what has already been
ruled out.

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
  bobdil build` names exactly which ones, every time — today that is three
  `variability='fixed'` parameters and one (`brake_bias`) the FMU does not
  export at all.

Scope, so the above is not read as more than it is: every A/B number produced so
far describes `Reduced14Dof`'s response, not the real car's, and the blind
protocol has never been run with a human driver.

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
| the header comment on any module | why that file exists at all — they are written for exactly this |
| `make help` | the authoritative list of what you can actually run |
