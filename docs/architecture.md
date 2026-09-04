# BobDil Architecture

## Context

BobLib is a Modelica FSAE vehicle model (MultiBody double-wishbone corners with
bellcranks and anti-roll bars, MF5.2 tires with transient slip, EV powertrain).
BobSim wraps it for offline studies. Neither answers the question a race engineer
actually asks: *does this setup feel better?*

BobDil closes that loop — a driver drives the model, and the response they feel
is derived from the model rather than authored. `README.md` specifies real-time
physics-informed response, off-the-shelf wheel/pedal support, a driver's-POV UI,
`vehicle.yml` reconfigurability, and hermetic distribution.

**Verdict: viable, with four caveats that change the product's claims.** They are
in "Concerns" and they are not cosmetic — one of them (pedal stiffness) is not
buildable as written, and one of them (motion cues) bounds what a DIL verdict
means. The architecture below is shaped around them.

### What already exists (and is reused, not rebuilt)

| Asset | Path | Role in BobDil |
|---|---|---|
| DIL boundary model | `BobLib/Experiments/Standards/VehicleFMI.mo` | **Already the right contract.** 3 inputs (steering angle, accel, brake); outputs include `handwheelTorque`, `Fz_1..4`, `accX/accY`, `roll`, `sideslip`, `yawVel`. Angle-in / reaction-torque-out is exactly the causality a FFB wheel needs. |
| Reduced-order plant | `BobSim/_0_Utils/dyn_py/models.py` | 3/6/10/14-DOF NumPy vehicle, 28 states at 14-DOF, nonlinear DWB kinematics via lookup. |
| MBD↔reduced correlation | `BobSim/_3_StandardSim/ReducedOrderEval/{suspension_correlation,fidelity_suite}.py` | The evidence that a reduced plant tracks BobLib. Becomes BobDil's fidelity gate. |
| yml → Modelica records | `BobSim/_5_App/modelica_generator.py` | Reused verbatim for the reconfigure path. |
| External toolchain detection | `BobSim/_5_App/toolchain.py` | Reused for "is OpenModelica available?" — it already probes `BOBSIM_OMC`/`OPENMODELICAHOME` and verifies `Modelica`+`VehicleInterfaces`. |
| Desktop packaging | `BobSim/_0_Utils/deploy/deploy.py` | Pattern for the PyInstaller/per-OS release. |

---

## 1. Architecture

### 1.1 The governing constraint

Everything below follows from one fact: **a Windows/Linux gaming PC is not a
real-time system.** There is no hard deadline guarantee. What is achievable is
*soft* real-time — 1 kHz with p99.9 jitter under ~200 µs — by pinning a thread to
a core, raising priority, preallocating, and never touching a syscall on the hot
path. Overruns *will* happen (GPU driver, Defender, thermal). The architecture is
therefore defined as much by **what happens when a deadline is missed** as by the
nominal loop.

### 1.2 Process and thread topology

Two processes. The split is not incidental — it is the isolation boundary that
keeps a GC pause, a shader compile, or a stalled UI out of the physics loop.

```
┌─ bobdil-kernel (Rust) ── soft-real-time, never blocks ────────────────┐
│                                                                       │
│  HidThread (normal prio)          StepThread (RT prio, pinned core)   │
│  ┌──────────────────┐             ┌──────────────────────────────┐    │
│  │ SDL3 read wheel  │──input_ring─▶│ 1. read DriverInput (latest) │    │
│  │ + pedals @1kHz   │             │ 2. plant.step(dt)  ← PlantKernel  │
│  │                  │◀──ffb_ring──│ 3. condition FFB torque      │    │
│  │ SDL3 write FFB   │             │ 4. publish VehicleState      │    │
│  │ + FFB watchdog   │             │ 5. push telemetry            │    │
│  └──────────────────┘             └──────────────────────────────┘    │
│         ▲ SAFETY: decays torque to 0                │                 │
│           if StepThread misses N deadlines          │                 │
│                                    TelemetryThread ─┘                 │
│                                    (drains ring → disk)               │
└────────────────────────────────┬──────────────────────────────────────┘
                    state_seqlock │ (shared memory)          │ control (protobuf/UDS)
                                 ▼                           ▼
              ┌─ bobdil-view (Godot 4) ─┐      ┌─ bobdil-session (Python) ─┐
              │ driver POV @60–144 Hz   │      │ vehicle.yml → records → FMU│
              │ interpolates latest     │      │ FMU cache, tunables, RTF   │
              │ state; NEVER writes     │      │ monitor, replay, A/B       │
              └─────────────────────────┘      └────────────────────────────┘
```

`bobdil-session` is deliberately *outside* the loop. Compilation, disk, config
UI, and anything that can block lives there. It can crash without stopping the
drive.

### 1.3 The `PlantKernel` interface — the decoupling the spec asks for

> *"Decouple the BobLib physics kernel from anything else"*

One trait, one integration path, three implementations:

```rust
trait PlantKernel {
    fn reset(&mut self, init: &InitialConditions) -> Result<()>;
    fn step(&mut self, u: &DriverInput, dt: f64) -> Result<VehicleState>;
    fn set_tunable(&mut self, id: TunableId, value: f64) -> Result<()>;
    fn capabilities(&self) -> KernelCaps;   // max stable dt, tunables, RTF class
}
```

| Impl | Source | Role |
|---|---|---|
| `FmuMe<VehicleFMI>` | existing `VehicleFMI.mo` | **Reference.** Full MultiBody. Ground truth; may or may not hold 1 kHz. |
| `FmuMe<VehicleRT>` | **new** `VehicleRT.mo` in BobLib | **The real-time target.** Same physics source, table-driven suspension kinematics. |
| `Reduced14Dof` | `dyn_py` port / FFI | Last-resort floor. Guarantees a drivable loop on any hardware. |

**A refinement on the kernel choice you selected.** You picked "FMU primary,
reduced-order fallback". I'd insert `VehicleRT` between them, and here is why it
is strictly better than falling straight from the MBD FMU to Python:

- It keeps **one** integration path in the kernel (FMI 2.0 ME) instead of two,
  so there is no second physics implementation to drift.
- The driver is still feeling *Modelica*, from the *same* BobLib source — which
  is the actual point of the product. A drop to the Python model quietly changes
  what the driver is validating.
- `dyn_py` is not wasted: it becomes the **oracle** that certifies `VehicleRT`
  (`fidelity_suite.py` already does this comparison), plus the generator for the
  kinematics tables `VehicleRT` consumes, plus the floor if even `VehicleRT`
  misses.

So the ladder is `VehicleFMI` → `VehicleRT` → `Reduced14Dof`, selected once at
session start from measured RTF, never mid-drive.

### 1.4 Why FMI Model Exchange, not Co-Simulation

This is the most consequential technical choice and the spec's biggest hidden trap.

An OpenModelica **Co-Simulation** FMU owns its own solver. `fmi2DoStep` runs
variable-step DASSL/CVODE internally — **unbounded work per call**. There is no
way to impose a deadline on it. A CS FMU cannot be made soft-real-time safe.

**Model Exchange** hands us `fmi2GetDerivatives` and we own the integrator. That
buys the three things real-time requires: a fixed step, a bounded iteration
count, and the ability to abort a step that is running long. Integrator: explicit
RK4 at 1 ms baseline, semi-implicit for stiff configurations, with substepping
for fast subsystems if the eigenvalue sweep (Phase 0) demands it.

`VehicleFMI`'s current annotation is the exact opposite of a real-time build —
`s = "dassl"`, `--indexReductionMethod=dynamicStateSelection` (state selection
*at runtime* = nondeterministic timing), `jacobian = "internalNumerical"`. The RT
build uses a different flag set; that is a build-target difference, not a model
fork.

### 1.5 `VehicleRT` — what actually makes it real-time

The MultiBody double-wishbone-with-bellcrank corners form **closed kinematic
loops**. Each one becomes a nonlinear algebraic system solved by Newton every
derivative evaluation, four corners deep, with a numerical Jacobian. This is the
single largest real-time risk in the model, and it is the thing to remove.

`VehicleRT` (a BobLib PR — per BobSim's `AGENTS.md`, BobLib changes ship
separately and get a pin bump) differs from `VehicleFMI` in four ways:

1. **Table-driven corner kinematics.** Replace the MultiBody linkage solve with
   bump/roll → camber, toe, track, half-track, motion-ratio lookups, *generated
   offline from the very same MultiBody model*. Physics preserved; loops gone.
   `dyn_py`'s `DoubleWishboneKinematicLookup` is the existing precedent.
2. **Smooth regularization instead of state events.** Differential stick-slip
   (`diff_w_transition`, `diff_kineticFrictionRatio`) and brake friction become
   `noEvent` smooth transitions. The tire model already uses this pattern.
3. **Quasi-static electrical.** Battery/inverter fast states collapse to a torque
   map plus a first-order lag, unless Phase 0 shows their eigenvalues are slow
   enough to keep.
4. **Zero-inertia handwheel boundary** — see §1.6.

Fidelity is not asserted, it is *measured*: `VehicleRT` must match `VehicleFMI`
on the existing BobSim maneuver suite within stated tolerances before it may be
selected. That gate lives in CI.

### 1.6 Force feedback — where naive DIL rigs fail

The model's boundary is position-in / torque-out, with a steering column that has
inertia *inside* the model. If we feed it the raw measured wheel angle, the
reaction torque contains a numerically-differentiated `d²θ/dt²` term, and the
wheel chatters violently. Three mitigations, in order of preference:

1. **Move the column inertia out of the model** (`VehicleRT`): the driver's real
   wheel *is* the inertia. FFB torque then reflects rack force through the
   steering ratio plus a friction/damping model. This is standard admittance
   coupling and it is the correct fix.
2. **Reference-shape the position input** — critically damped 2nd-order filter at
   ~40 Hz — so the plant sees consistent θ, θ̇, θ̈.
3. **Condition the output**, always, in a fixed chain: `NaN/inf guard → scale →
   slew-rate limit → soft end-stop at rack travel → absolute clamp → watchdog`.
   Every stage is parameterized and individually bypassable, because "does the
   FFB tuning or the vehicle change explain what the driver felt?" is a question
   you will need to answer.

**Safety, and it is not optional.** A direct-drive wheel outputs 20+ Nm — enough
to break a wrist. A stale, NaN, or maxed torque is a physical hazard. Mandatory
and non-negotiable in `io/ffb.rs` and `io/watchdog.rs`:

- NaN/inf → immediate zero, session fault.
- Absolute torque clamp below device capability, configured before first launch.
- Watchdog: if `StepThread` misses N consecutive deadlines, ramp torque to zero
  within 50 ms. Never hold the last value.
- Zero-on-exit on *every* path — clean shutdown, panic, signal, parent death.

### 1.7 Buffers and the wire format

> *"Protect the transfer of data between components with buffers"* /
> *"Prefer Protobufs or packed binary structs where possible"*

Two channel types, chosen by what the consumer actually needs — a ring buffer
everywhere would be wrong:

| Channel | Type | Why |
|---|---|---|
| `DriverInput` → StepThread | **seqlock** (latest-value, wait-free) | The plant wants the *newest* input, not a backlog. A queue would add latency. |
| `VehicleState` → view | **seqlock** in shared memory | The renderer wants the newest state. Dropped frames are correct behaviour. |
| FFB torque → HidThread | **seqlock** | Same; a stale queue is worse than a dropped sample. |
| Telemetry → disk | **SPSC ring**, lossless | Every sample matters for replay and A/B. Overflow is a recorded fault, never a silent drop. |
| Session ↔ kernel control | **Protobuf over UDS/named pipe** | Off the hot path; schema evolution matters more than latency. |

No mutex ever appears on the RT path.

**Format split, and one schema.** Protobuf serialization allocates and costs
microseconds — unacceptable at 1 kHz. So: **packed fixed-layout structs on the
real-time path, Protobuf on the control path.** To stop those two from drifting,
both are generated from a **single schema** (`proto/`), which emits: the C header
for the packed frame, the Rust binding, the Python binding, the Godot reader, and
the FMU value-reference table. Adding a signal is one edit in one file.

This is the direct answer to *"standardize components, then their inputs/outputs,
then the interactions"* — the signal set is defined once and every component's
view of it is derived.

### 1.8 Deadline policy — the part most sims get wrong

The plant is the master clock. Sim time advances by **exactly** `dt` per step,
paced against a monotonic clock. On overrun:

- **< 1 step behind:** absorb, continue.
- **Sustained:** *do not burst-catch-up.* Running four steps back-to-back spikes
  FFB and diverges from wall clock. Instead, degrade — drop to a slower fixed
  step, or (between sessions) down the kernel ladder — and **tell the driver**.
  A live RTF indicator is a first-class UI element, not diagnostics.
- **Never** freeze FFB at a stale value. Decay it.

A "verification tool" that silently varies its own timescale is worse than
useless, because the driver's verdict then encodes the stutter rather than the
setup.

### 1.9 The features that make it a *tool*, not a toy

These are not in the spec. They are what converts "physics-informed response"
into "driver-informed development", and they fall directly out of the concerns
in §5:

- **Live tunables — the highest-value feature in the product.** Spring rate, ARB
  rate, damper curve, brake bias, aero balance, diff preload exposed as FMI
  `tunable` parameters. The driver says "more front bar", you dial it, they drive
  again in **5 seconds** instead of a 15-minute recompile. Requires a BobLib
  change: those record fields must not be `Evaluate=true` and must route as FMU
  parameters rather than record constants. Structural changes (hardpoints,
  topology) still recompile.
- **Deterministic replay.** Record the 1 kHz driver input stream plus
  `hash(vehicle + kernel + build flags)`. Replaying must reproduce bit-identically.
- **Paired A/B.** Drive a lap, then replay *identical* driver inputs against
  setup B and diff the trajectories. This removes the driver's own lap-to-lap
  variance — the dominant noise source — from the comparison.
- **Blind A/B.** Driver doesn't know which setup is loaded. Given §5.1, this is
  the difference between data and confirmation bias.

---

## 2. Software used

| Layer | Choice | Why this one |
|---|---|---|
| Physics kernel | **Rust** | No GC, no GIL, deterministic teardown. The hot path is lock-free multi-threaded shared memory — precisely where C++ bugs are catastrophic and hardest to reproduce. Cargo makes 3-OS builds tractable. FMI 2.0's C API binds trivially. |
| FMI runtime | **FMI 2.0 Model Exchange**, `fmi`/`fmi-sys` crate or a thin hand-rolled loader | §1.4. FMI 3.0 is a later option; 2.0 has the mature OpenModelica export path. |
| Model compiler | **OpenModelica ≥1.26**, `buildModelFMU` | Already the team's toolchain; BobSim pins `openmodelica/openmodelica:v1.26.3-ompython`. |
| Input + FFB | **SDL3** (`SDL_haptic`, gamepad/joystick) | One API covering Windows DirectInput/XInput and Linux evdev-FFB. Directly satisfies "as long as the wheel works in a standard simulator, it works here". |
| Renderer / driver POV | **Godot 4** (MIT) | Window, 3D scene, camera, cone/track authoring, cross-platform export out of the box. BobDil owns physics; Godot is a **dumb client** reading a seqlock — it can never affect the plant. |
| Orchestrator | **Python 3.11** | Matches BobSim exactly; lets `modelica_generator.py` and `toolchain.py` be imported rather than reimplemented. |
| Control-plane schema | **Protobuf** | Spec-requested, and right for the non-RT path. |
| Correlation / tables | **NumPy/SciPy** via `dyn_py` | Already written and already correlated. |
| Packaging | **PyInstaller** (session) + native binaries, per BobSim `deploy.py` | Reuses a working release pipeline. |

**The Rust decision, stated honestly.** It adds a fourth language to a
Python+Modelica+GDScript stack, on a team with turnover. The alternative is C++
(same performance, more sim-industry precedent, more foot-guns) — that is a
defensible substitution and the architecture does not change. What is *not*
viable is Python on the hot path: GC pauses and per-op interpreter overhead put a
1 kHz FFB loop out of reach. If Rust is rejected, choose C++, not Python.

---

## 3. Structure

```
BobDil/
├── proto/                      # SINGLE SOURCE OF TRUTH for every signal
│   ├── driver_input.proto      #   3 driver commands
│   ├── vehicle_state.proto     #   handwheelTorque, Fz_*, accX/Y, roll, ...
│   ├── session.proto           #   control plane: load, tune, record, replay
│   └── rt_frame.yaml           #   packed-struct layout for the RT path
├── codegen/                    # schema → C hdr / Rust / Python / GDScript / FMU vref table
│
├── kernel/                     # Rust. Soft-real-time. Never blocks.
│   ├── clock.rs                #   monotonic pacing, deadline policy (§1.8)
│   ├── transport/
│   │   ├── seqlock.rs          #   latest-value, wait-free
│   │   ├── spsc_ring.rs        #   lossless telemetry
│   │   └── shm.rs              #   cross-process mapping
│   ├── plant/
│   │   ├── mod.rs              #   PlantKernel trait (§1.3)
│   │   ├── fmu_me.rs           #   FMI 2.0 ME loader + value refs
│   │   ├── integrator.rs       #   fixed-step RK4 / semi-implicit, bounded iters
│   │   ├── reduced.rs          #   14-DOF floor
│   │   └── ladder.rs           #   RTF-based kernel selection at session start
│   ├── io/
│   │   ├── hid.rs              #   SDL3 device thread
│   │   ├── ffb.rs              #   conditioning chain (§1.6)
│   │   └── watchdog.rs         #   SAFETY: torque decay. Reviewed like safety code.
│   └── telemetry/recorder.rs
│
├── session/bobdil/             # Python. Not real-time.
│   ├── toolchain.py            #   thin wrapper over BobSim _5_App/toolchain.py
│   ├── vehicle_build.py        #   modelica_generator.py → buildModelFMU
│   ├── fmu_cache.py            #   content-addressed: hash(yml, BobLib sha, omc ver, flags, platform)
│   ├── tunables.py             #   live FMI tunable parameters (§1.9)
│   ├── session.py              #   lifecycle, RTF monitor, kernel selection
│   ├── replay.py               #   deterministic replay, paired + blind A/B
│   └── server.py               #   local UI
│
├── view/                       # Godot 4 project
│   ├── scripts/state_reader.gd #   seqlock reader — read-only, by construction
│   └── events/                 #   skidpad, acceleration, autocross, endurance cone layouts
│
└── tools/rt_bench/             # PHASE 0 — the viability gate (§6)
```

**One responsibility per file, per the spec's OO requirement.** The rules that
keep it that way: nothing in `kernel/plant/` knows a wheel exists; nothing in
`kernel/io/` knows what a tire is; `view/` has no write path to physics, enforced
by mapping the state segment read-only; `session/` never appears in the loop.

**Road model scope — a deliberate cut.** `VehicleFMI` uses
`VehicleInterfaces.Roads.FlatRoad`: a flat infinite plane, no elevation, banking,
or grip variation. Rather than build a road system first, note that **every FSAE
event is run on flat pavement.** Phase 1 keeps `FlatRoad` and puts cone layouts
for skidpad, acceleration, autocross, and endurance in the *visual* layer only.
This is correct for the domain and removes a large subsystem from the critical
path. A heightmap-backed `Road` implementation slots in later behind the same
`VehicleInterfaces.Roads` contract without touching anything else.

---

## 4. Tradeoffs

| Decision | Chosen | Given up | Why it is right here |
|---|---|---|---|
| ME + our integrator | Fixed step, bounded work, abortable | The FMU's own adaptive-step accuracy | A CS FMU cannot be made deadline-safe at all (§1.4). Accuracy is recovered by fidelity-gating `VehicleRT` against `VehicleFMI` offline. |
| `VehicleRT` with table kinematics | Removes the dominant RT risk; still Modelica, still BobLib | Exact MultiBody linkage compliance at the corner | Tables are generated *from* the MBD model. The residual error is measured, not assumed — and it is far smaller than the tire-data error (§5.1). |
| Rust kernel | Determinism, memory safety in lock-free code | A fourth language on the team | Python cannot hold 1 kHz. C++ is an acceptable substitution; Python is not. |
| Two processes | A stalled UI or GPU driver cannot touch physics | One extra IPC hop (~µs via shm) | The hop is far cheaper than the failure it prevents. |
| Packed structs on RT path, protobuf on control path | Zero-allocation hot path, evolvable control plane | Two representations | Single-schema codegen makes drift impossible by construction (§1.7). |
| Godot as a dumb client | 3D, input, packaging for free; physics stays ours | A Godot dependency and a GDScript surface | The client is read-only; if Godot is ever replaced, the seqlock contract is all that must be reimplemented. |
| Split hermetic/recompile | Drive instantly with zero toolchain | Custom `vehicle.yml` needs an external OpenModelica | Consistent with BobSim today, and honest — see §5.2. |
| FlatRoad + visual cones | Ships the FSAE events immediately | Elevation, banking, camber, grip variation | Matches the actual competition surface; the road contract stays open. |
| Kernel selected at session start, never mid-drive | Predictable feel; no discontinuity under load | Cannot recover fidelity mid-session | Swapping the plant under a driver at speed is both unsafe and scientifically meaningless. |

---

## 5. Concerns — the parts of the spec that do not survive contact

The spec explicitly invites this, so here it is plainly, worst first.

### 5.1 The dominant fidelity risk is the tire data, not the solver

Everything about real-time computation is a solved engineering problem. **Tire
data is not.** A driver at the limit is feeling the MF5.2 fit almost exclusively.
Without TTC-grade fitted data for the *actual* tire at the *actual* pressures and
temperatures, drivers will confidently "verify" an artifact of the tire fit — and
because the response is physics-derived, they will trust it more, not less. The
model also has **no thermal or pressure-sensitivity model**, so the tire cannot
degrade, which is exactly what drivers use to judge a setup over a run.

This does not sink the product, but it bounds the claim: **BobDil compares
setups against each other; it does not predict absolute grip.** Every A/B feature
in §1.9 exists because of this sentence.

### 5.2 A static rig systematically biases the driver

No motion platform means no vestibular cue. Drivers detect the limit primarily
through lateral and longitudinal acceleration onset, and DIL literature
consistently shows that static rigs shift driver preference toward setups that
are more forgiving than they need to be. Combined with §5.1: **BobDil is a valid
instrument for relative comparison of steering feel, response timing, and
balance, and an invalid one for absolute limit judgement.** State this in the UI,
not just the docs — a tool that overstates itself will get a design decision
wrong eventually.

### 5.3 "Pedal stiffness fed back to the user" is not buildable as written

This is a direct internal contradiction in the spec. Key feature 3 requires
off-the-shelf pedals; key feature 2 requires pedal stiffness feedback. **Almost
no off-the-shelf pedal set can render variable stiffness.** Load-cell brake
pedals — the good ones — *measure* force; their stiffness is a fixed elastomer
stack. Active-force pedals exist (Simucube ActivePedal and similar) but are rare
and ~$1500+, and are not what "works in a standard simulator" means.

Resolution: a `HapticSink` interface with **capability negotiation**. Wheel FFB
is the mandatory implementation. Active pedals are an optional implementation
behind the same interface. For ordinary pedals, degrade *honestly* — surface
brake-pressure and lock-up cues visually and through a rumble motor if one is
present, and do not claim stiffness feedback in the UI. Do not silently pretend.

### 5.4 "Regardless of OS, system state, architecture" is not deliverable

An FMU contains **compiled native binaries**. A vehicle built on Windows x86_64
will not load on macOS ARM. "Hermetic regardless of architecture" is achievable
only by building per platform. Recommendation: **Windows x86_64 and Linux x86_64
as tier 1** (OpenModelica on macOS ARM is materially weaker), with prebuilt FMUs
shipped per tier-1 platform. Everything else is best-effort and should say so.

The hermetic/recompile conflict itself is resolved as you chose — ship prebuilt
FMUs for stock vehicles so the app drives with zero toolchain, and detect an
external OpenModelica for custom `vehicle.yml` recompiles using BobSim's existing
`toolchain.py` probe.

### 5.5 Licensing — check before distributing anything

BobLib is **GPL-3.0**. An FMU built from BobLib contains C generated from GPL-3.0
Modelica sources, which is a strong candidate for a derivative work. If BobDil is
distributed as an app bundling BobLib-derived FMUs, **BobDil itself likely must be
GPL-3.0**, which in turn constrains what may be linked into the kernel. This is a
question for whoever owns licensing at BobDyn, and it should be answered *before*
the first release build, not after. Flagging it, not deciding it.

### 5.6 Latency budget is tight, and most of it is not ours

| Segment | Budget |
|---|---|
| USB HID input poll | ~1 ms (hardware) |
| Kernel: read → step → condition → publish | **≤ 1 ms (ours)** |
| USB HID FFB write + wheel firmware | ~1–4 ms (hardware) |
| **Total torque loop** | **~3–6 ms** |

Above roughly 6–8 ms the wheel feels rubber-banded and stiff-wheel setups can go
unstable. **Roughly half the budget is device firmware we do not control**, so
this must be measured with the actual wheel early — a wheel with poor firmware
latency can fail the requirement no matter how good the kernel is.

### 5.7 What Phase 0 might still tell us

If the benchmark shows even `VehicleRT` cannot hold 1 ms, the fallbacks in order
are: 2 ms step with FFB interpolation (usually still acceptable); multi-rate
(suspension/tire at 1 kHz, powertrain at 100 Hz); then `Reduced14Dof`. The
product survives all three. It only fails if `Reduced14Dof` also misses, which
the existing `dyn_py` timings make very unlikely.

---

## 6. Phase 0 — the viability gate (build this first)

Nothing in either repo measures the real-time factor of the full vehicle.
`Tests/modelica_runtime_baseline.csv` looks relevant but is not: it times
*compile + run* of 0.01-second toy models, so it is dominated by compilation and
says nothing about RTF. **Every schedule estimate below Phase 0 is unfounded
until this runs.** Build `tools/rt_bench/` before anything else.

It must report, for `VehicleFMI` and then `VehicleRT`:

1. **Structure** — continuous states, total equations, count and max size of
   nonlinear algebraic systems, number of state events, whether dynamic state
   selection is active.
2. **Eigenvalue sweep** at representative operating points → the largest stable
   explicit step, and which subsystem sets it.
3. **Timing** — mean / p99 / p99.9 / max wall-clock per fixed 1 ms RK4 step over
   ≥60 s of representative driving: standing start, step steer, threshold
   braking, and steady-state skidpad. Standing start matters specifically because
   transient slip divides by velocity, and a DIL session always begins at rest.
4. **Flag comparison** — `dynamicStateSelection` vs `uode`; numerical vs
   symbolic Jacobian.

**Gate: p99.9 step time < 500 µs** (half the 1 ms budget, leaving headroom for
OS jitter) → that kernel is cleared for the ladder.

Run it on a representative gaming PC, not a laptop. Note the machine this was
scoped on is an i7-1360P laptop; a gaming desktop is roughly 1.5–2× faster
single-threaded, and single-thread performance is what matters here.

---

## 7. Build order

| Phase | Deliverable | Done when |
|---|---|---|
| **0** | `tools/rt_bench/` | RTF measured for `VehicleFMI`; kernel ladder decided on data. |
| **1** | `proto/` + `codegen/` | One schema emits C, Rust, Python, GDScript, FMU vref table. |
| **2** | `kernel/` skeleton: clock, seqlock, ring, `PlantKernel` trait, `Reduced14Dof` | 1 kHz loop holds with jitter measured; no plant risk yet. |
| **3** | `io/` — SDL3 input, FFB chain, **watchdog** | Torque loop measured end-to-end with the real wheel (§5.6). Watchdog tested by deliberately stalling `StepThread`. |
| **4** | `plant/fmu_me.rs` + `VehicleFMI` FMU | A driver drives the real Modelica model. **First moment of product truth.** |
| **5** | `view/` Godot client + FSAE cone events | Driver POV at 144 Hz; skidpad and autocross drivable. |
| **6** | `VehicleRT` BobLib PR + fidelity gate in CI | `VehicleRT` matches `VehicleFMI` within tolerance on the BobSim maneuver suite. |
| **7** | `session/` — yml→FMU rebuild, FMU cache, **live tunables** | Driver says "more front bar"; it is there in 5 s. |
| **8** | Replay, paired + blind A/B | Identical inputs reproduce bit-identically across two setups. |
| **9** | Packaging, tier-1 installers | Fresh machine, no toolchain, drives the stock car. |

Phases 2 and 3 deliberately precede any FMU work: the loop, the buffers, and the
safety path are provable with a trivial plant, and debugging jitter and debugging
Modelica at the same time is how these projects stall.

---

## 8. Verification

**Kernel timing** — `rt_bench` and the live RTF monitor report p50/p99/p99.9 step
time and deadline-miss count. CI fails a regression in p99.9. Soak: 30 minutes
continuous at 1 kHz with zero unhandled misses.

**Safety (highest priority, tested adversarially)** — inject NaN into the plant
output and assert torque reaches zero within one step. Suspend `StepThread` with
a debugger and assert the watchdog ramps to zero within 50 ms. `SIGKILL` the
kernel mid-drive and assert the device is left at zero torque. These are run
before any human drives the rig, and re-run on every kernel change.

**Physics fidelity** — `VehicleRT` and `Reduced14Dof` run the existing BobSim
maneuver suite (`_3_StandardSim/ReducedOrderEval/fidelity_suite.py`) against
`VehicleFMI` and must stay inside per-signal tolerances on `accY`, `yawVel`,
`Fz_*`, and `handwheelTorque`. Checked into CI as the gate that permits a kernel
onto the ladder.

**Determinism** — record a 60 s drive; replay it 10× and assert bit-identical
telemetry. Then replay against a tuned setup and assert the *inputs* are
identical while the outputs differ. This is the test that makes A/B trustworthy.

**Transport** — property tests on the seqlock (a reader never observes a torn
frame under a hammering writer) and on the SPSC ring (no loss below capacity;
overflow is reported, never silent).

**End-to-end** — on a clean machine with no OpenModelica: install, launch, drive
the stock car on skidpad with FFB. Then, with OpenModelica present: edit
`vehicle.yml`, rebuild, and drive the new model. Those two runs are the spec's
acceptance criteria.

**Human-in-the-loop, and it is a real test** — a blind A/B with a known-different
setup (e.g. a large front ARB change). If experienced drivers cannot reliably
identify which is which above chance, the rig is not yet sensitive enough to
support a design decision, regardless of what the timing numbers say. Run this
before anyone uses BobDil to justify a change to the car.
