# BobDil — implementation handoff

Resume point for a fresh session. The design is `docs/architecture.md`
(published at https://claude.ai/code/artifact/f5871938-3861-4689-bf4e-a0ca52e2d1b3).
This file records what is **built and verified**, what is **left**, and the
things that were learned the hard way.

Roughly 9k lines of Rust, 4.5k of Python, plus GDScript and Modelica.

Against the design's build order: phases 1, 2, 5 and 8 are done. Phases 3 and 4
are **built but not signed off against their own acceptance criteria** (design
§7): phase 3 wants the torque loop measured with a real wheel, and no torque has
ever reached hardware; phase 4 wants a driver driving the real Modelica model,
and `VehicleFMI` fails on its first step (§1). **Phase 0's tooling is complete
and its headline measurement is in** — and the answer is that the real car is
not steppable at 1 kHz as it stands (§1). Only the stability half of Phase 0 is
still owed, and it is blocked inside `omc`, not here (§4 item 0). Phase 6
(`VehicleRT`) and phase 9 (packaging) are untouched, and **phase 7 is only
half done**: the FMU cache and the manifest are built and the kernel takes live
tunables, but `vehicle.yml` -> Modelica regeneration is not wired to BobSim's
`modelica_generator.py`, and the tunables themselves are blocked on a BobLib
change. Phase 7's acceptance test -- "driver says more front bar, it is there
in 5 s" -- cannot pass yet, and nothing here should claim it can.

---

## 1. What works right now, verified by running it

| Piece | State | Evidence |
|---|---|---|
| Schema + codegen (5 targets) | done | `make codegen-check` |
| Rust kernel, zero external crates | done | 88 unit + 1 integration test pass |
| Reduced14Dof floor plant (29 states) | done | 14 behavioural tests |
| FFB chain + watchdog (safety) | done | `bobdil-kernel selftest` — 5/5 |
| FMI 2.0 **Model Exchange** loader | done | drives a real OpenModelica FMU |
| Seqlock / SPSC transports | done | torn-read + loss property tests |
| Telemetry record + replay | done | bit-identical replay verified |
| Godot 4 view reading `/dev/shm` | done | 120/120 fresh frames, 0 torn |
| FSAE cone layouts in `view/` | done | `events.gd`: skidpad, acceleration, autocross; drawn as one MultiMesh |
| SDL3 wheel/FFB backend | compiles, links | **never tested with a real wheel** |
| Session layer: toolchain, FMU build, manifest | done | built + loaded an FMU |
| BobDil Modelica fixture plant | done | `DilSmokePlant`, 5 states, 0 events |
| **`tools/rt_bench/`** — Phase 0 structure + stability | structure done, sweep blocked | `make rt-bench`, 45 Python tests |
| **Real `VehicleFMI` compiled + loaded** | done, but **does not step** | 45 states; `make vehicle-fmu`, then fails at step 1 (below) |
| **`python -m bobdil`** — the session CLI | done | 6 verbs, `make` targets now wrap it |
| **Paired A/B** | done | `make ab-paired` on the reduced kernel |
| **Blind A/B** | done | balanced sealed order + exact binomial score |
| **Containers** | done | `make omc-image` has MSL 4.1.0 + VehicleInterfaces 2.0.2 |
| The hermetic gate, in the container | done | `make test-container` — same 45 + 88 + 1, green |
| `.gitignore`, `AGENTS.md`, rewritten `README.md` | done | — |

Measured on this laptop (i7-1360P, unprivileged, `--release`):

```
DilSmokePlant (Modelica FMU)  5 states  mean 2.5  p99 7.0  p99.9 10.0 us  PASS
Reduced14Dof                 29 states  mean 3.8  p99 6.0  p99.9  8.0 us  PASS
gate: p99.9 <= 500 us
full session: 8000 steps, rtf 0.997, 0 deadline misses, 0 telemetry drops
```

The OS sleep path (`p99.9 ≈ 230 µs`, max 2.1 ms) is 20× more expensive than the
plant. **On this class of model the constraint is the operating system, not the
physics** — which is what the design predicted, and it is why the ladder exists.

`rt_bench` now supplies the two halves of Phase 0 that timing cannot. On the
fixture plant:

```
structure   5 states, 0 non-linear systems, 0 state events, static state selection
stability   tightest operating point (threshold braking) admits a 5.52 ms step,
            6x the 1 ms budget; set by yawRate at lambda = -504
```

The threshold-braking point is **6× stiffer than the other three**. That is
precisely why the sweep runs four operating points instead of one: a single
linearization at a comfortable point would have reported 34 ms and been wrong
by a factor of six about the model's actual constraint.

### The real vehicle — `BobLib.Experiments.Standards.VehicleFMI`

Compiled in the omc container (`make vehicle-fmu`), FMI 2.0 **Model Exchange**,
`--indexReductionMethod=uode`. This is the measurement Phase 0 existed to get.

```
continuous states     45          event indicators  3
flattened equations   24961 (19668 trivial)
non-linear systems    28 (largest 2)      linear systems 63 (largest 1)
state events          3           time events       0
state selection       static      <- the uode override worked
```

**The car does not currently hold a 1 kHz fixed step.** `make bench` against the
real FMU:

```
VehicleFMI    FAILED: step 1 at t=0.001: fmi2GetEventIndicators returned status 3
              (non-linear system 28062 failed at time=0.002)
Reduced14Dof  29 states  mean 6.2  p99 9.0  p99.9 14.0  max 36.9 us   PASS
```

The structural warnings and the runtime failure agree, which is the useful part:
28 Newton solves whose iteration counts vary with the operating point, and 3
state events whose crossings are located by bisection *inside* a step. Neither
has an upper bound that can be read off the model, and that is what a deadline
needs. The ladder behaved as designed — the session degraded to `Reduced14Dof`
rather than refusing to start.

**None of the four tunables is settable live on the real model.** Previously a
prediction, now confirmed: `front_arb_rate`, `rear_arb_rate` and `diff_preload`
export as `variability='fixed'`, and `brake_bias` is not exported at all. Live
setup changes need `Evaluate=true` removed in BobLib. See §4 item 2.

The stability sweep is **not yet measured on the real car**. Two defects were
fixed getting this far (§5: operating points carried the fixture's own state
name; omc's library path). It still fails in omc's symbolic initialization on a
MultiBody position variable, at all four operating points; `headless = true` has
since been tried and only moves the variable it fails on (§4 item 0). Do not
report a step bound for this model until the sweep runs.

---

## 2. Environment (already installed on this machine)

```bash
rustup                  ~/.cargo/bin      rustc 1.98.1
python venv             ./.venv           pyyaml numpy scipy pytest ruff protobuf
omc                     /usr/sbin/omc     v1.27.0-cmake   (MSL 3.2.3 — cannot build BobLib)
godot                   4.7.2
SDL3                    3.4.14 + headers
docker                  29.7.2, daemon up
bobdyn/bobdil-omc       built             omc 1.26.3, MSL 4.1.0, VehicleInterfaces 2.0.2
```

**Host omc cannot build BobLib**; the container can, and `make doctor` says so
and names the target that fixes it. The image deliberately mirrors BobSim's
`Dockerfile`: a BobDil FMU and a BobSim FMU must come from the same compiler and
the same library versions or the two repositories' results are not comparable,
and that comparison is the gate that admits a kernel to the ladder. **Bump the
two images together.**

One deviation from BobSim's image, and it is deliberate:
`docker/Dockerfile.omc` installs Python 3.13, not 3.11, because
`requirements-dev.txt` pins numpy 2.5 and scipy 1.18 which need ≥ 3.12. It is an
`ARG PYTHON_VERSION` so the version lives in one place; bump it with the pins.

---

## 3. Commands that work today

`make help` lists them. Everything goes through the makefile, so nothing needs
`PATH` or venv juggling by hand.

```bash
make venv        # once, on a fresh machine
make ci          # the whole gate: ~15 s from a cold target/ on this laptop
```

Most targets are now one line over `python -m bobdil` or the kernel's own
command line. That is on purpose: a makefile recipe cannot be unit tested, and
behaviour that only exists in a recipe can only be run there. Two long inline
scripts (`DOCTOR_PY`, `FIXTURE_PY`) became `session/bobdil/doctor.py` and a CLI
verb for exactly this reason.

The groups, and why the split matters:

| Group | Targets | What it means |
|---|---|---|
| **testing** | `test` `test-python` `test-kernel` `test-kernel-sdl3` `lint` `codegen-check` | Hermetic. Same answer on any machine. Safe in CI. |
| **validation** | `validate` `doctor` `selftest` `bench` `record` `replay-check` `view-check` | Measures *this* box: its clock, its scheduler, its OS. The numbers do not travel. |
| **model** | `rt-bench` `rt-bench-vehicle` `ab-paired` | Measures the *model*: work per step, largest stable step, what a setup change does. These numbers **do** travel. |
| **containers** | `omc-image` `kernel-image` `vehicle-fmu` `test-container` `shell-omc` | The toolchain this host does not have. |
| **production** | `build` `build-headless` `codegen` `fixture-fmu` `drive` `drive-fixture` `view` `release` | Artefacts, and driving the rig. |

Worth knowing individually:

```bash
make doctor          # first thing in a new session: what is missing and what it costs
make selftest        # SAFETY. Five checks. Run before a person touches a wheel.
make rt-bench        # Phase 0's model half: is this thing steppable at all?
make ab-paired       # one lap, two setups, diffed
make drive-fixture   # drives the real OpenModelica FMU, not the reduced plant
```

Gate decisions a future agent should not helpfully undo:

- **Generated code is linted but not formatted.** `src/generated/frames.rs` and
  `session/bobdil/generated/frames.py` are excluded from `rustfmt` and
  `ruff format`. Their shape is the emitter's contract and `codegen-check`
  already guards those bytes exactly; adding a formatter's opinion on the same
  bytes only gives two gates something to disagree about. Lint still covers them.
- **`make lint` runs clippy twice**, once `--no-default-features` and once with
  SDL3. A headless-only gate compiles none of `io/sdl3.rs`.
- **Container targets wrap the same makefile**, they do not duplicate it. A
  container recipe that reimplements a host recipe is a second thing to keep
  true, and the one run less often is the one that rots.

---

## 4. What is left

0. **Finish the stability sweep on the real car.** `make rt-bench-vehicle`
   now gets the structural half right (§1) and fails the linearization half:
   omc's `preBalanceInitialSystem2` reports a MultiBody position variable that
   `does not appear in any equation in the initial system and is not fixable`.
   Ruled out so far: the operating-point binding (fixed, §5) and MultiBody
   *animation* (`headless = true` removes the shape variables and the same
   failure reappears on `spaceFrame.torsionalRevolute.frame_b.r_0[3]`). Note
   that `buildModelFMU` on the same model with the same flags **succeeds** —
   only `linearize` fails — so suspect the initial system `linearize` builds,
   not the model. Next: whether it reproduces without rt_bench's wrapper, and
   under the model's own annotation flags (`dynamicStateSelection`, plus
   `-d=initialization,NLSanalyticJacobian,disableStartCalc`, which
   `REALTIME_FLAGS` deliberately drops). Re-run 2026-09-06 in the container
   (omc 1.26.3): unchanged, 0 of 4 operating points measured, failing on
   `$DER.plant.chassis.detailedChassis.spaceFrame.midToFore.shape.r[3]`.

1. **`VehicleRT`** (design §1.5) — the table-driven-suspension BobLib variant.
   The biggest remaining engineering item, and it is a **BobLib PR**, not work
   in this repository: BobLib is a read-only input here. **The structural half
   of Phase 0 already argues for it**: 28 non-linear systems and 3 state events
   have no readable upper bound on step cost, and the real FMU does in fact fail
   to step at 1 kHz (§1). Do not start it on that evidence alone — get the
   sweep (item 0) first, because it says which states are responsible.
2. **Live tunables end to end.** Kernel side is done (`set_tunable`, FMI
   `tunable` writes, `replay --tunable`). Blocked on BobLib: `python -m bobdil
   build` prints a `not tunable:` line naming every parameter compiled as a
   constant, and each of those record fields must lose `Evaluate=true` and be
   re-exported with `variability="tunable"` to be settable live. `brake_bias`
   is a separate case: it prints as `absent:` because `pVehicle.pVCU.brakeBias`
   is not exported by the FMU at all, so it needs adding, not just re-flagging.
3. **Test with a real wheel.** `io/sdl3.rs` links and the ABI was verified
   against `sizeof`/`offsetof` on the real headers, but no torque has ever been
   delivered to hardware. Run `selftest` first, then `devices`, then `run` with
   `--torque-limit` set **below** the wheel's capability.
4. **Correlate `Reduced14Dof` against BobLib** (design §8, using BobSim's
   `fidelity_suite.py`). This is the gate that should permit it onto the ladder,
   and it cannot run until `VehicleFMI` builds — which it now can.
5. **Phase 9: packaging and tier-1 installers.** Untouched.
6. **Look at the view on a screen.** `events.gd` now draws skidpad,
   acceleration and autocross to competition dimensions and `driver_view.gd`
   renders them, but every check so far has been `make view-check`, which is
   headless: it proves the view reads live physics, not that the scene looks
   right to a driver.

---

## 5. Things learned the hard way — do not re-derive these

**`mlockall(MCL_FUTURE)` breaks the process on stock Linux.** `RLIMIT_MEMLOCK`
defaults to 8 MB and cannot be raised without `CAP_SYS_RESOURCE`; under
`MCL_FUTURE` every later allocation must fit inside it, so allocations start
failing outright. `sys/sched.rs` now checks the limit, falls back to
`MCL_CURRENT`, and prints exactly what to change. **Hardening must happen after
preallocation** or `MCL_CURRENT` covers nothing.

**The deadline must be advanced after the sleep, not before.** Advancing first
made every step be judged against a slot it had already spent: a loop running at
1.5% budget reported a 99.98% miss rate. Fixed in `loop_runner.rs`.

**Godot's `FileAccess` buffers.** A handle held open across frames returns the
bytes it first read, forever, while looking completely healthy — the view
rendered one frozen frame and reported success. `state_link.gd` re-opens the
segment on every read. Do not "optimise" this back.

**A relative host path in `docker/compose.yml` resolves against `docker/`, not
the repository root** — and getting it wrong does not fail. Docker silently
creates an empty directory and mounts *that*, so `/boblib` existed, `doctor`
reported the mount as healthy, and the build failed with "BobLib was not found".
`make vehicle-fmu` now passes absolute paths resolved by `bobdil.paths`, and
`require-boblib` refuses to start rather than mounting nothing.

**A container that writes into a bind mount must run as the invoking user.**
Both images originally ran as root, and one `make test-container` left 778
root-owned files across `kernel/target/`, `build/` and — worst — inside `.venv/`,
where a failed `python3 -m venv` on the host's mounted venv created a dangling
`bin/python3.13`. `make clean` then fails on the user's own repository.
`compose.yml` now sets `user:` from `BOBDIL_UID`/`BOBDIL_GID`, which the
makefile fills in. Two consequences fall out of that and are handled in the
images: `installPackage` puts Modelica libraries under `/root/.openmodelica`
where a non-root user cannot read them (so `Dockerfile.omc` copies them to
`/usr/lib/omlibrary`), and both cargo and pip need a writable `HOME`.

**Debian bookworm has no `libsdl3-dev`.** `Dockerfile.kernel` is on trixie for
that reason alone: on bookworm the image can build the headless kernel and
nothing else, and `make lint` clippies the SDL3 backend, so the gate would fail
rather than skip.

**A venv inside a bind-mounted workspace is the host's venv.** The kernel image
builds its own at `/opt/venv` and points `$PYTHON` at it. `make venv` inside the
container would otherwise try to reuse the host's `.venv`, whose interpreter is
symlinked to a Python that does not exist in the image — and the error names a
path that looks perfectly healthy from outside.

**A Python subprocess's output arrives before anything the parent printed**
unless the parent flushes first. The A/B report printed both kernel runs and
*then* both `--- A ---` / `--- B ---` headings. `kernel.run` now flushes stdout
before every uncaptured invocation.

**Bisection on the stability polynomial finds a fake step for an unstable
mode.** Forward Euler's `|1 + i·h·ω|` is mathematically greater than one for
every positive `h`, but rounds to exactly one once `(hω)²` drops under the
double epsilon — so the search "found" a stable step of a few nanoseconds.
`eigen.MINIMUM_USEFUL_STEP_S` reports a bound that small as no bound at all.

**`-d=symbolicJacobian` is not a valid omc 1.27 flag**, and is not needed: the
integrator is explicit RK4. `--fmiSources=false` matters for a different reason
— BobLib is GPL-3.0 and shipping generated C inside a distributed artefact is a
licensing question best not answered by accident (design §5.5, still unanswered).

**`translateModel` is enough for the structural report.** It runs the whole
front and back end — which is where every structural fact is decided — and skips
the C compile, which is the expensive half. `<model>_info.json` and
`<model>_init.xml` both appear.

**omc's `linearize()` needs the inputs bound, not fed.** `rt_bench` generates a
one-line wrapper model per operating point that pins the three driver commands
as modifiers. With no free inputs, omc's `A` is the whole state matrix and its
eigenvalues are the model's modes rather than a closed loop that includes
whatever driver model happened to be attached. The wrapper is generated rather
than checked in, because the operating point is part of the measurement.

**GDScript `const` cannot hold a constructor call.** `PackedStringArray([...])`
in a `const` fails to resolve at parse time; the emitter uses a plain `Array`
literal. Do not "fix" the type back.

**Godot needs `--path view` from the repo root**, not a `cd`.

**Copying Modelica libraries out of `/root` is not enough to make them
visible.** Once the container runs as the host's user, `installPackage`'s
libraries under `~/.openmodelica` are unreadable, so the image copies them to
`/usr/lib/omlibrary`. That still reported both libraries MISSING: omc 1.26
builds MODELICAPATH from `$HOME/.openmodelica/libraries` and never consults
`/usr/lib/omlibrary`. **`ENV OPENMODELICALIBRARY=/usr/lib/omlibrary` is the part
that works** — verified with `getModelicaPath()` under `--network none`, which
also proves omc is not quietly downloading the package index to cover the gap.
Watch for that download: it makes the failure disappear on a networked machine
and come back in CI.

**Absolute paths baked into a build artefact are wrong the moment the artefact
moves.** `manifest.py` wrote `resources=file:///workspace/...` — correct only
inside the container that compiled it. On the host the kernel handed that URI
straight to `fmi2Instantiate` and the FMU could not find its sparsity patterns.
`library=` was already relative; `resources=` was the one that was not. Both are
now relative and resolved by the kernel against the plant directory, and
`build --link` writes a *relative* symlink for the same reason. Beware the
follow-on: `file://` requires an absolute path, so a relative `--plant` produced
`file://build/x/resources`, where `build` parses as the **authority** and the
FMU receives `/x/resources`. `resource_uri()` canonicalises first.

**A wall-clock assertion inside the unit suite measures the machine, not the
code.** `ladder.rs` asserted the floor kernel's p99.9 against the 1 kHz gate;
cargo runs tests on every core at once, so the suite went red purely because a
Modelica compile was running beside it. The gate lives in `make bench`, which
reports it next to the scheduling policy and memlock limit actually obtained.
A test failing should mean the code is wrong.

**`docker/` had no `.dockerignore`.** The images `COPY` exactly one file and
bind-mount the rest, but every `make omc-image` shipped 170 MB of build output
to the daemon first.

**`toolchain.require()` used to demand BobLib's libraries for every build.** The
requirement is now per-model: `require(libraries)` / `build(..., requires=...)`,
with `fmu_build.BOBDIL_ONLY` for the fixture.

---

## 6. Design decisions already made, with reasons — do not relitigate

- **Model Exchange only.** `fmi2DoStep` in a CS FMU is unbounded work with no
  deadline. `model_description.py` refuses a CS-only FMU with that explanation.
- **Zero external Rust crates.** All FFI is hand-written and audited
  (`sys/`, `plant/fmi2.rs`, `io/sdl3.rs`). Builds offline, no supply chain.
- **The kernel never parses XML.** The session resolves value references and
  writes a flat `key=value` manifest (`bobdil_plant.manifest`).
- **BobLib and BobSim are read-only inputs**, located by `$BOBDIL_BOBLIB` /
  `$BOBDIL_BOBSIM` or as siblings, mounted `:ro` in the containers. BobDil-specific
  Modelica lives in `modelica/BobDil/`. Nothing here writes to either repo.
- **Packed structs on the RT path, protobuf on the control path**, both from
  `schema/bobdil_signals.yaml`. A `layout_hash` in every shm header makes a
  mismatched reader refuse to attach rather than misread bytes.
- **The kernel is selected once at session start, never mid-drive.**
- **Pose is dead-reckoned in one place** (`kernel/src/pose.rs`) and shared by the
  live loop and replay. Two copies would let a paired A/B show a trajectory
  difference that came from the integrator rather than from the setup — the
  exact failure A/B exists to rule out.
- **Every replay verifies its own determinism**, by running twice and comparing
  bit-for-bit before reporting anything. A comparison built on a plant that
  turned out to be non-deterministic is worse than no comparison, because it
  still looks like a result.
- **Paired A/B replays both sides**, including the baseline, rather than reusing
  the original recording. The original came off a live loop on a real clock and
  its health fields carry this machine's jitter; replaying both leaves the setup
  as the only asymmetry.
- **Blind A/B's run order is balanced, not independently random.** With free
  coin flips a driver who always guesses the more frequent setup beats chance
  without feeling anything, and the binomial test would believe them.

## 7. Honest gaps in what is built

- The **fixture plant is not a vehicle.** `DilSmokePlant` has no combined-slip
  limit, so the probe maneuver spins it — expected, and documented as a fixture
  for the FMI path. Do not read physics conclusions from it.
- `Reduced14Dof` is **plausible, not validated**. It has not been correlated
  against BobLib. That correlation is item 4 of section 4.
- **Paired A/B has only ever been run against `Reduced14Dof`.** The machinery is
  plant-agnostic and `--plant` is wired through, but every number produced so far
  describes the reduced model's response to a setup change, not the real car's.
- **Blind A/B has never been run with a driver.** The order generation and the
  scoring are tested; the human half of the protocol is untested by definition.
- The default `--torque-limit 8` clamps the reduced plant's steering torque a
  lot during the aggressive probe maneuver (reported as `FFB_CLAMPED`, correctly).
  Real driving inputs are far gentler; revisit with a real wheel.
- **Nothing zeroes the wheel on a signal.** `io/watchdog.rs`'s rule 4 claims
  torque is zeroed on "clean shutdown, panic, signal, and the parent process
  dying", but the only mechanism is `impl Drop for SdlDevice`. There is no
  signal handler and no `PR_SET_PDEATHSIG`, so a `SIGTERM`ed or `SIGKILL`ed
  session is relying on the OS to drop the effect when the fd closes — which is
  probably what happens on Linux, and has never been tested on hardware. Either
  install the handler or correct the comment; do not leave the two disagreeing.
- No Windows path has been exercised at all. `shm.rs` and `sched.rs` are POSIX.
- The Godot view has only ever been exercised headless (`make view-check`), so
  the cone layouts are code, not something anyone has seen drawn (§4 item 6).
