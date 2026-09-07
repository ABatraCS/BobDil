# Agent notes

BobDil is a driver-in-the-loop rig over BobLib's Modelica vehicle model. It is
three languages in one repository and the boundaries between them are the whole
design, so read [`docs/architecture.md`](docs/architecture.md) before any
non-trivial change. [`HANDOFF.md`](HANDOFF.md) is the current state: what is
built and verified, what is left, and the things already learned the hard way.

## Before you start

```bash
make doctor    # what is installed, what is missing, and what each gap costs
make help      # the real target list -- authoritative over any doc
make ci        # the full gate: hermetic tests, then this machine's validations
```

`make doctor` first, every time. It prints the schema `layout_hash`, whether
BobLib and BobSim were located, whether this machine's `omc` can build BobLib
(on most it cannot -- see *Containers* below), and the real-time posture the
loop will actually get.

## The rules that are load-bearing

- **The schema is the single source of truth.** `schema/bobdil_signals.yaml`
  emits the Rust struct, the C header, the Python dataclass, the GDScript
  reader and the `.proto` files. Never hand-edit anything under a `generated/`
  directory; edit the schema and run `make codegen`. `make codegen-check` is the
  drift gate, and the `layout_hash` in every shared-memory header is its runtime
  half: a reader with the wrong hash refuses to attach rather than misreading
  bytes.
- **BobLib and BobSim are read-only inputs.** Located by `$BOBDIL_BOBLIB` /
  `$BOBDIL_BOBSIM` or as sibling checkouts, mounted `:ro` in the containers, and
  never written to. Modelica that BobDil needs and BobLib does not provide goes
  in `modelica/BobDil/`. A BobLib change is a PR in that repository, not an edit
  from in here.
- **The kernel has no external crates, and that is deliberate.** All FFI is
  hand-written and audited (`kernel/src/sys/`, `plant/fmi2.rs`, `io/sdl3.rs`).
  It builds offline with no supply chain. Do not add a dependency to solve a
  problem that 200 lines solves.
- **The kernel never parses XML, allocates on the hot path, or blocks.** The
  session resolves value references and writes a flat `key=value` manifest
  (`bobdil_plant.manifest`) that the kernel reads.
- **`plant/` must not reference `io/`, and `io/` must not reference `plant/`.**
  Nothing in the physics knows a wheel exists; nothing in the device layer knows
  what a tyre is. That is what makes the kernel swappable and the device layer
  testable with no hardware.
- **Model Exchange only, never Co-Simulation.** `fmi2DoStep` runs the FMU's own
  variable-step solver: unbounded work per call with no way to impose a
  deadline. `model_description.py` refuses a CS-only FMU and says why.
- **Safety code is reviewed as safety code.** `io/watchdog.rs`, `io/ffb.rs` and
  the `selftest` verb exist because a direct-drive wheel can break a wrist. Run
  `make selftest` before any person touches the rig, and re-run it on every
  kernel change. `python -m bobdil drive` runs it for you and refuses to drive
  if it fails -- do not add a flag to skip that.

## Testing versus validation -- the split matters

| Group | Targets | Means |
| --- | --- | --- |
| **testing** | `test` `test-python` `test-kernel` `test-kernel-sdl3` `lint` `codegen-check` | Hermetic. Same answer on any machine. Safe in CI. |
| **validation** | `validate` `doctor` `selftest` `bench` `record` `replay-check` `view-check` `trace` | Measures *this* box: its clock, its scheduler, its OS. **The numbers do not travel.** |
| **model** | `rt-bench` `ab-paired` | Measures the *model*: structural work per step, largest stable step, what a setup change does. These numbers do travel. |
| **production** | `build` `codegen` `fixture-fmu` `vehicle-fmu` `drive` `view` `release` | Artefacts, and driving the rig. |

Never quote a validation number as a property of the model, or a `rt-bench`
number as a property of a machine. Phase 0 needs both and says so.

## Containers

Most machines' `omc` ships MSL 3.2.3 and no VehicleInterfaces, and BobLib needs
MSL 4.1.0 with VehicleInterfaces 2.0.2. `docker/Dockerfile.omc` is the
toolchain that can actually build the car, and it deliberately mirrors BobSim's
image: a BobDil FMU and a BobSim FMU must come from the same compiler and the
same library versions or the two repositories' results are not comparable.
Bump the two together.

```bash
make omc-image  vehicle-fmu       # compile the real BobLib.…VehicleFMI
make rt-bench-vehicle             # Phase 0 on the real vehicle
make kernel-image  test-container # the hermetic gate in a fixed environment
```

Container targets wrap the same makefile rather than duplicating it. Keep it
that way: a container recipe that reimplements a host recipe is a second thing
to keep true, and the one run less often is the one that rots.

## Where things live

```
schema/       the single source of truth for every signal that crosses a boundary
codegen/      the five emitters
kernel/       Rust. Soft-real-time. No crates. Never blocks.
session/      Python. Everything allowed to be slow: compile, configure, A/B.
tools/rt_bench/  Phase 0: is a model steppable at all?
tools/roundtrip/ where round-trip latency goes, from a --trace .bdtrace
view/         Godot 4. Read-only view of the state segment, by construction.
modelica/     BobDil's own Modelica. Depends on no library, so it builds anywhere.
docker/       the two toolchains
tests/        Python tests. Hermetic -- no omc, no kernel, no hardware.
```

One responsibility per file. If what you are adding does not fit what a file is
for, add a file.

## Writing code here

- Comments say *why*, not *what*. Most of this repository's comments record a
  decision and the failure it prevents; match that. If a line exists because
  something broke, say what broke.
- New behaviour goes in a module, not in a makefile recipe. A recipe cannot be
  unit tested, and anything that only exists in the makefile can only be run
  there. Makefile targets should be one line over `python -m bobdil` or the
  kernel's own command line.
- A gate that measures the same bytes twice with two different opinions is
  worse than one gate. Generated files are linted but not formatted, for exactly
  that reason -- see the comments in `ruff.toml` and the makefile.
- Report what you actually ran. If a test failed, say so with the output; if a
  step was skipped, say that. `HANDOFF.md` section 7 is an honest list of gaps
  and should stay one.
