# BobDil development targets.
#
# Five groups, in the order you need them:
#
#   testing      cheap, hermetic, no hardware -- the gate a change must pass
#   validation   runs the built kernel against this machine's clock and OS
#   model        measures the model rather than the machine (Phase 0, A/B)
#   containers   the toolchain this host does not have
#   production   build the artefacts and drive the rig
#
# Every target here works today. Targets for things that are designed but not
# built (VehicleRT, and testing with a real wheel) are deliberately absent
# rather than present-and-failing -- see HANDOFF.md section 4 for that list.
#
# Most targets are one line, because the behaviour lives in `python -m bobdil`
# and in the kernel's own command line rather than in a makefile recipe. That is
# deliberate: a recipe cannot be unit tested, and anything that only exists here
# can only be run here.
#
# The split between testing, validation and model matters, and confusing them
# is how a laptop number ends up in a decision about a car.
#
#   A *test* asserts something about the code and gives the same answer on every
#   machine. Safe in CI.
#
#   A *validation* measures this box: whether it holds a 1 ms deadline, whether
#   the safety path reaches the device, whether the view sees live physics. Its
#   numbers describe the machine it ran on and nothing else.
#
#   A *model* measurement -- `rt-bench`, `ab-paired` -- measures the physics:
#   how much work is inside a step, the largest stable step, what a setup change
#   actually does. Those numbers do travel, and they are the ones worth quoting.
#
# Phase 0 needs a validation and a model measurement together. Neither alone is
# a viability answer: a model this box steps in 3 us but which needs a 0.2 ms
# step is a physics problem, and a bounded model this box cannot keep up with is
# an implementation problem.

SHELL := /bin/bash

# rustup installs outside the default PATH for a non-login shell.
export PATH := $(HOME)/.cargo/bin:$(PATH)

PYTHON  ?= .venv/bin/python
# The session layer's CLI. `make` targets are thin wrappers over it so that
# every one of them can also be run by hand, and so no behaviour lives only in
# a makefile recipe.
BOBDIL  := PYTHONPATH=session:tools $(PYTHON) -m bobdil
RUFF    ?= .venv/bin/ruff
CARGO   ?= cargo
RUSTFMT ?= rustfmt
GODOT   ?= godot
DOCKER  ?= docker
# Absolute paths for the read-only mounts, resolved by the same code the session
# layer uses so there is one answer to "where is BobLib". Passed explicitly
# because a *relative* path in compose.yml resolves against docker/, and getting
# that wrong does not fail loudly -- docker creates an empty directory, mounts
# it, and the build reports BobLib as missing while the mount looks healthy.
REPO_PATHS := PYTHONPATH=session $(PYTHON) -c
BOBLIB_PATH ?= $(shell $(REPO_PATHS) 'from bobdil import paths; r = paths.boblib(); print(r.path or "")' 2>/dev/null)
BOBSIM_PATH ?= $(shell $(REPO_PATHS) 'from bobdil import paths; r = paths.bobsim(); print(r.path or "")' 2>/dev/null)
# The containers run as this user, so that build output in the bind-mounted
# workspace belongs to the person who ran make rather than to root.
COMPOSE := BOBDIL_BOBLIB=$(BOBLIB_PATH) BOBDIL_BOBSIM=$(BOBSIM_PATH) \
           BOBDIL_UID=$(shell id -u) BOBDIL_GID=$(shell id -g) \
           $(DOCKER) compose -f docker/compose.yml

KERNEL_MANIFEST := kernel/Cargo.toml
KERNEL          := kernel/target/release/bobdil-kernel

# Headless: the null device stands in for the wheel, so the loop, the buffers,
# the safety path and telemetry are all exercised with no libSDL3 present.
# This is the configuration CI uses.
HEADLESS := --no-default-features

# Rust sources held to `cargo fmt`. src/generated/ is excluded on purpose: its
# shape is the emitter's contract and `codegen-check` already guards it byte for
# byte, so giving rustfmt an opinion on the same bytes only creates a gate that
# fights another gate. skip_children stops rustfmt following `mod generated;`.
RUST_SOURCES := $(shell find kernel/src kernel/tests kernel/examples -name '*.rs' -not -path '*/generated/*' 2>/dev/null | sort)
RUSTFMT_FLAGS := --edition 2021 --config skip_children=true

BENCH_STEPS ?= 60000
# fixture | vehicle | a fully qualified Modelica model name
RT_BENCH_TARGET ?= fixture
RT_BENCH_ARGS   ?=
# Setup changes for `make ab-paired`, as repeated -a/-b flags. The default asks
# the one question the reduced kernel can actually answer today: what does a
# large front anti-roll bar change do to the car over the recorded lap?
AB_A ?=
AB_B ?= -b front_arb_rate=45000
RECORDING   ?= build/recordings/validate.bdt
DRIVE_ARGS  ?=

.DEFAULT_GOAL := help

.PHONY: help venv codegen codegen-check \
        test test-python test-kernel test-kernel-sdl3 lint lint-python lint-rust fmt fmt-check \
        validate doctor selftest bench record replay-check view-check \
        build build-headless fixture-fmu rt-bench ab-paired drive drive-fixture view release ci \
        omc-image kernel-image require-boblib vehicle-fmu vehicle-fmu-inner rt-bench-vehicle \
        test-container shell-omc shell-kernel \
        clean clean-fmu

help:
	@printf '%s\n' \
	  'BobDil targets. `make ci` is the full unattended gate.' \
	  '' \
	  'testing -- hermetic, no hardware, safe in CI' \
	  '  make test            codegen drift + Python + kernel tests + every lint' \
	  '  make test-python     parsers and step-bound maths (no Modelica needed)' \
	  '  make test-kernel     74 Rust tests, headless (null device)' \
	  '  make test-kernel-sdl3  the same tests with libSDL3 linked in' \
	  '  make lint            ruff + clippy + rustfmt --check' \
	  '  make fmt             rewrite Rust formatting in place' \
	  '  make codegen-check   fail if any generated binding drifted from the schema' \
	  '' \
	  'validation -- measures THIS machine; numbers do not travel' \
	  '  make validate        doctor + selftest + bench + replay + view' \
	  '  make doctor          what is installed, what is missing, what that costs' \
	  '  make selftest        SAFETY: the force-feedback path. Run before driving.' \
	  '  make bench           step time vs the 1 ms deadline (BENCH_STEPS=$(BENCH_STEPS))' \
	  '  make record          drive the scripted input and record it' \
	  '  make replay-check    assert a recording replays bit-identically' \
	  '  make view-check      assert the Godot view reads live physics' \
	  '' \
	  'production' \
	  '  make build           release kernel, with wheel support' \
	  '  make build-headless  release kernel, no libSDL3 dependency' \
	  '  make codegen         regenerate all five bindings from the schema' \
	  '  make fixture-fmu     compile the BobDil Modelica fixture to an FMU' \
	  '  make drive           DRIVE IT. Add DRIVE_ARGS="--torque-limit 4"' \
	  '  make drive-fixture   drive the Modelica fixture instead of the reduced plant' \
	  '  make view            open the driver POV (needs a kernel running)' \
	  '  make release         build, then the whole gate, then say it is shippable' \
	  '' \
	  'model -- measures the PHYSICS, not this machine; these numbers travel' \
	  '  make rt-bench        Phase 0: is this model steppable? (RT_BENCH_TARGET=$(RT_BENCH_TARGET))' \
	  '  make ab-paired       one lap, two setups, diffed (AB_B="$(AB_B)")' \
	  '' \
	  'containers -- the toolchain this host does not have' \
	  '  make omc-image       build the image that CAN compile BobLib' \
	  '  make kernel-image    build the Rust + SDL3 image' \
	  '  make vehicle-fmu     compile the real BobLib VehicleFMI, in the container' \
	  '  make rt-bench-vehicle  Phase 0 on the real vehicle, in the container' \
	  '  make test-container   run the hermetic gate inside the kernel image' \
	  '  make shell-omc        a shell in the Modelica container' \
	  '' \
	  'housekeeping' \
	  '  make venv            create .venv from requirements-dev.txt' \
	  '  make clean           remove build products' \
	  '  make clean-fmu       drop the FMU cache (forces a recompile)'

# --- prerequisites ---------------------------------------------------------

$(PYTHON):
	@printf '%s\n' \
	  'No Python environment at .venv -- run `make venv` first.' \
	  'BobDil keeps its own venv rather than installing into the system' \
	  'Python, because the session layer pins versions (requirements-dev.txt)' \
	  'and must not disturb BobLib or BobSim, which are read-only inputs.' >&2
	@exit 1

venv:
	python3 -m venv .venv
	.venv/bin/python -m pip install --upgrade pip
	.venv/bin/python -m pip install -r requirements-dev.txt
	@echo 'venv ready. Now: make test'

# =========================================================================
#  testing
# =========================================================================

test: codegen-check lint test-python test-kernel
	@echo
	@echo 'gate passed: schema in sync, lints clean, all Python and Rust tests green.'

# The Python half of the gate. It covers the parsers that read what omc emits
# and the step-bound maths, against compiler output captured verbatim -- so it
# runs on a machine with no Modelica at all, which is the point.
test-python: $(PYTHON)
	$(PYTHON) -m pytest

test-kernel:
	$(CARGO) test --manifest-path $(KERNEL_MANIFEST) $(HEADLESS)

# Worth running separately: the SDL3 backend is behind a feature flag, so a
# headless-only gate compiles none of io/sdl3.rs and cannot catch a break in it.
test-kernel-sdl3:
	$(CARGO) test --manifest-path $(KERNEL_MANIFEST)

lint: lint-python lint-rust

PYTHON_SOURCES := codegen session tools tests

lint-python: $(PYTHON)
	$(RUFF) check $(PYTHON_SOURCES)
	$(RUFF) format --check --diff $(PYTHON_SOURCES)

lint-rust: fmt-check
	$(CARGO) clippy --manifest-path $(KERNEL_MANIFEST) $(HEADLESS) --all-targets -- -D warnings
	$(CARGO) clippy --manifest-path $(KERNEL_MANIFEST) --all-targets -- -D warnings

fmt-check:
	$(RUSTFMT) $(RUSTFMT_FLAGS) --check $(RUST_SOURCES)

fmt:
	$(RUSTFMT) $(RUSTFMT_FLAGS) $(RUST_SOURCES)

codegen: $(PYTHON)
	PYTHONPATH=codegen $(PYTHON) -m bobdil_codegen

# The drift gate. One schema emits the Rust struct, the C header, the Python
# dataclass, the GDScript reader and the .proto files; if any of them is stale
# then two processes disagree about the bytes in shared memory, which is a
# failure that surfaces as physics that looks subtly wrong rather than as an
# error. The layout_hash in every segment header is the runtime half of this.
codegen-check: $(PYTHON)
	PYTHONPATH=codegen $(PYTHON) -m bobdil_codegen --check

# =========================================================================
#  validation
# =========================================================================

validate: doctor selftest bench replay-check view-check
	@echo
	@echo 'validated on this machine: safety path intact, deadline held,'
	@echo 'replay deterministic, view reading live physics.'

doctor: $(PYTHON)
	@$(BOBDIL) doctor

# SAFETY. A direct-drive wheel puts out enough torque to break a wrist, so this
# is the target that must pass before a person touches the rig: NaN goes to zero
# in one step, the clamp holds, a stalled loop ramps down, shutdown leaves the
# device at zero, and a stale command is refused.
selftest: build
	$(KERNEL) selftest

bench: build
	$(KERNEL) bench --steps $(BENCH_STEPS)

record: build
	@mkdir -p $(dir $(RECORDING))
	$(KERNEL) run --duration 3 --telemetry $(RECORDING)

# Determinism is what makes paired A/B worth anything: if the same inputs do not
# reproduce the same states, a difference between two setups cannot be
# attributed to the setup.
replay-check: record
	$(KERNEL) replay --file $(RECORDING)

# The one interface between the two processes. Godot maps the state segment and
# must see frames that are fresh, untorn, and advancing -- a view that renders
# one frozen frame while reporting success is the failure this catches.
view-check: build
	@set -e ;\
	rm -f build/view-check.log ;\
	$(KERNEL) run --duration 30 --telemetry /dev/null > build/view-check.log 2>&1 & \
	kernel_pid=$$! ;\
	trap "kill $$kernel_pid 2>/dev/null || true" EXIT ;\
	for _ in $$(seq 1 60); do [ -e /dev/shm/bobdil_state ] && break; sleep 0.1; done ;\
	if [ ! -e /dev/shm/bobdil_state ]; then \
	  echo 'kernel never published a state segment; its log:' >&2 ;\
	  cat build/view-check.log >&2 ;\
	  exit 1 ;\
	fi ;\
	$(GODOT) --headless --path view --script scripts/smoke_test.gd

# =========================================================================
#  production
# =========================================================================

build:
	$(CARGO) build --release --manifest-path $(KERNEL_MANIFEST)

# Same output path as `build`, so switching between them relinks. That is
# cargo's normal feature behaviour and is what you want: whichever you ran last
# is what `make bench`, `make selftest` and `make drive` will actually run.
build-headless:
	$(CARGO) build --release --manifest-path $(KERNEL_MANIFEST) $(HEADLESS)

# BobDil's own Modelica, which needs no MSL and so builds against a bare omc.
# This is the fixture that proves the FMI Model Exchange path end to end; it is
# NOT a vehicle, and no physics conclusion should be drawn from it. Building the
# real BobLib.Experiments.Standards.VehicleFMI needs MSL 4.1.0 and
# VehicleInterfaces 2.0.2, which is what docker/Dockerfile.omc is for.
fixture-fmu: $(PYTHON)
	@$(BOBDIL) build fixture --link fixture-plant

# Phase 0, the half that is not timing. Unlike the validation group this
# measures the *model*, not this machine: the structural work inside a step and
# the largest stable explicit step both travel between machines. It needs omc,
# which is why it is not in the hermetic testing group; `make test-python`
# covers its parsers with no Modelica at all.
#
# Read it together with `make bench`. Structure says whether the work per step
# is bounded, stability says whether the step size is admissible, and only bench
# says whether it fits in the budget on the box in front of you.
rt-bench: $(PYTHON)
	PYTHONPATH=session:tools $(PYTHON) -m rt_bench $(RT_BENCH_TARGET) $(RT_BENCH_ARGS)

# One lap, two setups, diffed. The lap comes from `make record`; both sides are
# replayed, so the only difference between them is the setup named in AB_B.
#
# It is in the production group rather than validation because it produces a
# result about the *car*, not about this machine -- and unlike everything in
# validation, its numbers do travel.
ab-paired: build $(PYTHON) $(RECORDING)
	@$(BOBDIL) ab paired $(RECORDING) $(AB_A) $(AB_B)

$(RECORDING): build
	@$(MAKE) record

drive: build selftest
	$(KERNEL) run $(DRIVE_ARGS)

drive-fixture: build selftest fixture-fmu
	@test -d build/fixture-plant || { echo 'no fixture plant; run `make fixture-fmu`' >&2; exit 1; }
	$(KERNEL) run --plant build/fixture-plant $(DRIVE_ARGS)

# Godot needs --path from the repo root; do not turn this into a cd.
view:
	$(GODOT) --path view

release: build test validate
	@echo
	@echo 'release build: kernel at $(KERNEL)'
	@echo 'gates passed on this machine. Read HANDOFF.md section 7 before'
	@echo 'quoting any of it as a claim about the car -- the reduced plant is'
	@echo 'not yet correlated against BobLib, and no wheel has been tested.'

ci: test validate

# =========================================================================
#  containers
#
#  Two images, because there are two toolchains with nothing to say to each
#  other. Both mount this repository and both wrap the same makefile rather
#  than duplicating it -- a container target that reimplements a host target is
#  a second thing to keep true, and the one that is run less often is the one
#  that rots.
#
#  BobLib and BobSim are mounted READ-ONLY. That is the rule the whole layout
#  rests on (architecture.md 3), and mounting them any other way would let a
#  build in here modify an input it does not own.
# =========================================================================

omc-image:
	$(COMPOSE) build omc

kernel-image:
	$(COMPOSE) build kernel

# The first moment of product truth (architecture.md 7, phase 4): the pipeline
# has been proven end to end against BobDil's fixture, but never yet against
# the real car. Expect a long compile -- this is a MultiBody full-vehicle model,
# not the five-state fixture.
# Refuses to start rather than mounting an empty directory over /boblib, which
# is what docker does with a host path that does not exist.
require-boblib:
	@test -n "$(BOBLIB_PATH)" || { \
	  echo 'BobLib was not found. Set $$BOBDIL_BOBLIB to the checkout or place' >&2 ;\
	  echo 'it at ../BobLib. `make doctor` shows what was searched.' >&2 ;\
	  exit 1 ;\
	}

vehicle-fmu: omc-image require-boblib
	$(COMPOSE) run --rm omc make vehicle-fmu-inner

# The half that runs *inside* the container. Split out so the host target stays
# one line and so the same recipe can be run by hand from `make shell-omc`.
# PYTHON is `?=`, so the image's own /opt/venv interpreter wins without the
# container needing to know where this repo keeps its venv.
vehicle-fmu-inner: $(PYTHON)
	$(BOBDIL) build vehicle --link vehicle-plant

rt-bench-vehicle: omc-image require-boblib
	$(COMPOSE) run --rm omc make rt-bench RT_BENCH_TARGET=vehicle

# The same hermetic gate `make test` runs, in a fixed environment. Useful for
# reproducing a CI failure, and for checking that the gate does not depend on
# anything this laptop happens to have.
# No `make venv` here: the image builds its own at /opt/venv and points $PYTHON
# at it. A venv inside the workspace would be the *host's*, bind-mounted in,
# with an interpreter symlinked to a Python this image does not have.
test-container: kernel-image
	$(COMPOSE) run --rm kernel make test

shell-omc: omc-image
	$(COMPOSE) run --rm omc

shell-kernel: kernel-image
	$(COMPOSE) run --rm kernel

# =========================================================================
#  housekeeping
# =========================================================================

clean:
	$(CARGO) clean --manifest-path $(KERNEL_MANIFEST)
	rm -rf build/recordings build/view-check.log
	find . -name '__pycache__' -type d -prune -exec rm -rf {} +

clean-fmu:
	rm -rf build/fmu-cache build/fixture-plant
