"""Run omc, join the three measurements, and print one answer.

This is the only module here that shells out. The parsers in
:mod:`rt_bench.structure` and :mod:`rt_bench.eigen` are deliberately free of it
so they can be tested against captured compiler output on a machine with no
Modelica at all; everything that needs a real omc is here.

The report joins measurements of three different kinds and is careful to keep
them distinct, because they fail for different reasons and are fixed by
different changes:

* **structure** is a property of the model, and travels between machines;
* **stability** is a property of the physics, and travels between machines;
* **timing** is a property of *this box*, and does not travel at all.

Only the third is what ``make bench`` measures, and quoting it as if it were the
first two is exactly how a laptop number ends up in a decision about a car.
"""

from __future__ import annotations

import shutil
import subprocess
import tempfile
from dataclasses import dataclass, field
from pathlib import Path

from bobdil import fmu_build
from bobdil.toolchain import REQUIRED_LIBRARIES, Toolchain
from bobdil.toolchain import require as require_toolchain

from . import eigen, operating_points, structure
from .operating_points import OperatingPoint

#: The wrapper model rt_bench generates around the model under test. It is
#: top-level rather than inside BobDil or BobLib on purpose: rt_bench must not
#: add a class to a package it does not own, and BobLib is a read-only input.
WRAPPER_NAME = "RtBenchOperatingPoint"

#: omc's linearize() always writes here, in the working directory.
LINEARIZED_FILE = "linearized_model.mo"

#: How long to allow omc for one translate or one linearization. A MultiBody
#: vehicle takes minutes in the front end alone.
OMC_TIMEOUT_S = 3600


class BenchError(RuntimeError):
    pass


@dataclass
class Phase0Report:
    model: str
    step_s: float
    method: str
    structure: structure.StructuralReport | None = None
    sweeps: list[eigen.Sweep] = field(default_factory=list)
    failures: list[str] = field(default_factory=list)

    @property
    def limiting(self) -> eigen.Sweep | None:
        """The operating point with the smallest stable step -- the one that binds."""
        bounded = [sweep for sweep in self.sweeps if sweep.max_step_s < float("inf")]
        return min(bounded, key=lambda sweep: sweep.max_step_s, default=None)

    def stability_verdict(self) -> str:
        binding = self.limiting
        if binding is None:
            return (
                "No operating point imposed a step bound, which for a vehicle model "
                "means the sweep did not run, not that the model has no dynamics."
            )
        margin = binding.margin(self.step_s)
        if margin < 1.0:
            return (
                f"FAIL: {binding.model} is unstable at {self.step_s * 1e3:.1f} ms with "
                f"{self.method}. The largest stable step is {binding.max_step_s * 1e3:.3f} ms, "
                f"set by {', '.join(binding.binding_states) or 'an unattributed mode'}. "
                "This is a physics result: a faster CPU does not fix it. The options are "
                "a smaller step, an implicit method, or removing that mode from the model."
            )
        if margin < 3.0:
            return (
                f"MARGINAL: stable, but only {margin:.1f}x over the {self.step_s * 1e3:.1f} ms "
                f"step, and the bound moves with the operating point. Sweep more points "
                "before relying on it."
            )
        return (
            f"PASS: the tightest operating point still admits a "
            f"{binding.max_step_s * 1e3:.2f} ms step, {margin:.0f}x the "
            f"{self.step_s * 1e3:.1f} ms budget."
        )

    def describe(self) -> str:
        lines = [
            f"rt_bench -- Phase 0 viability gate for {self.model}",
            f"target: fixed {self.step_s * 1e3:.1f} ms step, {self.method}",
            "",
        ]
        if self.structure is not None:
            lines.append(self.structure.describe())
            lines.append("")
        lines.append(f"stability sweep ({len(self.sweeps)} operating point(s))")
        for sweep in self.sweeps:
            lines.append(sweep.describe(self.step_s))
        lines.append("")
        lines.append(self.stability_verdict())
        if self.failures:
            lines.append("")
            lines.append("not measured:")
            lines.extend(f"  ! {failure}" for failure in self.failures)
        lines.append("")
        lines.append(
            "Structure and stability are properties of the model and travel between "
            "machines. Step *timing* does not: run `make bench` on the machine that "
            "will actually drive, and read the two together."
        )
        return "\n".join(lines)


def _run_omc(toolchain: Toolchain, script: str, work_dir: Path, timeout_s: int) -> str:
    script_path = work_dir / "rt_bench.mos"
    script_path.write_text(script, encoding="utf-8")
    completed = subprocess.run(
        [str(toolchain.omc), str(script_path)],
        capture_output=True,
        text=True,
        timeout=timeout_s,
        check=False,
        cwd=work_dir,
    )
    return completed.stdout + completed.stderr


def _preamble(sources: list[Path], flags: str, work_dir: Path) -> list[str]:
    lines = [f'cd("{work_dir.as_posix()}");']
    for library, version in (("Modelica", "4.1.0"), ("VehicleInterfaces", "2.0.2")):
        lines.append(f'loadModel({library}, {{"{version}"}}); getErrorString();')
    for source in sources:
        lines.append(f'loadFile("{source.as_posix()}"); getErrorString();')
    lines.append(f'setCommandLineOptions("{flags}"); getErrorString();')
    return lines


def wrapper_source(model: str, point: OperatingPoint) -> str:
    """A one-line Modelica model that pins ``model``'s inputs to this point.

    Binding the inputs rather than feeding them from a source block is what makes
    the linearization meaningful: with no free inputs, omc's ``A`` is the whole
    state matrix, and its eigenvalues are the model's modes rather than a
    closed loop that includes whatever driver model was attached.

    It is also why the wrapper is generated instead of checked in: the point is
    part of the measurement, and a checked-in wrapper would silently linearize
    the wrong car the moment the operating points changed.
    """
    modifiers = [f"{name} = {value!r}" for name, value in point.inputs.items()]
    modifiers += operating_points.bind_starts(model, point.starts)
    return (
        f'model {WRAPPER_NAME} "{point.name}" '
        f"{model} plant({', '.join(modifiers)}); "
        f"end {WRAPPER_NAME};"
    )


def structural_report(
    model: str,
    sources: list[Path],
    *,
    flags: str = fmu_build.REALTIME_FLAGS,
    toolchain: Toolchain | None = None,
    work_dir: Path | None = None,
    timeout_s: int = OMC_TIMEOUT_S,
) -> structure.StructuralReport:
    """Translate the model and read the structure omc computed while doing it.

    ``translateModel`` rather than ``buildModelFMU``: the C compile is the
    expensive half and none of it is needed here, because everything this
    reports is decided in the back end.
    """
    toolchain = toolchain or require_toolchain(REQUIRED_LIBRARIES)
    with _scratch(work_dir) as directory:
        script = _preamble(sources, flags, directory) + [
            f"checkModel({model}); getErrorString();",
            f"translateModel({model}); getErrorString();",
        ]
        log = _run_omc(toolchain, "\n".join(script) + "\n", directory, timeout_s)
        try:
            return structure.read(directory, model, check_model_output=log)
        except FileNotFoundError as error:
            raise BenchError(f"{error}\n\nomc said:\n{log[-4000:]}") from error


def sweep(
    model: str,
    sources: list[Path],
    points: tuple[OperatingPoint, ...] = operating_points.DEFAULT,
    *,
    method: str = "rk4",
    flags: str = fmu_build.REALTIME_FLAGS,
    toolchain: Toolchain | None = None,
    work_dir: Path | None = None,
    timeout_s: int = OMC_TIMEOUT_S,
) -> tuple[list[eigen.Sweep], list[str]]:
    """Linearize at every operating point. Returns the sweeps and what failed.

    A point that fails is recorded and the sweep continues. One operating point
    the model cannot reach -- a standing start on a model that divides by speed,
    say -- is itself a finding, and losing the other three to it would hide it.
    """
    toolchain = toolchain or require_toolchain(REQUIRED_LIBRARIES)
    sweeps: list[eigen.Sweep] = []
    failures: list[str] = []

    for point in points:
        with _scratch(work_dir, suffix=point.name) as directory:
            script = _preamble(sources, flags, directory) + [
                f'loadString("{wrapper_source(model, point).replace(chr(34), chr(92) + chr(34))}");'
                " getErrorString();",
                f"linearize({WRAPPER_NAME}, startTime = 0.0, "
                f"stopTime = {point.settle_s}, tolerance = 1e-8); getErrorString();",
            ]
            log = _run_omc(toolchain, "\n".join(script) + "\n", directory, timeout_s)
            produced = directory / LINEARIZED_FILE
            if not produced.exists():
                failures.append(
                    f"{point.name}: omc produced no linear model. It said:\n      {_tail(log)}"
                )
                continue
            try:
                linear = eigen.parse_linearized_model(produced.read_text(encoding="utf-8"))
            except eigen.LinearizationError as error:
                failures.append(f"{point.name}: {error}")
                continue
            linear.name = point.name
            linear.operating_point = {**point.inputs, **point.starts}
            sweeps.append(eigen.analyse(linear, method))
    return sweeps, failures


def run(
    model: str,
    sources: list[Path],
    *,
    step_s: float = 1e-3,
    method: str = "rk4",
    points: tuple[OperatingPoint, ...] = operating_points.DEFAULT,
    flags: str = fmu_build.REALTIME_FLAGS,
    requires: tuple[str, ...] = REQUIRED_LIBRARIES,
    work_dir: Path | None = None,
) -> Phase0Report:
    """The whole Phase 0 gate for one model, minus the timing half."""
    toolchain = require_toolchain(requires)
    report = Phase0Report(model=model, step_s=step_s, method=method)

    try:
        report.structure = structural_report(
            model, sources, flags=flags, toolchain=toolchain, work_dir=work_dir
        )
    except (BenchError, subprocess.SubprocessError) as error:
        report.failures.append(f"structure: {error}")

    report.sweeps, sweep_failures = sweep(
        model,
        sources,
        points,
        method=method,
        flags=flags,
        toolchain=toolchain,
        work_dir=work_dir,
    )
    report.failures.extend(sweep_failures)
    return report


def _tail(log: str, lines: int = 6) -> str:
    interesting = [
        line.strip()
        for line in log.splitlines()
        if line.strip() and line.strip() not in {'""', "true", "false"}
    ]
    return "\n      ".join(interesting[-lines:]) or "(nothing)"


class _scratch:
    """A working directory for one omc invocation.

    Kept when the caller named one, so a failed run can be inspected; thrown
    away otherwise, because a translate leaves several hundred generated C files
    behind and a sweep does it four times.
    """

    def __init__(self, base: Path | None, suffix: str = "structure"):
        self.base = base
        self.suffix = suffix
        self._temporary: str | None = None

    def __enter__(self) -> Path:
        if self.base is not None:
            directory = self.base / self.suffix
            if directory.exists():
                shutil.rmtree(directory)
            directory.mkdir(parents=True)
            return directory
        self._temporary = tempfile.mkdtemp(prefix=f"bobdil-rtbench-{self.suffix}-")
        return Path(self._temporary)

    def __exit__(self, *exception: object) -> None:
        if self._temporary is not None:
            shutil.rmtree(self._temporary, ignore_errors=True)
