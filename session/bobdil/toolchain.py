"""Finding, and honestly reporting on, an OpenModelica installation.

The spec asks for a hermetic app that works "regardless of OS, system state,
architecture" *and* for the user to be able to edit ``vehicle.yml`` and
recompile. Those cannot both be true: recompiling needs a Modelica compiler,
and an FMU contains a compiled native binary for one platform
(architecture.md 5.4).

The resolution is a split, and it is stated rather than hidden:

* **Driving a stock vehicle needs no toolchain at all.** Prebuilt FMUs ship per
  tier-1 platform, and the built-in reduced kernel needs nothing whatsoever.
* **Recompiling a custom vehicle needs OpenModelica**, detected here.

The environment keys below are deliberately the same ones BobSim already
probes, so a machine set up for one is set up for the other.
"""

from __future__ import annotations

import os
import shutil
import subprocess
from dataclasses import dataclass, field
from pathlib import Path

OMC_ENV_KEYS = ("BOBDIL_OMC", "BOBSIM_OMC", "BOBDYN_OMC", "OMC")
HOME_ENV_KEYS = (
    "BOBDIL_OPENMODELICA_HOME",
    "BOBSIM_OPENMODELICA_HOME",
    "BOBDYN_OPENMODELICA_HOME",
    "OPENMODELICAHOME",
)
#: BobLib's declared dependencies. A toolchain missing either cannot build it.
REQUIRED_LIBRARIES = ("Modelica", "VehicleInterfaces")
VERIFY_TIMEOUT_S = 60


@dataclass
class Toolchain:
    omc: Path | None = None
    version: str = ""
    source: str = ""
    libraries: dict[str, str] = field(default_factory=dict)
    errors: list[str] = field(default_factory=list)

    @property
    def available(self) -> bool:
        return self.omc is not None

    def satisfies(self, libraries: tuple[str, ...]) -> bool:
        """True when omc is present and every named library is installed.

        Taking the requirement as an argument rather than assuming BobLib's is
        what lets BobDil's own Modelica build against a bare omc. The fixture
        plant under ``modelica/BobDil/`` deliberately depends on neither MSL nor
        VehicleInterfaces, precisely so the FMI path can be exercised on a
        machine that cannot yet compile the real vehicle.
        """
        return self.available and all(self.libraries.get(name) for name in libraries)

    @property
    def can_build(self) -> bool:
        """True only when every library BobLib needs is actually installed."""
        return self.satisfies(REQUIRED_LIBRARIES)

    def describe(self) -> str:
        if not self.available:
            return (
                "OpenModelica: not found. "
                "BobDil will still run prebuilt FMUs and the built-in reduced kernel; "
                "rebuilding a vehicle from vehicle.yml needs omc on PATH or $BOBDIL_OMC set."
            )
        lines = [
            f"OpenModelica: {self.omc} ({self.version or 'version unknown'}) from {self.source}"
        ]
        for name in REQUIRED_LIBRARIES:
            found = self.libraries.get(name)
            lines.append(f"  {name}: {found or 'MISSING'}")
        lines.extend(f"  ! {error}" for error in self.errors)
        return "\n".join(lines)


def _candidates() -> list[tuple[Path, str]]:
    found: list[tuple[Path, str]] = []
    for key in OMC_ENV_KEYS:
        value = os.environ.get(key)
        if value:
            found.append((Path(value).expanduser(), f"${key}"))
    for key in HOME_ENV_KEYS:
        value = os.environ.get(key)
        if value:
            found.append((Path(value).expanduser() / "bin" / "omc", f"${key}"))
    on_path = shutil.which("omc")
    if on_path:
        found.append((Path(on_path), "PATH"))
    for common in (
        Path("/usr/bin/omc"),
        Path("/usr/local/bin/omc"),
        Path("/opt/openmodelica/bin/omc"),
    ):
        found.append((common, "well-known location"))
    return found


def _run_script(omc: Path, script: str, timeout: int = VERIFY_TIMEOUT_S) -> str:
    """Run a Modelica script through omc and return its stdout."""
    import tempfile

    with tempfile.NamedTemporaryFile("w", suffix=".mos", delete=False) as handle:
        handle.write(script)
        script_path = Path(handle.name)
    try:
        result = subprocess.run(
            [str(omc), str(script_path)],
            capture_output=True,
            text=True,
            timeout=timeout,
            check=False,
        )
        return result.stdout
    finally:
        script_path.unlink(missing_ok=True)


def detect() -> Toolchain:
    """Find omc and check that it can actually load what BobLib needs.

    Deliberately verifies the libraries rather than only the binary: an omc that
    cannot load VehicleInterfaces will fail deep inside a build with an error
    that reads like a model problem.
    """
    toolchain = Toolchain()
    for candidate, source in _candidates():
        if not candidate.exists():
            continue
        try:
            version = subprocess.run(
                [str(candidate), "--version"],
                capture_output=True,
                text=True,
                timeout=15,
                check=False,
            ).stdout.strip()
        except (OSError, subprocess.SubprocessError) as error:
            toolchain.errors.append(f"{candidate}: {error}")
            continue
        toolchain.omc = candidate
        toolchain.version = version
        toolchain.source = source
        break

    if toolchain.omc is None:
        return toolchain

    checks = "\n".join(f"getVersion({name}); getErrorString();" for name in REQUIRED_LIBRARIES)
    script = "\n".join([f"loadModel({name});" for name in REQUIRED_LIBRARIES] + [checks])
    output = _run_script(toolchain.omc, script)
    # strict=False on purpose: an omc without VehicleInterfaces echoes fewer
    # lines than we asked about, and detecting that is the whole point here.
    for name, line in zip(REQUIRED_LIBRARIES, _quoted_lines(output), strict=False):
        toolchain.libraries[name] = line
    return toolchain


def _quoted_lines(output: str) -> list[str]:
    """Pull the quoted results out of omc's echoed output."""
    results: list[str] = []
    for raw in output.splitlines():
        line = raw.strip()
        if line.startswith('"') and line.endswith('"') and len(line) > 2:
            results.append(line.strip('"'))
    return results


def require(libraries: tuple[str, ...] = REQUIRED_LIBRARIES) -> Toolchain:
    """Detect, or explain precisely what is missing.

    ``libraries`` is what the model being built actually needs. It defaults to
    BobLib's dependencies; pass ``()`` for a model that has none.
    """
    toolchain = detect()
    if not toolchain.satisfies(libraries):
        raise RuntimeError(
            "cannot build a vehicle without a working OpenModelica toolchain.\n"
            + toolchain.describe()
            + "\n\nBobDil does not need this to drive a prebuilt vehicle or the "
            "built-in reduced kernel -- only to rebuild from vehicle.yml."
        )
    return toolchain
