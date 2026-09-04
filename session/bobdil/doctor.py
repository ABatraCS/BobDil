"""What is installed, what is missing, and what each gap actually costs.

This is the first thing to run in a new session or on a new machine. It is
deliberately opinionated about the *consequences*: "VehicleInterfaces: MISSING"
is not useful on its own, and "so this omc cannot build BobLib, but the fixture
FMU still builds because BobDil's own Modelica depends on neither" is.

It lives here rather than inline in the makefile because both `make doctor` and
`python -m bobdil doctor` need it, and a second copy of an environment report is
a second thing to keep true.
"""

from __future__ import annotations

import shutil
import subprocess
import sys
from pathlib import Path

from . import kernel, paths, toolchain


def _version(*command: str) -> str:
    executable = shutil.which(command[0])
    if executable is None:
        return "NOT FOUND"
    try:
        completed = subprocess.run(command, capture_output=True, text=True, timeout=30, check=False)
        return ((completed.stdout or completed.stderr).strip().splitlines() or ["(silent)"])[0]
    except (OSError, subprocess.SubprocessError) as error:
        return f"present at {executable} but did not answer: {error}"


def _layout_hash() -> str:
    """The schema hash both processes must agree on before they share memory."""
    sys.path.insert(0, str(paths.REPO_ROOT / "codegen"))
    try:
        from bobdil_codegen import schema
    except ImportError:
        return "unavailable (codegen/ not on the path)"
    return f"{schema.load().layout_hash:#018x}"


def _memlock_lines() -> list[str]:
    limits = Path("/proc/self/limits")
    if not limits.exists():
        return ["  memlock            unknown on this platform"]
    soft = None
    for line in limits.read_text().splitlines():
        if line.startswith("Max locked memory"):
            soft = line.split()[3]
    if soft == "unlimited":
        return ["  memlock            unlimited -- the loop can pin every page it touches"]
    if soft is None:
        return ["  memlock            not reported"]
    return [
        f"  memlock            {int(soft) / (1024 * 1024):.0f} MB soft limit",
        "                     Too small to pin future allocations, so a late",
        "                     allocation can page-fault inside a step. Raise it",
        "                     with `ulimit -l unlimited` or, on a container,",
        "                     `--ulimit memlock=-1`. `make bench` reports what",
        "                     the loop actually got.",
    ]


def report() -> str:
    lines = [
        "BobDil doctor",
        "",
        "-- this repo ------------------------------------------------------",
        f"  root               {paths.REPO_ROOT}",
        f"  schema layout_hash {_layout_hash()}",
    ]
    try:
        lines.append(f"  release kernel     {kernel.locate()}")
    except kernel.KernelMissing:
        lines.append("  release kernel     not built -- run `make build`")

    lines += ["", "-- read-only inputs ----------------------------------------------"]
    lines += [f"  {repo.describe()}" for repo in (paths.boblib(), paths.bobsim())]

    lines += ["", "-- toolchain -----------------------------------------------------"]
    for label, command in (
        ("python", (sys.executable, "--version")),
        ("cargo", ("cargo", "--version")),
        ("clippy", ("cargo-clippy", "--version")),
        ("rustfmt", ("rustfmt", "--version")),
        ("godot", ("godot", "--version")),
        ("docker", ("docker", "--version")),
    ):
        lines.append(f"  {label:18} {_version(*command)}")

    found = toolchain.detect()
    lines.append("")
    lines += [f"  {line}" for line in found.describe().splitlines()]
    if found.available and not found.can_build:
        lines += [
            "",
            "  This omc cannot build BobLib: BobLib needs MSL 4.1.0 and",
            "  VehicleInterfaces 2.0.2. `make fixture-fmu` still works, because",
            "  BobDil's own Modelica deliberately depends on neither. To build the",
            "  real vehicle, use the container: `make omc-image` then",
            "  `make vehicle-fmu`.",
        ]

    lines += ["", "-- real-time posture ---------------------------------------------"]
    lines += _memlock_lines()
    lines += [
        "  SCHED_FIFO         needs cap_sys_nice, so an unprivileged run gets",
        "                     SCHED_OTHER and a longer jitter tail. It still holds",
        "                     the deadline on this class of machine; `make bench`",
        "                     prints the scheduling it obtained, not the one asked",
        "                     for. Grant it with",
        f"                     `sudo setcap cap_sys_nice=eip {kernel.RELEASE_BINARY}`.",
    ]
    return "\n".join(lines)
