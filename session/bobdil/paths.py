"""Where everything lives.

BobDil is self-contained: nothing in this repository writes to BobLib or BobSim,
and neither is required to be present for the kernel, the view, the schema or
the tests to work. They are *inputs*, located by configuration, mounted
read-only in the container, and consumed without modification.

That containment is deliberate. A BobDil-specific Modelica model belongs in
``modelica/`` here, not as a patch to BobLib, so that BobDil can be cloned,
built and run on its own and so that a BobLib bump is never blocked on BobDil.
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]

SCHEMA_DIR = REPO_ROOT / "schema"
GENERATED_SCHEMA = SCHEMA_DIR / "generated"
KERNEL_DIR = REPO_ROOT / "kernel"
VIEW_DIR = REPO_ROOT / "view"
MODELICA_DIR = REPO_ROOT / "modelica"
BUILD_DIR = REPO_ROOT / "build"
FMU_CACHE_DIR = BUILD_DIR / "fmu-cache"
RECORDING_DIR = BUILD_DIR / "recordings"

#: Environment variables consulted for the external repositories, in order.
BOBLIB_ENV_KEYS = ("BOBDIL_BOBLIB", "BOBLIB_PATH", "BOBLIB")
BOBSIM_ENV_KEYS = ("BOBDIL_BOBSIM", "BOBSIM_PATH", "BOBSIM")


@dataclass(frozen=True)
class ExternalRepo:
    """An external repository BobDil reads but never writes."""

    name: str
    path: Path | None
    source: str

    @property
    def available(self) -> bool:
        return self.path is not None

    def describe(self) -> str:
        if self.path is None:
            return f"{self.name}: not found ({self.source})"
        return f"{self.name}: {self.path} (from {self.source})"


def _from_env(keys: tuple[str, ...]) -> tuple[Path | None, str]:
    for key in keys:
        value = os.environ.get(key)
        if value:
            candidate = Path(value).expanduser()
            if candidate.exists():
                return candidate, f"${key}"
            return None, f"${key} points at {candidate}, which does not exist"
    return None, "no environment override"


def boblib() -> ExternalRepo:
    """Locate the BobLib checkout that holds the Modelica vehicle model."""
    path, source = _from_env(BOBLIB_ENV_KEYS)
    if path is not None:
        return ExternalRepo("BobLib", path, source)

    sibling = REPO_ROOT.parent / "BobLib"
    if (sibling / "BobLib" / "package.mo").exists():
        return ExternalRepo("BobLib", sibling, "sibling checkout")
    return ExternalRepo(
        "BobLib",
        None,
        f"{source}; no ../BobLib sibling. Set $BOBDIL_BOBLIB.",
    )


def boblib_package() -> Path | None:
    """The directory containing BobLib's ``package.mo``."""
    repo = boblib()
    if repo.path is None:
        return None
    inner = repo.path / "BobLib"
    return inner if (inner / "package.mo").exists() else None


def bobsim() -> ExternalRepo:
    """Locate BobSim, used only for the optional vehicle.yml to Modelica step."""
    path, source = _from_env(BOBSIM_ENV_KEYS)
    if path is not None:
        return ExternalRepo("BobSim", path, source)
    sibling = REPO_ROOT.parent / "BobSim"
    if (sibling / "_5_App" / "modelica_generator.py").exists():
        return ExternalRepo("BobSim", sibling, "sibling checkout")
    return ExternalRepo("BobSim", None, f"{source}; no ../BobSim sibling.")


def ensure_build_dirs() -> None:
    for directory in (BUILD_DIR, FMU_CACHE_DIR, RECORDING_DIR):
        directory.mkdir(parents=True, exist_ok=True)
