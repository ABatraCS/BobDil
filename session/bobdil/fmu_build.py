"""Build an FMU from Modelica, unpack it, and make it loadable by the kernel.

Two things here are load-bearing.

**Model Exchange, never Co-Simulation.** ``fmuType="me"`` is not a preference.
A Co-Simulation FMU owns its solver: ``fmi2DoStep`` runs variable-step DASSL
internally, which is unbounded work per call with no way to impose a deadline,
so a CS FMU cannot be made soft-real-time safe at all (architecture.md 1.4).

**The compiler flags are the opposite of BobLib's defaults.** VehicleFMI's own
annotation asks for ``s="dassl"``, ``indexReductionMethod=dynamicStateSelection``
and ``jacobian="internalNumerical"`` -- a configuration tuned for offline
accuracy, and precisely wrong for a fixed deadline. Selecting states at *runtime*
makes step timing nondeterministic, which is the one thing a real-time loop
cannot tolerate. Overriding them here is a build-target difference, not a fork
of the model: BobLib is read, never modified.
"""

from __future__ import annotations

import hashlib
import shutil
import subprocess
import tempfile
import zipfile
from dataclasses import dataclass
from pathlib import Path

from . import manifest, model_description, paths
from .model_description import ModelDescription
from .toolchain import REQUIRED_LIBRARIES, Toolchain
from .toolchain import require as require_toolchain

#: Compiler options for a real-time build.
#:
#: ``uode``            index reduction that fixes the state set at *compile*
#:                     time. dynamicStateSelection re-selects states while the
#:                     model runs, which makes step cost vary unpredictably.
#: ``PFPlusExt``       the matching algorithm BobLib is already validated with.
#: ``maxSizeLinearTearing`` matches BobLib's setting so tearing behaviour is
#:                     comparable between an offline and a real-time build.
#: ``--fmiSources=false`` keeps the FMU small; the C sources are not needed to
#:                     run one and BobLib is GPL-3.0, so shipping them inside a
#:                     distributed artefact is a licensing question (5.5) best
#:                     not answered by accident.
REALTIME_FLAGS = (
    "--matchingAlgorithm=PFPlusExt "
    "--indexReductionMethod=uode "
    "--maxSizeLinearTearing=5000 "
    "--fmiSources=false"
)

#: The offline flag set, for building a reference FMU to compare against.
REFERENCE_FLAGS = (
    "--matchingAlgorithm=PFPlusExt "
    "--indexReductionMethod=dynamicStateSelection "
    "--maxSizeLinearTearing=5000 "
    "--fmiSources=false"
)


@dataclass
class BuildResult:
    model: str
    fmu_path: Path
    unpacked_dir: Path
    manifest_path: Path
    description: ModelDescription
    log: str
    cached: bool = False

    def describe(self) -> str:
        return (
            f"{self.description.describe()}\n"
            f"  fmu                {self.fmu_path}\n"
            f"  unpacked           {self.unpacked_dir}\n"
            f"  manifest           {self.manifest_path}\n"
            f"  from cache         {self.cached}"
        )


class BuildError(RuntimeError):
    pass


def cache_key(model: str, sources: list[Path], flags: str, toolchain: Toolchain) -> str:
    """Content-addressed key: the model, its sources, the flags and the compiler.

    Every one of these changes the resulting binary, so every one is in the key.
    Leaving the flags out is the classic way to get a cache that silently serves
    an offline-tuned FMU to a real-time session.
    """
    digest = hashlib.blake2b(digest_size=16)
    digest.update(model.encode())
    digest.update(flags.encode())
    digest.update(toolchain.version.encode())
    for source in sorted(sources):
        for path in sorted(source.rglob("*.mo")) if source.is_dir() else [source]:
            digest.update(path.name.encode())
            digest.update(path.read_bytes())
    return digest.hexdigest()


def _mos_script(model: str, sources: list[Path], flags: str, output_dir: Path) -> str:
    lines = [f'cd("{output_dir.as_posix()}");']
    for library, version in (("Modelica", "4.1.0"), ("VehicleInterfaces", "2.0.2")):
        lines.append(f'loadModel({library}, {{"{version}"}}); getErrorString();')
    for source in sources:
        lines.append(f'loadFile("{source.as_posix()}"); getErrorString();')
    lines.append(f'setCommandLineOptions("{flags}"); getErrorString();')
    lines.append(f'buildModelFMU({model}, version="2.0", fmuType="me", platforms={{"dynamic"}});')
    lines.append("getErrorString();")
    return "\n".join(lines) + "\n"


def build(
    model: str,
    sources: list[Path],
    *,
    flags: str = REALTIME_FLAGS,
    toolchain: Toolchain | None = None,
    output_dir: Path | None = None,
    use_cache: bool = True,
    timeout_s: int = 3600,
    vehicle_hash: int = 0,
    requires: tuple[str, ...] = REQUIRED_LIBRARIES,
) -> BuildResult:
    """Compile ``model`` to an FMI 2.0 Model Exchange FMU and prepare it for the kernel.

    ``requires`` is the set of Modelica libraries this model needs, checked
    before omc is invoked so a missing dependency is a clear message rather than
    a wall of translation errors. It defaults to BobLib's; ``bobdil_sources()``
    needs none, which is what lets the fixture build on a bare omc.
    """
    toolchain = toolchain or require_toolchain(requires)
    paths.ensure_build_dirs()

    key = cache_key(model, sources, flags, toolchain)
    target = output_dir or (paths.FMU_CACHE_DIR / f"{model.replace('.', '_')}-{key[:16]}")
    unpacked = target / "unpacked"
    fmu_path = target / f"{model.rsplit('.', 1)[-1]}.fmu"

    if use_cache and (unpacked / manifest.MANIFEST_NAME).exists() and fmu_path.exists():
        description = model_description.parse_file(unpacked / "modelDescription.xml")
        return BuildResult(
            model=model,
            fmu_path=fmu_path,
            unpacked_dir=unpacked,
            manifest_path=unpacked / manifest.MANIFEST_NAME,
            description=description,
            log="(cached)",
            cached=True,
        )

    target.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="bobdil-omc-") as scratch:
        scratch_dir = Path(scratch)
        script = scratch_dir / "build.mos"
        script.write_text(_mos_script(model, sources, flags, scratch_dir), encoding="utf-8")
        completed = subprocess.run(
            [str(toolchain.omc), str(script)],
            capture_output=True,
            text=True,
            timeout=timeout_s,
            check=False,
            cwd=scratch_dir,
        )
        log = completed.stdout + completed.stderr

        produced = sorted(scratch_dir.glob("*.fmu"))
        if not produced:
            raise BuildError(
                f"omc did not produce an FMU for {model}.\n"
                "This is almost always a missing library or a model error; the compiler "
                f"said:\n{log[-4000:]}"
            )
        shutil.copy2(produced[0], fmu_path)

    if unpacked.exists():
        shutil.rmtree(unpacked)
    unpacked.mkdir(parents=True)
    with zipfile.ZipFile(fmu_path) as archive:
        archive.extractall(unpacked)
    # Zip does not preserve the executable bit, and dlopen does not need it, but
    # a readable binary is required.
    for binary in (unpacked / "binaries").rglob("*"):
        if binary.is_file():
            binary.chmod(0o755)

    description = model_description.parse_file(unpacked / "modelDescription.xml")
    manifest_path = manifest.write(description, unpacked, vehicle_hash=vehicle_hash)

    return BuildResult(
        model=model,
        fmu_path=fmu_path,
        unpacked_dir=unpacked,
        manifest_path=manifest_path,
        description=description,
        log=log,
    )


def boblib_sources() -> list[Path]:
    """The Modelica BobDil builds from: BobLib's package, then BobDil's own."""
    package = paths.boblib_package()
    if package is None:
        raise BuildError(
            "BobLib was not found. Set $BOBDIL_BOBLIB to the checkout, or place it "
            "at ../BobLib. BobDil reads BobLib and never writes to it."
        )
    return [package / "package.mo", paths.MODELICA_DIR / "BobDil" / "package.mo"]


def bobdil_sources() -> list[Path]:
    """BobDil's own Modelica only -- no BobLib, so it builds with a bare omc.

    Pair this with ``requires=BOBDIL_ONLY`` so the toolchain check asks for what
    these sources actually need, which is nothing beyond omc itself.
    """
    return [paths.MODELICA_DIR / "BobDil" / "package.mo"]


#: The library requirement that goes with :func:`bobdil_sources`: none at all.
BOBDIL_ONLY: tuple[str, ...] = ()
