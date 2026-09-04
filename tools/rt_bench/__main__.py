"""``python -m rt_bench`` -- the Phase 0 gate, on one model.

Also reachable as ``python -m bobdil bench --structure``; this entry point
exists so the gate can be run with nothing but ``tools/`` and ``session/`` on
the path, which is what a CI job or a machine that is not set up for BobDil has.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from bobdil import fmu_build, paths

from . import operating_points, report


def _sources(target: str) -> tuple[str, list[Path], tuple[str, ...]]:
    """Resolve a friendly target name to a model, its sources and its libraries."""
    if target == "fixture":
        return (
            "BobDil.Experiments.DilSmokePlant",
            fmu_build.bobdil_sources(),
            fmu_build.BOBDIL_ONLY,
        )
    if target == "vehicle":
        return (
            "BobLib.Experiments.Standards.VehicleFMI",
            fmu_build.boblib_sources(),
            fmu_build.REQUIRED_LIBRARIES,
        )
    # An explicit model name: assume it lives in BobLib, which is the only other
    # place a Modelica model BobDil drives can come from.
    return target, fmu_build.boblib_sources(), fmu_build.REQUIRED_LIBRARIES


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="rt_bench",
        description=(
            "Phase 0: is this Modelica model steppable in real time? Reports the "
            "structural work per step and the largest stable explicit step. Step "
            "timing is measured separately, by the kernel, on the machine that "
            "will drive."
        ),
    )
    parser.add_argument(
        "target",
        nargs="?",
        default="fixture",
        help="fixture | vehicle | a fully qualified Modelica model name",
    )
    parser.add_argument("--dt", type=float, default=1e-3, help="target fixed step, seconds")
    parser.add_argument(
        "--method",
        default="rk4",
        choices=sorted(("euler", "midpoint", "rk4")),
        help="integrator whose stability region bounds the step",
    )
    parser.add_argument(
        "--point",
        action="append",
        choices=[point.name for point in operating_points.DEFAULT],
        help="linearize only at these operating points (default: all four)",
    )
    parser.add_argument(
        "--reference-flags",
        action="store_true",
        help=(
            "build with BobLib's offline flags instead of the real-time ones, to "
            "see what dynamicStateSelection actually costs"
        ),
    )
    parser.add_argument(
        "--keep",
        type=Path,
        help="keep omc's working directories here instead of discarding them",
    )
    arguments = parser.parse_args(argv)

    model, sources, requires = _sources(arguments.target)
    points = (
        tuple(operating_points.by_name(name) for name in arguments.point)
        if arguments.point
        else operating_points.DEFAULT
    )

    paths.ensure_build_dirs()
    result = report.run(
        model,
        sources,
        step_s=arguments.dt,
        method=arguments.method,
        points=points,
        flags=fmu_build.REFERENCE_FLAGS if arguments.reference_flags else fmu_build.REALTIME_FLAGS,
        requires=requires,
        work_dir=arguments.keep,
    )
    print(result.describe())

    binding = result.limiting
    if binding is not None and binding.margin(arguments.dt) < 1.0:
        return 1
    # A sweep that measured nothing is not a pass. Saying so here is what keeps
    # this a gate rather than a report that always succeeds.
    return 0 if result.sweeps else 2


if __name__ == "__main__":
    sys.exit(main())
