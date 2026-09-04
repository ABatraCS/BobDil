"""``python -m bobdil`` -- one command for a whole session.

The kernel has its own command line and keeps it: it must be runnable with
nothing else present, because it is the thing that has to work when everything
else is broken. This is the layer above, and it exists to do the parts the
kernel deliberately cannot -- compile a vehicle, resolve value references, hold
a recording, run two setups against one lap and diff them.

Six verbs, each answering one question:

    doctor    what is installed, and what does each gap cost?
    build     compile a vehicle to an FMU the kernel can load
    bench     can this model be stepped in real time, on this machine?
    drive     drive it
    replay    does this recording reproduce, and what did it do?
    ab        paired and blind A/B -- the reason the rig exists
"""

from __future__ import annotations

import argparse
import os
import shutil
import sys
from pathlib import Path

from . import ab, doctor, fmu_build, kernel, paths, recording

#: Friendly names for the two models BobDil knows how to build.
TARGETS = {
    "fixture": (
        "BobDil.Experiments.DilSmokePlant",
        "BobDil's own Modelica. Builds on a bare omc, proves the FMI path, "
        "and is NOT a vehicle -- draw no physics conclusion from it.",
    ),
    "vehicle": (
        "BobLib.Experiments.Standards.VehicleFMI",
        "The real car. Needs MSL 4.1.0 and VehicleInterfaces 2.0.2, which is "
        "what docker/Dockerfile.omc is for.",
    ),
}


def _resolve(target: str) -> tuple[str, list[Path], tuple[str, ...]]:
    if target == "fixture":
        return TARGETS["fixture"][0], fmu_build.bobdil_sources(), fmu_build.BOBDIL_ONLY
    model = TARGETS["vehicle"][0] if target == "vehicle" else target
    return model, fmu_build.boblib_sources(), fmu_build.REQUIRED_LIBRARIES


def _link(result_dir: Path, name: str) -> Path:
    """A stable path for a cache-keyed build, so nothing has to glob a hash.

    ``is_symlink`` is checked before ``exists``, because ``exists`` follows the
    link: a symlink pointing at a cache entry that has since been cleaned reads
    as absent, and ``symlink_to`` would then fail on a path that is very much
    there.
    """
    link = paths.BUILD_DIR / name
    if link.is_symlink() or link.is_file():
        link.unlink()
    elif link.is_dir():
        shutil.rmtree(link)
    # Relative, so the link survives the tree being seen at a different
    # absolute path than the one that built it. An absolute link written inside
    # the container pointed at /workspace/... and dangled on the host.
    link.symlink_to(os.path.relpath(result_dir.resolve(), link.parent))
    return link


# --- verbs -----------------------------------------------------------------


def command_doctor(_: argparse.Namespace) -> int:
    print(doctor.report())
    return 0


def command_build(arguments: argparse.Namespace) -> int:
    model, sources, requires = _resolve(arguments.target)
    result = fmu_build.build(
        model,
        sources,
        flags=fmu_build.REFERENCE_FLAGS if arguments.reference_flags else fmu_build.REALTIME_FLAGS,
        requires=requires,
        use_cache=not arguments.no_cache,
    )
    print(result.describe())
    link = _link(result.unpacked_dir, arguments.link or f"{arguments.target}-plant")
    print(f"  {link.relative_to(paths.REPO_ROOT)} -> {result.unpacked_dir}")

    # The manifest records, as comments, every tunable the FMU compiled as a
    # constant. That list is the BobLib work item for live tunables
    # (architecture.md 1.9), so it is surfaced here rather than left in a file.
    not_tunable = [
        line
        for line in result.manifest_path.read_text(encoding="utf-8").splitlines()
        if line.startswith("# not tunable:")
    ]
    if not_tunable:
        print()
        print("  These parameters exist but cannot be changed live:")
        for line in not_tunable:
            print(f"    {line[2:]}")
        print(
            "    Each needs Evaluate=true removed in BobLib and re-exporting with "
            "variability='tunable'."
        )
    return 0


def command_bench(arguments: argparse.Namespace) -> int:
    """Both halves of Phase 0: the model's structure, and this machine's timing.

    They are printed together because neither is a viability answer alone. A
    model with bounded work per step that this box cannot step in time is an
    implementation problem; a model this box steps in 3 microseconds that needs a
    0.2 ms step is a physics problem. Only the pair tells you which you have.
    """
    status = 0
    if not arguments.timing_only:
        sys.path.insert(0, str(paths.REPO_ROOT / "tools"))
        try:
            from rt_bench import __main__ as rt_bench_main
        except ImportError as error:
            print(f"structural analysis unavailable: {error}", file=sys.stderr)
            status = 2
        else:
            structural = ["--dt", str(arguments.dt), arguments.target]
            status |= rt_bench_main.main(structural)
            print()

    if arguments.structure_only:
        return status

    plant = ["--plant", str(arguments.plant)] if arguments.plant else []
    result = kernel.run("bench", "--steps", str(arguments.steps), "--dt", str(arguments.dt), *plant)
    return status | result.returncode


def command_drive(arguments: argparse.Namespace) -> int:
    # SAFETY: never skipped. A direct-drive wheel can break a wrist, and the
    # self-test is the check that the torque path fails safe. It costs
    # milliseconds; the alternative costs a person.
    selftest = kernel.run("selftest")
    if not selftest.ok:
        print("safety self-test FAILED -- not driving.", file=sys.stderr)
        return selftest.returncode

    extra: list[str] = []
    if arguments.plant:
        extra += ["--plant", str(arguments.plant)]
    if arguments.duration:
        extra += ["--duration", str(arguments.duration)]
    if arguments.telemetry:
        extra += ["--telemetry", str(arguments.telemetry)]
    return kernel.run(
        "run",
        "--torque-limit",
        str(arguments.torque_limit),
        *extra,
        *arguments.kernel_args,
    ).returncode


def command_replay(arguments: argparse.Namespace) -> int:
    extra: list[str] = []
    if arguments.plant:
        extra += ["--plant", str(arguments.plant)]
    if arguments.out:
        extra += ["--out", str(arguments.out)]
    for setting in arguments.tunable or []:
        extra += ["--tunable", setting]
    result = kernel.run("replay", "--file", str(arguments.file), *extra)
    if result.ok and not arguments.out:
        print()
        print(recording.read(arguments.file).describe())
    return result.returncode


def command_ab(arguments: argparse.Namespace) -> int:
    if arguments.mode == "paired":
        return _ab_paired(arguments)
    return _ab_blind(arguments)


def _ab_paired(arguments: argparse.Namespace) -> int:
    """Replay one lap against two setups and diff them.

    The A side is replayed too, rather than reusing the original recording. It
    would be tempting not to -- the inputs are identical either way -- but the
    original was produced by a live loop on a real clock, and its health fields
    carry this machine's jitter. Replaying both sides means the only asymmetry
    left between them is the setup.
    """
    paths.ensure_build_dirs()
    outputs = arguments.out_dir or paths.RECORDING_DIR
    outputs.mkdir(parents=True, exist_ok=True)
    side_a = outputs / "ab_a.bdt"
    side_b = outputs / "ab_b.bdt"

    plant = ["--plant", str(arguments.plant)] if arguments.plant else []
    for label, destination, tunables in (
        ("A (baseline)", side_a, arguments.a or []),
        ("B", side_b, arguments.b or []),
    ):
        print(f"--- {label} ---")
        flags: list[str] = []
        for setting in tunables:
            flags += ["--tunable", setting]
        result = kernel.run(
            "replay", "--file", str(arguments.file), "--out", str(destination), *plant, *flags
        )
        if not result.ok:
            return result.returncode
        print()

    comparison = ab.compare(
        recording.read(side_a),
        recording.read(side_b),
        label_a=", ".join(arguments.a or []) or "baseline",
        label_b=", ".join(arguments.b or []) or "baseline",
    )
    print(comparison.describe())
    return 0


def _ab_blind(arguments: argparse.Namespace) -> int:
    """Print a balanced, sealed run order for a blind session.

    The order is printed to a file rather than to the terminal, because the
    person running the session is usually also the driver, and a driver who has
    seen the order is no longer blind. Scoring is a separate invocation for the
    same reason.
    """
    if arguments.score is not None:
        order = [line.strip() for line in arguments.order.read_text().split() if line.strip()]
        guesses = [line.strip().upper() for line in arguments.score.read_text().split()]
        score = ab.score_blind(order, guesses)
        print(score.verdict())
        return 0 if score.above_chance else 1

    order = ab.blind_sequence(arguments.trials, seed=arguments.seed)
    arguments.order.parent.mkdir(parents=True, exist_ok=True)
    arguments.order.write_text("\n".join(order) + "\n", encoding="utf-8")
    print(
        f"wrote a balanced {arguments.trials}-trial order to {arguments.order}.\n"
        "Do NOT show it to the driver. Load setup A or B as it says, one trial at a\n"
        "time, and write the driver's guesses (A or B, one per line) to a second file.\n"
        f"Then score it:  python -m bobdil ab blind --order {arguments.order} "
        "--score guesses.txt"
    )
    return 0


# --- parser ----------------------------------------------------------------


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="bobdil",
        description=__doc__.split("\n\n")[0],
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    verbs = parser.add_subparsers(dest="verb", required=True)

    verbs.add_parser("doctor", help="what is installed, and what each gap costs").set_defaults(
        run=command_doctor
    )

    build = verbs.add_parser("build", help="compile a vehicle to an FMU the kernel can load")
    build.add_argument("target", nargs="?", default="fixture", help=" | ".join(TARGETS))
    build.add_argument("--link", help="name for the stable symlink under build/")
    build.add_argument("--no-cache", action="store_true", help="force a recompile")
    build.add_argument(
        "--reference-flags",
        action="store_true",
        help="build with BobLib's offline flags, for comparison against the real-time build",
    )
    build.set_defaults(run=command_build)

    bench = verbs.add_parser("bench", help="Phase 0: structure, stability, and step timing")
    bench.add_argument("target", nargs="?", default="fixture", help=" | ".join(TARGETS))
    bench.add_argument("--dt", type=float, default=1e-3, help="target fixed step, seconds")
    bench.add_argument("--steps", type=int, default=60_000, help="timing sample size")
    bench.add_argument("--plant", type=Path, help="unpacked FMU to time (default: reduced kernel)")
    bench.add_argument("--structure-only", action="store_true", help="skip the timing half")
    bench.add_argument("--timing-only", action="store_true", help="skip the omc analysis")
    bench.set_defaults(run=command_bench)

    drive = verbs.add_parser("drive", help="drive it (runs the safety self-test first)")
    drive.add_argument("--plant", type=Path, help="unpacked FMU (default: reduced kernel)")
    drive.add_argument("--duration", type=float, help="stop after this many seconds")
    drive.add_argument("--telemetry", type=Path, help="record every step here")
    drive.add_argument(
        "--torque-limit",
        type=float,
        default=8.0,
        help="absolute feedback clamp in N.m. SET THIS BELOW YOUR WHEEL'S CAPABILITY.",
    )
    drive.add_argument("kernel_args", nargs=argparse.REMAINDER, help="passed through to the kernel")
    drive.set_defaults(run=command_drive)

    replay = verbs.add_parser("replay", help="reproduce a recording, exactly")
    replay.add_argument("file", type=Path, help="the recording to reproduce")
    replay.add_argument("--plant", type=Path, help="unpacked FMU to replay against")
    replay.add_argument("--out", type=Path, help="write the replayed telemetry here")
    replay.add_argument(
        "--tunable", action="append", metavar="NAME=VALUE", help="setup change to apply first"
    )
    replay.set_defaults(run=command_replay)

    ab_parser = verbs.add_parser("ab", help="paired and blind A/B")
    modes = ab_parser.add_subparsers(dest="mode", required=True)

    paired = modes.add_parser("paired", help="one lap, two setups, diffed")
    paired.add_argument("file", type=Path, help="the recorded lap both setups replay")
    paired.add_argument("--plant", type=Path, help="unpacked FMU to replay against")
    paired.add_argument("-a", action="append", metavar="NAME=VALUE", help="setup A change")
    paired.add_argument("-b", action="append", metavar="NAME=VALUE", help="setup B change")
    paired.add_argument("--out-dir", type=Path, help="where to write the two replays")
    paired.set_defaults(run=command_ab)

    blind = modes.add_parser("blind", help="a sealed run order, and the scoring afterwards")
    blind.add_argument("--trials", type=int, default=10, help="how many runs (must be even)")
    blind.add_argument("--seed", type=int, help="make the order reproducible for an audit")
    blind.add_argument(
        "--order",
        type=Path,
        default=paths.RECORDING_DIR / "blind_order.txt",
        help="where the sealed order is written, and read back for scoring",
    )
    blind.add_argument("--score", type=Path, help="the driver's guesses, one per line")
    blind.set_defaults(run=command_ab)

    return parser


def main(argv: list[str] | None = None) -> int:
    arguments = build_parser().parse_args(argv)
    try:
        return arguments.run(arguments)
    except (RuntimeError, ValueError, OSError) as error:
        # Every failure this layer raises is one of these: NotComparable and
        # RecordingError are ValueErrors, KernelMissing and BuildError are
        # RuntimeErrors. They are caught as a group and printed without a
        # traceback because each one is already a sentence explaining what to do
        # -- a stack trace over the top of that helps nobody.
        print(f"error: {error}", file=sys.stderr)
        return 1
