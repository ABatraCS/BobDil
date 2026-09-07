"""``python -m roundtrip`` -- read a `.bdtrace` and say where the time went.

Two outputs, because they answer the question at different distances: the
summary is the number you quote, and the Chrome trace is the picture you open
when the number is surprising and you need to see which step did it.

    bobdil-kernel run --trace lap.bdtrace --duration 10
    python -m roundtrip lap.bdtrace                    # the summary
    python -m roundtrip lap.bdtrace -o lap.json        # and the flamegraph
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from . import chrome, reader, report, spans


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="python -m roundtrip",
        description="Where the time goes, from the driver's hand to the driver's hand.",
    )
    parser.add_argument("trace", type=Path, help="a .bdtrace written by --trace")
    parser.add_argument(
        "-o",
        "--out",
        type=Path,
        help="write Chrome Trace Event JSON here (open in Perfetto, "
        "chrome://tracing or speedscope)",
    )
    args = parser.parse_args(argv)

    try:
        trace = reader.read(args.trace)
    except (reader.TraceError, OSError) as error:
        # Loud, and fatal. A trace that cannot be interpreted is not a trace to
        # work around; every number in it would still look plausible.
        print(f"error: {error}", file=sys.stderr)
        return 2

    derived = spans.derive(trace.steps, trace.devices)
    if not derived.steps:
        print(f"{args.trace}: no steps recorded -- was --trace given?", file=sys.stderr)
        return 1

    print(report.render(derived, meta=trace.meta))

    if args.out:
        args.out.write_text(json.dumps(chrome.build(derived)), encoding="utf-8")
        print(f"\nwrote {args.out} -- open it in Perfetto (ui.perfetto.dev) or speedscope")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
