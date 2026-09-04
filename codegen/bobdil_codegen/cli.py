"""`python -m bobdil_codegen` -- regenerate every binding from the schema.

Writes are idempotent and content-compared, so `--check` can be used in CI to
fail a build whose generated files have drifted from the schema.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from . import emit_c, emit_gdscript, emit_manifest, emit_proto, emit_python, emit_rust, schema

OUTPUTS: dict[str, str] = {
    "kernel/src/generated/frames.rs": "rust",
    "include/bobdil_frames.h": "c",
    "session/bobdil/generated/frames.py": "python",
    "view/scripts/generated/frames.gd": "gdscript",
    "schema/generated/frames.proto": "proto_frames",
    "schema/generated/session.proto": "proto_session",
    "schema/generated/signals.json": "manifest",
}

RENDERERS = {
    "rust": emit_rust.render,
    "c": emit_c.render,
    "python": emit_python.render,
    "gdscript": emit_gdscript.render,
    "proto_frames": emit_proto.render_frames,
    "proto_session": emit_proto.render_session,
    "manifest": emit_manifest.render,
}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Generate BobDil signal bindings.")
    parser.add_argument(
        "--check",
        action="store_true",
        help="Do not write; exit non-zero if any generated file is stale.",
    )
    parser.add_argument("--root", type=Path, default=schema.REPO_ROOT)
    args = parser.parse_args(argv)

    model = schema.load()
    stale: list[str] = []
    written: list[str] = []

    for relative, kind in OUTPUTS.items():
        target = args.root / relative
        content = RENDERERS[kind](model)
        current = target.read_text(encoding="utf-8") if target.exists() else None
        if current == content:
            continue
        if args.check:
            stale.append(relative)
            continue
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(content, encoding="utf-8")
        written.append(relative)

    if args.check:
        if stale:
            print("stale generated files (run `make codegen`):", file=sys.stderr)
            for relative in stale:
                print(f"  {relative}", file=sys.stderr)
            return 1
        print(f"codegen up to date (layout_hash {model.layout_hash:#018x})")
        return 0

    for relative in written:
        print(f"wrote {relative}")
    if not written:
        print("codegen up to date")
    print(f"layout_hash {model.layout_hash:#018x}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
