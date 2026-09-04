"""Finding and running the kernel binary from the session layer.

The session never *is* the loop -- it starts one, waits for it, and reads what
it produced. Keeping that in one module means the CLI does not grow its own
opinions about where the binary lives or how it is invoked, and it is the single
place that knows the session layer must never sit between the loop and the
wheel (architecture.md 1.2).
"""

from __future__ import annotations

import shutil
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

from . import paths

BINARY_NAME = "bobdil-kernel"
#: Release only. A debug kernel misses the deadline on any machine and reporting
#: its timings would be actively misleading.
RELEASE_BINARY = paths.KERNEL_DIR / "target" / "release" / BINARY_NAME


class KernelMissing(RuntimeError):
    pass


def locate() -> Path:
    """The kernel this session will run, or an error saying how to build one."""
    if RELEASE_BINARY.exists():
        return RELEASE_BINARY
    on_path = shutil.which(BINARY_NAME)
    if on_path:
        return Path(on_path)
    raise KernelMissing(
        f"no kernel at {RELEASE_BINARY} and none on PATH. Build one with "
        "`make build` (with wheel support) or `make build-headless` (no libSDL3).\n"
        "Note that only a release build is used: a debug kernel misses the 1 ms "
        "deadline on every machine, so its numbers would describe the compiler "
        "rather than the model."
    )


@dataclass
class Result:
    command: list[str]
    returncode: int
    stdout: str
    stderr: str

    @property
    def ok(self) -> bool:
        return self.returncode == 0

    def raise_for_status(self) -> Result:
        if not self.ok:
            raise RuntimeError(
                f"{' '.join(self.command)} exited {self.returncode}\n"
                f"{self.stderr.strip() or self.stdout.strip()}"
            )
        return self


def run(
    *arguments: str,
    capture: bool = False,
    timeout_s: int | None = None,
) -> Result:
    """Invoke the kernel.

    ``capture`` is off by default so that a drive streams to the terminal as it
    happens. A session that swallowed the kernel's output would hide the safety
    self-test, which is the one thing a person must see before they touch a
    wheel.
    """
    command = [str(locate()), *arguments]
    if not capture:
        # The child writes straight to this terminal, so anything this process
        # has printed must reach it first. Without the flush, a heading printed
        # before the kernel runs appears after everything the kernel said.
        sys.stdout.flush()
    completed = subprocess.run(
        command,
        capture_output=capture,
        text=True,
        timeout=timeout_s,
        check=False,
    )
    return Result(
        command=command,
        returncode=completed.returncode,
        stdout=completed.stdout or "",
        stderr=completed.stderr or "",
    )
