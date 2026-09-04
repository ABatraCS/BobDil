"""BobDil session layer.

Everything that is allowed to be slow. Compilation, configuration, disk, and the
control UI live here, deliberately outside the real-time process, so that none
of them can stall a step. It can crash without stopping a drive.
"""


from . import (
    ab,
    doctor,
    fmu_build,
    kernel,
    manifest,
    model_description,
    paths,
    recording,
    toolchain,
)

__all__ = [
    "ab",
    "doctor",
    "fmu_build",
    "kernel",
    "manifest",
    "model_description",
    "paths",
    "recording",
    "toolchain",
]
