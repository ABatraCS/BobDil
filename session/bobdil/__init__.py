"""BobDil session layer.

Everything that is allowed to be slow. Compilation, configuration, disk, and the
control UI live here, deliberately outside the real-time process, so that none
of them can stall a step. It can crash without stopping a drive.
"""


from . import model_description, paths, toolchain

__all__ = ["model_description", "paths", "toolchain"]
