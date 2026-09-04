"""BobDil schema code generation.

One schema in, five language bindings out. See schema/bobdil_signals.yaml.
"""

from .schema import Field, Frame, Schema, SchemaError, load

__all__ = ["Field", "Frame", "Schema", "SchemaError", "load"]
