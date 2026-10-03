"""Canonical linear RGBA values, explicitly declared float32 (never implicit casts)."""

import math

from .values import ArrayValue


def validate_particle_colors(value: ArrayValue, shape: tuple[int, ...]) -> None:
    if not isinstance(value, ArrayValue) or value.dtype != "float32" or value.shape != shape:
        raise ValueError("particle colors must be an explicit float32 RGBA array matching particle axes")
    if any(not math.isfinite(float(v)) or not 0.0 <= float(v) <= 1.0 for v in value.values):
        raise ValueError("particle colors must contain finite linear RGBA in [0,1]")
