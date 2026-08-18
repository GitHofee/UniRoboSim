"""Portable immutable values and canonical coordinate/data conventions."""

from __future__ import annotations

import math
import re
from collections.abc import Iterable
from dataclasses import dataclass
from enum import StrEnum
from functools import reduce
from operator import mul
from typing import Any

from .errors import ValidationError

_PATH_SEGMENT = re.compile(r"^[A-Za-z0-9_][A-Za-z0-9_.-]*$")


class EntityKind(StrEnum):
    RIGID_BODY = "rigid_body"
    ARTICULATION = "articulation"
    SURFACE_DEFORMABLE = "surface_deformable"
    VOLUME_DEFORMABLE = "volume_deformable"
    PARTICLE_FLUID = "particle_fluid"
    CAMERA_SENSOR = "camera_sensor"


class CameraModality(StrEnum):
    """Portable camera channels supported by the M3 contract."""

    RGB = "rgb"
    DEPTH = "depth"


class CommandMode(StrEnum):
    POSITION = "position"
    VELOCITY = "velocity"
    EFFORT = "effort"


class PointCommandMode(StrEnum):
    """World-frame point control mode for deformable nodes and fluid particles."""

    POSITION = "position"
    VELOCITY = "velocity"
    FORCE = "force"


class DeformableTopology(StrEnum):
    """Portable simulation topology, independent of a backend solver implementation."""

    SURFACE = "surface"
    VOLUME = "volume"


class SessionState(StrEnum):
    OPEN = "open"
    READY = "ready"
    CLOSED = "closed"


class WorldState(StrEnum):
    READY = "ready"
    CLOSED = "closed"


class ArrayOwnership(StrEnum):
    OWNED = "owned"


def _validation(message: str, operation: str, **details: object) -> ValidationError:
    return ValidationError(message, operation=operation, details=details)


@dataclass(frozen=True, order=True)
class EntityPath:
    """Absolute backend-independent logical entity path."""

    value: str

    def __post_init__(self) -> None:
        if not isinstance(self.value, str):
            raise _validation("entity path must be a string", "entity_path.validate")
        if self.value == "/" or not self.value.startswith("/") or self.value.endswith("/"):
            raise _validation(
                "entity path must be absolute, non-root, and have no trailing slash",
                "entity_path.validate",
                value=self.value,
            )
        segments = self.value[1:].split("/")
        if any(not segment or segment in {".", ".."} or not _PATH_SEGMENT.fullmatch(segment) for segment in segments):
            raise _validation("entity path contains an invalid segment", "entity_path.validate", value=self.value)

    @property
    def name(self) -> str:
        return self.value.rsplit("/", 1)[-1]

    @property
    def parent(self) -> EntityPath | None:
        parent_value = self.value.rsplit("/", 1)[0]
        return None if not parent_value else EntityPath(parent_value)

    def child(self, segment: str) -> EntityPath:
        if not isinstance(segment, str) or not _PATH_SEGMENT.fullmatch(segment) or segment in {".", ".."}:
            raise _validation("child path segment is invalid", "entity_path.child", segment=segment)
        return EntityPath(f"{self.value}/{segment}")

    def __str__(self) -> str:
        return self.value


def _finite_tuple(values: Iterable[float], length: int, field: str) -> tuple[float, ...]:
    try:
        result = tuple(float(value) for value in values)
    except (TypeError, ValueError) as exc:
        raise _validation(f"{field} must be an iterable of numbers", "pose.validate", field=field) from exc
    if len(result) != length or not all(math.isfinite(value) for value in result):
        raise _validation(f"{field} must contain {length} finite numbers", "pose.validate", field=field)
    return result


def _flatten_nested(value: object) -> tuple[tuple[int, ...], tuple[float | int | bool, ...]]:
    if isinstance(value, (list, tuple)):
        if not value:
            raise _validation("nested arrays cannot contain an empty dimension", "array.from_nested")
        children = tuple(_flatten_nested(item) for item in value)
        child_shape = children[0][0]
        if any(shape != child_shape for shape, _ in children):
            raise _validation("nested arrays must be rectangular", "array.from_nested")
        return (len(children), *child_shape), tuple(item for _, values in children for item in values)
    if isinstance(value, (bool, int, float)):
        return (), (value,)
    raise _validation("nested arrays may only contain numeric or boolean scalars", "array.from_nested")


@dataclass(frozen=True)
class Pose:
    """Pose in SI metres and an XYZW unit quaternion."""

    position: tuple[float, float, float] = (0.0, 0.0, 0.0)
    orientation_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)

    def __post_init__(self) -> None:
        position = _finite_tuple(self.position, 3, "position")
        orientation = _finite_tuple(self.orientation_xyzw, 4, "orientation_xyzw")
        norm = math.sqrt(sum(value * value for value in orientation))
        if not math.isclose(norm, 1.0, rel_tol=0.0, abs_tol=1e-6):
            raise _validation(
                "orientation_xyzw must be a unit quaternion; implicit normalization is forbidden",
                "pose.validate",
                norm=norm,
            )
        object.__setattr__(self, "position", position)
        object.__setattr__(self, "orientation_xyzw", orientation)


@dataclass(frozen=True)
class ArrayValue:
    """Immutable CPU-owned n-dimensional data with explicit shape and dtype."""

    shape: tuple[int, ...]
    values: tuple[float | int | bool, ...]
    dtype: str = "float64"
    device: str = "cpu"
    ownership: ArrayOwnership = ArrayOwnership.OWNED

    def __post_init__(self) -> None:
        try:
            shape = tuple(self.shape)
            raw_values = tuple(self.values)
        except TypeError as exc:
            raise _validation("array shape and values must be iterable", "array.validate") from exc
        if not shape or any(not isinstance(size, int) or isinstance(size, bool) or size <= 0 for size in shape):
            raise _validation("array shape must contain positive integer dimensions", "array.validate", shape=shape)
        if self.dtype not in {"float32", "float64", "int32", "int64", "uint8", "bool"}:
            raise _validation("unsupported array dtype", "array.validate", dtype=self.dtype)
        if self.device != "cpu" or self.ownership is not ArrayOwnership.OWNED:
            raise _validation(
                "M0 only supports owned CPU arrays",
                "array.validate",
                device=self.device,
                ownership=str(self.ownership),
            )
        expected = reduce(mul, shape, 1)
        if len(raw_values) != expected:
            raise _validation(
                "array value count does not match shape",
                "array.validate",
                shape=shape,
                expected=expected,
                actual=len(raw_values),
            )
        if self.dtype.startswith("float"):
            if any(isinstance(value, bool) or not isinstance(value, (int, float)) for value in raw_values):
                raise _validation("floating array contains a non-numeric value", "array.validate", dtype=self.dtype)
            values = tuple(float(value) for value in raw_values)
            if not all(math.isfinite(value) for value in values):
                raise _validation("floating array values must be finite", "array.validate", dtype=self.dtype)
        elif self.dtype.startswith("int") or self.dtype == "uint8":
            if any(isinstance(value, bool) or not isinstance(value, int) for value in raw_values):
                raise _validation("integer array contains a non-integer value", "array.validate", dtype=self.dtype)
            if self.dtype == "uint8" and any(value < 0 or value > 255 for value in raw_values):
                raise _validation("uint8 array values must be in [0, 255]", "array.validate", dtype=self.dtype)
            values = raw_values
        else:
            if any(not isinstance(value, bool) for value in raw_values):
                raise _validation("boolean array contains a non-boolean value", "array.validate")
            values = raw_values
        object.__setattr__(self, "shape", shape)
        object.__setattr__(self, "values", values)

    @classmethod
    def from_rows(cls, rows: Iterable[Iterable[float]], *, dtype: str = "float64") -> ArrayValue:
        try:
            row_values = tuple(tuple(row) for row in rows)
        except TypeError as exc:
            raise _validation("array rows must be an iterable of iterables", "array.from_rows") from exc
        if not row_values or not row_values[0]:
            raise _validation("array rows must be non-empty", "array.from_rows")
        width = len(row_values[0])
        if any(len(row) != width for row in row_values):
            raise _validation("array rows must have equal length", "array.from_rows")
        return cls(
            shape=(len(row_values), width),
            values=tuple(value for row in row_values for value in row),
            dtype=dtype,
        )

    @classmethod
    def from_nested(cls, values: object, *, dtype: str = "float64") -> ArrayValue:
        """Build an ArrayValue from a non-empty rectangular nested sequence."""

        shape, flattened = _flatten_nested(values)
        if not shape:
            raise _validation("from_nested requires at least one array dimension", "array.from_nested")
        return cls(shape=shape, values=flattened, dtype=dtype)

    def rows(self) -> tuple[tuple[float | int | bool, ...], ...]:
        if len(self.shape) != 2:
            raise _validation("rows() requires a rank-2 array", "array.rows", shape=self.shape)
        width = self.shape[1]
        return tuple(tuple(self.values[offset : offset + width]) for offset in range(0, len(self.values), width))

    def nested(self) -> tuple[Any, ...]:
        """Return a tuple-nested representation with the declared shape."""

        def build(offset: int, shape: tuple[int, ...]) -> tuple[Any, ...]:
            if len(shape) == 1:
                return tuple(self.values[offset : offset + shape[0]])
            stride = reduce(mul, shape[1:], 1)
            return tuple(build(offset + index * stride, shape[1:]) for index in range(shape[0]))

        return build(0, self.shape)


@dataclass(frozen=True)
class EntityHandle:
    provider_id: str
    session_id: str
    world_id: str
    generation: int
    path: EntityPath
    entity_kind: EntityKind
    token: str

    def __post_init__(self) -> None:
        strings = (self.provider_id, self.session_id, self.world_id, self.token)
        if any(not isinstance(value, str) or not value for value in strings):
            raise _validation("handle identity strings must be non-empty", "entity_handle.validate")
        if not isinstance(self.generation, int) or isinstance(self.generation, bool) or self.generation <= 0:
            raise _validation("handle generation must be a positive integer", "entity_handle.validate")
        if not isinstance(self.path, EntityPath) or not isinstance(self.entity_kind, EntityKind):
            raise _validation("handle path and kind must use canonical value types", "entity_handle.validate")


@dataclass(frozen=True)
class Tick:
    step_index: int
    sim_time_seconds: float

    def __post_init__(self) -> None:
        if (
            not isinstance(self.step_index, int)
            or isinstance(self.step_index, bool)
            or self.step_index < 0
            or not isinstance(self.sim_time_seconds, (int, float))
            or isinstance(self.sim_time_seconds, bool)
            or not math.isfinite(self.sim_time_seconds)
            or self.sim_time_seconds < 0.0
        ):
            raise _validation("tick must be non-negative and finite", "tick.validate")
        object.__setattr__(self, "sim_time_seconds", float(self.sim_time_seconds))
