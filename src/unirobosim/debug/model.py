"""Versioned backend-neutral debug values and validation rules."""

from __future__ import annotations

import json
import math
import re
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from enum import StrEnum
from typing import cast

from unirobosim.api.errors import ValidationError
from unirobosim.api.frozen import FrozenMap
from unirobosim.api.values import ArrayValue

DEBUG_SCHEMA_VERSION = "unirobosim.debug/v1alpha1"
_DEBUG_NAME = re.compile(r"^[A-Za-z0-9_][A-Za-z0-9_.:/-]*$")
_EVENT_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.:/-]{0,127}$")


class DebugPrimitiveKind(StrEnum):
    POINT_SET = "point_set"
    LINE_LIST = "line_list"
    COORDINATE_AXES = "coordinate_axes"
    TEXT = "text"
    BOUNDING_BOX = "bounding_box"
    TRAJECTORY = "trajectory"


class DebugLifetimeMode(StrEnum):
    FRAME = "frame"
    STEPS = "steps"
    PERSISTENT = "persistent"
    MANUAL = "manual"


def _validate_name(value: str, field_name: str) -> None:
    if not isinstance(value, str) or not _DEBUG_NAME.fullmatch(value):
        raise ValidationError(
            f"debug {field_name} is invalid",
            operation="debug_primitive.validate",
            details={field_name: value},
        )


def _mapping(value: object, name: str, operation: str) -> Mapping[str, object]:
    if not isinstance(value, Mapping) or any(not isinstance(key, str) for key in value):
        raise ValidationError(f"{name} must be an object", operation=operation)
    return cast(Mapping[str, object], value)


def _sequence(value: object, name: str, operation: str) -> Sequence[object]:
    if not isinstance(value, (list, tuple)):
        raise ValidationError(f"{name} must be an array", operation=operation)
    return cast(Sequence[object], value)


@dataclass(frozen=True)
class DebugLifetime:
    """Explicit lifetime policy owned by the Debug Bus."""

    mode: DebugLifetimeMode
    step_count: int | None = None

    def __post_init__(self) -> None:
        operation = "debug_lifetime.validate"
        if not isinstance(self.mode, DebugLifetimeMode):
            raise ValidationError("debug lifetime mode is invalid", operation=operation)
        if self.mode is DebugLifetimeMode.STEPS:
            if not isinstance(self.step_count, int) or isinstance(self.step_count, bool) or self.step_count <= 0:
                raise ValidationError("steps lifetime requires a positive step count", operation=operation)
        elif self.step_count is not None:
            raise ValidationError("only steps lifetime accepts a step count", operation=operation)

    @classmethod
    def frame(cls) -> DebugLifetime:
        return cls(DebugLifetimeMode.FRAME)

    @classmethod
    def steps(cls, count: int) -> DebugLifetime:
        return cls(DebugLifetimeMode.STEPS, count)

    @classmethod
    def persistent(cls) -> DebugLifetime:
        return cls(DebugLifetimeMode.PERSISTENT)

    @classmethod
    def manual(cls) -> DebugLifetime:
        return cls(DebugLifetimeMode.MANUAL)

    def to_dict(self) -> dict[str, object]:
        result: dict[str, object] = {"mode": self.mode.value}
        if self.step_count is not None:
            result["step_count"] = self.step_count
        return result

    @classmethod
    def from_dict(cls, value: object) -> DebugLifetime:
        payload = _mapping(value, "lifetime", "debug_lifetime.decode")
        try:
            mode = DebugLifetimeMode(cast(str, payload.get("mode")))
        except (TypeError, ValueError) as exc:
            raise ValidationError("trace contains an invalid lifetime mode", operation="debug_lifetime.decode") from exc
        count = payload.get("step_count")
        return cls(mode, cast(int | None, count))


def _validate_environment_indices(values: object, environment_count: int) -> tuple[int, ...]:
    try:
        environments = tuple(cast(Sequence[int], values))
    except TypeError as exc:
        raise ValidationError(
            "debug environment selection must be iterable", operation="debug_primitive.validate"
        ) from exc
    if (
        not environments
        or len(environments) != environment_count
        or any(not isinstance(index, int) or isinstance(index, bool) or index < 0 for index in environments)
        or len(environments) != len(set(environments))
    ):
        raise ValidationError(
            "debug environment selection must be unique and match the geometry batch",
            operation="debug_primitive.validate",
        )
    return environments


def _validate_unit_quaternions(value: ArrayValue, width: int, offset: int, operation: str) -> None:
    row_count = math.prod(value.shape[:-1])
    for row_index in range(row_count):
        start = row_index * width + offset
        quaternion = value.values[start : start + 4]
        norm = math.sqrt(sum(float(item) ** 2 for item in quaternion))
        if not math.isclose(norm, 1.0, rel_tol=0.0, abs_tol=1.0e-5):
            raise ValidationError("debug orientations must be unit XYZW quaternions", operation=operation)


def _validate_geometry(kind: DebugPrimitiveKind, geometry: ArrayValue) -> None:
    operation = "debug_primitive.validate"
    if not geometry.dtype.startswith("float"):
        raise ValidationError("debug geometry must use a floating dtype", operation=operation)
    shape = geometry.shape
    valid = False
    if kind is DebugPrimitiveKind.POINT_SET:
        valid = len(shape) == 3 and shape[-1] == 3
    elif kind is DebugPrimitiveKind.LINE_LIST:
        valid = len(shape) == 4 and shape[-2:] == (2, 3)
    elif kind is DebugPrimitiveKind.COORDINATE_AXES:
        valid = len(shape) == 3 and shape[-1] == 7
        if valid:
            _validate_unit_quaternions(geometry, 7, 3, operation)
    elif kind is DebugPrimitiveKind.TEXT:
        valid = len(shape) == 3 and shape[-1] == 3
    elif kind is DebugPrimitiveKind.BOUNDING_BOX:
        valid = len(shape) == 3 and shape[-1] == 10
        if valid:
            for row_index in range(math.prod(shape[:-1])):
                start = row_index * 10
                if any(float(item) <= 0.0 for item in geometry.values[start + 3 : start + 6]):
                    raise ValidationError("debug bounding-box sizes must be positive", operation=operation)
            _validate_unit_quaternions(geometry, 10, 6, operation)
    elif kind is DebugPrimitiveKind.TRAJECTORY:
        valid = len(shape) == 3 and shape[-1] == 3 and shape[1] >= 2
    if not valid:
        raise ValidationError(
            "debug geometry shape does not match its primitive kind",
            operation=operation,
            details={"shape": list(shape), "kind": kind.value},
        )


def _normalize_text(value: object, shape: tuple[int, ...]) -> tuple[tuple[str, ...], ...]:
    operation = "debug_primitive.validate"
    outer = _sequence(value, "debug text", operation)
    if len(outer) != shape[0]:
        raise ValidationError("debug text batch must match geometry", operation=operation)
    result: list[tuple[str, ...]] = []
    for row in outer:
        values = _sequence(row, "debug text row", operation)
        if len(values) != shape[1]:
            raise ValidationError("debug text row must match geometry", operation=operation)
        strings: list[str] = []
        for item in values:
            if not isinstance(item, str) or not item or "\x00" in item or len(item.encode("utf-8")) > 4096:
                raise ValidationError("debug text must contain bounded non-empty UTF-8 strings", operation=operation)
            strings.append(item)
        result.append(tuple(strings))
    return tuple(result)


@dataclass(frozen=True)
class DebugPrimitive:
    """One stable, environment-batched primitive in environment-local world coordinates."""

    primitive_id: str
    layer: str
    kind: DebugPrimitiveKind
    geometry_m: ArrayValue
    environment_indices: tuple[int, ...]
    group: str = "default"
    source: str = "application"
    color_rgba: tuple[float, float, float, float] = (1.0, 1.0, 1.0, 1.0)
    size: float = 1.0
    lifetime: DebugLifetime = field(default_factory=DebugLifetime.persistent)
    text: tuple[tuple[str, ...], ...] | None = None
    sample_times_s: ArrayValue | None = None
    metadata: FrozenMap = field(default_factory=FrozenMap)

    def __post_init__(self) -> None:
        operation = "debug_primitive.validate"
        _validate_name(self.primitive_id, "primitive_id")
        _validate_name(self.layer, "layer")
        _validate_name(self.group, "group")
        _validate_name(self.source, "source")
        if not isinstance(self.kind, DebugPrimitiveKind) or not isinstance(self.geometry_m, ArrayValue):
            raise ValidationError("debug primitive kind/geometry is invalid", operation=operation)
        _validate_geometry(self.kind, self.geometry_m)
        environments = _validate_environment_indices(self.environment_indices, self.geometry_m.shape[0])
        try:
            color = tuple(float(value) for value in self.color_rgba)
        except (TypeError, ValueError) as exc:
            raise ValidationError("debug color values must be iterable", operation=operation) from exc
        if len(color) != 4 or not all(math.isfinite(value) and 0.0 <= value <= 1.0 for value in color):
            raise ValidationError("debug color must be four finite values in [0, 1]", operation=operation)
        if isinstance(self.size, bool) or not isinstance(self.size, (int, float)):
            raise ValidationError("debug size must be numeric", operation=operation)
        size = float(self.size)
        if not math.isfinite(size) or size <= 0.0:
            raise ValidationError("debug size must be positive and finite", operation=operation)
        if not isinstance(self.lifetime, DebugLifetime):
            raise ValidationError("debug lifetime is invalid", operation=operation)
        normalized_text = None
        if self.kind is DebugPrimitiveKind.TEXT:
            normalized_text = _normalize_text(self.text, self.geometry_m.shape)
        elif self.text is not None:
            raise ValidationError("only text primitives accept text payloads", operation=operation)
        if self.kind is DebugPrimitiveKind.TRAJECTORY:
            if self.sample_times_s is not None:
                expected_shape = self.geometry_m.shape[:2]
                if (
                    not isinstance(self.sample_times_s, ArrayValue)
                    or not self.sample_times_s.dtype.startswith("float")
                    or self.sample_times_s.shape != expected_shape
                ):
                    raise ValidationError(
                        "trajectory sample times must be a floating [environment, sample] array",
                        operation=operation,
                    )
                for row in self.sample_times_s.nested():
                    row_values = cast(tuple[float, ...], row)
                    if any(right <= left for left, right in zip(row_values, row_values[1:], strict=False)):
                        raise ValidationError("trajectory sample times must increase strictly", operation=operation)
        elif self.sample_times_s is not None:
            raise ValidationError("only trajectory primitives accept sample times", operation=operation)
        if not isinstance(self.metadata, FrozenMap):
            raise ValidationError("debug metadata must be a FrozenMap", operation=operation)
        object.__setattr__(self, "environment_indices", environments)
        object.__setattr__(self, "color_rgba", color)
        object.__setattr__(self, "size", size)
        object.__setattr__(self, "text", normalized_text)

    @property
    def key(self) -> tuple[str, str, str]:
        return self.layer, self.group, self.primitive_id

    @property
    def vertex_count(self) -> int:
        count = self.geometry_m.shape[0] * self.geometry_m.shape[1]
        multipliers = {
            DebugPrimitiveKind.POINT_SET: 1,
            DebugPrimitiveKind.LINE_LIST: 2,
            DebugPrimitiveKind.COORDINATE_AXES: 6,
            DebugPrimitiveKind.TEXT: 1,
            DebugPrimitiveKind.BOUNDING_BOX: 24,
            DebugPrimitiveKind.TRAJECTORY: 1,
        }
        return count * multipliers[self.kind]

    @property
    def estimated_payload_bytes(self) -> int:
        return len(json.dumps(self.to_dict(), separators=(",", ":"), ensure_ascii=False).encode("utf-8"))

    def select_environments(self, selected: frozenset[int]) -> DebugPrimitive | None:
        rows = tuple(index for index, environment in enumerate(self.environment_indices) if environment in selected)
        if not rows:
            return None
        row_width = math.prod(self.geometry_m.shape[1:])
        geometry_values = tuple(
            value for row in rows for value in self.geometry_m.values[row * row_width : (row + 1) * row_width]
        )
        geometry = ArrayValue(
            (len(rows), *self.geometry_m.shape[1:]),
            geometry_values,
            dtype=self.geometry_m.dtype,
        )
        text = None if self.text is None else tuple(self.text[row] for row in rows)
        sample_times = None
        if self.sample_times_s is not None:
            sample_width = self.sample_times_s.shape[1]
            sample_values = tuple(
                value
                for row in rows
                for value in self.sample_times_s.values[row * sample_width : (row + 1) * sample_width]
            )
            sample_times = ArrayValue((len(rows), sample_width), sample_values, dtype=self.sample_times_s.dtype)
        return DebugPrimitive(
            primitive_id=self.primitive_id,
            layer=self.layer,
            group=self.group,
            source=self.source,
            kind=self.kind,
            geometry_m=geometry,
            environment_indices=tuple(self.environment_indices[row] for row in rows),
            color_rgba=self.color_rgba,
            size=self.size,
            lifetime=self.lifetime,
            text=text,
            sample_times_s=sample_times,
            metadata=self.metadata,
        )

    def to_dict(self) -> dict[str, object]:
        result: dict[str, object] = {
            "id": self.primitive_id,
            "layer": self.layer,
            "group": self.group,
            "source": self.source,
            "kind": self.kind.value,
            "geometry_m": self.geometry_m.nested(),
            "geometry_dtype": self.geometry_m.dtype,
            "environment_indices": list(self.environment_indices),
            "color_rgba": list(self.color_rgba),
            "size": self.size,
            "lifetime": self.lifetime.to_dict(),
            "metadata": self.metadata.to_dict(),
        }
        if self.text is not None:
            result["text"] = self.text
        if self.sample_times_s is not None:
            result["sample_times_s"] = self.sample_times_s.nested()
            result["sample_times_dtype"] = self.sample_times_s.dtype
        return result

    @classmethod
    def from_dict(cls, value: object) -> DebugPrimitive:
        operation = "debug_primitive.decode"
        payload = _mapping(value, "primitive", operation)
        try:
            primitive_id = payload["id"]
            layer = payload["layer"]
            kind = DebugPrimitiveKind(cast(str, payload["kind"]))
            geometry = ArrayValue.from_nested(
                payload["geometry_m"], dtype=cast(str, payload.get("geometry_dtype", "float64"))
            )
            environments = tuple(cast(Sequence[int], payload["environment_indices"]))
            color = tuple(cast(Sequence[float], payload["color_rgba"]))
            lifetime = DebugLifetime.from_dict(payload["lifetime"])
            metadata = FrozenMap(_mapping(payload.get("metadata", {}), "metadata", operation))
        except (KeyError, TypeError, ValueError) as exc:
            raise ValidationError("trace contains an invalid debug primitive", operation=operation) from exc
        sample_times = None
        if "sample_times_s" in payload:
            sample_times = ArrayValue.from_nested(
                payload["sample_times_s"], dtype=cast(str, payload.get("sample_times_dtype", "float64"))
            )
        return cls(
            primitive_id=cast(str, primitive_id),
            layer=cast(str, layer),
            group=cast(str, payload.get("group", "default")),
            source=cast(str, payload.get("source", "application")),
            kind=kind,
            geometry_m=geometry,
            environment_indices=environments,
            color_rgba=cast(tuple[float, float, float, float], color),
            size=cast(float, payload.get("size", 1.0)),
            lifetime=lifetime,
            text=cast(tuple[tuple[str, ...], ...] | None, payload.get("text")),
            sample_times_s=sample_times,
            metadata=metadata,
        )


@dataclass(frozen=True)
class DebugBatch:
    primitives: tuple[DebugPrimitive, ...]
    step_index: int = 0
    sim_time_s: float = 0.0
    world_generation: int = 0
    event_id: str | None = None

    def __post_init__(self) -> None:
        operation = "debug_batch.validate"
        try:
            primitives = tuple(self.primitives)
        except TypeError as exc:
            raise ValidationError("debug primitives must be iterable", operation=operation) from exc
        keys = tuple(item.key for item in primitives if isinstance(item, DebugPrimitive))
        if not primitives or len(keys) != len(primitives) or len(keys) != len(set(keys)):
            raise ValidationError("debug batch must contain primitives with unique stable keys", operation=operation)
        if (
            not isinstance(self.step_index, int)
            or isinstance(self.step_index, bool)
            or self.step_index < 0
            or not isinstance(self.world_generation, int)
            or isinstance(self.world_generation, bool)
            or self.world_generation < 0
            or isinstance(self.sim_time_s, bool)
            or not isinstance(self.sim_time_s, (int, float))
            or not math.isfinite(float(self.sim_time_s))
            or float(self.sim_time_s) < 0.0
        ):
            raise ValidationError("debug batch time/generation values are invalid", operation=operation)
        if self.event_id is not None and (not isinstance(self.event_id, str) or not _EVENT_ID.fullmatch(self.event_id)):
            raise ValidationError("debug event ID is invalid", operation=operation)
        object.__setattr__(self, "primitives", primitives)
        object.__setattr__(self, "sim_time_s", float(self.sim_time_s))

    def with_primitives(self, primitives: Sequence[DebugPrimitive]) -> DebugBatch:
        return DebugBatch(tuple(primitives), self.step_index, self.sim_time_s, self.world_generation, self.event_id)

    def to_dict(self) -> dict[str, object]:
        result: dict[str, object] = {
            "primitives": [item.to_dict() for item in self.primitives],
            "step_index": self.step_index,
            "sim_time_s": self.sim_time_s,
            "world_generation": self.world_generation,
        }
        if self.event_id is not None:
            result["event_id"] = self.event_id
        return result

    @classmethod
    def from_dict(cls, value: object) -> DebugBatch:
        operation = "debug_batch.decode"
        payload = _mapping(value, "batch", operation)
        try:
            primitives = tuple(
                DebugPrimitive.from_dict(item) for item in _sequence(payload["primitives"], "primitives", operation)
            )
            return cls(
                primitives,
                step_index=cast(int, payload.get("step_index", 0)),
                sim_time_s=cast(float, payload.get("sim_time_s", 0.0)),
                world_generation=cast(int, payload.get("world_generation", 0)),
                event_id=cast(str | None, payload.get("event_id")),
            )
        except KeyError as exc:
            raise ValidationError("trace contains an invalid debug batch", operation=operation) from exc


@dataclass(frozen=True)
class DebugSelection:
    """Exact layer/group/environment selection applied before any sink sees data."""

    layers: tuple[str, ...] | None = None
    groups: tuple[str, ...] | None = None
    environment_indices: tuple[int, ...] | None = None

    def __post_init__(self) -> None:
        for name, values in (("layers", self.layers), ("groups", self.groups)):
            if values is not None:
                normalized = tuple(values)
                if not normalized or len(normalized) != len(set(normalized)):
                    raise ValidationError(
                        f"debug {name} selection must be non-empty and unique",
                        operation="debug_selection.validate",
                    )
                for value in normalized:
                    _validate_name(value, name)
                object.__setattr__(self, name, normalized)
        if self.environment_indices is not None:
            environments = tuple(self.environment_indices)
            if (
                not environments
                or len(environments) != len(set(environments))
                or any(not isinstance(item, int) or isinstance(item, bool) or item < 0 for item in environments)
            ):
                raise ValidationError(
                    "debug environment selection must be non-empty, unique non-negative integers",
                    operation="debug_selection.validate",
                )
            object.__setattr__(self, "environment_indices", environments)

    def apply(self, primitive: DebugPrimitive) -> DebugPrimitive | None:
        if self.layers is not None and primitive.layer not in self.layers:
            return None
        if self.groups is not None and primitive.group not in self.groups:
            return None
        if self.environment_indices is None:
            return primitive
        return primitive.select_environments(frozenset(self.environment_indices))
