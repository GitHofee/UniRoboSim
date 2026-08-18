"""Backend-neutral debug primitives, fan-out bus, and reference sinks."""

from __future__ import annotations

import json
import math
import re
from collections.abc import Iterable
from dataclasses import dataclass, field
from enum import StrEnum
from pathlib import Path
from typing import Protocol, runtime_checkable

from .errors import LifecycleError, ValidationError
from .frozen import FrozenMap
from .values import ArrayValue

_DEBUG_NAME = re.compile(r"^[A-Za-z0-9_][A-Za-z0-9_.:/-]*$")


class DebugPrimitiveKind(StrEnum):
    POINT_SET = "point_set"
    LINE_LIST = "line_list"


def _validate_name(value: str, field_name: str) -> None:
    if not isinstance(value, str) or not _DEBUG_NAME.fullmatch(value):
        raise ValidationError(
            f"debug {field_name} is invalid",
            operation="debug_primitive.validate",
            details={field_name: value},
        )


@dataclass(frozen=True)
class DebugPrimitive:
    """One stable, environment-batched primitive in environment-local world coordinates."""

    primitive_id: str
    layer: str
    kind: DebugPrimitiveKind
    geometry_m: ArrayValue
    environment_indices: tuple[int, ...]
    color_rgba: tuple[float, float, float, float] = (1.0, 1.0, 1.0, 1.0)
    size: float = 1.0
    lifetime_steps: int = 0
    metadata: FrozenMap = field(default_factory=FrozenMap)

    def __post_init__(self) -> None:
        operation = "debug_primitive.validate"
        _validate_name(self.primitive_id, "primitive_id")
        _validate_name(self.layer, "layer")
        if not isinstance(self.kind, DebugPrimitiveKind) or not isinstance(self.geometry_m, ArrayValue):
            raise ValidationError("debug primitive kind/geometry is invalid", operation=operation)
        if not self.geometry_m.dtype.startswith("float"):
            raise ValidationError("debug geometry must use a floating dtype", operation=operation)
        if self.kind is DebugPrimitiveKind.POINT_SET:
            valid_shape = len(self.geometry_m.shape) == 3 and self.geometry_m.shape[-1] == 3
        else:
            valid_shape = len(self.geometry_m.shape) == 4 and self.geometry_m.shape[-2:] == (2, 3)
        if not valid_shape:
            raise ValidationError(
                "debug geometry shape does not match its primitive kind",
                operation=operation,
                details={"shape": list(self.geometry_m.shape), "kind": self.kind.value},
            )
        try:
            environments = tuple(self.environment_indices)
            color = tuple(float(value) for value in self.color_rgba)
        except (TypeError, ValueError) as exc:
            raise ValidationError("debug environment/color values must be iterable", operation=operation) from exc
        if (
            not environments
            or len(environments) != self.geometry_m.shape[0]
            or any(not isinstance(index, int) or isinstance(index, bool) or index < 0 for index in environments)
            or len(environments) != len(set(environments))
        ):
            raise ValidationError(
                "debug environment selection must be unique and match the geometry batch",
                operation=operation,
            )
        if len(color) != 4 or not all(math.isfinite(value) and 0.0 <= value <= 1.0 for value in color):
            raise ValidationError("debug color must be four finite values in [0, 1]", operation=operation)
        if isinstance(self.size, bool) or not isinstance(self.size, (int, float)):
            raise ValidationError("debug size must be numeric", operation=operation)
        size = float(self.size)
        if not math.isfinite(size) or size <= 0.0:
            raise ValidationError("debug size must be positive and finite", operation=operation)
        if not isinstance(self.lifetime_steps, int) or isinstance(self.lifetime_steps, bool) or self.lifetime_steps < 0:
            raise ValidationError("debug lifetime must be a non-negative integer", operation=operation)
        if not isinstance(self.metadata, FrozenMap):
            raise ValidationError("debug metadata must be a FrozenMap", operation=operation)
        object.__setattr__(self, "environment_indices", environments)
        object.__setattr__(self, "color_rgba", color)
        object.__setattr__(self, "size", size)

    @property
    def key(self) -> tuple[str, str]:
        return self.layer, self.primitive_id

    def to_dict(self) -> dict[str, object]:
        return {
            "id": self.primitive_id,
            "layer": self.layer,
            "kind": self.kind.value,
            "geometry_m": self.geometry_m.nested(),
            "environment_indices": list(self.environment_indices),
            "color_rgba": list(self.color_rgba),
            "size": self.size,
            "lifetime_steps": self.lifetime_steps,
            "metadata": self.metadata.to_dict(),
        }


@dataclass(frozen=True)
class DebugBatch:
    primitives: tuple[DebugPrimitive, ...]

    def __post_init__(self) -> None:
        try:
            primitives = tuple(self.primitives)
        except TypeError as exc:
            raise ValidationError("debug primitives must be iterable", operation="debug_batch.validate") from exc
        keys = tuple(item.key for item in primitives if isinstance(item, DebugPrimitive))
        if not primitives or len(keys) != len(primitives) or len(keys) != len(set(keys)):
            raise ValidationError(
                "debug batch must contain primitives with unique stable keys",
                operation="debug_batch.validate",
            )
        object.__setattr__(self, "primitives", primitives)


@dataclass(frozen=True)
class DebugPublishReport:
    accepted_count: int
    dropped_count: int
    active_count: int
    sink_failures: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        counts = (self.accepted_count, self.dropped_count, self.active_count)
        if any(not isinstance(value, int) or isinstance(value, bool) or value < 0 for value in counts):
            raise ValidationError(
                "debug report counts must be non-negative integers",
                operation="debug_report.validate",
            )
        try:
            failures = tuple(self.sink_failures)
        except TypeError as exc:
            raise ValidationError("debug sink failures must be iterable", operation="debug_report.validate") from exc
        if any(not isinstance(item, str) or not item for item in failures):
            raise ValidationError("debug sink failures must be non-empty strings", operation="debug_report.validate")
        object.__setattr__(self, "sink_failures", failures)


@runtime_checkable
class DebugSink(Protocol):
    def publish(self, batch: DebugBatch) -> None: ...

    def clear(self, *, layer: str | None = None, primitive_id: str | None = None) -> int: ...

    def close(self) -> None: ...


def _matches(key: tuple[str, str], layer: str | None, primitive_id: str | None) -> bool:
    return (layer is None or key[0] == layer) and (primitive_id is None or key[1] == primitive_id)


class DebugBus:
    """Own stable IDs, budgets and physics-step lifetimes while isolating sink failures."""

    def __init__(self, sinks: Iterable[DebugSink], *, max_active_primitives: int = 10_000) -> None:
        try:
            sink_values = tuple(sinks)
        except TypeError as exc:
            raise ValidationError("debug sinks must be iterable", operation="debug_bus.init") from exc
        if not sink_values or any(not isinstance(item, DebugSink) for item in sink_values):
            raise ValidationError("debug bus requires at least one DebugSink", operation="debug_bus.init")
        if (
            not isinstance(max_active_primitives, int)
            or isinstance(max_active_primitives, bool)
            or max_active_primitives <= 0
        ):
            raise ValidationError("debug primitive budget must be a positive integer", operation="debug_bus.init")
        self._sinks = sink_values
        self._budget = max_active_primitives
        self._lifetimes: dict[tuple[str, str], int | None] = {}
        self._closed = False

    @property
    def active_count(self) -> int:
        return len(self._lifetimes)

    def _ensure_open(self, operation: str) -> None:
        if self._closed:
            raise LifecycleError("debug bus is closed", operation=operation)

    def publish(self, batch: DebugBatch) -> DebugPublishReport:
        self._ensure_open("debug_bus.publish")
        if not isinstance(batch, DebugBatch):
            raise ValidationError("publish requires a DebugBatch", operation="debug_bus.publish")
        next_keys = set(self._lifetimes)
        next_keys.update(item.key for item in batch.primitives)
        if len(next_keys) > self._budget:
            return DebugPublishReport(0, len(batch.primitives), len(self._lifetimes))
        failures: list[str] = []
        successful_sinks = 0
        for index, sink in enumerate(self._sinks):
            try:
                sink.publish(batch)
                successful_sinks += 1
            except Exception as exc:  # sinks are failure-isolated by contract
                failures.append(f"sink[{index}] {type(exc).__name__}: {exc}")
        if successful_sinks:
            for primitive in batch.primitives:
                self._lifetimes[primitive.key] = primitive.lifetime_steps or None
            return DebugPublishReport(len(batch.primitives), 0, len(self._lifetimes), tuple(failures))
        return DebugPublishReport(0, len(batch.primitives), len(self._lifetimes), tuple(failures))

    def clear(self, *, layer: str | None = None, primitive_id: str | None = None) -> int:
        self._ensure_open("debug_bus.clear")
        if layer is not None:
            _validate_name(layer, "layer")
        if primitive_id is not None:
            _validate_name(primitive_id, "primitive_id")
        keys = tuple(key for key in self._lifetimes if _matches(key, layer, primitive_id))
        for sink in self._sinks:
            try:
                sink.clear(layer=layer, primitive_id=primitive_id)
            except Exception:
                continue
        for key in keys:
            del self._lifetimes[key]
        return len(keys)

    def advance(self, count: int = 1) -> int:
        """Advance debug lifetimes after the caller has advanced physics by ``count`` steps."""

        self._ensure_open("debug_bus.advance")
        if not isinstance(count, int) or isinstance(count, bool) or count <= 0:
            raise ValidationError("debug advance count must be positive", operation="debug_bus.advance")
        expired: list[tuple[str, str]] = []
        for key, remaining in tuple(self._lifetimes.items()):
            if remaining is None:
                continue
            next_remaining = remaining - count
            if next_remaining <= 0:
                expired.append(key)
            else:
                self._lifetimes[key] = next_remaining
        for layer, primitive_id in expired:
            for sink in self._sinks:
                try:
                    sink.clear(layer=layer, primitive_id=primitive_id)
                except Exception:
                    continue
            del self._lifetimes[(layer, primitive_id)]
        return len(expired)

    def close(self) -> None:
        if self._closed:
            return
        for sink in self._sinks:
            try:
                sink.close()
            except Exception:
                continue
        self._lifetimes.clear()
        self._closed = True


class TestDebugSink:
    """In-memory stable-ID sink for assertions and SDK-free examples."""

    __test__ = False

    def __init__(self) -> None:
        self._primitives: dict[tuple[str, str], DebugPrimitive] = {}
        self._closed = False

    @property
    def primitives(self) -> tuple[DebugPrimitive, ...]:
        return tuple(self._primitives[key] for key in sorted(self._primitives))

    def publish(self, batch: DebugBatch) -> None:
        if self._closed:
            raise LifecycleError("debug sink is closed", operation="test_debug_sink.publish")
        if not isinstance(batch, DebugBatch):
            raise ValidationError("publish requires a DebugBatch", operation="test_debug_sink.publish")
        for primitive in batch.primitives:
            self._primitives[primitive.key] = primitive

    def clear(self, *, layer: str | None = None, primitive_id: str | None = None) -> int:
        if self._closed:
            raise LifecycleError("debug sink is closed", operation="test_debug_sink.clear")
        keys = tuple(key for key in self._primitives if _matches(key, layer, primitive_id))
        for key in keys:
            del self._primitives[key]
        return len(keys)

    def close(self) -> None:
        self._primitives.clear()
        self._closed = True


class TraceDebugSink:
    """Canonical JSON Lines sink for replayable debug evidence."""

    def __init__(self, path: str | Path) -> None:
        self._path = Path(path)
        self._path.parent.mkdir(parents=True, exist_ok=True)
        self._stream = self._path.open("w", encoding="utf-8", newline="\n")
        self._event_count = 0
        self._closed = False

    @property
    def path(self) -> Path:
        return self._path

    def _write(self, payload: dict[str, object]) -> None:
        if self._closed:
            raise LifecycleError("debug trace sink is closed", operation="trace_debug_sink.write")
        self._stream.write(json.dumps(payload, sort_keys=True, separators=(",", ":"), ensure_ascii=False) + "\n")
        self._stream.flush()
        self._event_count += 1

    def publish(self, batch: DebugBatch) -> None:
        if not isinstance(batch, DebugBatch):
            raise ValidationError("publish requires a DebugBatch", operation="trace_debug_sink.publish")
        self._write({"event": "publish", "primitives": [item.to_dict() for item in batch.primitives]})

    def clear(self, *, layer: str | None = None, primitive_id: str | None = None) -> int:
        self._write({"event": "clear", "layer": layer, "primitive_id": primitive_id})
        return 0

    def close(self) -> None:
        if self._closed:
            return
        self._stream.write(
            json.dumps(
                {"event": "close", "events_before_close": self._event_count},
                sort_keys=True,
                separators=(",", ":"),
                ensure_ascii=False,
            )
            + "\n"
        )
        self._stream.close()
        self._closed = True


class NativeWorldDebugSink:
    """Adapter from a capability-gated World debug endpoint to the DebugSink protocol."""

    def __init__(self, world: object) -> None:
        if not callable(getattr(world, "publish_debug", None)) or not callable(getattr(world, "clear_debug", None)):
            raise ValidationError("world has no native debug endpoint", operation="native_debug_sink.init")
        self._world = world
        self._closed = False

    def publish(self, batch: DebugBatch) -> None:
        if self._closed:
            raise LifecycleError("native debug sink is closed", operation="native_debug_sink.publish")
        self._world.publish_debug(batch)  # type: ignore[attr-defined]

    def clear(self, *, layer: str | None = None, primitive_id: str | None = None) -> int:
        if self._closed:
            raise LifecycleError("native debug sink is closed", operation="native_debug_sink.clear")
        result = self._world.clear_debug(layer=layer, primitive_id=primitive_id)  # type: ignore[attr-defined]
        return int(result)

    def close(self) -> None:
        self._closed = True
