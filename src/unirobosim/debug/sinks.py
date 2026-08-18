"""Small backend-neutral debug sinks and the public World sink bridge."""

from __future__ import annotations

from unirobosim.api.errors import LifecycleError, ValidationError

from .bus import _matches
from .model import DebugBatch, DebugLifetimeMode, DebugPrimitive


class TestDebugSink:
    """In-memory stable-key sink for assertions and SDK-free examples."""

    __test__ = False

    def __init__(self) -> None:
        self._primitives: dict[tuple[str, str, str], DebugPrimitive] = {}
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

    def clear(
        self,
        *,
        layer: str | None = None,
        group: str | None = None,
        primitive_id: str | None = None,
    ) -> int:
        if self._closed:
            raise LifecycleError("debug sink is closed", operation="test_debug_sink.clear")
        keys = tuple(key for key in self._primitives if _matches(key, layer, group, primitive_id))
        for key in keys:
            del self._primitives[key]
        return len(keys)

    def reset(self) -> int:
        if self._closed:
            raise LifecycleError("debug sink is closed", operation="test_debug_sink.reset")
        keys = tuple(
            key
            for key, primitive in self._primitives.items()
            if primitive.lifetime.mode is not DebugLifetimeMode.MANUAL
        )
        for key in keys:
            del self._primitives[key]
        return len(keys)

    def close(self) -> None:
        self._primitives.clear()
        self._closed = True


class NativeWorldDebugSink:
    """Adapter from a capability-gated World debug endpoint to the DebugSink protocol."""

    def __init__(self, world: object) -> None:
        if not callable(getattr(world, "publish_debug", None)) or not callable(getattr(world, "clear_debug", None)):
            raise ValidationError("world has no native debug endpoint", operation="native_debug_sink.init")
        self._world = world
        self._primitives: dict[tuple[str, str, str], DebugPrimitive] = {}
        self._closed = False

    def publish(self, batch: DebugBatch) -> None:
        if self._closed:
            raise LifecycleError("native debug sink is closed", operation="native_debug_sink.publish")
        self._world.publish_debug(batch)  # type: ignore[attr-defined]
        for primitive in batch.primitives:
            self._primitives[primitive.key] = primitive

    def clear(
        self,
        *,
        layer: str | None = None,
        group: str | None = None,
        primitive_id: str | None = None,
    ) -> int:
        if self._closed:
            raise LifecycleError("native debug sink is closed", operation="native_debug_sink.clear")
        result = self._world.clear_debug(  # type: ignore[attr-defined]
            layer=layer,
            group=group,
            primitive_id=primitive_id,
        )
        keys = tuple(key for key in self._primitives if _matches(key, layer, group, primitive_id))
        for key in keys:
            del self._primitives[key]
        return int(result)

    def reset(self) -> int:
        if self._closed:
            raise LifecycleError("native debug sink is closed", operation="native_debug_sink.reset")
        keys = tuple(
            key
            for key, primitive in self._primitives.items()
            if primitive.lifetime.mode is not DebugLifetimeMode.MANUAL
        )
        for layer, group, primitive_id in keys:
            self._world.clear_debug(  # type: ignore[attr-defined]
                layer=layer,
                group=group,
                primitive_id=primitive_id,
            )
            del self._primitives[(layer, group, primitive_id)]
        return len(keys)

    def close(self) -> None:
        self._primitives.clear()
        self._closed = True
