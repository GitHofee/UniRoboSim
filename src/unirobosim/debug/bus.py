"""Debug fan-out, filtering, lifecycle and deterministic budget accounting."""

from __future__ import annotations

import math
import time
from collections import deque
from collections.abc import Callable, Iterable
from dataclasses import dataclass, field
from typing import Protocol, runtime_checkable

from unirobosim.api.errors import LifecycleError, ValidationError
from unirobosim.api.frozen import FrozenMap

from .model import DebugBatch, DebugLifetime, DebugLifetimeMode, DebugPrimitive, DebugSelection, _validate_name


@runtime_checkable
class DebugSink(Protocol):
    def publish(self, batch: DebugBatch) -> None: ...

    def clear(
        self,
        *,
        layer: str | None = None,
        group: str | None = None,
        primitive_id: str | None = None,
    ) -> int: ...

    def reset(self) -> int: ...

    def close(self) -> None: ...


@dataclass(frozen=True)
class DebugBudget:
    max_active_primitives: int = 10_000
    max_primitives_per_publish: int = 1_000
    max_vertices_per_publish: int = 100_000
    max_events_per_second: int = 120
    max_payload_bytes_per_second: int = 16 * 1024 * 1024
    max_publish_duration_ms: float = 16.0

    def __post_init__(self) -> None:
        integer_values = (
            self.max_active_primitives,
            self.max_primitives_per_publish,
            self.max_vertices_per_publish,
            self.max_events_per_second,
            self.max_payload_bytes_per_second,
        )
        if any(not isinstance(value, int) or isinstance(value, bool) or value <= 0 for value in integer_values):
            raise ValidationError("debug integer budgets must be positive", operation="debug_budget.validate")
        if (
            isinstance(self.max_publish_duration_ms, bool)
            or not isinstance(self.max_publish_duration_ms, (int, float))
            or not math.isfinite(float(self.max_publish_duration_ms))
            or float(self.max_publish_duration_ms) <= 0.0
        ):
            raise ValidationError("debug publish duration budget must be positive", operation="debug_budget.validate")
        object.__setattr__(self, "max_publish_duration_ms", float(self.max_publish_duration_ms))


@dataclass(frozen=True)
class DebugPublishReport:
    accepted_count: int
    dropped_count: int
    active_count: int
    filtered_count: int = 0
    elapsed_ms: float = 0.0
    budget_exceeded: bool = False
    drop_reasons: FrozenMap = field(default_factory=FrozenMap)
    sink_failures: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        counts = (self.accepted_count, self.dropped_count, self.active_count, self.filtered_count)
        if any(not isinstance(value, int) or isinstance(value, bool) or value < 0 for value in counts):
            raise ValidationError(
                "debug report counts must be non-negative integers", operation="debug_report.validate"
            )
        if (
            isinstance(self.elapsed_ms, bool)
            or not isinstance(self.elapsed_ms, (int, float))
            or not math.isfinite(float(self.elapsed_ms))
            or float(self.elapsed_ms) < 0.0
            or not isinstance(self.budget_exceeded, bool)
            or not isinstance(self.drop_reasons, FrozenMap)
        ):
            raise ValidationError("debug report budget fields are invalid", operation="debug_report.validate")
        try:
            failures = tuple(self.sink_failures)
        except TypeError as exc:
            raise ValidationError("debug sink failures must be iterable", operation="debug_report.validate") from exc
        if any(not isinstance(item, str) or not item for item in failures):
            raise ValidationError("debug sink failures must be non-empty strings", operation="debug_report.validate")
        if any(
            not isinstance(value, int) or isinstance(value, bool) or value <= 0 for value in self.drop_reasons.values()
        ):
            raise ValidationError("debug drop reasons must contain positive counts", operation="debug_report.validate")
        object.__setattr__(self, "elapsed_ms", float(self.elapsed_ms))
        object.__setattr__(self, "sink_failures", failures)


@runtime_checkable
class DebugReportSink(Protocol):
    """Optional audit side-channel notified after each publish decision."""

    def publish_report(self, batch: DebugBatch, report: DebugPublishReport) -> None: ...


def _matches(
    key: tuple[str, str, str],
    layer: str | None,
    group: str | None,
    primitive_id: str | None,
) -> bool:
    return (
        (layer is None or key[0] == layer)
        and (group is None or key[1] == group)
        and (primitive_id is None or key[2] == primitive_id)
    )


def _increment(reasons: dict[str, int], reason: str, count: int = 1) -> None:
    if count > 0:
        reasons[reason] = reasons.get(reason, 0) + count


class DebugBus:
    """Own stable IDs, selection, budgets and step/reset lifetimes."""

    def __init__(
        self,
        sinks: Iterable[DebugSink],
        *,
        selection: DebugSelection | None = None,
        budget: DebugBudget | None = None,
        clock: Callable[[], float] = time.perf_counter,
        max_active_primitives: int | None = None,
    ) -> None:
        try:
            sink_values = tuple(sinks)
        except TypeError as exc:
            raise ValidationError("debug sinks must be iterable", operation="debug_bus.init") from exc
        if not sink_values or any(not isinstance(item, DebugSink) for item in sink_values):
            raise ValidationError("debug bus requires at least one DebugSink", operation="debug_bus.init")
        if selection is not None and not isinstance(selection, DebugSelection):
            raise ValidationError("debug selection is invalid", operation="debug_bus.init")
        if budget is not None and not isinstance(budget, DebugBudget):
            raise ValidationError("debug budget is invalid", operation="debug_bus.init")
        if max_active_primitives is not None:
            if budget is not None:
                raise ValidationError(
                    "use either budget or max_active_primitives compatibility option", operation="debug_bus.init"
                )
            budget = DebugBudget(max_active_primitives=max_active_primitives)
        if not callable(clock):
            raise ValidationError("debug clock must be callable", operation="debug_bus.init")
        self._sinks = sink_values
        self._selection = selection or DebugSelection()
        self._budget = budget or DebugBudget()
        self._clock = clock
        self._active: dict[tuple[str, str, str], DebugLifetime] = {}
        self._rate_window: deque[tuple[float, int]] = deque()
        self._closed = False

    @property
    def active_count(self) -> int:
        return len(self._active)

    @property
    def selection(self) -> DebugSelection:
        return self._selection

    @property
    def budget(self) -> DebugBudget:
        return self._budget

    def set_selection(self, selection: DebugSelection) -> None:
        self._ensure_open("debug_bus.set_selection")
        if not isinstance(selection, DebugSelection):
            raise ValidationError("debug selection is invalid", operation="debug_bus.set_selection")
        self._selection = selection

    def _ensure_open(self, operation: str) -> None:
        if self._closed:
            raise LifecycleError("debug bus is closed", operation=operation)

    def _purge_rate_window(self, now: float) -> None:
        while self._rate_window and now - self._rate_window[0][0] >= 1.0:
            self._rate_window.popleft()

    def _select_and_limit(self, batch: DebugBatch, now: float) -> tuple[list[DebugPrimitive], int, dict[str, int], int]:
        reasons: dict[str, int] = {}
        selected: list[DebugPrimitive] = []
        filtered = 0
        vertices = 0
        payload_bytes = 0
        self._purge_rate_window(now)
        if len(self._rate_window) >= self._budget.max_events_per_second:
            _increment(reasons, "event_rate", len(batch.primitives))
            return selected, filtered, reasons, payload_bytes
        bytes_used = sum(item[1] for item in self._rate_window)
        active_keys = set(self._active)
        for primitive in batch.primitives:
            filtered_primitive = self._selection.apply(primitive)
            if filtered_primitive is None:
                filtered += 1
                continue
            if len(selected) >= self._budget.max_primitives_per_publish:
                _increment(reasons, "batch_primitive_limit")
                continue
            if vertices + filtered_primitive.vertex_count > self._budget.max_vertices_per_publish:
                _increment(reasons, "batch_vertex_limit")
                continue
            primitive_bytes = filtered_primitive.estimated_payload_bytes
            if bytes_used + payload_bytes + primitive_bytes > self._budget.max_payload_bytes_per_second:
                _increment(reasons, "payload_rate")
                continue
            if filtered_primitive.key not in active_keys and len(active_keys) >= self._budget.max_active_primitives:
                _increment(reasons, "active_primitive_limit")
                continue
            selected.append(filtered_primitive)
            vertices += filtered_primitive.vertex_count
            payload_bytes += primitive_bytes
            active_keys.add(filtered_primitive.key)
        return selected, filtered, reasons, payload_bytes

    def publish(self, batch: DebugBatch) -> DebugPublishReport:
        self._ensure_open("debug_bus.publish")
        if not isinstance(batch, DebugBatch):
            raise ValidationError("publish requires a DebugBatch", operation="debug_bus.publish")
        started = self._clock()
        selected, filtered, reasons, payload_bytes = self._select_and_limit(batch, started)
        failures: list[str] = []
        successful_sinks = 0
        if selected:
            selected_batch = batch.with_primitives(selected)
            for index, sink in enumerate(self._sinks):
                elapsed = (self._clock() - started) * 1000.0
                if elapsed > self._budget.max_publish_duration_ms:
                    _increment(reasons, "dispatch_time", len(self._sinks) - index)
                    break
                try:
                    sink.publish(selected_batch)
                    successful_sinks += 1
                except Exception as exc:  # sinks are failure-isolated by contract
                    failures.append(f"sink[{index}] {type(exc).__name__}: {exc}")
        accepted = len(selected) if successful_sinks else 0
        if accepted:
            for primitive in selected:
                self._active[primitive.key] = primitive.lifetime
            self._rate_window.append((started, payload_bytes))
        elif selected:
            _increment(reasons, "sink_failure", len(selected))
        elapsed_ms = max(0.0, (self._clock() - started) * 1000.0)
        budget_exceeded = bool(reasons) or elapsed_ms > self._budget.max_publish_duration_ms
        dropped = sum(reasons.values())
        report = DebugPublishReport(
            accepted,
            dropped,
            len(self._active),
            filtered,
            elapsed_ms,
            budget_exceeded,
            FrozenMap(reasons),
            tuple(failures),
        )
        report_failures: list[str] = []
        for index, sink in enumerate(self._sinks):
            if isinstance(sink, DebugReportSink):
                try:
                    sink.publish_report(batch, report)
                except Exception as exc:
                    report_failures.append(f"sink[{index}] report {type(exc).__name__}: {exc}")
        if report_failures:
            report = DebugPublishReport(
                report.accepted_count,
                report.dropped_count,
                report.active_count,
                report.filtered_count,
                report.elapsed_ms,
                report.budget_exceeded,
                report.drop_reasons,
                report.sink_failures + tuple(report_failures),
            )
        return report

    def clear(
        self,
        *,
        layer: str | None = None,
        group: str | None = None,
        primitive_id: str | None = None,
    ) -> int:
        self._ensure_open("debug_bus.clear")
        for name, value in (("layer", layer), ("group", group), ("primitive_id", primitive_id)):
            if value is not None:
                _validate_name(value, name)
        keys = tuple(key for key in self._active if _matches(key, layer, group, primitive_id))
        for sink in self._sinks:
            try:
                sink.clear(layer=layer, group=group, primitive_id=primitive_id)
            except Exception:
                continue
        for key in keys:
            del self._active[key]
        return len(keys)

    def advance(self, count: int = 1) -> int:
        """Advance frame and step lifetimes after physics advances by ``count`` steps."""

        self._ensure_open("debug_bus.advance")
        if not isinstance(count, int) or isinstance(count, bool) or count <= 0:
            raise ValidationError("debug advance count must be positive", operation="debug_bus.advance")
        expired: list[tuple[str, str, str]] = []
        for key, lifetime in tuple(self._active.items()):
            if lifetime.mode is DebugLifetimeMode.FRAME:
                expired.append(key)
            elif lifetime.mode is DebugLifetimeMode.STEPS:
                assert lifetime.step_count is not None
                remaining = lifetime.step_count - count
                if remaining <= 0:
                    expired.append(key)
                else:
                    self._active[key] = DebugLifetime.steps(remaining)
        for layer, group, primitive_id in expired:
            for sink in self._sinks:
                try:
                    sink.clear(layer=layer, group=group, primitive_id=primitive_id)
                except Exception:
                    continue
            del self._active[(layer, group, primitive_id)]
        return len(expired)

    def reset(self) -> int:
        """Clear all reset-scoped primitives while preserving manual lifetime objects."""

        self._ensure_open("debug_bus.reset")
        keys = tuple(key for key, lifetime in self._active.items() if lifetime.mode is not DebugLifetimeMode.MANUAL)
        for sink in self._sinks:
            try:
                sink.reset()
            except Exception:
                continue
        for key in keys:
            del self._active[key]
        return len(keys)

    def close(self) -> None:
        if self._closed:
            return
        for sink in self._sinks:
            try:
                sink.close()
            except Exception:
                continue
        self._active.clear()
        self._rate_window.clear()
        self._closed = True
