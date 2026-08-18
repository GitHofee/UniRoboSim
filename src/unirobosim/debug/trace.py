"""Canonical JSONL trace recording, validation, inspection and backend-free replay."""

from __future__ import annotations

import json
import math
import re
from collections.abc import Mapping
from dataclasses import dataclass
from enum import StrEnum
from pathlib import Path
from typing import TextIO, cast

from unirobosim.api.errors import LifecycleError, ValidationError
from unirobosim.api.frozen import FrozenMap

from .bus import DebugPublishReport, DebugSink, _matches
from .model import DEBUG_SCHEMA_VERSION, DebugBatch, DebugLifetimeMode, DebugPrimitive

_RUN_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,127}$")
_EVENT_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.:/-]{0,127}$")


class DebugTraceEventKind(StrEnum):
    PUBLISH = "publish"
    CLEAR = "clear"
    RESET = "reset"


@dataclass(frozen=True)
class DebugTraceEvent:
    sequence: int
    kind: DebugTraceEventKind
    batch: DebugBatch | None = None
    layer: str | None = None
    group: str | None = None
    primitive_id: str | None = None

    def __post_init__(self) -> None:
        operation = "debug_trace_event.validate"
        if not isinstance(self.sequence, int) or isinstance(self.sequence, bool) or self.sequence <= 0:
            raise ValidationError("trace event sequence must be positive", operation=operation)
        if not isinstance(self.kind, DebugTraceEventKind):
            raise ValidationError("trace event kind is invalid", operation=operation)
        if self.kind is DebugTraceEventKind.PUBLISH:
            if not isinstance(self.batch, DebugBatch) or any(
                value is not None for value in (self.layer, self.group, self.primitive_id)
            ):
                raise ValidationError("publish trace event payload is invalid", operation=operation)
        elif self.batch is not None:
            raise ValidationError("non-publish trace event cannot carry a batch", operation=operation)
        if self.kind is DebugTraceEventKind.RESET and any(
            value is not None for value in (self.layer, self.group, self.primitive_id)
        ):
            raise ValidationError("reset trace event cannot carry selectors", operation=operation)


@dataclass(frozen=True)
class DebugTracePublishReport:
    """One persisted Debug Bus decision, including fully rejected publishes."""

    report_sequence: int
    event_id: str | None
    step_index: int
    sim_time_s: float
    world_generation: int
    requested_count: int
    accepted_count: int
    dropped_count: int
    filtered_count: int
    active_count: int
    elapsed_ms: float
    budget_exceeded: bool
    drop_reasons: FrozenMap
    sink_failures: tuple[str, ...]

    def __post_init__(self) -> None:
        integers = (
            self.report_sequence,
            self.step_index,
            self.world_generation,
            self.requested_count,
            self.accepted_count,
            self.dropped_count,
            self.filtered_count,
            self.active_count,
        )
        if (
            any(not isinstance(value, int) or isinstance(value, bool) or value < 0 for value in integers)
            or self.report_sequence <= 0
            or self.requested_count <= 0
            or self.accepted_count > self.requested_count
            or self.filtered_count > self.requested_count
            or self.accepted_count + self.filtered_count > self.requested_count
            or (
                self.event_id is not None
                and (not isinstance(self.event_id, str) or not _EVENT_ID.fullmatch(self.event_id))
            )
            or not isinstance(self.sim_time_s, (int, float))
            or isinstance(self.sim_time_s, bool)
            or not isinstance(self.elapsed_ms, (int, float))
            or isinstance(self.elapsed_ms, bool)
            or not math.isfinite(float(self.sim_time_s))
            or not math.isfinite(float(self.elapsed_ms))
            or float(self.sim_time_s) < 0.0
            or float(self.elapsed_ms) < 0.0
            or not isinstance(self.budget_exceeded, bool)
            or not isinstance(self.drop_reasons, FrozenMap)
            or any(
                not isinstance(value, int) or isinstance(value, bool) or value <= 0
                for value in self.drop_reasons.values()
            )
        ):
            raise ValidationError("debug trace publish report is invalid", operation="debug_trace_report.validate")
        try:
            failures = tuple(self.sink_failures)
        except TypeError as exc:
            raise ValidationError(
                "debug trace publish report failures are invalid",
                operation="debug_trace_report.validate",
            ) from exc
        if any(not isinstance(item, str) or not item for item in failures):
            raise ValidationError(
                "debug trace publish report failures are invalid",
                operation="debug_trace_report.validate",
            )
        object.__setattr__(self, "sim_time_s", float(self.sim_time_s))
        object.__setattr__(self, "elapsed_ms", float(self.elapsed_ms))
        object.__setattr__(self, "sink_failures", failures)


@dataclass(frozen=True)
class DebugTraceManifest:
    schema_version: str
    run_id: str
    metadata: FrozenMap
    event_count: int
    publish_count: int
    primitive_count: int
    active_count: int
    report_count: int
    accepted_count: int
    dropped_count: int
    filtered_count: int
    drop_reasons: FrozenMap
    layers: tuple[str, ...]
    groups: tuple[str, ...]
    environment_indices: tuple[int, ...]
    first_step_index: int | None
    last_step_index: int | None
    closed: bool


@dataclass(frozen=True)
class DebugTrace:
    manifest: DebugTraceManifest
    events: tuple[DebugTraceEvent, ...]
    reports: tuple[DebugTracePublishReport, ...]


@dataclass(frozen=True)
class DebugReplayReport:
    event_count: int
    publish_count: int
    clear_count: int
    reset_count: int
    final_active_count: int


def _record_mapping(value: object, line_number: int) -> Mapping[str, object]:
    if not isinstance(value, Mapping) or any(not isinstance(key, str) for key in value):
        raise ValidationError(
            "debug trace line must contain a JSON object",
            operation="debug_trace.read",
            details={"line": line_number},
        )
    return cast(Mapping[str, object], value)


class TraceDebugSink:
    """Canonical, flush-on-event JSON Lines sink for replayable debug evidence."""

    def __init__(
        self,
        path: str | Path,
        *,
        run_id: str = "debug-run",
        metadata: FrozenMap | None = None,
    ) -> None:
        if not isinstance(run_id, str) or not _RUN_ID.fullmatch(run_id):
            raise ValidationError("debug trace run ID is invalid", operation="trace_debug_sink.init")
        if metadata is not None and not isinstance(metadata, FrozenMap):
            raise ValidationError("debug trace metadata is invalid", operation="trace_debug_sink.init")
        self._path = Path(path)
        self._path.parent.mkdir(parents=True, exist_ok=True)
        self._stream: TextIO = self._path.open("w", encoding="utf-8", newline="\n")
        self._run_id = run_id
        self._metadata = metadata or FrozenMap()
        self._sequence = 0
        self._event_count = 0
        self._publish_count = 0
        self._primitive_count = 0
        self._report_count = 0
        self._accepted_count = 0
        self._dropped_count = 0
        self._filtered_count = 0
        self._drop_reasons: dict[str, int] = {}
        self._active: dict[tuple[str, str, str], DebugPrimitive] = {}
        self._closed = False
        self._write(
            {
                "record": "header",
                "schema": DEBUG_SCHEMA_VERSION,
                "run_id": run_id,
                "metadata": self._metadata.to_dict(),
            }
        )

    @property
    def path(self) -> Path:
        return self._path

    def _write(self, payload: dict[str, object]) -> None:
        if self._closed:
            raise LifecycleError("debug trace sink is closed", operation="trace_debug_sink.write")
        self._stream.write(json.dumps(payload, sort_keys=True, separators=(",", ":"), ensure_ascii=False) + "\n")
        self._stream.flush()

    def _event(self, kind: DebugTraceEventKind, payload: dict[str, object]) -> None:
        self._sequence += 1
        self._event_count += 1
        self._write(
            {
                "record": "event",
                "schema": DEBUG_SCHEMA_VERSION,
                "sequence": self._sequence,
                "event": kind.value,
                **payload,
            }
        )

    def publish(self, batch: DebugBatch) -> None:
        if not isinstance(batch, DebugBatch):
            raise ValidationError("publish requires a DebugBatch", operation="trace_debug_sink.publish")
        self._event(DebugTraceEventKind.PUBLISH, {"batch": batch.to_dict()})
        self._publish_count += 1
        self._primitive_count += len(batch.primitives)
        for primitive in batch.primitives:
            self._active[primitive.key] = primitive

    def publish_report(self, batch: DebugBatch, report: DebugPublishReport) -> None:
        if not isinstance(batch, DebugBatch) or not isinstance(report, DebugPublishReport):
            raise ValidationError(
                "trace publish report requires a DebugBatch and DebugPublishReport",
                operation="trace_debug_sink.publish_report",
            )
        self._report_count += 1
        self._accepted_count += report.accepted_count
        self._dropped_count += report.dropped_count
        self._filtered_count += report.filtered_count
        for reason, count in report.drop_reasons.items():
            assert isinstance(count, int)
            self._drop_reasons[reason] = self._drop_reasons.get(reason, 0) + count
        self._write(
            {
                "record": "report",
                "schema": DEBUG_SCHEMA_VERSION,
                "report_sequence": self._report_count,
                "event_id": batch.event_id,
                "step_index": batch.step_index,
                "sim_time_s": batch.sim_time_s,
                "world_generation": batch.world_generation,
                "requested_count": len(batch.primitives),
                "accepted_count": report.accepted_count,
                "dropped_count": report.dropped_count,
                "filtered_count": report.filtered_count,
                "active_count": report.active_count,
                "elapsed_ms": report.elapsed_ms,
                "budget_exceeded": report.budget_exceeded,
                "drop_reasons": report.drop_reasons.to_dict(),
                "sink_failures": report.sink_failures,
            }
        )

    def clear(
        self,
        *,
        layer: str | None = None,
        group: str | None = None,
        primitive_id: str | None = None,
    ) -> int:
        self._event(
            DebugTraceEventKind.CLEAR,
            {"layer": layer, "group": group, "primitive_id": primitive_id},
        )
        keys = tuple(key for key in self._active if _matches(key, layer, group, primitive_id))
        for key in keys:
            del self._active[key]
        return len(keys)

    def reset(self) -> int:
        self._event(DebugTraceEventKind.RESET, {})
        keys = tuple(
            key for key, primitive in self._active.items() if primitive.lifetime.mode is not DebugLifetimeMode.MANUAL
        )
        for key in keys:
            del self._active[key]
        return len(keys)

    def close(self) -> None:
        if self._closed:
            return
        self._sequence += 1
        self._write(
            {
                "record": "close",
                "schema": DEBUG_SCHEMA_VERSION,
                "sequence": self._sequence,
                "event_count": self._event_count,
                "publish_count": self._publish_count,
                "primitive_count": self._primitive_count,
                "active_count": len(self._active),
                "report_count": self._report_count,
                "accepted_count": self._accepted_count,
                "dropped_count": self._dropped_count,
                "filtered_count": self._filtered_count,
                "drop_reasons": self._drop_reasons,
            }
        )
        self._stream.close()
        self._closed = True


class DebugTraceReader:
    """Strict reader that validates the entire bounded trace before exposing events."""

    def __init__(self, *, max_bytes: int = 64 * 1024 * 1024, max_events: int = 1_000_000) -> None:
        if any(
            not isinstance(value, int) or isinstance(value, bool) or value <= 0 for value in (max_bytes, max_events)
        ):
            raise ValidationError("debug trace reader limits must be positive", operation="debug_trace_reader.init")
        self._max_bytes = max_bytes
        self._max_events = max_events

    def read(self, path: str | Path) -> DebugTrace:
        trace_path = Path(path)
        try:
            size = trace_path.stat().st_size
        except OSError as exc:
            raise ValidationError("debug trace is unavailable", operation="debug_trace.read") from exc
        if size > self._max_bytes:
            raise ValidationError(
                "debug trace exceeds the configured byte limit",
                operation="debug_trace.read",
                details={"size": size, "limit": self._max_bytes},
            )
        try:
            lines = trace_path.read_text(encoding="utf-8").splitlines()
        except (OSError, UnicodeError) as exc:
            raise ValidationError("debug trace cannot be decoded as UTF-8", operation="debug_trace.read") from exc
        if len(lines) < 2:
            raise ValidationError("debug trace must contain a header and close record", operation="debug_trace.read")
        decoded: list[Mapping[str, object]] = []
        for line_number, line in enumerate(lines, start=1):
            try:
                decoded.append(_record_mapping(json.loads(line), line_number))
            except json.JSONDecodeError as exc:
                raise ValidationError(
                    "debug trace contains invalid JSON",
                    operation="debug_trace.read",
                    details={"line": line_number},
                ) from exc
        header = decoded[0]
        if header.get("record") != "header" or header.get("schema") != DEBUG_SCHEMA_VERSION:
            raise ValidationError("debug trace header/schema is unsupported", operation="debug_trace.read")
        run_id = header.get("run_id")
        if not isinstance(run_id, str) or not _RUN_ID.fullmatch(run_id):
            raise ValidationError("debug trace run ID is invalid", operation="debug_trace.read")
        metadata_value = header.get("metadata", {})
        if not isinstance(metadata_value, Mapping):
            raise ValidationError("debug trace metadata is invalid", operation="debug_trace.read")
        metadata = FrozenMap(cast(Mapping[str, object], metadata_value))
        close = decoded[-1]
        if close.get("record") != "close" or close.get("schema") != DEBUG_SCHEMA_VERSION:
            raise ValidationError("debug trace is not closed", operation="debug_trace.read")
        events: list[DebugTraceEvent] = []
        reports: list[DebugTracePublishReport] = []
        expected_sequence = 1
        expected_report_sequence = 1
        for record in decoded[1:-1]:
            if record.get("record") == "report":
                if len(events) + len(reports) >= self._max_events:
                    raise ValidationError("debug trace exceeds the record limit", operation="debug_trace.read")
                if (
                    record.get("schema") != DEBUG_SCHEMA_VERSION
                    or record.get("report_sequence") != expected_report_sequence
                ):
                    raise ValidationError(
                        "debug trace report sequence/schema is invalid",
                        operation="debug_trace.read",
                    )
                reasons_value = record.get("drop_reasons", {})
                if not isinstance(reasons_value, Mapping):
                    raise ValidationError("debug trace report reasons are invalid", operation="debug_trace.read")
                failures_value = record.get("sink_failures", ())
                if not isinstance(failures_value, (list, tuple)):
                    raise ValidationError("debug trace report failures are invalid", operation="debug_trace.read")
                try:
                    reports.append(
                        DebugTracePublishReport(
                            report_sequence=expected_report_sequence,
                            event_id=cast(str | None, record.get("event_id")),
                            step_index=cast(int, record.get("step_index")),
                            sim_time_s=cast(float, record.get("sim_time_s")),
                            world_generation=cast(int, record.get("world_generation")),
                            requested_count=cast(int, record.get("requested_count")),
                            accepted_count=cast(int, record.get("accepted_count")),
                            dropped_count=cast(int, record.get("dropped_count")),
                            filtered_count=cast(int, record.get("filtered_count")),
                            active_count=cast(int, record.get("active_count")),
                            elapsed_ms=cast(float, record.get("elapsed_ms")),
                            budget_exceeded=cast(bool, record.get("budget_exceeded")),
                            drop_reasons=FrozenMap(cast(Mapping[str, object], reasons_value)),
                            sink_failures=tuple(cast(tuple[str, ...], failures_value)),
                        )
                    )
                except (TypeError, ValueError) as exc:
                    raise ValidationError("debug trace report is invalid", operation="debug_trace.read") from exc
                expected_report_sequence += 1
                continue
            if len(events) + len(reports) >= self._max_events:
                raise ValidationError("debug trace exceeds the record limit", operation="debug_trace.read")
            if (
                record.get("record") != "event"
                or record.get("schema") != DEBUG_SCHEMA_VERSION
                or record.get("sequence") != expected_sequence
            ):
                raise ValidationError("debug trace event sequence/schema is invalid", operation="debug_trace.read")
            try:
                kind = DebugTraceEventKind(cast(str, record.get("event")))
            except (TypeError, ValueError) as exc:
                raise ValidationError("debug trace event kind is invalid", operation="debug_trace.read") from exc
            if kind is DebugTraceEventKind.PUBLISH:
                event = DebugTraceEvent(expected_sequence, kind, batch=DebugBatch.from_dict(record.get("batch")))
            elif kind is DebugTraceEventKind.CLEAR:
                event = DebugTraceEvent(
                    expected_sequence,
                    kind,
                    layer=cast(str | None, record.get("layer")),
                    group=cast(str | None, record.get("group")),
                    primitive_id=cast(str | None, record.get("primitive_id")),
                )
            else:
                event = DebugTraceEvent(expected_sequence, kind)
            events.append(event)
            expected_sequence += 1
        if close.get("sequence") != expected_sequence or close.get("event_count") != len(events):
            raise ValidationError("debug trace close summary is inconsistent", operation="debug_trace.read")
        report_values = tuple(reports)
        if close.get("report_count", 0) != len(report_values):
            raise ValidationError("debug trace report summary is inconsistent", operation="debug_trace.read")
        totals = {
            "accepted_count": sum(item.accepted_count for item in report_values),
            "dropped_count": sum(item.dropped_count for item in report_values),
            "filtered_count": sum(item.filtered_count for item in report_values),
        }
        if any(close.get(name, 0) != value for name, value in totals.items()):
            raise ValidationError("debug trace report totals are inconsistent", operation="debug_trace.read")
        reasons: dict[str, int] = {}
        for report in report_values:
            for reason, count in report.drop_reasons.items():
                assert isinstance(count, int)
                reasons[reason] = reasons.get(reason, 0) + count
        if close.get("drop_reasons", {}) != reasons:
            raise ValidationError("debug trace drop-reason summary is inconsistent", operation="debug_trace.read")
        return DebugTrace(
            self._manifest(run_id, metadata, tuple(events), report_values, close),
            tuple(events),
            report_values,
        )

    @staticmethod
    def _manifest(
        run_id: str,
        metadata: FrozenMap,
        events: tuple[DebugTraceEvent, ...],
        reports: tuple[DebugTracePublishReport, ...],
        close: Mapping[str, object],
    ) -> DebugTraceManifest:
        active: dict[tuple[str, str, str], DebugPrimitive] = {}
        layers: set[str] = set()
        groups: set[str] = set()
        environments: set[int] = set()
        steps: list[int] = []
        publish_count = 0
        primitive_count = 0
        for event in events:
            if event.kind is DebugTraceEventKind.PUBLISH:
                assert event.batch is not None
                publish_count += 1
                primitive_count += len(event.batch.primitives)
                steps.append(event.batch.step_index)
                for primitive in event.batch.primitives:
                    active[primitive.key] = primitive
                    layers.add(primitive.layer)
                    groups.add(primitive.group)
                    environments.update(primitive.environment_indices)
            elif event.kind is DebugTraceEventKind.CLEAR:
                for key in tuple(active):
                    if _matches(key, event.layer, event.group, event.primitive_id):
                        del active[key]
            else:
                for key, primitive in tuple(active.items()):
                    if primitive.lifetime.mode is not DebugLifetimeMode.MANUAL:
                        del active[key]
        close_active = close.get("active_count")
        if close_active != len(active):
            raise ValidationError("debug trace active summary is inconsistent", operation="debug_trace.read")
        return DebugTraceManifest(
            DEBUG_SCHEMA_VERSION,
            run_id,
            metadata,
            len(events),
            publish_count,
            primitive_count,
            len(active),
            len(reports),
            sum(item.accepted_count for item in reports),
            sum(item.dropped_count for item in reports),
            sum(item.filtered_count for item in reports),
            FrozenMap(cast(Mapping[str, object], close.get("drop_reasons", {}))),
            tuple(sorted(layers)),
            tuple(sorted(groups)),
            tuple(sorted(environments)),
            min(steps) if steps else None,
            max(steps) if steps else None,
            True,
        )


def replay_debug_trace(trace: DebugTrace | str | Path, sink: DebugSink) -> DebugReplayReport:
    if not isinstance(sink, DebugSink):
        raise ValidationError("trace replay requires a DebugSink", operation="debug_trace.replay")
    debug_trace = DebugTraceReader().read(trace) if isinstance(trace, (str, Path)) else trace
    if not isinstance(debug_trace, DebugTrace):
        raise ValidationError("trace replay requires a DebugTrace or path", operation="debug_trace.replay")
    active: dict[tuple[str, str, str], DebugPrimitive] = {}
    publish_count = 0
    clear_count = 0
    reset_count = 0
    for event in debug_trace.events:
        if event.kind is DebugTraceEventKind.PUBLISH:
            assert event.batch is not None
            sink.publish(event.batch)
            publish_count += 1
            for primitive in event.batch.primitives:
                active[primitive.key] = primitive
        elif event.kind is DebugTraceEventKind.CLEAR:
            sink.clear(layer=event.layer, group=event.group, primitive_id=event.primitive_id)
            clear_count += 1
            for key in tuple(active):
                if _matches(key, event.layer, event.group, event.primitive_id):
                    del active[key]
        else:
            sink.reset()
            reset_count += 1
            for key, primitive in tuple(active.items()):
                if primitive.lifetime.mode is not DebugLifetimeMode.MANUAL:
                    del active[key]
    return DebugReplayReport(len(debug_trace.events), publish_count, clear_count, reset_count, len(active))
