from __future__ import annotations

import json
import sys
from dataclasses import replace
from pathlib import Path

import pytest

from unirobosim import (
    DEBUG_SCHEMA_VERSION,
    ArrayValue,
    DebugBatch,
    DebugBudget,
    DebugBus,
    DebugLifetime,
    DebugLifetimeMode,
    DebugPrimitive,
    DebugPrimitiveKind,
    DebugPublishReport,
    DebugSelection,
    DebugTraceReader,
    FrozenMap,
    LifecycleError,
    TestDebugSink,
    TraceDebugSink,
    ValidationError,
    build_portable_viewer,
    render_trace_svg,
    replay_debug_trace,
)
from unirobosim.debug.cli import main as viewer_main
from unirobosim.debug.trace import DebugTrace, DebugTraceEvent, DebugTraceEventKind


def primitive(
    kind: DebugPrimitiveKind,
    primitive_id: str | None = None,
    *,
    group: str = "geometry",
    lifetime: DebugLifetime | None = None,
) -> DebugPrimitive:
    common = {
        "primitive_id": primitive_id or kind.value,
        "layer": "debug.scene",
        "group": group,
        "source": "tests",
        "kind": kind,
        "environment_indices": (0, 2),
        "color_rgba": (0.1, 0.7, 1.0, 0.8),
        "size": 2.0,
        "lifetime": lifetime or DebugLifetime.persistent(),
        "metadata": FrozenMap({"test": True}),
    }
    if kind is DebugPrimitiveKind.POINT_SET:
        return DebugPrimitive(
            geometry_m=ArrayValue.from_nested([[[0.0, 0.0, 0.0]], [[2.0, 0.0, 0.0]]]),
            **common,
        )
    if kind is DebugPrimitiveKind.LINE_LIST:
        return DebugPrimitive(
            geometry_m=ArrayValue.from_nested(
                [
                    [[[-1.0, 0.0, 0.0], [1.0, 0.0, 0.0]]],
                    [[[1.0, 0.0, 0.0], [3.0, 0.0, 0.0]]],
                ]
            ),
            **common,
        )
    if kind is DebugPrimitiveKind.COORDINATE_AXES:
        return DebugPrimitive(
            geometry_m=ArrayValue.from_nested(
                [
                    [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]],
                    [[2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]],
                ]
            ),
            **common,
        )
    if kind is DebugPrimitiveKind.TEXT:
        return DebugPrimitive(
            geometry_m=ArrayValue.from_nested([[[0.0, 0.0, 0.5]], [[2.0, 0.0, 0.5]]]),
            text=(("origin",), ("环境二",)),
            **common,
        )
    if kind is DebugPrimitiveKind.BOUNDING_BOX:
        return DebugPrimitive(
            geometry_m=ArrayValue.from_nested(
                [
                    [[0.0, 0.0, 0.5, 1.0, 2.0, 1.0, 0.0, 0.0, 0.0, 1.0]],
                    [[2.0, 0.0, 0.5, 1.0, 2.0, 1.0, 0.0, 0.0, 0.0, 1.0]],
                ]
            ),
            **common,
        )
    return DebugPrimitive(
        geometry_m=ArrayValue.from_nested(
            [
                [[0.0, 0.0, 0.0], [0.5, 0.2, 0.1], [1.0, 0.0, 0.2]],
                [[2.0, 0.0, 0.0], [2.5, 0.2, 0.1], [3.0, 0.0, 0.2]],
            ]
        ),
        sample_times_s=ArrayValue.from_nested([[0.0, 0.5, 1.0], [0.0, 0.5, 1.0]]),
        **common,
    )


def all_primitives() -> tuple[DebugPrimitive, ...]:
    return tuple(primitive(kind) for kind in DebugPrimitiveKind)


class ManualClock:
    def __init__(self) -> None:
        self.value = 0.0

    def __call__(self) -> float:
        return self.value

    def advance(self, seconds: float) -> None:
        self.value += seconds


def test_every_required_primitive_round_trips_with_versioned_batch() -> None:
    batch = DebugBatch(all_primitives(), step_index=12, sim_time_s=0.2, world_generation=3, event_id="evt-12")
    decoded = DebugBatch.from_dict(batch.to_dict())
    assert decoded == batch
    assert DEBUG_SCHEMA_VERSION == "unirobosim.debug/v1alpha1"
    assert {item.kind for item in decoded.primitives} == set(DebugPrimitiveKind)
    assert decoded.primitives[2].vertex_count == 12
    assert decoded.primitives[4].vertex_count == 48


@pytest.mark.parametrize(
    ("kind", "geometry"),
    (
        (DebugPrimitiveKind.COORDINATE_AXES, [[[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0]]]),
        (DebugPrimitiveKind.BOUNDING_BOX, [[[0.0, 0.0, 0.0, -1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0]]]),
        (DebugPrimitiveKind.TRAJECTORY, [[[0.0, 0.0, 0.0]]]),
    ),
)
def test_semantic_geometry_validation_rejects_invalid_payload(kind: DebugPrimitiveKind, geometry: object) -> None:
    with pytest.raises(ValidationError):
        DebugPrimitive(
            "bad",
            "debug.scene",
            kind,
            ArrayValue.from_nested(geometry),
            (0,),
        )


def test_text_trajectory_and_lifetime_validation_edges() -> None:
    with pytest.raises(ValidationError):
        replace(primitive(DebugPrimitiveKind.TEXT), text=(("only-one-env",),))
    with pytest.raises(ValidationError):
        replace(primitive(DebugPrimitiveKind.POINT_SET), text=(("invalid",), ("invalid",)))
    with pytest.raises(ValidationError):
        replace(
            primitive(DebugPrimitiveKind.TRAJECTORY),
            sample_times_s=ArrayValue.from_nested([[0.0, 0.5, 0.4], [0.0, 0.5, 1.0]]),
        )
    for mode in (DebugLifetimeMode.FRAME, DebugLifetimeMode.PERSISTENT, DebugLifetimeMode.MANUAL):
        with pytest.raises(ValidationError):
            DebugLifetime(mode, 1)
    with pytest.raises(ValidationError):
        DebugLifetime.steps(0)
    with pytest.raises(ValidationError):
        DebugLifetime.from_dict({"mode": "unknown"})
    with pytest.raises(ValidationError):
        DebugLifetime.from_dict([])


def test_primitive_batch_selection_and_report_reject_malformed_values() -> None:
    point = primitive(DebugPrimitiveKind.POINT_SET)
    for override in ({"primitive_id": "bad name"}, {"group": ""}, {"source": "bad source"}):
        with pytest.raises(ValidationError):
            replace(point, **override)


@pytest.mark.parametrize(
    "override",
    (
        {"color_rgba": (1.0, 0.0, 0.0, 2.0)},
        {"color_rgba": None},
        {"size": True},
        {"size": 0.0},
        {"lifetime": "persistent"},
        {"metadata": {}},
        {"geometry_m": ArrayValue((2, 1, 3), (0, 0, 0, 1, 1, 1), dtype="int64")},
    ),
)
def test_common_primitive_validation_edges(override: dict[str, object]) -> None:
    point = primitive(DebugPrimitiveKind.POINT_SET)
    with pytest.raises(ValidationError):
        replace(point, **override)


def test_decode_batch_and_selection_validation_edges() -> None:
    point = primitive(DebugPrimitiveKind.POINT_SET)
    payload = point.to_dict()
    payload["kind"] = "future"
    with pytest.raises(ValidationError):
        DebugPrimitive.from_dict(payload)
    with pytest.raises(ValidationError):
        DebugPrimitive.from_dict([])
    with pytest.raises(ValidationError):
        DebugBatch.from_dict({})
    for kwargs in (
        {"step_index": -1},
        {"sim_time_s": float("inf")},
        {"world_generation": True},
        {"event_id": "bad event"},
    ):
        with pytest.raises(ValidationError):
            DebugBatch((point,), **kwargs)
    for kwargs in ({"layers": ()}, {"groups": ("bad group",)}, {"environment_indices": (True,)}):
        with pytest.raises(ValidationError):
            DebugSelection(**kwargs)


def test_selection_filters_layer_group_and_slices_every_environment_payload() -> None:
    selection = DebugSelection(layers=("debug.scene",), groups=("geometry",), environment_indices=(2,))
    for value in all_primitives():
        selected = selection.apply(value)
        assert selected is not None
        assert selected.environment_indices == (2,)
        assert selected.geometry_m.shape[0] == 1
        if selected.text is not None:
            assert selected.text == (("环境二",),)
        if selected.sample_times_s is not None:
            assert selected.sample_times_s.shape == (1, 3)
    assert DebugSelection(layers=("other",)).apply(primitive(DebugPrimitiveKind.POINT_SET)) is None
    with pytest.raises(ValidationError):
        DebugSelection(environment_indices=(0, 0))


def test_bus_lifetime_reset_group_and_stable_replacement() -> None:
    sink = TestDebugSink()
    bus = DebugBus((sink,))
    values = (
        primitive(DebugPrimitiveKind.POINT_SET, "frame", lifetime=DebugLifetime.frame()),
        primitive(DebugPrimitiveKind.POINT_SET, "steps", lifetime=DebugLifetime.steps(2)),
        primitive(DebugPrimitiveKind.POINT_SET, "persistent", lifetime=DebugLifetime.persistent()),
        primitive(DebugPrimitiveKind.POINT_SET, "manual", group="operator", lifetime=DebugLifetime.manual()),
    )
    assert bus.publish(DebugBatch(values)).accepted_count == 4
    assert bus.publish(DebugBatch((replace(values[2], size=7.0),))).active_count == 4
    assert next(item for item in sink.primitives if item.primitive_id == "persistent").size == 7.0
    assert bus.advance() == 1
    assert bus.advance() == 1
    assert bus.reset() == 1
    assert tuple(item.primitive_id for item in sink.primitives) == ("manual",)
    assert bus.clear(group="operator") == 1


def test_bus_filters_before_sink_and_reports_partial_budget_drops() -> None:
    sink = TestDebugSink()
    budget = DebugBudget(
        max_active_primitives=2,
        max_primitives_per_publish=2,
        max_vertices_per_publish=100,
        max_events_per_second=10,
        max_payload_bytes_per_second=1_000_000,
        max_publish_duration_ms=10.0,
    )
    bus = DebugBus(
        (sink,),
        selection=DebugSelection(groups=("geometry",), environment_indices=(2,)),
        budget=budget,
    )
    values = tuple(primitive(DebugPrimitiveKind.POINT_SET, f"point-{index}") for index in range(3))
    report = bus.publish(DebugBatch(values))
    assert (report.accepted_count, report.dropped_count, report.active_count) == (2, 1, 2)
    assert report.drop_reasons["batch_primitive_limit"] == 1
    assert all(item.environment_indices == (2,) for item in sink.primitives)


def test_event_payload_vertex_and_dispatch_time_budgets_are_accounted() -> None:
    clock = ManualClock()
    sink = TestDebugSink()
    event_budget = DebugBudget(max_events_per_second=1, max_payload_bytes_per_second=1_000_000)
    bus = DebugBus((sink,), budget=event_budget, clock=clock)
    batch = DebugBatch((primitive(DebugPrimitiveKind.POINT_SET),))
    assert bus.publish(batch).accepted_count == 1
    rate_drop = bus.publish(batch)
    assert rate_drop.drop_reasons["event_rate"] == 1
    clock.advance(1.0)
    assert bus.publish(batch).accepted_count == 1

    vertex_bus = DebugBus((TestDebugSink(),), budget=DebugBudget(max_vertices_per_publish=1))
    vertex_drop = vertex_bus.publish(DebugBatch((primitive(DebugPrimitiveKind.LINE_LIST),)))
    assert vertex_drop.drop_reasons["batch_vertex_limit"] == 1

    payload_bus = DebugBus((TestDebugSink(),), budget=DebugBudget(max_payload_bytes_per_second=1))
    payload_drop = payload_bus.publish(batch)
    assert payload_drop.drop_reasons["payload_rate"] == 1

    class SlowSink(TestDebugSink):
        def publish(self, value: DebugBatch) -> None:
            super().publish(value)
            clock.advance(0.02)

    clock.value = 0.0
    timing_bus = DebugBus(
        (SlowSink(), TestDebugSink()),
        budget=DebugBudget(max_publish_duration_ms=5.0),
        clock=clock,
    )
    timing = timing_bus.publish(batch)
    assert timing.accepted_count == 1
    assert timing.budget_exceeded
    assert timing.drop_reasons["dispatch_time"] == 1
    assert timing.elapsed_ms == pytest.approx(20.0)


def test_budget_bus_and_sink_failure_edges_are_structured() -> None:
    for kwargs in (
        {"max_active_primitives": 0},
        {"max_events_per_second": True},
        {"max_publish_duration_ms": 0.0},
        {"max_publish_duration_ms": float("nan")},
    ):
        with pytest.raises(ValidationError):
            DebugBudget(**kwargs)
    with pytest.raises(ValidationError):
        DebugBus(())
    with pytest.raises(ValidationError):
        DebugBus((TestDebugSink(),), selection=object())  # type: ignore[arg-type]
    with pytest.raises(ValidationError):
        DebugBus((TestDebugSink(),), budget=object())  # type: ignore[arg-type]
    with pytest.raises(ValidationError):
        DebugBus((TestDebugSink(),), clock=None)  # type: ignore[arg-type]
    with pytest.raises(ValidationError):
        DebugBus((TestDebugSink(),), budget=DebugBudget(), max_active_primitives=1)

    class BrokenSink:
        def publish(self, batch: DebugBatch) -> None:
            raise RuntimeError("publish")

        def clear(
            self,
            *,
            layer: str | None = None,
            group: str | None = None,
            primitive_id: str | None = None,
        ) -> int:
            raise RuntimeError("clear")

        def reset(self) -> int:
            raise RuntimeError("reset")

        def close(self) -> None:
            raise RuntimeError("close")

    bus = DebugBus((BrokenSink(),))
    report = bus.publish(DebugBatch((primitive(DebugPrimitiveKind.POINT_SET),)))
    assert report.accepted_count == 0
    assert report.drop_reasons["sink_failure"] == 1
    assert report.sink_failures and "RuntimeError" in report.sink_failures[0]
    assert bus.clear() == 0
    assert bus.reset() == 0
    bus.close()

    sink = TestDebugSink()
    active_bus = DebugBus((sink,), budget=DebugBudget(max_active_primitives=1))
    active_bus.publish(DebugBatch((primitive(DebugPrimitiveKind.POINT_SET, "one"),)))
    active_drop = active_bus.publish(DebugBatch((primitive(DebugPrimitiveKind.POINT_SET, "two"),)))
    assert active_drop.drop_reasons["active_primitive_limit"] == 1
    active_bus.set_selection(DebugSelection(layers=("hidden",)))
    filtered = active_bus.publish(DebugBatch((primitive(DebugPrimitiveKind.POINT_SET, "one"),)))
    assert filtered.filtered_count == 1 and filtered.accepted_count == 0


def test_native_sink_reset_preserves_manual_and_validates_closed_state() -> None:
    from unirobosim import (
        CameraSpec,
        EntityKind,
        EntityPath,
        EntitySpec,
        EnvironmentSpec,
        NativeWorldDebugSink,
        WorldSpec,
    )
    from unirobosim.testing import FakeProvider

    session = FakeProvider().open()
    world = session.build(
        WorldSpec(
            "native-sink",
            (
                EntitySpec(
                    EntityPath("/camera"),
                    EntityKind.CAMERA_SENSOR,
                    camera=CameraSpec(width_px=2, height_px=2),
                ),
            ),
            environments=EnvironmentSpec(3),
        )
    )
    sink = NativeWorldDebugSink(world)
    values = (
        primitive(DebugPrimitiveKind.POINT_SET, "persistent"),
        primitive(DebugPrimitiveKind.POINT_SET, "manual", lifetime=DebugLifetime.manual()),
    )
    sink.publish(DebugBatch(values))
    assert sink.reset() == 1
    assert world.clear_debug(primitive_id="persistent") == 0
    assert sink.clear(primitive_id="manual") == 1
    sink.close()
    with pytest.raises(LifecycleError):
        sink.reset()


def _write_trace(path: Path) -> None:
    sink = TraceDebugSink(path, run_id="m4-test", metadata=FrozenMap({"seed": 42}))
    sink.publish(DebugBatch(all_primitives(), step_index=1, sim_time_s=0.1, world_generation=2))
    sink.publish(
        DebugBatch(
            (primitive(DebugPrimitiveKind.POINT_SET, "manual", lifetime=DebugLifetime.manual()),),
            step_index=2,
            sim_time_s=0.2,
            world_generation=2,
        )
    )
    sink.clear(group="geometry", primitive_id=DebugPrimitiveKind.LINE_LIST.value)
    sink.reset()
    sink.close()


def test_trace_manifest_validation_and_replay_reproduce_final_state(tmp_path: Path) -> None:
    path = tmp_path / "run.urs-debug.jsonl"
    _write_trace(path)
    trace = DebugTraceReader().read(path)
    assert trace.manifest.run_id == "m4-test"
    assert trace.manifest.event_count == 4
    assert trace.manifest.publish_count == 2
    assert trace.manifest.primitive_count == 7
    assert trace.manifest.active_count == 1
    assert trace.manifest.report_count == 0
    assert trace.reports == ()
    assert trace.manifest.environment_indices == (0, 2)
    sink = TestDebugSink()
    report = replay_debug_trace(trace, sink)
    assert (report.event_count, report.publish_count, report.clear_count, report.reset_count) == (4, 2, 1, 1)
    assert report.final_active_count == 1


def test_trace_persists_accepted_and_fully_rejected_publish_reports(tmp_path: Path) -> None:
    clock = ManualClock()
    path = tmp_path / "reports.jsonl"
    sink = TraceDebugSink(path, run_id="report-test")
    bus = DebugBus(
        (sink,),
        budget=DebugBudget(max_events_per_second=1, max_payload_bytes_per_second=1_000_000),
        clock=clock,
    )
    batch = DebugBatch((primitive(DebugPrimitiveKind.POINT_SET),), event_id="budget-event")
    assert bus.publish(batch).accepted_count == 1
    rejected = bus.publish(batch)
    assert rejected.accepted_count == 0 and rejected.drop_reasons["event_rate"] == 1
    bus.close()

    trace = DebugTraceReader().read(path)
    assert len(trace.events) == 1
    assert len(trace.reports) == 2
    assert trace.reports[0].accepted_count == 1
    assert trace.reports[1].dropped_count == 1
    assert trace.reports[1].event_id == "budget-event"
    assert trace.manifest.report_count == 2
    assert trace.manifest.accepted_count == 1
    assert trace.manifest.dropped_count == 1
    assert trace.manifest.drop_reasons["event_rate"] == 1


def test_report_side_channel_failure_is_isolated() -> None:
    class BrokenReportSink(TestDebugSink):
        def publish_report(self, batch: DebugBatch, report: DebugPublishReport) -> None:
            raise RuntimeError("audit channel")

    sink = BrokenReportSink()
    report = DebugBus((sink,)).publish(DebugBatch((primitive(DebugPrimitiveKind.POINT_SET),)))
    assert report.accepted_count == 1
    assert any("report RuntimeError" in failure for failure in report.sink_failures)
    assert tuple(item.primitive_id for item in sink.primitives) == ("point_set",)


def test_trace_reader_rejects_schema_sequence_truncation_and_limits(tmp_path: Path) -> None:
    path = tmp_path / "run.jsonl"
    _write_trace(path)
    records = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]
    records[0]["schema"] = "future"
    path.write_text("\n".join(json.dumps(item) for item in records) + "\n", encoding="utf-8")
    with pytest.raises(ValidationError, match="schema"):
        DebugTraceReader().read(path)
    _write_trace(path)
    records = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]
    records[2]["sequence"] = 99
    path.write_text("\n".join(json.dumps(item) for item in records) + "\n", encoding="utf-8")
    with pytest.raises(ValidationError, match="sequence"):
        DebugTraceReader().read(path)
    _write_trace(path)
    path.write_text("\n".join(path.read_text(encoding="utf-8").splitlines()[:-1]), encoding="utf-8")
    with pytest.raises(ValidationError, match="closed"):
        DebugTraceReader().read(path)
    _write_trace(path)
    with pytest.raises(ValidationError, match="byte limit"):
        DebugTraceReader(max_bytes=10).read(path)
    with pytest.raises(ValidationError, match="record limit"):
        DebugTraceReader(max_events=1).read(path)


def test_trace_constructor_reader_and_replay_validation_edges(tmp_path: Path) -> None:
    with pytest.raises(ValidationError):
        TraceDebugSink(tmp_path / "bad.jsonl", run_id="bad run")
    with pytest.raises(ValidationError):
        TraceDebugSink(tmp_path / "bad.jsonl", metadata={})  # type: ignore[arg-type]
    with pytest.raises(ValidationError):
        DebugTraceReader(max_events=0)
    with pytest.raises(ValidationError, match="unavailable"):
        DebugTraceReader().read(tmp_path / "missing.jsonl")
    with pytest.raises(ValidationError):
        DebugTraceEvent(0, DebugTraceEventKind.RESET)
    with pytest.raises(ValidationError):
        DebugTraceEvent(1, DebugTraceEventKind.PUBLISH)
    with pytest.raises(ValidationError):
        DebugTraceEvent(1, DebugTraceEventKind.RESET, layer="bad")
    with pytest.raises(ValidationError):
        replay_debug_trace(DebugTrace, TestDebugSink())  # type: ignore[arg-type]
    with pytest.raises(ValidationError):
        replay_debug_trace(DebugTraceReader, object())  # type: ignore[arg-type]

    path = tmp_path / "bad-json.jsonl"
    path.write_text("{}\nnot-json\n", encoding="utf-8")
    with pytest.raises(ValidationError, match="invalid JSON"):
        DebugTraceReader().read(path)
    path.write_text("[]\n{}\n", encoding="utf-8")
    with pytest.raises(ValidationError, match="JSON object"):
        DebugTraceReader().read(path)


def test_portable_viewer_and_svg_are_self_contained_and_render_all_kinds(tmp_path: Path) -> None:
    trace_path = tmp_path / "run.jsonl"
    _write_trace(trace_path)
    html_path = tmp_path / "viewer.html"
    svg_path = tmp_path / "frame.svg"
    html_report = build_portable_viewer(trace_path, html_path, title="M4 evidence")
    svg_report = render_trace_svg(trace_path, svg_path, sequence=1, environment_indices=(0,))
    payload = html_path.read_text(encoding="utf-8")
    svg = svg_path.read_text(encoding="utf-8")
    assert html_report.frame_count == 5
    assert html_report.primitive_count == 7
    assert len(html_report.sha256) == 64
    assert "<script src=" not in payload
    assert "fetch(" not in payload
    assert "__URS_VIEWER_READY__" in payload
    assert all(f'"kind":"{kind.value}"' in payload for kind in DebugPrimitiveKind)
    assert svg_report.primitive_count == 6
    assert "origin" in svg
    assert "<line" in svg and "<circle" in svg and "<text" in svg


def test_viewer_validation_and_cli_html_svg_paths(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    trace_path = tmp_path / "run.jsonl"
    _write_trace(trace_path)
    with pytest.raises(ValidationError):
        build_portable_viewer(trace_path, tmp_path / "x.html", title="")
    with pytest.raises(ValidationError):
        render_trace_svg(trace_path, tmp_path / "x.svg", width=10)
    with pytest.raises(ValidationError):
        render_trace_svg(trace_path, tmp_path / "x.svg", sequence=-1)
    with pytest.raises(ValidationError):
        build_portable_viewer(object(), tmp_path / "x.html")  # type: ignore[arg-type]
    html_path = tmp_path / "cli.html"
    monkeypatch.setattr(sys, "argv", ["viewer", str(trace_path), "--output", str(html_path)])
    assert viewer_main() == 0 and html_path.exists()
    svg_path = tmp_path / "cli.svg"
    monkeypatch.setattr(
        sys,
        "argv",
        ["viewer", str(trace_path), "--output", str(svg_path), "--sequence", "1"],
    )
    assert viewer_main() == 0 and svg_path.exists()
