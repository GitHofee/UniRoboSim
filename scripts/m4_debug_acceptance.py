"""Generate deterministic M4 trace, viewer, SVG and budget acceptance evidence."""

from __future__ import annotations

import argparse
import hashlib
import json
import time
from pathlib import Path

from unirobosim import (
    ArrayValue,
    DebugBatch,
    DebugBudget,
    DebugBus,
    DebugLifetime,
    DebugPrimitive,
    DebugPrimitiveKind,
    DebugTraceReader,
    FrozenMap,
    TestDebugSink,
    TraceDebugSink,
    build_portable_viewer,
    render_trace_svg,
    replay_debug_trace,
)


def _primitive(kind: DebugPrimitiveKind, step: int) -> DebugPrimitive:
    phase = step / 60.0
    common = {
        "primitive_id": kind.value,
        "layer": "acceptance.debug",
        "group": "m4-primitives",
        "source": "unirobosim.acceptance",
        "kind": kind,
        "environment_indices": (0,),
        "color_rgba": (0.1, 0.85, 1.0, 1.0),
        "size": 3.0,
        "lifetime": DebugLifetime.persistent(),
        "metadata": FrozenMap({"step": step}),
    }
    if kind is DebugPrimitiveKind.POINT_SET:
        geometry = ArrayValue.from_nested([[[phase * 2.0 - 1.0, -0.8, 0.2]]])
        return DebugPrimitive(geometry_m=geometry, **common)
    if kind is DebugPrimitiveKind.LINE_LIST:
        geometry = ArrayValue.from_nested([[[[-1.2, -0.5, 0.0], [1.2, -0.5, 0.0]]]])
        return DebugPrimitive(geometry_m=geometry, **(common | {"color_rgba": (1.0, 0.75, 0.1, 1.0)}))
    if kind is DebugPrimitiveKind.COORDINATE_AXES:
        geometry = ArrayValue.from_nested([[[0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 1.0]]])
        return DebugPrimitive(geometry_m=geometry, **(common | {"size": 0.65}))
    if kind is DebugPrimitiveKind.TEXT:
        geometry = ArrayValue.from_nested([[[0.0, 0.0, 1.25]]])
        return DebugPrimitive(geometry_m=geometry, text=((f"M4 step {step:02d}",),), **common)
    if kind is DebugPrimitiveKind.BOUNDING_BOX:
        geometry = ArrayValue.from_nested([[[0.0, 0.0, 0.5, 1.6, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0]]])
        return DebugPrimitive(geometry_m=geometry, **(common | {"color_rgba": (1.0, 0.25, 0.3, 1.0)}))
    geometry = ArrayValue.from_nested(
        [
            [
                [-1.0, 0.7, 0.2],
                [-0.5, 0.7 + 0.2 * phase, 0.45],
                [0.0, 0.7, 0.7],
                [0.5, 0.7 - 0.2 * phase, 0.45],
                [1.0, 0.7, 0.2],
            ]
        ]
    )
    return DebugPrimitive(
        geometry_m=geometry,
        sample_times_s=ArrayValue.from_nested([[0.0, 0.25, 0.5, 0.75, 1.0]]),
        **(common | {"color_rgba": (0.8, 0.35, 1.0, 1.0)}),
    )


class _Clock:
    def __init__(self) -> None:
        self.value = 0.0

    def __call__(self) -> float:
        return self.value


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    trace_path = args.output_dir / "m4-debug-trace.urs-debug.jsonl"
    budget_trace_path = args.output_dir / "m4-budget-trace.urs-debug.jsonl"
    viewer_path = args.output_dir / "m4-portable-viewer.html"
    svg_path = args.output_dir / "m4-portable-frame.svg"
    report_path = args.output_dir / "portable-result.json"

    trace_sink = TraceDebugSink(
        trace_path,
        run_id="m4-portable-acceptance",
        metadata=FrozenMap({"seed": 42, "frames": 60}),
    )
    memory_sink = TestDebugSink()
    bus = DebugBus(
        (trace_sink, memory_sink),
        budget=DebugBudget(
            max_active_primitives=16,
            max_primitives_per_publish=16,
            max_vertices_per_publish=10_000,
            max_events_per_second=10_000,
            max_payload_bytes_per_second=64 * 1024 * 1024,
            max_publish_duration_ms=100.0,
        ),
    )
    publish_reports = []
    started = time.perf_counter()
    for step in range(60):
        batch = DebugBatch(
            tuple(_primitive(kind, step) for kind in DebugPrimitiveKind),
            step_index=step,
            sim_time_s=step / 30.0,
            world_generation=1,
            event_id=f"frame-{step:04d}",
        )
        publish_reports.append(bus.publish(batch))
    elapsed_s = time.perf_counter() - started
    bus.close()

    trace = DebugTraceReader().read(trace_path)
    replay_sink = TestDebugSink()
    replay = replay_debug_trace(trace, replay_sink)
    viewer = build_portable_viewer(trace, viewer_path, title="UniRoboSim M4 Debug Acceptance")
    svg = render_trace_svg(trace, svg_path, sequence=60, environment_indices=(0,))

    clock = _Clock()
    stress_trace_sink = TraceDebugSink(
        budget_trace_path,
        run_id="m4-budget-acceptance",
        metadata=FrozenMap({"seed": 42, "attempts": 100}),
    )
    stress_bus = DebugBus(
        (stress_trace_sink, TestDebugSink()),
        budget=DebugBudget(max_events_per_second=10, max_payload_bytes_per_second=64 * 1024 * 1024),
        clock=clock,
    )
    stress_reports = [
        stress_bus.publish(DebugBatch((_primitive(DebugPrimitiveKind.POINT_SET, index),))) for index in range(100)
    ]
    stress_bus.close()
    budget_trace = DebugTraceReader().read(budget_trace_path)

    payload = {
        "checks": {
            "all_six_primitives_active": len(memory_sink.primitives) == 0
            and len(replay_sink.primitives) == len(DebugPrimitiveKind),
            "all_publish_events_accepted": all(
                item.accepted_count == len(DebugPrimitiveKind) for item in publish_reports
            ),
            "trace_replay_active_match": replay.final_active_count == len(DebugPrimitiveKind),
            "portable_viewer_self_contained": "<script src=" not in viewer_path.read_text(encoding="utf-8"),
            "rate_limit_dropped_90": sum(item.dropped_count for item in stress_reports) == 90,
            "rate_limit_reports_persisted": budget_trace.manifest.report_count == 100
            and budget_trace.manifest.accepted_count == 10
            and budget_trace.manifest.dropped_count == 90
            and budget_trace.manifest.drop_reasons.to_dict() == {"event_rate": 90},
        },
        "trace": {
            "events": trace.manifest.event_count,
            "primitives": trace.manifest.primitive_count,
            "active": trace.manifest.active_count,
            "layers": trace.manifest.layers,
            "groups": trace.manifest.groups,
        },
        "performance": {
            "publish_events": len(publish_reports),
            "primitives_per_event": len(DebugPrimitiveKind),
            "total_seconds": elapsed_s,
            "mean_publish_ms": elapsed_s * 1000.0 / len(publish_reports),
            "stress_accepted": sum(item.accepted_count for item in stress_reports),
            "stress_dropped": sum(item.dropped_count for item in stress_reports),
            "stress_report_count": budget_trace.manifest.report_count,
        },
        "artifacts": {
            "trace": {"path": trace_path.name, "sha256": _sha256(trace_path)},
            "budget_trace": {
                "path": budget_trace_path.name,
                "sha256": _sha256(budget_trace_path),
            },
            "viewer": {"path": viewer_path.name, "sha256": viewer.sha256, "frames": viewer.frame_count},
            "svg": {"path": svg_path.name, "sha256": svg.sha256, "primitives": svg.primitive_count},
        },
    }
    if not all(payload["checks"].values()):
        raise RuntimeError(f"M4 portable acceptance failed: {payload['checks']}")
    report_path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(payload, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
