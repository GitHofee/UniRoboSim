from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from unirobosim import (
    RIGID_CONTACT_WORLD_SCHEMA_VERSION,
    ArrayValue,
    CameraModality,
    CameraSpec,
    CommandError,
    DebugBatch,
    DebugBus,
    DebugPrimitive,
    DebugPrimitiveKind,
    DebugPublishReport,
    EntityKind,
    EntityPath,
    EntitySpec,
    EnvironmentSpec,
    FrozenMap,
    LifecycleError,
    NativeWorldDebugSink,
    SensorChannel,
    SensorSample,
    TestDebugSink,
    TraceDebugSink,
    ValidationError,
    WorldSpec,
)
from unirobosim.testing import FakeProvider


def camera_world(*, modalities: tuple[CameraModality, ...] = (CameraModality.RGB, CameraModality.DEPTH)) -> WorldSpec:
    return WorldSpec(
        "camera-world",
        (
            EntitySpec(
                EntityPath("/camera"),
                EntityKind.CAMERA_SENSOR,
                camera=CameraSpec(width_px=4, height_px=3, modalities=modalities),
            ),
        ),
        environments=EnvironmentSpec(2),
    )


def point_primitive(
    primitive_id: str = "origin",
    *,
    layer: str = "planning",
    x: float = 0.0,
    lifetime_steps: int = 0,
) -> DebugPrimitive:
    return DebugPrimitive(
        primitive_id=primitive_id,
        layer=layer,
        kind=DebugPrimitiveKind.POINT_SET,
        geometry_m=ArrayValue.from_nested([[[x, 0.0, 0.0]], [[x + 1.0, 0.0, 0.0]]]),
        environment_indices=(0, 1),
        color_rgba=(1.0, 0.25, 0.0, 0.75),
        size=3.0,
        lifetime_steps=lifetime_steps,
    )


class CameraContractTests(unittest.TestCase):
    def test_uint8_is_owned_and_range_checked(self) -> None:
        value = ArrayValue((1, 1, 1, 3), (0, 127, 255), dtype="uint8")
        self.assertEqual(value.values, (0, 127, 255))
        with self.assertRaises(ValidationError):
            ArrayValue((1,), (256,), dtype="uint8")
        with self.assertRaises(ValidationError):
            ArrayValue((1,), (-1,), dtype="uint8")

    def test_camera_spec_and_schema_requirements_are_strict(self) -> None:
        spec = camera_world()
        capability_ids = {item.capability.value for item in spec.requirements}
        self.assertTrue({"sensor.camera@1", "sensor.camera.rgb@1", "sensor.camera.depth@1"} <= capability_ids)
        with self.assertRaises(ValidationError):
            CameraSpec(width_px=0)
        with self.assertRaises(ValidationError):
            CameraSpec(modalities=(CameraModality.RGB, CameraModality.RGB))
        with self.assertRaises(ValidationError):
            CameraSpec(near_plane_m=1.0, far_plane_m=1.0)
        with self.assertRaises(ValidationError):
            WorldSpec(
                "old-camera",
                spec.entities,
                schema_version=RIGID_CONTACT_WORLD_SCHEMA_VERSION,
            )

    def test_camera_entity_rejects_mixed_specs(self) -> None:
        with self.assertRaises(ValidationError):
            EntitySpec(EntityPath("/camera"), EntityKind.CAMERA_SENSOR)
        with self.assertRaises(ValidationError):
            EntitySpec(
                EntityPath("/rigid"),
                EntityKind.RIGID_BODY,
                camera=CameraSpec(),
            )

    def test_fake_rgb_depth_sample_is_ordered_and_does_not_step(self) -> None:
        session = FakeProvider().open()
        world = session.build(camera_world())
        handle = world.resolve(EntityPath("/camera"))
        before = world.tick
        sample = world.read_sensor(handle)
        self.assertIsInstance(sample, SensorSample)
        self.assertEqual(sample.tick, before)
        self.assertEqual(world.tick, before)
        self.assertEqual(
            tuple(channel.modality for channel in sample.channels),
            (CameraModality.RGB, CameraModality.DEPTH),
        )
        rgb = sample.channel(CameraModality.RGB)
        depth = sample.channel(CameraModality.DEPTH)
        self.assertEqual((rgb.shape, rgb.dtype), ((2, 3, 4, 3), "uint8"))
        self.assertEqual((depth.shape, depth.dtype), ((2, 3, 4), "float32"))
        self.assertGreater(len(set(rgb.values)), 1)
        self.assertGreater(len(set(depth.values)), 1)
        with self.assertRaises(ValidationError):
            sample.channel("rgb")  # type: ignore[arg-type]

    def test_sensor_report_validation_rejects_wrong_shapes(self) -> None:
        session = FakeProvider().open()
        world = session.build(camera_world(modalities=(CameraModality.RGB,)))
        handle = world.resolve(EntityPath("/camera"))
        with self.assertRaises(ValidationError):
            SensorChannel(CameraModality.RGB, ArrayValue((2, 3, 4), (0,) * 24, dtype="uint8"))
        with self.assertRaises(ValidationError):
            SensorSample(handle, (), world.tick)
        world.close()
        mixed_world = session.build(
            WorldSpec(
                "mixed-sensor-world",
                (
                    EntitySpec(EntityPath("/rigid"), EntityKind.RIGID_BODY),
                    EntitySpec(
                        EntityPath("/camera"),
                        EntityKind.CAMERA_SENSOR,
                        camera=CameraSpec(width_px=2, height_px=2),
                    ),
                ),
            )
        )
        with self.assertRaises(CommandError) as caught:
            mixed_world.read_sensor(mixed_world.resolve(EntityPath("/rigid")))
        self.assertEqual(caught.exception.operation, "world.read_sensor")


class DebugContractTests(unittest.TestCase):
    def test_primitive_validation_edges_and_serialization(self) -> None:
        primitive = point_primitive()
        self.assertEqual(primitive.to_dict()["id"], "origin")
        invalid_kwargs = (
            {"primitive_id": "bad name"},
            {"layer": ""},
            {"kind": "point_set"},
            {"geometry_m": ArrayValue((1, 1, 3), (0, 0, 0), dtype="int64")},
            {"environment_indices": ()},
            {"environment_indices": (0, 0)},
            {"environment_indices": (0, True)},
            {"color_rgba": (1.0, 0.0, 0.0, 2.0)},
            {"color_rgba": None},
            {"size": True},
            {"size": 0.0},
            {"lifetime_steps": True},
            {"lifetime_steps": -1},
            {"metadata": {}},
        )
        base = {
            "primitive_id": "valid",
            "layer": "layer",
            "kind": DebugPrimitiveKind.POINT_SET,
            "geometry_m": ArrayValue.from_nested([[[0.0, 0.0, 0.0]], [[1.0, 0.0, 0.0]]]),
            "environment_indices": (0, 1),
            "metadata": FrozenMap(),
        }
        for override in invalid_kwargs:
            with self.subTest(override=override), self.assertRaises(ValidationError):
                DebugPrimitive(**(base | override))  # type: ignore[arg-type]

    def test_batch_and_report_validation_edges(self) -> None:
        primitive = point_primitive()
        for value in (None, (), (object(),), (primitive, primitive)):
            with self.subTest(value=value), self.assertRaises(ValidationError):
                DebugBatch(value)  # type: ignore[arg-type]
        for counts in ((-1, 0, 0), (0, True, 0)):
            with self.subTest(counts=counts), self.assertRaises(ValidationError):
                DebugPublishReport(*counts)  # type: ignore[arg-type]
        with self.assertRaises(ValidationError):
            DebugPublishReport(0, 0, 0, None)  # type: ignore[arg-type]
        with self.assertRaises(ValidationError):
            DebugPublishReport(0, 0, 0, ("",))

    def test_bus_and_reference_sink_lifecycle_edges(self) -> None:
        for sinks, budget in ((None, 1), ((), 1), ((object(),), 1), ((TestDebugSink(),), 0)):
            with self.subTest(sinks=sinks, budget=budget), self.assertRaises(ValidationError):
                DebugBus(sinks, max_active_primitives=budget)  # type: ignore[arg-type]

        class AlwaysBrokenSink:
            def publish(self, batch: DebugBatch) -> None:
                raise RuntimeError("publish")

            def clear(self, *, layer: str | None = None, primitive_id: str | None = None) -> int:
                raise RuntimeError("clear")

            def close(self) -> None:
                raise RuntimeError("close")

        bus = DebugBus((AlwaysBrokenSink(),))
        with self.assertRaises(ValidationError):
            bus.publish(object())  # type: ignore[arg-type]
        report = bus.publish(DebugBatch((point_primitive(),)))
        self.assertEqual((report.accepted_count, report.dropped_count, bus.active_count), (0, 1, 0))
        with self.assertRaises(ValidationError):
            bus.clear(layer="bad layer")
        with self.assertRaises(ValidationError):
            bus.advance(0)
        bus.close()
        bus.close()
        with self.assertRaises(LifecycleError):
            bus.clear()
        with self.assertRaises(LifecycleError):
            bus.advance()

        sink = TestDebugSink()
        with self.assertRaises(ValidationError):
            sink.publish(object())  # type: ignore[arg-type]
        sink.close()
        with self.assertRaises(LifecycleError):
            sink.clear()

    def test_trace_and_native_sink_validation_edges(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            sink = TraceDebugSink(Path(directory) / "trace.jsonl")
            with self.assertRaises(ValidationError):
                sink.publish(object())  # type: ignore[arg-type]
            sink.close()
            sink.close()
            with self.assertRaises(LifecycleError):
                sink.clear()
        with self.assertRaises(ValidationError):
            NativeWorldDebugSink(object())
        session = FakeProvider().open()
        world = session.build(camera_world())
        native = NativeWorldDebugSink(world)
        native.close()
        with self.assertRaises(LifecycleError):
            native.clear()

    def test_primitive_shapes_and_environment_batches_are_strict(self) -> None:
        primitive = point_primitive()
        self.assertEqual(primitive.key, ("planning", "origin"))
        lines = DebugPrimitive(
            "axes",
            "frames",
            DebugPrimitiveKind.LINE_LIST,
            ArrayValue.from_nested([[[[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]]]]),
            (0,),
        )
        self.assertEqual(lines.geometry_m.shape, (1, 1, 2, 3))
        with self.assertRaises(ValidationError):
            DebugPrimitive(
                "bad",
                "frames",
                DebugPrimitiveKind.LINE_LIST,
                ArrayValue.from_nested([[[0.0, 0.0, 0.0]]]),
                (0,),
            )
        with self.assertRaises(ValidationError):
            DebugPrimitive(
                "bad",
                "frames",
                DebugPrimitiveKind.POINT_SET,
                ArrayValue.from_nested([[[0.0, 0.0, 0.0]]]),
                (0, 1),
            )

    def test_bus_replaces_stable_ids_enforces_budget_and_lifetime(self) -> None:
        sink = TestDebugSink()
        bus = DebugBus((sink,), max_active_primitives=2)
        first = bus.publish(DebugBatch((point_primitive(x=0.0),)))
        replaced = bus.publish(DebugBatch((point_primitive(x=2.0, lifetime_steps=2),)))
        self.assertEqual((first.accepted_count, replaced.active_count), (1, 1))
        self.assertEqual(sink.primitives[0].geometry_m.values[0], 2.0)
        bus.publish(DebugBatch((point_primitive("second", layer="other"),)))
        dropped = bus.publish(DebugBatch((point_primitive("third"),)))
        self.assertEqual((dropped.accepted_count, dropped.dropped_count, dropped.active_count), (0, 1, 2))
        self.assertEqual(bus.advance(), 0)
        self.assertEqual(bus.advance(), 1)
        self.assertEqual(bus.active_count, 1)
        self.assertEqual(bus.clear(layer="other"), 1)
        bus.close()
        with self.assertRaises(LifecycleError):
            bus.publish(DebugBatch((point_primitive(),)))

    def test_sink_failures_are_reported_without_blocking_other_sinks(self) -> None:
        class BrokenSink:
            def publish(self, batch: DebugBatch) -> None:
                raise RuntimeError("broken")

            def clear(self, *, layer: str | None = None, primitive_id: str | None = None) -> int:
                raise RuntimeError("broken")

            def close(self) -> None:
                raise RuntimeError("broken")

        sink = TestDebugSink()
        bus = DebugBus((BrokenSink(), sink))
        report = bus.publish(DebugBatch((point_primitive(),)))
        self.assertEqual(report.accepted_count, 1)
        self.assertEqual(len(report.sink_failures), 1)
        self.assertEqual(len(sink.primitives), 1)
        bus.close()

    def test_trace_sink_writes_canonical_events_and_close_summary(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "debug.jsonl"
            sink = TraceDebugSink(path)
            bus = DebugBus((sink,))
            bus.publish(DebugBatch((point_primitive(),)))
            bus.clear(layer="planning")
            bus.close()
            lines = path.read_text(encoding="utf-8").splitlines()
            events = [json.loads(line) for line in lines]
            self.assertEqual([event["event"] for event in events], ["publish", "clear", "close"])
            self.assertEqual(events[-1]["events_before_close"], 2)
            self.assertEqual(lines[0], json.dumps(events[0], sort_keys=True, separators=(",", ":")))

    def test_native_world_sink_forwards_and_fake_world_expires(self) -> None:
        session = FakeProvider().open()
        world = session.build(camera_world())
        sink = NativeWorldDebugSink(world)
        sink.publish(DebugBatch((point_primitive(lifetime_steps=1),)))
        world.step()
        self.assertEqual(world.clear_debug(), 0)
        sink.publish(DebugBatch((point_primitive(),)))
        self.assertEqual(sink.clear(layer="planning", primitive_id="origin"), 1)
        sink.close()
        with self.assertRaises(LifecycleError):
            sink.publish(DebugBatch((point_primitive(),)))

    def test_fake_world_rejects_out_of_range_debug_environment(self) -> None:
        session = FakeProvider().open()
        world = session.build(camera_world())
        primitive = DebugPrimitive(
            "bad-env",
            "debug",
            DebugPrimitiveKind.POINT_SET,
            ArrayValue.from_nested([[[0.0, 0.0, 0.0]]]),
            (2,),
        )
        with self.assertRaises(ValidationError):
            world.publish_debug(DebugBatch((primitive,)))


if __name__ == "__main__":
    unittest.main()
