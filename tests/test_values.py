from __future__ import annotations

import gc
import math
import tracemalloc
import unittest
import weakref
from collections.abc import Iterator, Mapping
from unittest import mock

import unirobosim.api.frozen as frozen_contract
from unirobosim import (
    ArrayValue,
    EntityHandle,
    EntityKind,
    EntityPath,
    FrozenMap,
    Pose,
    Tick,
    UniRoboSimError,
    ValidationError,
    freeze_json,
    thaw_json,
)


class EntityPathTests(unittest.TestCase):
    def test_path_navigation(self) -> None:
        path = EntityPath("/env_0/robot.arm")
        self.assertEqual(path.name, "robot.arm")
        self.assertEqual(path.parent, EntityPath("/env_0"))
        self.assertEqual(path.child("gripper-1"), EntityPath("/env_0/robot.arm/gripper-1"))
        self.assertIsNone(EntityPath("/robot").parent)

    def test_invalid_paths_are_rejected(self) -> None:
        for value in ("", "/", "robot", "/robot/", "//robot", "/robot//link", "/./robot", "/../robot", "/robot link"):
            with self.subTest(value=value), self.assertRaises(ValidationError):
                EntityPath(value)

    def test_invalid_child_is_rejected(self) -> None:
        for child in ("", "a/b", ".", "..", "bad child"):
            with self.subTest(child=child), self.assertRaises(ValidationError):
                EntityPath("/robot").child(child)


class PoseTests(unittest.TestCase):
    def test_numeric_input_is_canonicalized(self) -> None:
        pose = Pose((1, 2, 3), (0, 0, 0, 1))
        self.assertEqual(pose.position, (1.0, 2.0, 3.0))
        self.assertEqual(pose.orientation_xyzw, (0.0, 0.0, 0.0, 1.0))

    def test_non_unit_quaternion_is_not_silently_normalized(self) -> None:
        with self.assertRaisesRegex(ValidationError, "implicit normalization"):
            Pose(orientation_xyzw=(0.0, 0.0, 0.0, 2.0))

    def test_non_finite_pose_is_rejected(self) -> None:
        with self.assertRaises(ValidationError):
            Pose(position=(math.inf, 0.0, 0.0))
        with self.assertRaises(ValidationError):
            Pose(orientation_xyzw=(0.0, 0.0, math.nan, 1.0))


class FrozenJsonTests(unittest.TestCase):
    def test_recursive_freeze_and_thaw(self) -> None:
        frozen = FrozenMap({"b": [1, {"x": True}], "a": "value"})
        self.assertEqual(tuple(frozen), ("a", "b"))
        self.assertEqual(thaw_json(frozen), {"a": "value", "b": [1, {"x": True}]})
        self.assertEqual(hash(frozen), hash(FrozenMap({"a": "value", "b": [1, {"x": True}]})))
        with self.assertRaises(TypeError):
            frozen["new"] = 1  # type: ignore[index]

    def test_invalid_json_values_are_rejected(self) -> None:
        for value in (math.inf, math.nan, object(), {1: "bad-key"}, {1, 2}):
            with self.subTest(value=repr(value)), self.assertRaises(ValidationError):
                freeze_json(value)

    def test_invalid_frozen_map_input_is_structured(self) -> None:
        with self.assertRaises(ValidationError):
            FrozenMap(1)  # type: ignore[arg-type]
        with self.assertRaises(ValidationError):
            FrozenMap([object()])  # type: ignore[list-item]
        with self.assertRaises(ValidationError):
            FrozenMap([("key", 1, 2)])  # type: ignore[list-item]

    def test_scalar_and_builtin_container_subclasses_are_detached_before_use(self) -> None:
        class HostileText(str):
            calls = 0
            armed = False

            def __hash__(self):
                if not type(self).armed:
                    return str.__hash__(self)
                type(self).calls += 1
                raise KeyboardInterrupt("subclass hash must not run")

            def __eq__(self, other):
                if not type(self).armed:
                    return str.__eq__(self, other)
                type(self).calls += 1
                raise KeyboardInterrupt("subclass equality must not run")

            def __str__(self):
                type(self).calls += 1
                raise KeyboardInterrupt("subclass string conversion must not run")

        class HostileInt(int):
            calls = 0

            def __int__(self):
                type(self).calls += 1
                raise KeyboardInterrupt("subclass integer conversion must not run")

        class HostileFloat(float):
            calls = 0

            def __float__(self):
                type(self).calls += 1
                raise KeyboardInterrupt("subclass float conversion must not run")

        class HostileList(list):
            calls = 0

            def __iter__(self):
                type(self).calls += 1
                raise KeyboardInterrupt("subclass list iteration must not run")

        class HostileDict(dict):
            calls = 0

            def items(self):
                type(self).calls += 1
                raise KeyboardInterrupt("subclass mapping iteration must not run")

        key = HostileText("key")
        source = HostileDict(
            {
                key: HostileList(
                    [
                        HostileText("value"),
                        HostileInt(7),
                        HostileFloat(2.5),
                    ]
                )
            }
        )
        HostileText.armed = True
        frozen = FrozenMap(source)
        self.assertEqual(frozen[HostileText("key")], ("value", 7, 2.5))
        self.assertIs(type(next(iter(frozen))), str)
        self.assertEqual(
            (HostileText.calls, HostileInt.calls, HostileFloat.calls, HostileList.calls, HostileDict.calls),
            (0, 0, 0, 0, 0),
        )
        self.assertTrue(all(type(value) in {str, int, float} for value in frozen["key"]))

    def test_type_classification_never_consults_forged_instance_or_metaclass_fields(self) -> None:
        class HostileNameMeta(type):
            name_calls = 0
            mro_calls = 0

            @property
            def __name__(cls):
                del cls
                HostileNameMeta.name_calls += 1
                raise KeyboardInterrupt("metaclass name must not run")

            @property
            def __mro__(cls):
                del cls
                HostileNameMeta.mro_calls += 1
                raise KeyboardInterrupt("metaclass mro must not run")

        class HostileUnknown(metaclass=HostileNameMeta):
            class_calls = 0

            @property
            def __class__(self):
                type(self).class_calls += 1
                raise KeyboardInterrupt("instance class must not run")

        with self.assertRaises(ValidationError) as caught:
            freeze_json(HostileUnknown())
        self.assertEqual(caught.exception.details, {"value_type": "unsupported"})
        self.assertEqual(
            (HostileUnknown.class_calls, HostileNameMeta.name_calls, HostileNameMeta.mro_calls),
            (0, 0, 0),
        )

    def test_actual_mapping_subclass_remains_supported_without_virtual_isinstance(self) -> None:
        class CustomMapping(Mapping[str, object]):
            def __init__(self) -> None:
                self._values = {"payload": [1, 2, 3]}

            def __getitem__(self, key: str) -> object:
                return self._values[key]

            def __iter__(self) -> Iterator[str]:
                return iter(self._values)

            def __len__(self) -> int:
                return len(self._values)

        frozen = FrozenMap(CustomMapping())
        self.assertEqual(frozen, FrozenMap({"payload": (1, 2, 3)}))

    def test_streaming_container_budget_counts_duplicates_and_accepts_exact_boundary(self) -> None:
        class RepeatingMapping(Mapping[str, object]):
            def __init__(self, repetitions: int) -> None:
                self.repetitions = repetitions
                self.yielded = 0

            def __getitem__(self, key: str) -> object:
                if key != "key":
                    raise KeyError(key)
                return 7

            def __iter__(self) -> Iterator[str]:
                return iter(("key",))

            def __len__(self) -> int:
                return 1

            def items(self):
                for _ in range(self.repetitions):
                    self.yielded += 1
                    yield "key", 7

        boundary = RepeatingMapping(100_000)
        frozen = FrozenMap(boundary)
        self.assertEqual((boundary.yielded, len(frozen), frozen["key"]), (100_000, 1, 7))

        oversized = RepeatingMapping(200_000)
        tracemalloc.start()
        try:
            with self.assertRaises(ValidationError) as caught:
                FrozenMap(oversized)
            _, peak = tracemalloc.get_traced_memory()
        finally:
            tracemalloc.stop()
        self.assertEqual(oversized.yielded, 100_001)
        self.assertLess(peak, 512 * 1024)
        self.assertEqual(caught.exception.message, "JSON container item budget exceeded")
        self.assertIsNone(caught.exception.__cause__)
        self.assertIsNone(caught.exception.__context__)

    def test_nested_frozen_maps_share_the_same_occurrence_budget(self) -> None:
        inner = FrozenMap((("payload", [None] * 99_999),))
        self.assertEqual(FrozenMap(inner), inner)
        self.assertEqual(freeze_json(inner), inner)

        with self.assertRaises(ValidationError, msg="a nested map adds one pair occurrence") as nested:
            FrozenMap({"nested": inner})
        self.assertEqual(nested.exception.message, "JSON container item budget exceeded")

        with self.assertRaises(ValidationError, msg="aliased maps count once per occurrence") as aliased:
            freeze_json([inner] * 100_000)
        self.assertEqual(aliased.exception.message, "JSON container item budget exceeded")

    def test_container_hook_failures_are_fixed_typed_and_do_not_retain_source_graph(self) -> None:
        class Sidecar:
            pass

        class InterruptingMapping(Mapping[str, object]):
            def __init__(self, sidecar: object) -> None:
                self.sidecar = sidecar

            def __getitem__(self, key: str) -> object:
                raise AssertionError(key)

            def __iter__(self) -> Iterator[str]:
                raise AssertionError("mapping iteration must not run")

            def __len__(self) -> int:
                return 1

            def items(self):
                raise KeyboardInterrupt("private input", self.sidecar)

        sidecar = Sidecar()
        reference = weakref.ref(sidecar)
        source = InterruptingMapping(sidecar)
        failure: ValidationError | None = None
        try:
            FrozenMap(source)
        except ValidationError as caught:
            failure = caught
        self.assertIsNotNone(failure)
        assert failure is not None
        self.assertEqual(
            failure.message,
            "FrozenMap input must contain JSON-compatible string-keyed pairs",
        )
        self.assertIsNone(failure.__cause__)
        self.assertIsNone(failure.__context__)
        traceback = failure.__traceback__
        while traceback is not None:
            if traceback.tb_frame.f_code.co_filename.endswith("frozen.py"):
                self.assertFalse(any(value is source for value in traceback.tb_frame.f_locals.values()))
            traceback = traceback.tb_next
        failure.__traceback__ = None
        source = None
        sidecar = None
        gc.collect()
        self.assertIsNone(reference())

    def test_iterator_and_pair_baseexceptions_share_the_fixed_failure_boundary(self) -> None:
        class FailingIterator:
            def __iter__(self):
                return self

            def __next__(self):
                raise SystemExit("private iterator payload")

        class Pair:
            def __iter__(self):
                raise GeneratorExit("private pair payload")

        for source in (FailingIterator(), (Pair(),)):
            with self.subTest(source_type=type(source)), self.assertRaises(ValidationError) as caught:
                FrozenMap(source)  # type: ignore[arg-type]
            self.assertEqual(
                caught.exception.message,
                "FrozenMap input must contain JSON-compatible string-keyed pairs",
            )
            self.assertIsNone(caught.exception.__cause__)
            self.assertIsNone(caught.exception.__context__)

    def test_streaming_detachment_shape_and_defensive_boundary_matrix(self) -> None:
        class CustomPair:
            def __iter__(self):
                return iter(("custom", [1, 2]))

        self.assertEqual(FrozenMap((["list", 1],))["list"], 1)
        self.assertEqual(FrozenMap(("xy",))["x"], "y")
        self.assertEqual(FrozenMap((CustomPair(),))["custom"], (1, 2))
        self.assertEqual(FrozenMap(FrozenMap({"copy": True})), FrozenMap({"copy": True}))
        for value in ((["too", "many", "items"],), (CustomPair(), object())):
            with self.subTest(value=value), self.assertRaises(ValidationError):
                FrozenMap(value)  # type: ignore[arg-type]
        with self.assertRaises(ValidationError, msg="nested invalid values use the shared boundary"):
            FrozenMap({"invalid": object()})
        with self.assertRaises(ValidationError, msg="oversized arrays share the graph budget"):
            freeze_json([None] * 100_001)
        with self.assertRaises(ValidationError, msg="recursive graphs fail through a typed boundary"):
            recursive: list[object] = []
            recursive.append(recursive)
            freeze_json(recursive)
        frozen = FrozenMap({"key": 1})
        with self.assertRaises(ValidationError):
            frozen[object()]  # type: ignore[index]

        deep_type: type = object
        for index in range(257):
            deep_type = type(f"Deep{index}", (deep_type,), {})
        with self.assertRaises(ValidationError):
            freeze_json(deep_type())

        budget = frozen_contract._FreezeBudget()
        self.assertFalse(budget.consume(-1))
        self.assertFalse(budget.consume("1"))  # type: ignore[arg-type]
        frozen_contract._scrub_failure(object())  # type: ignore[arg-type]

        with (
            mock.patch.object(
                frozen_contract,
                "_freeze_value",
                side_effect=KeyboardInterrupt("private value boundary"),
            ),
            self.assertRaises(ValidationError) as value_failure,
        ):
            freeze_json("value")
        self.assertEqual(value_failure.exception.message, "JSON value could not be detached")
        self.assertIsNone(value_failure.exception.__context__)

        with (
            mock.patch.object(
                frozen_contract,
                "_freeze_mapping_items",
                side_effect=(SystemExit("private mapping boundary"), ()),
            ),
            self.assertRaises(ValidationError) as mapping_failure,
        ):
            FrozenMap({"key": "value"})
        self.assertEqual(
            mapping_failure.exception.message,
            "FrozenMap input must contain JSON-compatible string-keyed pairs",
        )
        self.assertIsNone(mapping_failure.exception.__context__)


class ArrayValueTests(unittest.TestCase):
    def test_rows_round_trip(self) -> None:
        value = ArrayValue.from_rows(((1, 2), (3.5, 4)), dtype="float32")
        self.assertEqual(value.shape, (2, 2))
        self.assertEqual(value.values, (1.0, 2.0, 3.5, 4.0))
        self.assertEqual(value.rows(), ((1.0, 2.0), (3.5, 4.0)))

    def test_shape_dtype_and_finite_values_are_strict(self) -> None:
        invalid = (
            {"shape": (), "values": ()},
            {"shape": (1, 2), "values": (1.0,)},
            {"shape": (1,), "values": (math.inf,)},
            {"shape": (1,), "values": (1,), "dtype": "unknown"},
            {"shape": (1,), "values": (1.5,), "dtype": "int64"},
            {"shape": (1,), "values": (1.0,), "device": "cuda:0"},
        )
        for kwargs in invalid:
            with self.subTest(kwargs=kwargs), self.assertRaises(ValidationError):
                ArrayValue(**kwargs)

    def test_ragged_rows_are_rejected(self) -> None:
        with self.assertRaises(ValidationError):
            ArrayValue.from_rows(((1.0,), (2.0, 3.0)))

    def test_non_iterable_array_inputs_are_structured(self) -> None:
        with self.assertRaises(ValidationError):
            ArrayValue(shape=None, values=(1.0,))  # type: ignore[arg-type]
        with self.assertRaises(ValidationError):
            ArrayValue.from_rows(None)  # type: ignore[arg-type]
        with self.assertRaises(ValidationError):
            ArrayValue(shape=(1,), values=(1.0,), ownership="borrowed")  # type: ignore[arg-type]


class IdentityValueTests(unittest.TestCase):
    def test_handle_and_tick_validate_identity(self) -> None:
        handle = EntityHandle("provider", "session", "world", 1, EntityPath("/robot"), EntityKind.ARTICULATION, "x")
        self.assertEqual(handle.generation, 1)
        self.assertEqual(Tick(1, 0).sim_time_seconds, 0.0)
        with self.assertRaises(ValidationError):
            EntityHandle("", "session", "world", 1, EntityPath("/robot"), EntityKind.ARTICULATION, "x")
        with self.assertRaises(ValidationError):
            EntityHandle("provider", "session", "world", 0, EntityPath("/robot"), EntityKind.ARTICULATION, "x")
        for step, time in ((True, 0.0), (-1, 0.0), (0, math.inf), (0, -1.0)):
            with self.subTest(step=step, time=time), self.assertRaises(ValidationError):
                Tick(step, time)


class ErrorTests(unittest.TestCase):
    def test_error_is_machine_readable_and_keeps_cause(self) -> None:
        cause = RuntimeError("native failure")
        error = UniRoboSimError(
            "wrapped",
            operation="test.operation",
            backend_id="backend.test",
            world_id="world",
            entity_path="/robot",
            details={"nested": [1, 2]},
            cause=cause,
        )
        self.assertIs(error.__cause__, cause)
        self.assertEqual(
            error.to_dict(),
            {
                "code": "unirobosim.error",
                "message": "wrapped",
                "operation": "test.operation",
                "backend_id": "backend.test",
                "world_id": "world",
                "entity_path": "/robot",
                "details": {"nested": [1, 2]},
            },
        )


if __name__ == "__main__":
    unittest.main()
