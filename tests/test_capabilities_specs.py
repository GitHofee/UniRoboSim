from __future__ import annotations

import math
import unittest

from unirobosim import (
    CapabilityDeclaration,
    CapabilityId,
    CapabilityRequirement,
    CapabilitySet,
    EntityKind,
    EntityPath,
    EntitySpec,
    EnvironmentSpec,
    FrozenMap,
    PhysicsSpec,
    ValidationError,
    WorldSpec,
)


class CapabilityTests(unittest.TestCase):
    def setUp(self) -> None:
        self.identifier = CapabilityId("profile.core-robotics@1")
        self.capabilities = CapabilitySet(
            (CapabilityDeclaration(self.identifier, FrozenMap({"layout": "batch-first", "nested": {"v": 1}})),)
        )

    def test_identifier_parsing(self) -> None:
        self.assertEqual(self.identifier.name, "profile.core-robotics")
        self.assertEqual(self.identifier.major, 1)
        for value in ("", "Core@1", "core", "core@0", "core@1.0", ".core@1"):
            with self.subTest(value=value), self.assertRaises(ValidationError):
                CapabilityId(value)

    def test_exact_property_match(self) -> None:
        report = self.capabilities.negotiate(
            (
                CapabilityRequirement(
                    self.identifier, constraints=FrozenMap({"layout": "batch-first", "nested": {"v": 1}})
                ),
            )
        )
        self.assertTrue(report.accepted)
        self.assertEqual(report.matched, (self.identifier,))

    def test_required_missing_and_mismatch_reject(self) -> None:
        missing = self.capabilities.negotiate((CapabilityRequirement(CapabilityId("sensor.camera@1")),))
        mismatch = self.capabilities.negotiate(
            (CapabilityRequirement(self.identifier, constraints=FrozenMap({"layout": "dof-first"})),)
        )
        self.assertFalse(missing.accepted)
        self.assertEqual(missing.required_issues[0].reason, "missing")
        self.assertFalse(mismatch.accepted)
        self.assertEqual(mismatch.required_issues[0].reason, "property_mismatch")

    def test_optional_missing_is_reported_but_accepted(self) -> None:
        report = self.capabilities.negotiate((CapabilityRequirement(CapabilityId("sensor.camera@1"), required=False),))
        self.assertTrue(report.accepted)
        self.assertEqual(len(report.optional_issues), 1)

    def test_duplicate_declarations_and_requirements_are_rejected(self) -> None:
        declaration = CapabilityDeclaration(self.identifier)
        with self.assertRaises(ValidationError):
            CapabilitySet((declaration, declaration))
        with self.assertRaises(ValidationError):
            self.capabilities.negotiate(
                (CapabilityRequirement(self.identifier), CapabilityRequirement(self.identifier))
            )

    def test_capability_digest_is_order_independent(self) -> None:
        other = CapabilityDeclaration(CapabilityId("state.articulation@1"))
        first = CapabilitySet((CapabilityDeclaration(self.identifier), other))
        second = CapabilitySet((other, CapabilityDeclaration(self.identifier)))
        self.assertEqual(first.digest, second.digest)
        self.assertEqual(first, second)
        self.assertEqual(hash(first), hash(second))

    def test_malformed_capability_collections_are_structured(self) -> None:
        with self.assertRaises(ValidationError):
            CapabilitySet((object(),))  # type: ignore[arg-type]
        with self.assertRaises(ValidationError):
            CapabilitySet(None)  # type: ignore[arg-type]
        with self.assertRaises(ValidationError):
            self.capabilities.negotiate(None)  # type: ignore[arg-type]


class WorldSpecTests(unittest.TestCase):
    def _entities(self):
        robot = EntitySpec(EntityPath("/robot"), EntityKind.ARTICULATION, joint_names=("a", "b"))
        ground = EntitySpec(EntityPath("/ground"), EntityKind.RIGID_BODY)
        return robot, ground

    def test_default_joint_positions_and_canonical_entity_order(self) -> None:
        robot, ground = self._entities()
        spec = WorldSpec(world_id="world-1", entities=(robot, ground))
        self.assertEqual(spec.entities[0].path, EntityPath("/ground"))
        self.assertEqual(spec.entities[1].initial_joint_positions, (0.0, 0.0))
        self.assertEqual(spec.requirements[0].capability, CapabilityId("profile.core-robotics@1"))
        self.assertIn(CapabilityId("state.rigid_body@1"), {item.capability for item in spec.requirements})

    def test_digest_is_stable_across_mapping_and_entity_order(self) -> None:
        robot, ground = self._entities()
        first = WorldSpec(world_id="world", entities=(robot, ground), metadata=FrozenMap({"b": 2, "a": {"x": 1}}))
        second = WorldSpec(world_id="world", entities=(ground, robot), metadata=FrozenMap({"a": {"x": 1}, "b": 2}))
        changed = WorldSpec(world_id="world", entities=(ground, robot), metadata=FrozenMap({"a": {"x": 9}, "b": 2}))
        self.assertEqual(first.canonical_json, second.canonical_json)
        self.assertEqual(first.digest, second.digest)
        self.assertNotEqual(first.digest, changed.digest)
        self.assertEqual(len(first.digest), 64)

    def test_duplicate_paths_are_rejected(self) -> None:
        robot, _ = self._entities()
        with self.assertRaises(ValidationError):
            WorldSpec(world_id="world", entities=(robot, robot))

    def test_entity_invariants_are_rejected(self) -> None:
        with self.assertRaises(ValidationError):
            EntitySpec(EntityPath("/rigid"), EntityKind.RIGID_BODY, joint_names=("bad",))
        with self.assertRaises(ValidationError):
            EntitySpec(EntityPath("/robot"), EntityKind.ARTICULATION)
        with self.assertRaises(ValidationError):
            EntitySpec(EntityPath("/robot"), EntityKind.ARTICULATION, joint_names=("a", "a"))
        with self.assertRaises(ValidationError):
            EntitySpec(
                EntityPath("/robot"),
                EntityKind.ARTICULATION,
                joint_names=("a", "b"),
                initial_joint_positions=(0.0,),
            )

    def test_world_and_physics_invariants_are_rejected(self) -> None:
        robot, _ = self._entities()
        for world_id in ("", "bad world", "/world"):
            with self.subTest(world_id=world_id), self.assertRaises(ValidationError):
                WorldSpec(world_id=world_id, entities=(robot,))
        with self.assertRaises(ValidationError):
            WorldSpec(world_id="world", entities=())
        with self.assertRaises(ValidationError):
            WorldSpec(world_id="world", entities=(robot,), schema_version="future")
        for time_step in (0.0, -1.0, math.inf):
            with self.subTest(time_step=time_step), self.assertRaises(ValidationError):
                PhysicsSpec(time_step_seconds=time_step)
        with self.assertRaises(ValidationError):
            EnvironmentSpec(0)

    def test_core_profile_is_mandatory_and_malformed_collections_are_structured(self) -> None:
        robot, _ = self._entities()
        optional_core = CapabilityRequirement(CapabilityId("profile.core-robotics@1"), required=False)
        with self.assertRaises(ValidationError):
            WorldSpec(world_id="world", entities=(robot,), requirements=(optional_core,))
        extra = CapabilityRequirement(CapabilityId("sensor.camera@1"), required=False)
        normalized = WorldSpec(world_id="world", entities=(robot,), requirements=(extra,))
        self.assertEqual(
            {item.capability for item in normalized.requirements},
            {CapabilityId("profile.core-robotics@1"), CapabilityId("sensor.camera@1")},
        )
        with self.assertRaises(ValidationError):
            WorldSpec(world_id="world", entities=(object(),))  # type: ignore[arg-type]
        with self.assertRaises(ValidationError):
            WorldSpec(world_id="world", entities=(robot,), requirements=(object(),))  # type: ignore[arg-type]


if __name__ == "__main__":
    unittest.main()
