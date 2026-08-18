from __future__ import annotations

import math
import unittest

from unirobosim import (
    LEGACY_WORLD_SCHEMA_VERSION,
    SOFT_MATTER_WORLD_SCHEMA_VERSION,
    WORLD_SCHEMA_VERSION,
    ArrayValue,
    CapabilityId,
    DeformableBodySpec,
    DeformableTopology,
    EntityKind,
    EntityPath,
    EntitySpec,
    ParticleFluidSpec,
    ValidationError,
    WorldSpec,
)


def surface_body(**overrides: object) -> DeformableBodySpec:
    values: dict[str, object] = {
        "topology": DeformableTopology.SURFACE,
        "rest_positions_m": ArrayValue.from_rows(((0, 0, 1), (1, 0, 1), (0, 1, 1), (1, 1, 1))),
        "surface_triangles": ArrayValue.from_rows(((0, 1, 2), (1, 3, 2)), dtype="int64"),
        "kinematic_node_indices": (0, 1),
        "node_mass_kg": 0.5,
        "linear_damping_per_s": 0.25,
        "material_id": "cotton",
    }
    values.update(overrides)
    return DeformableBodySpec(**values)  # type: ignore[arg-type]


def volume_body(**overrides: object) -> DeformableBodySpec:
    values: dict[str, object] = {
        "topology": DeformableTopology.VOLUME,
        "rest_positions_m": ArrayValue.from_rows(((0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1))),
        "tetrahedra": ArrayValue.from_rows(((0, 1, 2, 3),), dtype="int32"),
    }
    values.update(overrides)
    return DeformableBodySpec(**values)  # type: ignore[arg-type]


def fluid_body(**overrides: object) -> ParticleFluidSpec:
    values: dict[str, object] = {
        "initial_particle_positions_m": ArrayValue.from_rows(((0, 0, 1), (0.02, 0, 1))),
        "particle_radius_m": 0.01,
        "rest_density_kg_m3": 1000.0,
        "dynamic_viscosity_pa_s": 0.001,
        "surface_tension_n_m": 0.072,
        "material_id": "water",
    }
    values.update(overrides)
    return ParticleFluidSpec(**values)  # type: ignore[arg-type]


class ArrayValueNestedTests(unittest.TestCase):
    def test_round_trip_rectangular_rank_three_values(self) -> None:
        array = ArrayValue.from_nested((((1, 2, 3), (4, 5, 6)), ((7, 8, 9), (10, 11, 12))))
        self.assertEqual(array.shape, (2, 2, 3))
        self.assertEqual(array.nested()[1][0], (7.0, 8.0, 9.0))

    def test_nested_constructor_rejects_scalar_empty_ragged_and_non_numeric_values(self) -> None:
        for value in (1.0, (), ((1, 2), (3,)), ((1, 2), (3, "bad"))):
            with self.subTest(value=value), self.assertRaises(ValidationError):
                ArrayValue.from_nested(value)

    def test_rows_rejects_rank_three_array(self) -> None:
        with self.assertRaises(ValidationError):
            ArrayValue.from_nested((((1, 2, 3),),)).rows()


class DeformableSpecTests(unittest.TestCase):
    def test_surface_and_volume_shapes_and_serialization(self) -> None:
        surface = surface_body()
        volume = volume_body(surface_triangles=ArrayValue.from_rows(((0, 1, 2),), dtype="int64"))
        self.assertEqual(surface.node_count, 4)
        self.assertEqual(surface.initial_velocities().shape, (4, 3))
        self.assertEqual(surface.to_dict()["surface_triangles"], ((0, 1, 2), (1, 3, 2)))
        self.assertEqual(volume.to_dict()["tetrahedra"], ((0, 1, 2, 3),))
        self.assertIn("surface_triangles", volume.to_dict())

    def test_explicit_initial_velocity_is_preserved(self) -> None:
        velocities = ArrayValue.from_rows(((0, 0, 1),) * 4)
        body = surface_body(initial_node_velocities_m_s=velocities)
        self.assertIs(body.initial_velocities(), velocities)
        self.assertIn("initial_node_velocities_m_s", body.to_dict())

    def test_invalid_geometry_and_topology_are_rejected(self) -> None:
        invalid = (
            {"topology": "surface"},
            {"rest_positions_m": ArrayValue.from_rows(((0, 0, 0), (1, 0, 0)))},
            {"surface_triangles": None},
            {"surface_triangles": object()},
            {"surface_triangles": ArrayValue.from_rows(((0.0, 1.0, 2.0),))},
            {"surface_triangles": ArrayValue.from_rows(((0, 1),), dtype="int64")},
            {"surface_triangles": ArrayValue.from_rows(((0, 1, 4),), dtype="int64")},
            {"surface_triangles": ArrayValue.from_rows(((0, 1, 1),), dtype="int64")},
            {"tetrahedra": ArrayValue.from_rows(((0, 1, 2, 3),), dtype="int64")},
            {"initial_node_velocities_m_s": ArrayValue.from_rows(((0, 0, 0),))},
            {"kinematic_node_indices": (0, 0)},
            {"kinematic_node_indices": (4,)},
            {"node_mass_kg": 0.0},
            {"node_mass_kg": True},
            {"linear_damping_per_s": -1.0},
            {"self_collision": 1},
            {"material_id": ""},
        )
        for overrides in invalid:
            with self.subTest(overrides=overrides), self.assertRaises(ValidationError):
                surface_body(**overrides)
        with self.assertRaises(ValidationError):
            volume_body(tetrahedra=None)

    def test_malformed_numeric_inputs_raise_validation_errors(self) -> None:
        for overrides in (
            {"node_mass_kg": "bad"},
            {"linear_damping_per_s": math.inf},
            {"kinematic_node_indices": None},
        ):
            with self.subTest(overrides=overrides), self.assertRaises(ValidationError):
                surface_body(**overrides)


class ParticleFluidSpecTests(unittest.TestCase):
    def test_mass_defaults_to_density_times_particle_sphere_volume(self) -> None:
        fluid = fluid_body()
        expected = 1000.0 * 4.0 / 3.0 * math.pi * 0.01**3
        self.assertAlmostEqual(fluid.resolved_particle_mass_kg, expected)
        self.assertEqual(fluid.particle_count, 2)
        self.assertEqual(fluid.initial_velocities().shape, (2, 3))
        self.assertNotIn("initial_particle_velocities_m_s", fluid.to_dict())

    def test_explicit_mass_and_velocity_are_preserved(self) -> None:
        velocities = ArrayValue.from_rows(((1, 0, 0), (0, 1, 0)))
        fluid = fluid_body(particle_mass_kg=0.25, initial_particle_velocities_m_s=velocities)
        self.assertEqual(fluid.resolved_particle_mass_kg, 0.25)
        self.assertIs(fluid.initial_velocities(), velocities)
        self.assertIn("initial_particle_velocities_m_s", fluid.to_dict())

    def test_invalid_particle_properties_are_rejected(self) -> None:
        invalid = (
            {"initial_particle_positions_m": ArrayValue((2, 2), (0, 0, 0, 0))},
            {"initial_particle_velocities_m_s": ArrayValue.from_rows(((0, 0, 0),))},
            {"particle_radius_m": 0},
            {"particle_radius_m": True},
            {"rest_density_kg_m3": -1},
            {"particle_mass_kg": 0},
            {"dynamic_viscosity_pa_s": -1},
            {"surface_tension_n_m": -1},
            {"surface_tension_n_m": "bad"},
            {"material_id": ""},
        )
        for overrides in invalid:
            with self.subTest(overrides=overrides), self.assertRaises(ValidationError):
                fluid_body(**overrides)


class SoftMatterWorldSpecTests(unittest.TestCase):
    def test_entity_kinds_require_matching_nested_specs(self) -> None:
        surface = surface_body()
        volume = volume_body()
        fluid = fluid_body()
        valid = (
            EntitySpec(EntityPath("/cloth"), EntityKind.SURFACE_DEFORMABLE, deformable=surface),
            EntitySpec(EntityPath("/gel"), EntityKind.VOLUME_DEFORMABLE, deformable=volume),
            EntitySpec(EntityPath("/water"), EntityKind.PARTICLE_FLUID, particle_fluid=fluid),
        )
        self.assertEqual([entity.kind for entity in valid], list(EntityKind)[2:])
        invalid = (
            {"kind": EntityKind.SURFACE_DEFORMABLE},
            {"kind": EntityKind.SURFACE_DEFORMABLE, "deformable": volume},
            {"kind": EntityKind.VOLUME_DEFORMABLE, "deformable": surface},
            {"kind": EntityKind.PARTICLE_FLUID, "deformable": surface},
            {"kind": EntityKind.RIGID_BODY, "particle_fluid": fluid},
        )
        for overrides in invalid:
            with self.subTest(overrides=overrides), self.assertRaises(ValidationError):
                EntitySpec(EntityPath("/invalid"), **overrides)  # type: ignore[arg-type]

    def test_world_injects_state_requirements_and_uses_current_schema(self) -> None:
        entities = (
            EntitySpec(EntityPath("/cloth"), EntityKind.SURFACE_DEFORMABLE, deformable=surface_body()),
            EntitySpec(EntityPath("/gel"), EntityKind.VOLUME_DEFORMABLE, deformable=volume_body()),
            EntitySpec(EntityPath("/water"), EntityKind.PARTICLE_FLUID, particle_fluid=fluid_body()),
        )
        world = WorldSpec("soft", entities)
        requirement_ids = {requirement.capability for requirement in world.requirements}
        self.assertEqual(world.schema_version, WORLD_SCHEMA_VERSION)
        self.assertTrue(
            {
                CapabilityId("profile.core-robotics@1"),
                CapabilityId("state.deformable.surface@1"),
                CapabilityId("state.deformable.volume@1"),
                CapabilityId("state.fluid.particles@1"),
            }.issubset(requirement_ids)
        )
        self.assertIn("deformable", world.to_dict()["entities"][0])
        self.assertEqual(world.digest, WorldSpec("soft", tuple(reversed(entities))).digest)

    def test_self_collision_is_an_explicit_required_capability(self) -> None:
        cloth = EntitySpec(
            EntityPath("/cloth"),
            EntityKind.SURFACE_DEFORMABLE,
            deformable=surface_body(self_collision=True),
        )
        world = WorldSpec("self-collision", (cloth,))
        self.assertIn(
            CapabilityId("physics.deformable.self-collision@1"),
            {requirement.capability for requirement in world.requirements},
        )

    def test_v0alpha1_rejects_soft_matter_but_still_accepts_rigid_entities(self) -> None:
        rigid = EntitySpec(EntityPath("/box"), EntityKind.RIGID_BODY)
        self.assertEqual(
            WorldSpec("legacy", (rigid,), schema_version=LEGACY_WORLD_SCHEMA_VERSION).schema_version,
            LEGACY_WORLD_SCHEMA_VERSION,
        )
        cloth = EntitySpec(EntityPath("/cloth"), EntityKind.SURFACE_DEFORMABLE, deformable=surface_body())
        with self.assertRaises(ValidationError):
            WorldSpec("legacy-soft", (cloth,), schema_version=LEGACY_WORLD_SCHEMA_VERSION)

    def test_explicit_v0alpha2_preserves_its_previous_rigid_requirement_normalization(self) -> None:
        rigid = EntitySpec(EntityPath("/box"), EntityKind.RIGID_BODY)
        world = WorldSpec("soft-era", (rigid,), schema_version=SOFT_MATTER_WORLD_SCHEMA_VERSION)
        self.assertEqual(world.schema_version, SOFT_MATTER_WORLD_SCHEMA_VERSION)
        self.assertNotIn(
            CapabilityId("state.rigid_body@1"),
            {requirement.capability for requirement in world.requirements},
        )


if __name__ == "__main__":
    unittest.main()
