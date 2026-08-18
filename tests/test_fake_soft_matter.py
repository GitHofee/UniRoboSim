from __future__ import annotations

import random
import unittest

from tests.test_soft_matter_specs import fluid_body, surface_body, volume_body
from unirobosim import (
    ArrayValue,
    CapabilityId,
    CapabilityNegotiationError,
    CommandError,
    DeformableCommand,
    EntityKind,
    EntityPath,
    EntitySpec,
    EnvironmentSpec,
    ParticleFluidCommand,
    PhysicsSpec,
    PointCommandMode,
    Pose,
    StaleHandleError,
    ValidationError,
    World,
    WorldSpec,
)
from unirobosim.testing import FakeProvider


def make_soft_world(*, environments: int = 2, self_collision: bool = False) -> WorldSpec:
    return WorldSpec(
        "soft-world",
        (
            EntitySpec(
                EntityPath("/cloth"),
                EntityKind.SURFACE_DEFORMABLE,
                deformable=surface_body(
                    kinematic_node_indices=(0,),
                    node_mass_kg=2.0,
                    linear_damping_per_s=1.0,
                    self_collision=self_collision,
                ),
            ),
            EntitySpec(EntityPath("/gel"), EntityKind.VOLUME_DEFORMABLE, deformable=volume_body()),
            EntitySpec(
                EntityPath("/water"),
                EntityKind.PARTICLE_FLUID,
                particle_fluid=fluid_body(particle_mass_kg=0.5),
            ),
            EntitySpec(EntityPath("/box"), EntityKind.RIGID_BODY),
        ),
        environments=EnvironmentSpec(environments),
        physics=PhysicsSpec(time_step_seconds=0.1, gravity_m_s2=(0, 0, -10)),
    )


class FakeSoftMatterTests(unittest.TestCase):
    def setUp(self) -> None:
        self.session = FakeProvider().open()
        self.world = self.session.build(make_soft_world())
        self.cloth = self.world.resolve(EntityPath("/cloth"))
        self.gel = self.world.resolve(EntityPath("/gel"))
        self.water = self.world.resolve(EntityPath("/water"))

    def tearDown(self) -> None:
        self.session.close()

    def test_fake_world_satisfies_extended_world_protocol(self) -> None:
        self.assertIsInstance(self.world, World)
        profile = self.session.descriptor.capabilities.get(CapabilityId("profile.soft-matter@1"))
        self.assertIsNotNone(profile)
        assert profile is not None
        self.assertIn("no elasticity", profile.limitations[0])

    def test_initial_states_are_batch_point_xyz(self) -> None:
        cloth = self.world.read_deformable(self.cloth)
        gel = self.world.read_deformable(self.gel)
        water = self.world.read_particle_fluid(self.water)
        self.assertEqual(cloth.node_positions_m.shape, (2, 4, 3))
        self.assertEqual(gel.node_positions_m.shape, (2, 4, 3))
        self.assertEqual(water.particle_positions_m.shape, (2, 2, 3))
        self.assertEqual(set(cloth.node_velocities_m_s.values), {0.0})
        self.assertEqual(cloth.tick, self.world.tick)

    def test_gravity_damping_and_kinematic_nodes_follow_reference_rule(self) -> None:
        self.world.step()
        cloth = self.world.read_deformable(self.cloth)
        water = self.world.read_particle_fluid(self.water)
        cloth_positions = cloth.node_positions_m.nested()
        cloth_velocities = cloth.node_velocities_m_s.nested()
        water_positions = water.particle_positions_m.nested()
        water_velocities = water.particle_velocities_m_s.nested()
        for environment in range(2):
            self.assertEqual(cloth_positions[environment][0], (0.0, 0.0, 1.0))
            self.assertEqual(cloth_velocities[environment][0], (0.0, 0.0, 0.0))
            self.assertAlmostEqual(cloth_positions[environment][1][2], 0.91)
            self.assertAlmostEqual(cloth_velocities[environment][1][2], -0.9)
            self.assertAlmostEqual(water_positions[environment][0][2], 0.9)
            self.assertAlmostEqual(water_velocities[environment][0][2], -1.0)

    def test_deformable_commands_preserve_environment_and_node_selection_order(self) -> None:
        self.world.apply_deformable_command(
            DeformableCommand(
                self.cloth,
                PointCommandMode.VELOCITY,
                ArrayValue.from_nested((((1, 2, 3),), ((4, 5, 6),))),
                environment_indices=(1, 0),
                node_indices=(2,),
            )
        )
        self.world.step()
        state = self.world.read_deformable(self.cloth)
        positions = state.node_positions_m.nested()
        velocities = state.node_velocities_m_s.nested()
        self.assertEqual(velocities[0][2], (4.0, 5.0, 6.0))
        self.assertEqual(velocities[1][2], (1.0, 2.0, 3.0))
        self.assertEqual(positions[0][2], (0.4, 1.5, 1.6))
        self.assertEqual(positions[1][2], (0.1, 1.2, 1.3))

    def test_position_moves_kinematic_node_and_non_position_commands_are_atomic_rejections(self) -> None:
        self.world.apply_deformable_command(
            DeformableCommand(
                self.cloth,
                PointCommandMode.POSITION,
                ArrayValue.from_nested((((3, 2, 1),),)),
                environment_indices=(0,),
                node_indices=(0,),
            )
        )
        self.world.step()
        state = self.world.read_deformable(self.cloth)
        self.assertEqual(state.node_positions_m.nested()[0][0], (3.0, 2.0, 1.0))
        self.assertEqual(state.node_velocities_m_s.nested()[0][0], (30.0, 20.0, 0.0))
        before = state.node_positions_m
        with self.assertRaises(CommandError):
            self.world.apply_deformable_command(
                DeformableCommand(
                    self.cloth,
                    PointCommandMode.FORCE,
                    ArrayValue.from_nested((((1, 0, 0), (1, 0, 0)),)),
                    environment_indices=(0,),
                    node_indices=(0, 1),
                )
            )
        self.assertEqual(self.world.read_deformable(self.cloth).node_positions_m, before)

    def test_force_uses_mass_gravity_and_deformable_damping(self) -> None:
        self.world.apply_deformable_command(
            DeformableCommand(
                self.cloth,
                PointCommandMode.FORCE,
                ArrayValue.from_nested((((2, 0, 10),),)),
                environment_indices=(0,),
                node_indices=(1,),
            )
        )
        self.world.step()
        state = self.world.read_deformable(self.cloth)
        velocity = state.node_velocities_m_s.nested()[0][1]
        position = state.node_positions_m.nested()[0][1]
        for actual, expected in zip(velocity, (0.09, 0.0, -0.45), strict=True):
            self.assertAlmostEqual(actual, expected)
        for actual, expected in zip(position, (1.009, 0.0, 0.955), strict=True):
            self.assertAlmostEqual(actual, expected)

    def test_fluid_position_velocity_and_force_commands(self) -> None:
        self.world.apply_particle_fluid_command(
            ParticleFluidCommand(
                self.water,
                PointCommandMode.POSITION,
                ArrayValue.from_nested((((1, 2, 3),),)),
                environment_indices=(0,),
                particle_indices=(0,),
            )
        )
        self.world.apply_particle_fluid_command(
            ParticleFluidCommand(
                self.water,
                PointCommandMode.FORCE,
                ArrayValue.from_nested((((0, 0, 5),),)),
                environment_indices=(1,),
                particle_indices=(1,),
            )
        )
        self.world.step()
        state = self.world.read_particle_fluid(self.water)
        self.assertEqual(state.particle_positions_m.nested()[0][0], (1.0, 2.0, 3.0))
        self.assertEqual(state.particle_velocities_m_s.nested()[0][0], (10.0, 20.0, 20.0))
        self.assertAlmostEqual(state.particle_velocities_m_s.nested()[1][1][2], 0.0)
        self.world.apply_particle_fluid_command(
            ParticleFluidCommand(
                self.water,
                PointCommandMode.VELOCITY,
                ArrayValue.from_nested((((2, 0, 0),),)),
                environment_indices=(1,),
                particle_indices=(0,),
            )
        )
        self.world.step(2)
        state = self.world.read_particle_fluid(self.water)
        self.assertAlmostEqual(state.particle_positions_m.nested()[1][0][0], 0.4)

    def test_partial_reset_restores_only_selected_soft_state_and_control_mode(self) -> None:
        self.world.apply_particle_fluid_command(
            ParticleFluidCommand(
                self.water,
                PointCommandMode.POSITION,
                ArrayValue.from_nested((((5, 5, 5),), ((6, 6, 6),))),
                environment_indices=(0, 1),
                particle_indices=(0,),
            )
        )
        self.world.step()
        tick_before = self.world.tick
        result = self.world.reset((0,))
        self.assertEqual(result.tick, tick_before)
        reset_state = self.world.read_particle_fluid(self.water)
        self.assertEqual(reset_state.particle_positions_m.nested()[0][0], (0.0, 0.0, 1.0))
        self.assertEqual(reset_state.particle_positions_m.nested()[1][0], (6.0, 6.0, 6.0))
        self.world.step()
        stepped = self.world.read_particle_fluid(self.water)
        self.assertAlmostEqual(stepped.particle_positions_m.nested()[0][0][2], 0.9)
        self.assertEqual(stepped.particle_positions_m.nested()[1][0], (6.0, 6.0, 6.0))

    def test_shape_selection_and_wrong_operation_errors_are_explicit(self) -> None:
        bad_shape = DeformableCommand(
            self.cloth,
            PointCommandMode.POSITION,
            ArrayValue.from_nested((((1, 2, 3),),)),
        )
        with self.assertRaises(CommandError):
            self.world.apply_deformable_command(bad_shape)
        out_of_range = ParticleFluidCommand(
            self.water,
            PointCommandMode.FORCE,
            ArrayValue.from_nested((((0, 0, 0),),)),
            environment_indices=(2,),
            particle_indices=(0,),
        )
        with self.assertRaises(ValidationError):
            self.world.apply_particle_fluid_command(out_of_range)
        box = self.world.resolve(EntityPath("/box"))
        with self.assertRaises(CommandError):
            self.world.read_deformable(box)
        with self.assertRaises(CommandError):
            self.world.read_particle_fluid(self.cloth)
        with self.assertRaises(CommandError):
            self.world.apply_deformable_command(object())  # type: ignore[arg-type]
        with self.assertRaises(CommandError):
            self.world.apply_particle_fluid_command(object())  # type: ignore[arg-type]

    def test_soft_handle_is_stale_after_rebuild(self) -> None:
        old_cloth = self.cloth
        self.world.close()
        self.world = self.session.build(make_soft_world())
        with self.assertRaises(StaleHandleError):
            self.world.read_deformable(old_cloth)


class FakeSoftMatterCapabilityTests(unittest.TestCase):
    def test_fake_rejects_self_collision_instead_of_silently_degrading(self) -> None:
        session = FakeProvider().open()
        try:
            with self.assertRaises(CapabilityNegotiationError):
                session.build(make_soft_world(self_collision=True))
        finally:
            session.close()

    def test_entity_pose_transforms_local_soft_geometry_and_velocity_to_world_frame(self) -> None:
        velocities = ArrayValue.from_rows(((1, 0, 0),) * 4)
        cloth = EntitySpec(
            EntityPath("/posed-cloth"),
            EntityKind.SURFACE_DEFORMABLE,
            pose=Pose((10, 20, 30), (0, 0, 2**-0.5, 2**-0.5)),
            deformable=surface_body(
                kinematic_node_indices=(),
                initial_node_velocities_m_s=velocities,
            ),
        )
        session = FakeProvider().open()
        try:
            world = session.build(WorldSpec("posed-soft", (cloth,)))
            state = world.read_deformable(world.resolve(EntityPath("/posed-cloth")))
            position = state.node_positions_m.nested()[0][1]
            velocity = state.node_velocities_m_s.nested()[0][1]
            for actual, expected in zip(position, (10.0, 21.0, 31.0), strict=True):
                self.assertAlmostEqual(actual, expected)
            for actual, expected in zip(velocity, (0.0, 1.0, 0.0), strict=True):
                self.assertAlmostEqual(actual, expected)
        finally:
            session.close()


class FakeFluidRandomizedReferenceTests(unittest.TestCase):
    def test_fixed_seed_commands_match_independent_reference_model(self) -> None:
        random_source = random.Random(20260818)
        session = FakeProvider().open()
        world = session.build(make_soft_world(environments=3))
        handle = world.resolve(EntityPath("/water"))
        positions = [[list(point) for point in ((0, 0, 1), (0.02, 0, 1))] for _ in range(3)]
        velocities = [[[0.0, 0.0, 0.0] for _ in range(2)] for _ in range(3)]
        modes = [[PointCommandMode.FORCE for _ in range(2)] for _ in range(3)]
        targets = [[[[0.0, 0.0, 0.0][axis] for axis in range(3)] for _ in range(2)] for _ in range(3)]
        try:
            for _ in range(50):
                environment = random_source.randrange(3)
                particle = random_source.randrange(2)
                mode = random_source.choice(list(PointCommandMode))
                target = [random_source.uniform(-2.0, 2.0) for _ in range(3)]
                world.apply_particle_fluid_command(
                    ParticleFluidCommand(
                        handle,
                        mode,
                        ArrayValue.from_nested(((target,),)),
                        environment_indices=(environment,),
                        particle_indices=(particle,),
                    )
                )
                modes[environment][particle] = mode
                targets[environment][particle] = target
                for env_index in range(3):
                    for particle_index in range(2):
                        current_mode = modes[env_index][particle_index]
                        current_target = targets[env_index][particle_index]
                        if current_mode is PointCommandMode.POSITION:
                            previous = positions[env_index][particle_index]
                            positions[env_index][particle_index] = current_target.copy()
                            velocities[env_index][particle_index] = [
                                (current_target[axis] - previous[axis]) / 0.1 for axis in range(3)
                            ]
                        elif current_mode is PointCommandMode.VELOCITY:
                            velocities[env_index][particle_index] = current_target.copy()
                            positions[env_index][particle_index] = [
                                positions[env_index][particle_index][axis] + current_target[axis] * 0.1
                                for axis in range(3)
                            ]
                        else:
                            acceleration = [
                                current_target[0] / 0.5,
                                current_target[1] / 0.5,
                                current_target[2] / 0.5 - 10.0,
                            ]
                            velocities[env_index][particle_index] = [
                                velocities[env_index][particle_index][axis] + acceleration[axis] * 0.1
                                for axis in range(3)
                            ]
                            positions[env_index][particle_index] = [
                                positions[env_index][particle_index][axis]
                                + velocities[env_index][particle_index][axis] * 0.1
                                for axis in range(3)
                            ]
                world.step()
                state = world.read_particle_fluid(handle)
                for actual, expected in zip(
                    state.particle_positions_m.values,
                    ArrayValue.from_nested(positions).values,
                    strict=True,
                ):
                    self.assertAlmostEqual(actual, expected, places=12)
                for actual, expected in zip(
                    state.particle_velocities_m_s.values,
                    ArrayValue.from_nested(velocities).values,
                    strict=True,
                ):
                    self.assertAlmostEqual(actual, expected, places=12)
        finally:
            session.close()


if __name__ == "__main__":
    unittest.main()
