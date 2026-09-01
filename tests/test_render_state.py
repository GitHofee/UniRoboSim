from __future__ import annotations

import struct
from dataclasses import replace

import pytest

from tests.test_soft_matter_specs import fluid_body
from unirobosim import (
    RENDER_STATE_CAPABILITY_ID,
    ArrayValue,
    CapabilitySet,
    CommandError,
    EntityKind,
    EntityPath,
    EntitySpec,
    EnvironmentSpec,
    PackedFloat32Array,
    RenderArticulationState,
    RenderParticleFluidState,
    RenderRigidBodyState,
    RenderStateFrame,
    RenderStateWorld,
    UnsupportedCapabilityError,
    WorldSpec,
)
from unirobosim.testing import FAKE_DESCRIPTOR, FakeProvider, FakeSession


def _world_spec() -> WorldSpec:
    return WorldSpec(
        "render-state",
        (
            EntitySpec(
                EntityPath("/robot"),
                EntityKind.ARTICULATION,
                joint_names=("shoulder", "elbow"),
            ),
            EntitySpec(EntityPath("/box"), EntityKind.RIGID_BODY),
            EntitySpec(EntityPath("/water"), EntityKind.PARTICLE_FLUID, particle_fluid=fluid_body()),
        ),
        environments=EnvironmentSpec(2),
    )


def _frame(world: RenderStateWorld, *, first_particle_index: int = 0) -> RenderStateFrame:
    resolve = world.resolve  # type: ignore[attr-defined]
    return RenderStateFrame(
        articulations=(
            RenderArticulationState(
                resolve(EntityPath("/robot")),
                ArrayValue.from_rows(((1.25, 0.4),)),
                ArrayValue.from_rows(((0.5, -0.2),)),
                environment_indices=(1,),
                root_positions_m=ArrayValue.from_rows(((9.0, 8.0, 7.0),)),
                root_orientations_xyzw=ArrayValue.from_rows(((0.0, 0.0, 0.0, 1.0),)),
                root_linear_velocities_m_s=ArrayValue.from_rows(((0.6, 0.5, 0.4),)),
                root_angular_velocities_rad_s=ArrayValue.from_rows(((0.3, 0.2, 0.1),)),
            ),
        ),
        rigid_bodies=(
            RenderRigidBodyState(
                resolve(EntityPath("/box")),
                ArrayValue.from_rows(((3.0, 2.0, 1.0),)),
                ArrayValue.from_rows(((0.0, 0.0, 0.0, 1.0),)),
                ArrayValue.from_rows(((0.1, 0.2, 0.3),)),
                ArrayValue.from_rows(((0.4, 0.5, 0.6),)),
                environment_indices=(0,),
            ),
        ),
        particle_fluids=(
            RenderParticleFluidState(
                resolve(EntityPath("/water")),
                PackedFloat32Array((1, 1, 3), struct.pack("<3f", 8.0, 7.0, 6.0)),
                PackedFloat32Array((1, 1, 3), struct.pack("<3f", 0.3, 0.2, 0.1)),
                environment_indices=(1,),
                first_particle_index=first_particle_index,
            ),
        ),
    )


def test_render_state_applies_immediately_without_advancing_physics() -> None:
    session = FakeProvider().open()
    try:
        world = session.build(_world_spec())
        assert isinstance(world, RenderStateWorld)
        tick = world.tick

        result = world.apply_render_state(_frame(world))

        assert world.tick == tick == result.tick
        assert result.state_revision == 1
        assert (result.articulation_count, result.rigid_body_count, result.particle_fluid_count) == (1, 1, 1)
        articulation = world.read_articulation(world.resolve(EntityPath("/robot")))
        assert articulation.joint_positions.rows()[1] == (1.25, 0.4)
        assert articulation.joint_velocities.rows()[1] == (0.5, -0.2)
        robot_scene = next(
            entity
            for entity in world.scene_snapshot().entities
            if entity.path == EntityPath("/robot") and entity.environment_index == 1
        )
        assert robot_scene.pose.position == (0.0, 0.0, 0.0)
        assert robot_scene.linear_velocity_m_s == (0.0, 0.0, 0.0)
        robot_physics = world._articulations[EntityPath("/robot")]  # type: ignore[attr-defined]
        assert robot_physics.root_positions[1] == [9.0, 8.0, 7.0]
        assert robot_physics.root_linear_velocities[1] == [0.6, 0.5, 0.4]
        rigid = world.read_rigid_body(world.resolve(EntityPath("/box")))
        assert rigid.positions_m.rows()[0] == (3.0, 2.0, 1.0)
        assert rigid.linear_velocities_m_s.rows()[0] == (0.1, 0.2, 0.3)
        fluid = world.read_particle_fluid(world.resolve(EntityPath("/water")))
        assert fluid.particle_positions_m.nested()[1][0] == (8.0, 7.0, 6.0)
        assert fluid.particle_velocities_m_s.nested()[1][0] == pytest.approx((0.3, 0.2, 0.1))
        assert world.apply_render_state(_frame(world)).state_revision == 2
    finally:
        session.close()


def test_render_state_prevalidation_rejects_the_whole_frame_atomically() -> None:
    session = FakeProvider().open()
    try:
        world = session.build(_world_spec())
        robot = world.resolve(EntityPath("/robot"))
        before = world.read_articulation(robot)

        with pytest.raises(CommandError, match="range or shape"):
            world.apply_render_state(_frame(world, first_particle_index=2))

        after = world.read_articulation(robot)
        assert after.joint_positions == before.joint_positions
        assert after.joint_velocities == before.joint_velocities
        assert world.tick == before.tick
    finally:
        session.close()


def test_render_state_method_fails_closed_when_capability_is_not_declared() -> None:
    capabilities = CapabilitySet(
        tuple(item for item in FAKE_DESCRIPTOR.capabilities if item.capability != RENDER_STATE_CAPABILITY_ID)
    )
    descriptor = replace(FAKE_DESCRIPTOR, capabilities=capabilities)
    session = FakeSession(descriptor)
    try:
        world = session.build(_world_spec())
        with pytest.raises(UnsupportedCapabilityError):
            world.apply_render_state(_frame(world))
    finally:
        session.close()


def test_render_state_value_objects_reject_ambiguous_or_invalid_frames() -> None:
    session = FakeProvider().open()
    try:
        world = session.build(_world_spec())
        robot = world.resolve(EntityPath("/robot"))
        update = RenderArticulationState(
            robot,
            ArrayValue.from_rows(((0.0, 0.0),)),
            ArrayValue.from_rows(((0.0, 0.0),)),
            environment_indices=(0,),
        )
        with pytest.raises(ValueError, match="at most once"):
            RenderStateFrame(articulations=(update, update))
        box = world.resolve(EntityPath("/box"))
        with pytest.raises(ValueError, match="unit quaternions"):
            RenderRigidBodyState(
                box,
                ArrayValue.from_rows(((0.0, 0.0, 0.0),)),
                ArrayValue.from_rows(((0.0, 0.0, 0.0, 2.0),)),
                ArrayValue.from_rows(((0.0, 0.0, 0.0),)),
                ArrayValue.from_rows(((0.0, 0.0, 0.0),)),
            )
    finally:
        session.close()
