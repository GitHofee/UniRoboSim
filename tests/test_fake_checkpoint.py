from __future__ import annotations

import json

import pytest

from tests.helpers import make_world_spec
from unirobosim import (
    ArrayValue,
    ArticulationCommand,
    CheckpointFidelity,
    CommandMode,
    EntityPath,
    SceneCommand,
    SceneCommandKind,
    SceneCommandStatus,
    ValidationError,
    WorldCheckpoint,
)
from unirobosim.testing import FakeProvider


def test_checkpoint_restores_physical_state_without_rewinding_clock() -> None:
    session = FakeProvider().open()
    try:
        world = session.build(make_world_spec(environments=2, initial=(0.1, -0.2)))
        robot = world.resolve(EntityPath("/robot"))
        ground = world.resolve(EntityPath("/ground"))
        world.apply_articulation_command(
            ArticulationCommand(
                robot,
                CommandMode.POSITION,
                ArrayValue.from_rows(((0.5, 0.6), (0.7, 0.8))),
            )
        )
        world.step()
        expected_robot = world.read_articulation(robot)
        expected_ground = world.read_rigid_body(ground)
        checkpoint = world.create_checkpoint()

        world.apply_articulation_command(
            ArticulationCommand(
                robot,
                CommandMode.POSITION,
                ArrayValue.from_rows(((-0.5, -0.6), (-0.7, -0.8))),
            )
        )
        world.step(3)
        live_tick = world.tick
        assert world.read_articulation(robot).joint_positions != expected_robot.joint_positions

        result = world.restore_checkpoint(checkpoint)

        assert result.tick == live_tick
        assert world.tick == live_tick
        restored_robot = world.read_articulation(robot)
        restored_ground = world.read_rigid_body(ground)
        assert restored_robot.joint_positions == expected_robot.joint_positions
        assert restored_robot.joint_velocities == expected_robot.joint_velocities
        assert restored_ground.positions_m == expected_ground.positions_m
        assert restored_ground.linear_velocities_m_s == expected_ground.linear_velocities_m_s
    finally:
        session.close()


def test_checkpoint_from_equivalent_rebuilt_world_can_be_restored() -> None:
    session = FakeProvider().open()
    try:
        spec = make_world_spec(environments=1)
        first = session.build(spec)
        checkpoint = first.create_checkpoint()
        first.close()
        second = session.build(spec)

        result = second.restore_checkpoint(checkpoint)

        assert result.generation == second.generation
        assert result.restored_entity_count == 2
    finally:
        session.close()


def test_invalid_payload_is_rejected_before_any_state_changes() -> None:
    session = FakeProvider().open()
    try:
        world = session.build(make_world_spec(environments=1))
        robot = world.resolve(EntityPath("/robot"))
        checkpoint = world.create_checkpoint()
        before = world.read_articulation(robot)
        document = json.loads(checkpoint.payload)
        document["articulations"]["/robot"]["positions"] = [[float("nan"), 0.0]]
        corrupt = WorldCheckpoint(
            checkpoint.provider_id,
            checkpoint.world_id,
            checkpoint.source_generation,
            checkpoint.source_tick,
            checkpoint.payload_schema,
            CheckpointFidelity.PHYSICAL,
            json.dumps(document).encode("utf-8"),
            checkpoint.entity_count,
        )

        with pytest.raises(ValidationError):
            world.restore_checkpoint(corrupt)

        assert world.read_articulation(robot) == before
    finally:
        session.close()


def test_checkpoint_restores_runtime_attachment_registry() -> None:
    session = FakeProvider().open()
    try:
        world = session.build(make_world_spec(environments=1))
        attached = world.apply_scene_command(
            SceneCommand(
                "attach-ground",
                "plugin",
                "lease",
                world.generation,
                SceneCommandKind.ATTACH,
                EntityPath("/ground"),
                attachment_id="held-object",
                parent_entity_path=EntityPath("/robot"),
            )
        )
        assert attached.status is SceneCommandStatus.APPLIED
        checkpoint = world.create_checkpoint()
        detached = world.apply_scene_command(
            SceneCommand(
                "detach-ground",
                "plugin",
                "lease",
                world.generation,
                SceneCommandKind.DETACH,
                EntityPath("/ground"),
                attachment_id="held-object",
            )
        )
        assert detached.status is SceneCommandStatus.APPLIED
        assert all(item.attachment_id != "held-object" for item in world._attachments.values())

        world.restore_checkpoint(checkpoint)

        assert any(item.attachment_id == "held-object" for item in world._attachments.values())
    finally:
        session.close()
