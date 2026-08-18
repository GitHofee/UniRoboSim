from __future__ import annotations

from dataclasses import replace

import pytest

from tests.helpers import make_world_spec
from unirobosim import (
    EntityKind,
    EntityPath,
    FrozenMap,
    LifecycleError,
    Pose,
    SceneCommand,
    SceneCommandKind,
    SceneCommandResult,
    SceneCommandStatus,
    SceneControlWorld,
    SceneDelta,
    SceneDragMode,
    SceneEntityState,
    SceneSnapshot,
    SceneVisual,
    SceneVisualKind,
    Tick,
    ValidationError,
)
from unirobosim.testing import FakeProvider


def command(
    kind: SceneCommandKind,
    command_id: str,
    generation: int,
    *,
    pose: Pose | None = None,
    drag_id: str | None = None,
    mode: SceneDragMode | None = None,
) -> SceneCommand:
    return SceneCommand(
        command_id,
        "browser-1",
        "lease-1",
        generation,
        kind,
        EntityPath("/ground"),
        1,
        pose,
        drag_id,
        mode,
        (0.0, 0.0, 0.0) if kind is SceneCommandKind.DRAG_BEGIN else None,
    )


def test_scene_value_contract_and_serialization() -> None:
    visual = SceneVisual(
        "box",
        SceneVisualKind.BOX,
        dimensions_m=(1.0, 2.0, 3.0),
        color_rgba=(0.1, 0.2, 0.3, 1.0),
        metadata=FrozenMap({"source": "test"}),
    )
    entity = SceneEntityState(
        EntityPath("/box"),
        EntityKind.RIGID_BODY,
        0,
        Pose((1.0, 2.0, 3.0)),
        visuals=(visual,),
        draggable=True,
    )
    snapshot = SceneSnapshot("reference.fake", "scene", 1, 3, Tick(2, 0.02), (entity,))
    assert snapshot.to_dict()["entities"][0]["visuals"][0]["dimensions_m"] == [1.0, 2.0, 3.0]
    heartbeat = SceneDelta("scene", 1, 3, 3, Tick(2, 0.02))
    assert heartbeat.to_dict()["upserts"] == []
    with pytest.raises(ValidationError):
        replace(heartbeat, upserts=(entity,))
    with pytest.raises(ValidationError):
        SceneVisual("mesh", SceneVisualKind.MESH)
    with pytest.raises(ValidationError):
        replace(visual, color_rgba=(2.0, 0.0, 0.0, 1.0))


def test_fake_snapshot_delta_pose_and_idempotency() -> None:
    session = FakeProvider().open()
    world = session.build(make_world_spec(environments=2))
    try:
        assert isinstance(world, SceneControlWorld)
        initial = world.scene_snapshot()
        assert initial.sequence == 0 and len(initial.entities) == 4
        ground = next(item for item in initial.entities if item.key == ("/ground", 1))
        assert ground.draggable and ground.visuals[0].kind is SceneVisualKind.BOX

        target = Pose((1.0, -2.0, 0.5))
        result = world.apply_scene_command(command(SceneCommandKind.SET_POSE, "pose-1", world.generation, pose=target))
        assert result.status is SceneCommandStatus.APPLIED and result.scene_sequence == 1
        duplicate = world.apply_scene_command(
            command(SceneCommandKind.SET_POSE, "pose-1", world.generation, pose=Pose((9.0, 9.0, 9.0)))
        )
        assert duplicate.status is SceneCommandStatus.DUPLICATE
        assert world.scene_snapshot().entities[1].pose == target
        delta = world.scene_delta(initial.sequence)
        assert delta.base_sequence == 0 and delta.sequence == 1 and len(delta.upserts) == 4
        assert world.scene_delta(delta.sequence).upserts == ()

        stale = world.apply_scene_command(
            command(SceneCommandKind.SET_POSE, "stale-1", world.generation + 1, pose=target)
        )
        assert stale.status is SceneCommandStatus.REJECTED and stale.error_code == "stale_generation"
        world.step(2)
        assert world.scene_delta(1).sequence == 3
        with pytest.raises(ValidationError):
            world.scene_delta(4)
    finally:
        session.close()


def test_fake_drag_transaction_commit_cancel_and_failures() -> None:
    session = FakeProvider().open()
    world = session.build(make_world_spec(environments=2))
    try:
        begin = command(
            SceneCommandKind.DRAG_BEGIN,
            "begin-1",
            world.generation,
            drag_id="drag-1",
            mode=SceneDragMode.KINEMATIC,
        )
        assert world.apply_scene_command(begin).status is SceneCommandStatus.APPLIED
        target = Pose((2.0, 3.0, 4.0))
        update = command(
            SceneCommandKind.DRAG_UPDATE,
            "update-1",
            world.generation,
            pose=target,
            drag_id="drag-1",
        )
        assert world.apply_scene_command(update).status is SceneCommandStatus.APPLIED
        end = command(SceneCommandKind.DRAG_END, "end-1", world.generation, drag_id="drag-1")
        assert world.apply_scene_command(end).status is SceneCommandStatus.APPLIED
        assert world.scene_snapshot().entities[1].pose == target

        begin_cancel = replace(begin, command_id="begin-2", drag_id="drag-2")
        world.apply_scene_command(begin_cancel)
        world.apply_scene_command(replace(update, command_id="update-2", drag_id="drag-2", target_pose=Pose((8, 8, 8))))
        cancelled = world.apply_scene_command(
            command(SceneCommandKind.DRAG_CANCEL, "cancel-2", world.generation, drag_id="drag-2")
        )
        assert cancelled.status is SceneCommandStatus.APPLIED
        assert world.scene_snapshot().entities[1].pose == target

        missing = world.apply_scene_command(
            command(SceneCommandKind.DRAG_END, "end-missing", world.generation, drag_id="missing")
        )
        assert missing.error_code == "drag_not_active"
        constraint = replace(begin, command_id="constraint", drag_id="constraint", drag_mode=SceneDragMode.CONSTRAINT)
        assert world.apply_scene_command(constraint).error_code == "unsupported_drag_mode"
        articulation = replace(begin, command_id="robot", drag_id="robot", entity_path=EntityPath("/robot"))
        assert world.apply_scene_command(articulation).error_code == "unsupported_entity_kind"
    finally:
        session.close()


def test_scene_command_validation_and_closed_world() -> None:
    with pytest.raises(ValidationError):
        command(SceneCommandKind.SET_POSE, "missing-pose", 1)
    with pytest.raises(ValidationError):
        command(SceneCommandKind.DRAG_BEGIN, "missing-mode", 1, drag_id="drag")
    with pytest.raises(ValidationError):
        command(SceneCommandKind.DRAG_END, "bad-extra", 1, drag_id="drag", mode=SceneDragMode.KINEMATIC)
    session = FakeProvider().open()
    world = session.build(make_world_spec())
    world.close()
    with pytest.raises(LifecycleError):
        world.scene_snapshot()
    session.close()


def test_scene_values_reject_ambiguous_or_nonportable_data() -> None:
    visual = SceneVisual("body", SceneVisualKind.BOX)
    mesh = SceneVisual("mesh", SceneVisualKind.MESH, asset_uri="file:///asset.glb")
    assert mesh.to_dict()["asset_uri"] == "file:///asset.glb"
    invalid_visuals = (
        {"visual_id": "bad id"},
        {"kind": "box"},
        {"dimensions_m": (1.0, float("nan"), 1.0)},
        {"asset_uri": "file:///not-a-mesh.glb"},
        {"metadata": {}},
    )
    for override in invalid_visuals:
        with pytest.raises(ValidationError):
            replace(visual, **override)  # type: ignore[arg-type]

    entity = SceneEntityState(
        EntityPath("/body"),
        EntityKind.RIGID_BODY,
        0,
        Pose(),
        visuals=(visual,),
    )
    invalid_entities = (
        {"environment_index": -1},
        {"linear_velocity_m_s": (0.0, 1.0)},
        {"joint_names": ("same", "same"), "joint_positions": (0.0, 0.0)},
        {"joint_names": ("joint",), "joint_positions": ()},
        {"visuals": (visual, visual)},
        {"selectable": 1},
        {"metadata": {}},
    )
    for override in invalid_entities:
        with pytest.raises(ValidationError):
            replace(entity, **override)  # type: ignore[arg-type]
    with pytest.raises(ValidationError):
        replace(entity, joint_positions=(object(),))


def test_snapshot_delta_and_result_validation_boundaries() -> None:
    visual = SceneVisual("body", SceneVisualKind.BOX)
    first = SceneEntityState(EntityPath("/z"), EntityKind.RIGID_BODY, 0, Pose(), visuals=(visual,))
    second = replace(first, path=EntityPath("/a"))
    snapshot = SceneSnapshot("reference.fake", "world", 1, 2, Tick(2, 0.02), (first, second))
    assert [item.path.value for item in snapshot.entities] == ["/a", "/z"]
    with pytest.raises(ValidationError):
        replace(snapshot, entities=None)  # type: ignore[arg-type]
    with pytest.raises(ValidationError):
        replace(snapshot, entities=(first, first))
    with pytest.raises(ValidationError):
        replace(snapshot, generation=0)

    delta = SceneDelta(
        "world",
        1,
        2,
        3,
        Tick(3, 0.03),
        upserts=(first,),
        removals=((EntityPath("/gone"), 0),),
    )
    assert delta.to_dict()["removals"] == [{"path": "/gone", "environment_index": 0}]
    with pytest.raises(ValidationError):
        replace(delta, upserts=None)  # type: ignore[arg-type]
    with pytest.raises(ValidationError):
        replace(delta, removals=((EntityPath("/z"), 0),))
    with pytest.raises(ValidationError):
        replace(delta, sequence=1)

    applied = SceneCommandResult("result", SceneCommandStatus.APPLIED, 1, 3, Tick(3, 0.03))
    assert applied.to_dict()["status"] == "applied"
    with pytest.raises(ValidationError):
        replace(applied, generation=0)
    with pytest.raises(ValidationError):
        replace(applied, error_code="unexpected")
    with pytest.raises(ValidationError):
        replace(applied, status=SceneCommandStatus.REJECTED)
    with pytest.raises(ValidationError):
        replace(applied, message="")


def test_scene_command_identity_target_and_drag_field_boundaries() -> None:
    pose = Pose((1.0, 2.0, 3.0))
    set_pose = SceneCommand(
        "set",
        "client",
        "lease",
        1,
        SceneCommandKind.SET_POSE,
        EntityPath("/body"),
        target_pose=pose,
    )
    invalid_commands = (
        {"command_id": "bad id"},
        {"expected_generation": 0},
        {"kind": "set_pose"},
        {"environment_index": -1},
        {"target_pose": object()},
        {"drag_id": "unexpected"},
    )
    for override in invalid_commands:
        with pytest.raises(ValidationError):
            replace(set_pose, **override)  # type: ignore[arg-type]
    with pytest.raises(ValidationError):
        SceneCommand(
            "end",
            "client",
            "lease",
            1,
            SceneCommandKind.DRAG_END,
            EntityPath("/body"),
            drag_id="bad id",
        )
