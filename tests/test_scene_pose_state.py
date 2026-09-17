from dataclasses import replace

import pytest

from tests.test_planning_scene import planning_world as planning_world
from unirobosim.api import PlanningSceneContractError, PlanningScenePoseState


def _poses(world):
    catalog = world.planning_scene_catalog()
    state = world.planning_scene_state()
    return catalog, PlanningScenePoseState(
        state.provider_id,
        state.world_id,
        state.generation,
        state.environment_index,
        state.tick,
        state.catalog_revision,
        state.catalog_content_sha256,
        state.world_frame_id,
        state.entities,
        state.links,
    )


def test_pose_state_preserves_complete_physical_link_coverage(planning_world):
    catalog, poses = _poses(planning_world)
    poses.validate_against(catalog)
    assert poses.links
    assert not hasattr(poses, "geometry_transforms")
    for changed in (
        replace(poses, links=poses.links[:-1]),
        replace(poses, entities=poses.entities[:-1]),
        replace(poses, generation=poses.generation + 1),
        replace(poses, catalog_content_sha256="a" * 64),
    ):
        with pytest.raises(PlanningSceneContractError):
            changed.validate_against(catalog)


@pytest.mark.parametrize(
    "field,value",
    [("generation", 0), ("environment_index", -1), ("tick", None), ("entities", []), ("catalog_content_sha256", "bad")],
)
def test_pose_state_rejects_invalid_envelopes(planning_world, field, value):
    _, poses = _poses(planning_world)
    with pytest.raises(PlanningSceneContractError):
        replace(poses, **{field: value})


@pytest.mark.parametrize(
    "field,value",
    [
        ("provider_id", "other"),
        ("world_id", "other"),
        ("environment_index", 1),
        ("catalog_revision", 2),
        ("catalog_content_sha256", "f" * 64),
    ],
)
def test_pose_state_rejects_foreign_catalog_identity(planning_world, field, value):
    catalog, poses = _poses(planning_world)
    with pytest.raises(PlanningSceneContractError):
        replace(poses, **{field: value}).validate_against(catalog)


@pytest.mark.parametrize("field", ["entities", "links"])
def test_pose_state_rejects_duplicate_unsorted_and_wrong_types(planning_world, field):
    _, poses = _poses(planning_world)
    values = getattr(poses, field)
    for changed in ((values[0], values[0]), tuple(reversed(values)), (object(),)):
        if changed == values:
            continue
        with pytest.raises(PlanningSceneContractError):
            replace(poses, **{field: changed})


def test_pose_state_rejects_nonworld_frames_and_total_budget(planning_world, monkeypatch):
    from unirobosim.api import planning_scene

    _, poses = _poses(planning_world)
    item = poses.entities[0]
    foreign = replace(item, pose=replace(item.pose, frame_id="other"), twist=replace(item.twist, frame_id="other"))
    with pytest.raises(PlanningSceneContractError):
        replace(poses, entities=(foreign, *poses.entities[1:]))
    monkeypatch.setattr(planning_scene, "_MAX_ITEMS", max(len(poses.entities), len(poses.links)))
    with pytest.raises(PlanningSceneContractError, match="aggregate"):
        replace(poses)


def test_pose_protocol_is_optional_and_runtime_checkable():
    from unirobosim.api import PlanningScenePoseWorld

    assert not isinstance(object(), PlanningScenePoseWorld)

    class World:
        def planning_scene_pose_state(self, environment_index=0):
            raise NotImplementedError("unavailable")

    assert isinstance(World(), PlanningScenePoseWorld)
