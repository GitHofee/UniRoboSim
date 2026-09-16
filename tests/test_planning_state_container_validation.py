"""State validation keeps defensive behavior while avoiding unused work."""

from dataclasses import replace

import pytest

from unirobosim import (
    BoxGeometrySpec,
    CapabilityId,
    CapabilityRequirement,
    EntityKind,
    EntityPath,
    EntitySpec,
    FrozenMap,
    PlanningArticulationState,
    PlanningPose,
    PlanningSceneContractError,
    WorldSpec,
)
from unirobosim.testing import FakeProvider


class HostileTuple(tuple):
    def __iter__(self):
        raise AssertionError("tuple subclass iteration dispatched")

    def __len__(self):
        raise AssertionError("tuple subclass length dispatched")

    def __getitem__(self, index):
        raise AssertionError("tuple subclass item dispatched")


@pytest.fixture(params=[False, True], ids=["without-attachment", "with-attachment"])
def publication(request):
    metadata = {}
    if request.param:
        metadata = {
            "planning_attachment_authority": "exclusive_registry",
            "planning_attachments": (
                {"attachment_id": "attachment.a-b", "parent_path": "/a", "child_path": "/b"},
            ),
        }
    session = FakeProvider().open()
    try:
        world = session.build(
            WorldSpec(
                "container-validation",
                tuple(
                    EntitySpec(EntityPath(path), EntityKind.RIGID_BODY, box=BoxGeometrySpec()) for path in ("/a", "/b")
                ),
                requirements=(CapabilityRequirement(CapabilityId("planning.scene@2")),),
                metadata=FrozenMap(metadata),
            )
        )
        yield world.planning_scene_state(), world.planning_scene_catalog()
    finally:
        session.close()


@pytest.mark.parametrize("field", ["position_m", "orientation_xyzw"])
def test_pose_rejects_tuple_subclass_without_dispatching_hooks(field):
    pose = PlanningPose("frame.world", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    with pytest.raises(PlanningSceneContractError, match="must be an immutable"):
        replace(pose, **{field: HostileTuple(getattr(pose, field))})


def test_identifier_sequence_rejects_tuple_subclass_without_dispatching_hooks():
    with pytest.raises(
        PlanningSceneContractError, match="articulation state joint_ids must be a bounded immutable tuple"
    ):
        PlanningArticulationState("robot", HostileTuple(("joint",)), (0.0,), (0.0,), ("rad",))


def test_state_records_reject_tuple_subclass_without_dispatching_hooks(publication):
    state, _ = publication
    with pytest.raises(PlanningSceneContractError, match="state entities must be a bounded immutable tuple"):
        replace(state, entities=HostileTuple(state.entities))


@pytest.mark.parametrize(
    "position,reason",
    [
        ((1.0, float("nan"), True), r"pose position_m\[1\] must be finite"),
        ((True, float("nan"), 0.0), r"pose position_m\[0\] must be numeric"),
        ((1.0, 2.0, float("inf")), r"pose position_m\[2\] must be finite"),
    ],
)
def test_vector_failure_identifies_first_invalid_coordinate(position, reason):
    with pytest.raises(PlanningSceneContractError, match=reason):
        PlanningPose("frame.world", position, (0.0, 0.0, 0.0, 1.0))


def test_reconstructed_state_retains_values_and_catalog_validation(publication):
    state, catalog = publication
    rebuilt = replace(state, entities=tuple(replace(item) for item in state.entities))
    assert rebuilt == state
    rebuilt.validate_against(catalog)


def test_geometry_consistency_is_checked_even_without_attachments(publication):
    state, catalog = publication
    geometry = state.geometry_transforms[0]
    bad_geometry = replace(geometry, world_pose=replace(geometry.world_pose, position_m=(10.0, 0.0, 0.0)))
    bad_state = replace(state, geometry_transforms=(bad_geometry,) + state.geometry_transforms[1:])
    with pytest.raises(PlanningSceneContractError, match="geometry world pose contradicts"):
        bad_state.validate_against(catalog)


def test_nonempty_attachment_references_and_catalog_ownership_are_still_checked(publication):
    state, catalog = publication
    if not state.attachments:
        return
    attachment = state.attachments[0]
    with pytest.raises(PlanningSceneContractError, match="attachment references do not close"):
        replace(state, attachments=(replace(attachment, parent_entity_id="entity.missing"),))
    # The link exists, so state closure still holds; catalog ownership must reject
    # assigning a link to an attachment endpoint that names another frame/link.
    other_link = next(item for item in catalog.links if item.link_id != attachment.parent_link_id)
    invalid = replace(state, attachments=(replace(attachment, parent_link_id=other_link.link_id),))
    with pytest.raises(PlanningSceneContractError, match="attachment parent link ownership is invalid"):
        invalid.validate_against(catalog)
