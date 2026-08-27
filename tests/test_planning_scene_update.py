from __future__ import annotations

import inspect
from dataclasses import replace

import pytest

from unirobosim import (
    CapabilityId,
    CapabilityRequirement,
    EntityKind,
    EntityPath,
    EntitySpec,
    FrozenMap,
    PhysicsSpec,
    PlanningSceneContractError,
    PlanningSceneStatePatch,
    PlanningSceneUpdateKind,
    PlanningSceneUpdateWorld,
    PlanningSceneWorld,
    Pose,
    SceneCommand,
    SceneCommandKind,
    SceneCommandStatus,
    WorldSpec,
)
from unirobosim.testing import FakeProvider


@pytest.fixture
def update_world():
    entities = tuple(
        EntitySpec(
            EntityPath(f"/body_{index}"),
            EntityKind.RIGID_BODY,
            asset_uri=f"asset://body-{index}",
            metadata=FrozenMap(
                {
                    "planning_motion_class": "static",
                    "fake_planning_collision_authority": "effective_native",
                }
            ),
        )
        for index in range(2)
    )
    spec = WorldSpec(
        "planning-update-contract",
        entities,
        requirements=(CapabilityRequirement(CapabilityId("planning.scene@2")),),
        physics=PhysicsSpec(gravity_m_s2=(0.0, 0.0, 0.0)),
        metadata=FrozenMap(
            {
                "planning_attachment_authority": "exclusive_registry",
                "planning_attachments": (),
            }
        ),
    )
    session = FakeProvider().open()
    world = session.build(spec)
    try:
        yield world
    finally:
        session.close()


def _flatten_unchanged(prior, update):
    return replace(
        prior,
        tick=update.tick,
        sequence=update.sequence,
        world_revision=update.world_revision,
        catalog_revision=update.catalog_revision,
        geometry_revision=update.geometry_revision,
        catalog_content_sha256=update.catalog_content_sha256,
        transform_revision=update.transform_revision,
        attachment_revision=update.attachment_revision,
    )


def _merge(values, patch, identity: str):
    merged = {getattr(item, identity): item for item in values}
    merged.update({getattr(item, identity): item for item in patch})
    return tuple(sorted(merged.values(), key=lambda item: getattr(item, identity)))


def test_update_protocol_is_additive_and_runtime_detectable(update_world) -> None:
    assert isinstance(update_world, PlanningSceneWorld)
    assert isinstance(update_world, PlanningSceneUpdateWorld)
    assert "planning_scene_update" not in PlanningSceneWorld.__dict__
    assert inspect.signature(PlanningSceneUpdateWorld.planning_scene_update) == inspect.signature(
        type(update_world).planning_scene_update
    )


def test_same_sequence_unchanged_is_payload_free_and_does_no_content_work(update_world) -> None:
    current = update_world.planning_scene_state()
    runtime = update_world._planning_runtime
    update = update_world.planning_scene_update(current.sequence)
    assert update.kind is PlanningSceneUpdateKind.UNCHANGED
    assert update.sequence == update.base_sequence == current.sequence
    assert update.state_patch is update.attachment_patch is None
    assert update.catalog is update.state is None
    assert update.world_revision == update.previous_world_revision
    assert update.transform_revision == update.previous_transform_revision
    assert runtime.geometry_hash_calls == runtime.geometry_materializations == 0


def test_cross_tick_unchanged_reuses_records_when_flattened(update_world) -> None:
    prior = update_world.planning_scene_state()
    update_world.step()
    current = update_world.planning_scene_state()
    update = update_world.planning_scene_update(prior.sequence)
    assert update.kind is PlanningSceneUpdateKind.UNCHANGED
    assert update.sequence > update.base_sequence
    assert update.world_revision > update.previous_world_revision
    assert update.state_patch is update.attachment_patch is None
    flattened = _flatten_unchanged(prior, update)
    assert flattened == current
    assert flattened.entities is prior.entities
    assert flattened.links is prior.links
    assert flattened.frames is prior.frames
    assert flattened.articulations is prior.articulations
    assert flattened.geometry_transforms is prior.geometry_transforms
    assert update_world._planning_runtime.geometry_hash_calls == 0
    assert update_world._planning_runtime.geometry_materializations == 0


def test_state_patch_contains_only_changed_canonical_records(update_world) -> None:
    prior = update_world.planning_scene_state()
    result = update_world.apply_scene_command(
        SceneCommand(
            "move-body-zero",
            "update-contract-test",
            "lease",
            update_world.generation,
            SceneCommandKind.SET_POSE,
            EntityPath("/body_0"),
            target_pose=Pose((1.0, 2.0, 3.0)),
        )
    )
    assert result.status is SceneCommandStatus.APPLIED
    current = update_world.planning_scene_state()
    update = update_world.planning_scene_update(prior.sequence)
    assert update.kind is PlanningSceneUpdateKind.STATE_PATCH
    assert update.state_patch is not None and not update.state_patch.empty
    patch = update.state_patch
    assert tuple(item.entity_id for item in patch.entities) == tuple(
        sorted(item.entity_id for item in patch.entities)
    )
    flattened = replace(
        prior,
        tick=update.tick,
        sequence=update.sequence,
        world_revision=update.world_revision,
        transform_revision=update.transform_revision,
        entities=_merge(prior.entities, patch.entities, "entity_id"),
        links=_merge(prior.links, patch.links, "link_id"),
        frames=_merge(prior.frames, patch.frames, "frame_id"),
        articulations=_merge(prior.articulations, patch.articulations, "entity_id"),
        geometry_transforms=_merge(
            prior.geometry_transforms,
            patch.geometry_transforms,
            "geometry_id",
        ),
    )
    assert flattened == current
    assert update_world._planning_runtime.geometry_hash_calls == 0
    assert update_world._planning_runtime.geometry_materializations == 0


def test_patch_requires_canonical_unique_order(update_world) -> None:
    state = update_world.planning_scene_state()
    with pytest.raises(PlanningSceneContractError, match="deterministic identifier order"):
        PlanningSceneStatePatch(entities=tuple(reversed(state.entities)))
    with pytest.raises(PlanningSceneContractError, match="unique identifiers"):
        PlanningSceneStatePatch(entities=(state.entities[0], state.entities[0]))


def test_history_or_generation_gap_returns_payload_free_resync(update_world) -> None:
    prior = update_world.planning_scene_state()
    update_world.step()
    del update_world._planning_runtime.environments[0].history[prior.sequence]
    gap = update_world.planning_scene_update(prior.sequence)
    assert gap.kind is PlanningSceneUpdateKind.RESYNC and gap.resync_required
    assert gap.state_patch is gap.attachment_patch is gap.catalog is gap.state is None

    before_reset_sequence = update_world.planning_scene_state().sequence
    update_world.reset((0,))
    reset = update_world.planning_scene_update(before_reset_sequence)
    assert reset.kind is PlanningSceneUpdateKind.RESYNC and reset.resync_required


def test_update_kind_revision_and_payload_closure_rejects_contradictions(update_world) -> None:
    state = update_world.planning_scene_state()
    unchanged = update_world.planning_scene_update(state.sequence)
    with pytest.raises(PlanningSceneContractError, match="changed update sequence"):
        replace(unchanged, kind=PlanningSceneUpdateKind.STATE_PATCH)
    with pytest.raises(PlanningSceneContractError, match="same-sequence unchanged"):
        replace(unchanged, world_revision=unchanged.world_revision + 1)
    with pytest.raises(PlanningSceneContractError, match="non-structural update"):
        replace(unchanged, geometry_revision=unchanged.geometry_revision + 1)
