"""Point constraints remain facts outside the kinematic tree and digest every field."""

from __future__ import annotations

import hashlib
import json
import pickle
from dataclasses import asdict, replace

import pytest

import unirobosim.api.planning_scene as contract
from unirobosim import (
    PLANNING_POINT_CLOSURES_CAPABILITY_ID,
    PLANNING_POINT_CLOSURES_SCHEMA_VERSION,
    PLANNING_SCENE_SCHEMA_VERSION,
    CapabilityDeclaration,
    CapabilityId,
    CapabilityRequirement,
    CapabilitySet,
    PlanningEntityDescriptor,
    PlanningEntityKind,
    PlanningFrameDescriptor,
    PlanningFrameKind,
    PlanningJointDescriptor,
    PlanningJointType,
    PlanningLinkDescriptor,
    PlanningPointClosureDescriptor,
    PlanningSceneCatalog,
    PlanningSceneContractError,
)


def _catalog(point_closures=()):
    # The a->b edge is fixed; closure c->d crosses c,b,d but not common a.
    edges = (
        ("a", "root", PlanningJointType.REVOLUTE),
        ("b", "a", PlanningJointType.FIXED),
        ("c", "b", PlanningJointType.PRISMATIC),
        ("d", "a", PlanningJointType.REVOLUTE),
    )
    entity_id = "robot.one"
    links = [PlanningLinkDescriptor("link.root", entity_id, "root", "frame.root")]
    frames = [
        PlanningFrameDescriptor("frame.world", PlanningFrameKind.WORLD, None, None, None),
        PlanningFrameDescriptor("frame.entity", PlanningFrameKind.ENTITY, "frame.world", entity_id, None),
        PlanningFrameDescriptor("frame.root", PlanningFrameKind.LINK, "frame.entity", entity_id, "link.root"),
    ]
    joints = []
    for child, parent, kind in edges:
        links.append(PlanningLinkDescriptor(f"link.{child}", entity_id, child, f"frame.{child}", f"link.{parent}"))
        frames.append(
            PlanningFrameDescriptor(
                f"frame.{child}", PlanningFrameKind.LINK, f"frame.{parent}", entity_id, f"link.{child}"
            )
        )
        joints.append(
            PlanningJointDescriptor(
                f"joint.{child}",
                entity_id,
                child,
                f"link.{parent}",
                f"link.{child}",
                kind,
                f"frame.{parent}",
                (0.0, 0.0, 1.0),
                "m" if kind is PlanningJointType.PRISMATIC else "rad",
            )
        )
    entity = PlanningEntityDescriptor(
        entity_id,
        "/robot",
        PlanningEntityKind.ROBOT,
        True,
        "frame.entity",
        tuple(sorted(link.link_id for link in links)),
        tuple(sorted(frame.frame_id for frame in frames if frame.kind is not PlanningFrameKind.WORLD)),
        (),
        tuple(joint.joint_id for joint in joints),
    )
    return PlanningSceneCatalog.build(
        "reference.fake",
        "point-closure",
        1,
        0,
        1,
        1,
        (entity,),
        tuple(sorted(links, key=lambda item: item.link_id)),
        tuple(joints),
        tuple(sorted(frames, key=lambda item: item.frame_id)),
        (),
        point_closures=point_closures,
    )


def _closure(**changes):
    values = dict(
        closure_id="closure.one",
        entity_id="robot.one",
        authored_name="point closure",
        link_a_id="link.c",
        link_b_id="link.d",
        anchor_a_m=(0.01, 0.02, 0.03),
        anchor_b_m=(-0.01, 0.0, 0.03),
        constraint_sha256="a" * 64,
        coupled_joint_ids=("joint.c", "joint.d"),
    )
    values.update(changes)
    return PlanningPointClosureDescriptor(**values)


def _catalog_payload(catalog):
    return contract._catalog_content(
        schema_version=catalog.schema_version,
        provider_id=catalog.provider_id,
        world_id=catalog.world_id,
        generation=catalog.generation,
        environment_index=catalog.environment_index,
        catalog_revision=catalog.catalog_revision,
        geometry_revision=catalog.geometry_revision,
        entities=catalog.entities,
        links=catalog.links,
        joints=catalog.joints,
        frames_=catalog.frames,
        geometries=catalog.geometries,
        point_closures=catalog.point_closures,
    )


def test_empty_default_retains_v2_bytes_and_positional_constructor_compatibility():
    catalog = _catalog()
    assert catalog.schema_version == PLANNING_SCENE_SCHEMA_VERSION
    assert catalog.point_closures == ()
    payload = _catalog_payload(catalog)
    assert "point_closures" not in payload
    explicit_empty = replace(catalog, point_closures=())
    assert explicit_empty.content_sha256 == catalog.content_sha256
    positional = PlanningSceneCatalog(
        catalog.provider_id,
        catalog.world_id,
        catalog.generation,
        catalog.environment_index,
        catalog.catalog_revision,
        catalog.geometry_revision,
        catalog.content_sha256,
        catalog.entities,
        catalog.links,
        catalog.joints,
        catalog.frames,
        catalog.geometries,
        catalog.schema_version,
    )
    assert positional == catalog


def test_nonempty_schema_v3_and_constraint_fact_roundtrip():
    closure = _closure()
    catalog = _catalog((closure,))
    assert catalog.schema_version == PLANNING_POINT_CLOSURES_SCHEMA_VERSION
    assert catalog.schema_version != PLANNING_SCENE_SCHEMA_VERSION
    assert len(catalog.joints) == 4  # a closure is never inserted into the tree.
    payload = _catalog_payload(catalog)
    assert payload["point_closures"] == [
        asdict(closure)
        | {
            "anchor_a_m": list(closure.anchor_a_m),
            "anchor_b_m": list(closure.anchor_b_m),
            "coupled_joint_ids": list(closure.coupled_joint_ids),
        }
    ]
    assert (
        hashlib.sha256(
            json.dumps(payload, sort_keys=True, ensure_ascii=False, separators=(",", ":")).encode()
        ).hexdigest()
        == catalog.content_sha256
    )
    # Backend worker transport uses trusted pickle. Reconstruct to rerun public validation.
    transported = pickle.loads(pickle.dumps(catalog))
    validated = replace(transported, point_closures=tuple(replace(item) for item in transported.point_closures))
    assert validated == catalog
    assert (
        _closure(
            **json.loads(json.dumps(asdict(closure)))
            | {
                "anchor_a_m": closure.anchor_a_m,
                "anchor_b_m": closure.anchor_b_m,
                "coupled_joint_ids": closure.coupled_joint_ids,
            }
        )
        == closure
    )


@pytest.mark.parametrize(
    "changes",
    [
        {"closure_id": ""},
        {"link_b_id": "link.c"},
        {"anchor_a_m": [0.0, 0.0, 0.0]},
        {"anchor_a_m": (0.0, True, 0.0)},
        {"anchor_a_m": (0.0, float("nan"), 0.0)},
        {"anchor_b_m": (0.0, 0.0)},
        {"constraint_sha256": "A" * 64},
        {"constraint_sha256": None},
        {"coupled_joint_ids": ["joint.c", "joint.d"]},
        {"coupled_joint_ids": ("joint.d", "joint.c")},
        {"coupled_joint_ids": ("joint.c", "joint.c")},
        {"content_sha256": "0" * 64},
    ],
)
def test_descriptor_rejects_invalid_and_tampered_values(changes):
    with pytest.raises(PlanningSceneContractError):
        _closure(**changes)


@pytest.mark.parametrize(
    "field,value",
    [
        ("closure_id", "closure.other"),
        ("entity_id", "robot.other"),
        ("authored_name", "other"),
        ("link_a_id", "link.b"),
        ("link_b_id", "link.root"),
        ("anchor_a_m", (0.02, 0.02, 0.03)),
        ("anchor_b_m", (0.02, 0.02, 0.03)),
        ("constraint_sha256", "b" * 64),
        ("coupled_joint_ids", ("joint.c",)),
    ],
)
def test_each_constraint_fact_participates_in_descriptor_digest(field, value):
    original = _closure()
    with pytest.raises(PlanningSceneContractError, match="content_sha256"):
        replace(original, **{field: value})
    changed = replace(original, **{field: value}, content_sha256="")
    assert changed.content_sha256 != original.content_sha256


@pytest.mark.parametrize(
    "changes",
    [
        {"entity_id": "robot.absent"},
        {"link_a_id": "link.absent"},
        {"coupled_joint_ids": ()},
        {"coupled_joint_ids": ("joint.c",)},
        {"coupled_joint_ids": ("joint.b", "joint.c", "joint.d")},  # fixed must not be coupled
        {"coupled_joint_ids": ("joint.a", "joint.c", "joint.d")},  # shared ancestor must not be coupled
        {"coupled_joint_ids": ("joint.c", "joint.unknown")},
        {"closure_id": "joint.a"},
    ],
)
def test_catalog_rejects_ownership_or_inexact_coupled_path(changes):
    with pytest.raises(PlanningSceneContractError, match="point closure"):
        _catalog((_closure(**changes),))


def test_all_fixed_path_has_no_coupled_motion_coordinates():
    closure = _closure(link_a_id="link.a", link_b_id="link.b", coupled_joint_ids=())
    assert _catalog((closure,)).point_closures == (closure,)


def test_unknown_or_legacy_schema_cannot_silently_discard_closures():
    catalog = _catalog((_closure(),))
    for version in (PLANNING_SCENE_SCHEMA_VERSION, "unirobosim.planning-scene/v999"):
        with pytest.raises(PlanningSceneContractError, match="schema"):
            replace(catalog, schema_version=version)
    with pytest.raises(PlanningSceneContractError, match="content_sha256"):
        replace(catalog, point_closures=())


def test_constraint_change_requires_new_catalog_digest_and_distinct_ids_sorted():
    closure = _closure()
    catalog = _catalog((closure,))
    changed = replace(closure, constraint_sha256="b" * 64, content_sha256="")
    with pytest.raises(PlanningSceneContractError, match="content_sha256"):
        replace(catalog, point_closures=(changed,))
    assert _catalog((changed,)).content_sha256 != catalog.content_sha256
    other = replace(closure, closure_id="closure.two", content_sha256="")
    for values in ((closure, closure), (other, closure)):
        with pytest.raises(PlanningSceneContractError):
            _catalog(values)


def test_capability_is_explicit_read_knowledge_not_solver_support():
    requirement = CapabilityRequirement(CapabilityId(PLANNING_POINT_CLOSURES_CAPABILITY_ID))
    legacy = CapabilitySet((CapabilityDeclaration(CapabilityId("planning.scene@2")),))
    assert not legacy.negotiate((requirement,)).accepted
    capable = CapabilitySet((*legacy, CapabilityDeclaration(requirement.capability)))
    assert capable.negotiate((requirement,)).accepted
    assert PLANNING_POINT_CLOSURES_CAPABILITY_ID == "scene.point_closures.read@1"


def test_traversal_work_is_bounded(monkeypatch):
    monkeypatch.setattr(contract, "_MAX_RELATIONSHIP_REFERENCES", 30)
    # 30 is enough for the descriptor/reference census, but not many repeated paths.
    closures = tuple(_closure(closure_id=f"closure.{i:02}") for i in range(5))
    with pytest.raises(PlanningSceneContractError, match="budget"):
        _catalog(closures)


def test_existing_state_cannot_bind_changed_closure_facts_and_structural_delta_preserves_them():
    from tests.test_planning_scene import planning_spec
    from tests.test_planning_scene_validation import _structural_delta
    from unirobosim.testing import FakeProvider

    session = FakeProvider().open()
    try:
        world = session.build(planning_spec())
        before_catalog = world.planning_scene_catalog()
        before_state = world.planning_scene_state()
        entity = next(item for item in before_catalog.entities if item.path == "/robot")
        joints = tuple(item for item in before_catalog.joints if item.entity_id == entity.entity_id)
        by_parent = {item.parent_link_id: item for item in joints}
        child_links = {item.child_link_id for item in joints}
        root = next(item.parent_link_id for item in joints if item.parent_link_id not in child_links)
        current = root
        path = []
        while current in by_parent:
            joint = by_parent[current]
            path.append(joint.joint_id)
            current = joint.child_link_id
        closure = _closure(
            entity_id=entity.entity_id, link_a_id=root, link_b_id=current, coupled_joint_ids=tuple(sorted(path))
        )
        v3 = replace(
            before_catalog,
            schema_version=PLANNING_POINT_CLOSURES_SCHEMA_VERSION,
            point_closures=(closure,),
            content_sha256="",
        )
        with pytest.raises(PlanningSceneContractError, match="envelopes"):
            before_state.validate_against(v3)
        state = replace(before_state, catalog_content_sha256=v3.content_sha256)
        state.validate_against(v3)
        delta = _structural_delta(v3, state)
        transported = pickle.loads(pickle.dumps(delta))
        advanced = transported.apply(v3, state)
        assert advanced is not None
        assert advanced[0].point_closures == (closure,)
        assert advanced[0].schema_version == PLANNING_POINT_CLOSURES_SCHEMA_VERSION
        assert advanced[1].catalog_content_sha256 == advanced[0].content_sha256
    finally:
        session.close()


def test_same_entity_but_disconnected_rigid_roots_cannot_form_a_tree_path():
    catalog = _catalog()
    extra_link = PlanningLinkDescriptor("link.z", "robot.one", "z", "frame.z")
    extra_frame = PlanningFrameDescriptor("frame.z", PlanningFrameKind.LINK, "frame.entity", "robot.one", "link.z")
    entity = replace(
        catalog.entities[0],
        link_ids=(*catalog.entities[0].link_ids, "link.z"),
        frame_ids=(*catalog.entities[0].frame_ids, "frame.z"),
    )
    with pytest.raises(PlanningSceneContractError, match="unique connected tree path"):
        replace(
            catalog,
            entities=(entity,),
            links=(*catalog.links, extra_link),
            frames=tuple(sorted((*catalog.frames, extra_frame), key=lambda item: item.frame_id)),
            point_closures=(_closure(link_b_id="link.z", coupled_joint_ids=()),),
            schema_version=PLANNING_POINT_CLOSURES_SCHEMA_VERSION,
            content_sha256="",
        )


def test_empty_v2_catalog_bytes_match_prechange_protocol4_and5_fixtures():
    catalog = PlanningSceneCatalog.build(
        "reference.fake",
        "legacy",
        1,
        0,
        1,
        1,
        (),
        (),
        (),
        (PlanningFrameDescriptor("frame.world", PlanningFrameKind.WORLD, None, None, None),),
        (),
    )
    # Pinned from an independent prechange Core source import, not this implementation.
    assert catalog.content_sha256 == "5a9f17e762fdd8d4b13419393074f7c05146535d5b70331db76fee21546ca014"
    expected = {
        4: "81bfb1feb67fc784451b24a896948fd16adc48dfa16eb026dbf84274923c6b57",
        5: "ad1ea5f18a0e04a76d3db78a060eaa41b9298c749decbdb6f757973596a2ea19",
    }
    for protocol, digest in expected.items():
        raw = pickle.dumps(catalog, protocol=protocol)
        assert hashlib.sha256(raw).hexdigest() == digest
        loaded = pickle.loads(raw)
        assert loaded == catalog
        assert loaded.point_closures == ()


def test_transport_revalidates_descriptor_and_catalog_hashes():
    closure = _closure()
    object.__setattr__(closure, "anchor_a_m", (99.0, 0.0, 0.0))
    with pytest.raises(PlanningSceneContractError, match="content_sha256"):
        pickle.loads(pickle.dumps(closure))
    catalog = _catalog((_closure(),))
    object.__setattr__(catalog, "point_closures", (_closure(constraint_sha256="b" * 64),))
    with pytest.raises(PlanningSceneContractError, match="content_sha256"):
        pickle.loads(pickle.dumps(catalog))
    catalog = _catalog((_closure(),))
    object.__setattr__(catalog, "point_closures", None)
    with pytest.raises(PlanningSceneContractError, match="immutable tuple"):
        pickle.loads(pickle.dumps(catalog))


def test_transport_rejects_unknown_pickle_field_shape():
    catalog = _catalog()
    with pytest.raises(PlanningSceneContractError, match="pickle state"):
        catalog.__setstate__([None])
