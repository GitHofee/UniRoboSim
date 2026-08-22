from __future__ import annotations

import math
from dataclasses import replace

import pytest

import unirobosim.api.planning_scene as planning_contract
from tests.test_planning_scene import entity_by_path, planning_spec
from unirobosim import (
    PLANNING_FRAME_DECLARATIONS_SCHEMA_VERSION,
    PLANNING_SYSTEM_ENTITY_PATH,
    FrozenMap,
    PlanningAttachment,
    PlanningCompoundGeometry,
    PlanningCompoundPart,
    PlanningEntityDescriptor,
    PlanningEntityKind,
    PlanningEntityState,
    PlanningFrameDeclaration,
    PlanningFrameDescriptor,
    PlanningFrameKind,
    PlanningFrameRole,
    PlanningFrameSourceKind,
    PlanningFrameState,
    PlanningGeometryContentProfile,
    PlanningGeometryDType,
    PlanningGeometryMotionClass,
    PlanningGeometryRepresentation,
    PlanningGeometryResourceDescriptor,
    PlanningGeometryResourceLayout,
    PlanningGeometryStorageKind,
    PlanningGeometryTransform,
    PlanningJointType,
    PlanningLinkState,
    PlanningPose,
    PlanningPrimitiveGeometry,
    PlanningSceneCatalog,
    PlanningSceneContractError,
    PlanningSceneDelta,
    PlanningSceneDeltaContinuityError,
    PlanningSceneDeltaKind,
    PlanningSceneIncompleteError,
    PlanningSceneStaleGenerationError,
    PlanningSceneState,
    Tick,
    parse_planning_frame_declarations,
)
from unirobosim.testing import FakeProvider


@pytest.fixture
def planning_values():
    session = FakeProvider().open()
    world = session.build(planning_spec())
    try:
        yield world, world.planning_scene_catalog(), world.planning_scene_state()
    finally:
        session.close()


def test_scalar_container_enum_and_hash_validation_is_closed(planning_values) -> None:
    _, catalog, state = planning_values
    entity = entity_by_path(catalog, "/payload")
    primitive = next(
        geometry.inline
        for geometry in catalog.geometries
        if geometry.representation is PlanningGeometryRepresentation.BOX
    )
    assert type(primitive) is PlanningPrimitiveGeometry
    failures = (
        lambda: PlanningPose(object(), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        lambda: PlanningPose("", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        lambda: PlanningPose("frame.\ud800", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        lambda: PlanningPose("四" * 400, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        lambda: PlanningPose("bad frame", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        lambda: PlanningPose("frame.safe", (0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        lambda: PlanningPose("frame.safe", (math.nan, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        lambda: PlanningPose("frame.safe", (object(), 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        lambda: replace(entity, enabled=1),
        lambda: replace(entity, root_frame_id="frame.missing"),
        lambda: replace(entity, link_ids=[]),
        lambda: replace(entity, link_ids=("link.z", "link.a")),
        lambda: replace(entity, link_ids=("link.a", "link.a")),
        lambda: replace(state, generation=True),
        lambda: replace(state, generation=0),
        lambda: replace(state, tick=object()),
        lambda: PlanningPrimitiveGeometry("unsupported", (1.0,)),
        lambda: replace(primitive, representation=PlanningGeometryRepresentation.TRIANGLE_MESH),
        lambda: replace(primitive, dimensions_m=(0.0, 1.0, 1.0)),
    )
    for construct in failures:
        with pytest.raises(PlanningSceneContractError):
            construct()


def test_quaternion_canonicalization_and_compound_validation(planning_values) -> None:
    _, catalog, _ = planning_values
    negative_w = PlanningPose("frame.safe", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, -1.0))
    negative_axis = PlanningPose("frame.safe", (0.0, 0.0, 0.0), (-1.0, 0.0, 0.0, 0.0))
    assert negative_w.orientation_xyzw == (0.0, 0.0, 0.0, 1.0)
    assert negative_axis.orientation_xyzw == (1.0, 0.0, 0.0, 0.0)
    with pytest.raises(PlanningSceneContractError):
        PlanningPose("frame.safe", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0))
    for representation, dimensions in (
        (PlanningGeometryRepresentation.BOX, (1.0, 2.0, 3.0)),
        (PlanningGeometryRepresentation.SPHERE, (1.0,)),
        (PlanningGeometryRepresentation.CAPSULE, (0.1, 0.5)),
        (PlanningGeometryRepresentation.CYLINDER, (0.1, 0.5)),
    ):
        assert PlanningPrimitiveGeometry(representation, dimensions).representation is representation
    compound = next(
        geometry.inline
        for geometry in catalog.geometries
        if geometry.representation is PlanningGeometryRepresentation.COMPOUND
    )
    assert type(compound) is PlanningCompoundGeometry
    part = compound.parts[0]
    for construct in (
        lambda: PlanningCompoundPart("part.invalid", object(), part.primitive),
        lambda: PlanningCompoundPart("part.invalid", part.local_pose, object()),
        lambda: PlanningCompoundGeometry(()),
        lambda: PlanningCompoundGeometry((part, part)),
        lambda: PlanningCompoundGeometry(tuple(reversed(compound.parts))),
    ):
        with pytest.raises(PlanningSceneContractError):
            construct()


def test_link_joint_and_frame_local_invariants(planning_values) -> None:
    _, catalog, _ = planning_values
    link = catalog.links[0]
    joint = catalog.joints[0]
    owned_frame = next(frame for frame in catalog.frames if frame.owner_entity_id is not None)
    with pytest.raises(PlanningSceneContractError):
        replace(link, parent_link_id=link.link_id)
    invalid_joints = (
        {"child_link_id": joint.parent_link_id},
        {"axis_xyz": (0.0, 0.0, 2.0)},
        {"position_unit": "m"},
        {"lower": -1.0, "upper": None},
        {"lower": 1.0, "upper": -1.0},
        {"max_velocity": 0.0},
        {"max_effort": -1.0},
    )
    for override in invalid_joints:
        with pytest.raises(PlanningSceneContractError):
            replace(joint, **override)
    prismatic = replace(
        joint,
        joint_type=PlanningJointType.PRISMATIC,
        position_unit="m",
        lower=-0.5,
        upper=0.5,
        max_velocity=1.0,
        max_effort=2.0,
    )
    assert prismatic.position_unit == "m" and prismatic.lower == -0.5
    for construct in (
        lambda: PlanningFrameDescriptor("frame.bad", PlanningFrameKind.WORLD, "frame.world", None, None),
        lambda: PlanningFrameDescriptor("frame.bad", PlanningFrameKind.NAMED, None, None, None),
        lambda: replace(owned_frame, parent_frame_id=owned_frame.frame_id),
    ):
        with pytest.raises(PlanningSceneContractError):
            construct()


def test_planning_frame_declaration_projection_is_canonical_and_fail_closed() -> None:
    valid = FrozenMap(
        {
            "schema": PLANNING_FRAME_DECLARATIONS_SCHEMA_VERSION,
            "component_sha256": "b" * 64,
            "entries": (
                {
                    "semantic_key": "ee.left",
                    "role": "ee",
                    "owner_link": "left wrist",
                    "source": {"kind": "link", "name": "left wrist"},
                },
                {
                    "semantic_key": "tool.tcp",
                    "role": "tool",
                    "owner_link": "tool flange",
                    "source": {"kind": "native_named", "name": "tool0"},
                },
            ),
        }
    )
    declarations = parse_planning_frame_declarations(valid)
    assert declarations is not None
    assert declarations.schema_version == PLANNING_FRAME_DECLARATIONS_SCHEMA_VERSION
    assert declarations.component_sha256 == "b" * 64
    assert tuple(item.semantic_key for item in declarations.entries) == ("ee.left", "tool.tcp")
    assert declarations.entries[0].source.kind is PlanningFrameSourceKind.LINK
    assert declarations.entries[1].source.kind is PlanningFrameSourceKind.NATIVE_NAMED
    assert parse_planning_frame_declarations(None) is None

    invalid = (
        object(),
        FrozenMap({"schema": PLANNING_FRAME_DECLARATIONS_SCHEMA_VERSION}),
        FrozenMap(
            {
                "schema": "unirobosim.planning-frame-declarations/v2",
                "component_sha256": "b" * 64,
                "entries": (),
            }
        ),
        FrozenMap(
            {
                "schema": PLANNING_FRAME_DECLARATIONS_SCHEMA_VERSION,
                "component_sha256": "not-a-digest",
                "entries": (),
            }
        ),
        FrozenMap(
            {
                "schema": PLANNING_FRAME_DECLARATIONS_SCHEMA_VERSION,
                "component_sha256": "b" * 64,
                "entries": "not-a-tuple",
            }
        ),
        FrozenMap(
            {
                "schema": PLANNING_FRAME_DECLARATIONS_SCHEMA_VERSION,
                "component_sha256": "b" * 64,
                "entries": ("not-a-record",),
            }
        ),
        FrozenMap(
            {
                "schema": PLANNING_FRAME_DECLARATIONS_SCHEMA_VERSION,
                "component_sha256": "b" * 64,
                "entries": (
                    {
                        "semantic_key": "tool.bad-source",
                        "role": "tool",
                        "owner_link": "base",
                        "source": "not-a-record",
                    },
                ),
            }
        ),
        FrozenMap(
            {
                "schema": PLANNING_FRAME_DECLARATIONS_SCHEMA_VERSION,
                "component_sha256": "b" * 64,
                "entries": tuple(reversed(valid["entries"])),
            }
        ),
        FrozenMap(
            {
                "schema": PLANNING_FRAME_DECLARATIONS_SCHEMA_VERSION,
                "component_sha256": "b" * 64,
                "entries": (
                    {
                        "semantic_key": "bad",
                        "role": "camera",
                        "owner_link": None,
                        "source": {"kind": "link", "name": "base"},
                    },
                ),
            }
        ),
    )
    for value in invalid:
        with pytest.raises(PlanningSceneIncompleteError) as caught:
            parse_planning_frame_declarations(value)
        assert caught.value.operation == "planning_scene.preflight"
    with pytest.raises(PlanningSceneContractError, match="exact source"):
        PlanningFrameDeclaration("tool.bad", PlanningFrameRole.TOOL, "base", object())


def test_additional_v2_scalar_halfspace_catalog_and_named_frame_edges(planning_values) -> None:
    _, catalog, _ = planning_values
    with pytest.raises(PlanningSceneContractError, match="opaque scoped identifier"):
        planning_contract._text("bad space", "opaque", opaque=True)
    with pytest.raises(PlanningSceneContractError, match="non-negative"):
        planning_contract._finite(-1.0, "non-negative", non_negative=True)
    with pytest.raises(PlanningSceneContractError, match="invalid value"):
        planning_contract._typed_tuple((object(),), PlanningPose, "typed tuple")
    with pytest.raises(PlanningSceneContractError, match="non-portable"):
        planning_contract._portable_payload(object())
    with pytest.raises(PlanningSceneContractError, match="schema version"):
        replace(catalog, schema_version="unirobosim.planning-scene/v1", content_sha256="")

    system = entity_by_path(catalog, PLANNING_SYSTEM_ENTITY_PATH)
    ground = next(geometry for geometry in catalog.geometries if geometry.owner_entity_id == system.entity_id)
    with pytest.raises(PlanningSceneContractError, match="unit scale"):
        replace(ground, scale=(1.0, 1.0, 2.0))
    with pytest.raises(PlanningSceneContractError, match="system geometry"):
        _rebuild_catalog(
            catalog,
            geometries=_replace_sorted(
                catalog.geometries,
                ground,
                replace(ground, motion_class=PlanningGeometryMotionClass.DYNAMIC),
                "geometry_id",
            ),
        )

    spoofed_path = replace(system, path="/system/not-effective")
    with pytest.raises(PlanningSceneContractError, match="identity and path"):
        _rebuild_catalog(
            catalog,
            entities=_replace_sorted(catalog.entities, system, spoofed_path, "entity_id"),
        )
    invalid_system_shape = replace(system, enabled=False)
    with pytest.raises(PlanningSceneContractError, match="entity shape"):
        _rebuild_catalog(
            catalog,
            entities=_replace_sorted(catalog.entities, system, invalid_system_shape, "entity_id"),
        )

    robot = entity_by_path(catalog, "/robot")
    robot_links = tuple(link for link in catalog.links if link.entity_id == robot.entity_id)
    root_link = next(link for link in robot_links if link.parent_link_id is None)
    child_link = next(link for link in robot_links if link.parent_link_id == root_link.link_id)
    joint = next(item for item in catalog.joints if item.child_link_id == child_link.link_id)
    with pytest.raises(PlanningSceneContractError, match="axis frame"):
        _rebuild_catalog(
            catalog,
            joints=_replace_sorted(
                catalog.joints,
                joint,
                replace(joint, axis_frame_id=robot.root_frame_id),
                "joint_id",
            ),
        )

    unknown_parent = PlanningFrameDescriptor(
        "frame.robot.unknown-parent",
        PlanningFrameKind.NAMED,
        "frame.unknown",
        robot.entity_id,
        None,
        PlanningFrameRole.ANNOTATION,
        "annotation.unknown-parent",
    )
    robot_with_unknown_parent = replace(robot, frame_ids=tuple(sorted((*robot.frame_ids, unknown_parent.frame_id))))
    with pytest.raises(PlanningSceneContractError, match="parent is unknown"):
        _rebuild_catalog(
            catalog,
            entities=_replace_sorted(catalog.entities, robot, robot_with_unknown_parent, "entity_id"),
            frames=tuple(sorted((*catalog.frames, unknown_parent), key=lambda item: item.frame_id)),
        )

    entity_owned_on_link = PlanningFrameDescriptor(
        "frame.robot.entity-owned-on-link",
        PlanningFrameKind.NAMED,
        root_link.frame_id,
        robot.entity_id,
        None,
        PlanningFrameRole.ANNOTATION,
        "annotation.entity-owned-on-link",
    )
    robot_with_entity_owned = replace(
        robot,
        frame_ids=tuple(sorted((*robot.frame_ids, entity_owned_on_link.frame_id))),
    )
    with pytest.raises(PlanningSceneContractError, match="entity root"):
        _rebuild_catalog(
            catalog,
            entities=_replace_sorted(catalog.entities, robot, robot_with_entity_owned, "entity_id"),
            frames=tuple(sorted((*catalog.frames, entity_owned_on_link), key=lambda item: item.frame_id)),
        )

    link_owned_on_wrong_link = PlanningFrameDescriptor(
        "frame.robot.link-owned-on-wrong-link",
        PlanningFrameKind.NAMED,
        child_link.frame_id,
        robot.entity_id,
        root_link.link_id,
        PlanningFrameRole.TOOL,
        "tool.wrong-link",
    )
    robot_with_link_owned = replace(
        robot,
        frame_ids=tuple(sorted((*robot.frame_ids, link_owned_on_wrong_link.frame_id))),
    )
    with pytest.raises(PlanningSceneContractError, match="owner link frame"):
        _rebuild_catalog(
            catalog,
            entities=_replace_sorted(catalog.entities, robot, robot_with_link_owned, "entity_id"),
            frames=tuple(sorted((*catalog.frames, link_owned_on_wrong_link), key=lambda item: item.frame_id)),
        )

    extra_entity_frame = PlanningFrameDescriptor(
        "frame.robot.extra-entity",
        PlanningFrameKind.ENTITY,
        catalog.world_frame_id,
        robot.entity_id,
        None,
    )
    robot_with_extra_entity = replace(
        robot,
        frame_ids=tuple(sorted((*robot.frame_ids, extra_entity_frame.frame_id))),
    )
    with pytest.raises(PlanningSceneContractError, match="correspond exactly"):
        _rebuild_catalog(
            catalog,
            entities=_replace_sorted(catalog.entities, robot, robot_with_extra_entity, "entity_id"),
            frames=tuple(sorted((*catalog.frames, extra_entity_frame), key=lambda item: item.frame_id)),
        )

    extra_joint_frame = PlanningFrameDescriptor(
        "frame.robot.extra-joint",
        PlanningFrameKind.JOINT,
        root_link.frame_id,
        robot.entity_id,
        root_link.link_id,
    )
    robot_with_extra_joint = replace(
        robot,
        frame_ids=tuple(sorted((*robot.frame_ids, extra_joint_frame.frame_id))),
    )
    with pytest.raises(PlanningSceneContractError, match="joint frames must correspond"):
        _rebuild_catalog(
            catalog,
            entities=_replace_sorted(catalog.entities, robot, robot_with_extra_joint, "entity_id"),
            frames=tuple(sorted((*catalog.frames, extra_joint_frame), key=lambda item: item.frame_id)),
        )

    named = next(
        frame
        for frame in catalog.frames
        if frame.owner_entity_id == robot.entity_id and frame.kind is PlanningFrameKind.NAMED
    )
    duplicate_named = PlanningFrameDescriptor(
        "frame.robot.duplicate-semantic",
        PlanningFrameKind.NAMED,
        named.parent_frame_id,
        robot.entity_id,
        named.owner_link_id,
        named.role,
        named.semantic_key,
    )
    robot_with_duplicate_named = replace(
        robot,
        frame_ids=tuple(sorted((*robot.frame_ids, duplicate_named.frame_id))),
    )
    with pytest.raises(PlanningSceneContractError, match="semantic keys"):
        _rebuild_catalog(
            catalog,
            entities=_replace_sorted(catalog.entities, robot, robot_with_duplicate_named, "entity_id"),
            frames=tuple(sorted((*catalog.frames, duplicate_named), key=lambda item: item.frame_id)),
        )


def test_geometry_descriptor_local_and_resource_identity_invariants(planning_values) -> None:
    _, catalog, _ = planning_values
    inline = next(
        geometry for geometry in catalog.geometries if geometry.representation is PlanningGeometryRepresentation.BOX
    )
    resource = next(
        geometry
        for geometry in catalog.geometries
        if geometry.representation is PlanningGeometryRepresentation.TRIANGLE_MESH
    )
    assert inline.resolution_key is None
    assert resource.resolution_key == (resource.geometry_id, resource.representation, resource.sha256)
    failures = (
        lambda: replace(inline, parent_frame_T_geometry=object()),
        lambda: replace(inline, scale=(1.0, 0.0, 1.0)),
        lambda: replace(inline, provenance_sha256="BAD"),
        lambda: replace(inline, inline=None),
        lambda: replace(inline, resource_id="resource.bad", sha256="0" * 64),
        lambda: replace(inline, representation=PlanningGeometryRepresentation.COMPOUND),
        lambda: replace(
            inline,
            representation=PlanningGeometryRepresentation.SPHERE,
            inline=PlanningPrimitiveGeometry(PlanningGeometryRepresentation.BOX, (1.0, 1.0, 1.0)),
        ),
        lambda: replace(resource, resource_id=None),
        lambda: replace(resource, inline=inline.inline),
        lambda: replace(resource, content_profile="arbitrary.profile"),
        lambda: replace(
            resource,
            resource_layout=replace(
                resource.resource_layout,
                representation=PlanningGeometryRepresentation.CONVEX_MESH,
            ),
        ),
    )
    for construct in failures:
        with pytest.raises(PlanningSceneContractError):
            construct()


def test_catalog_rejects_alternative_representation_under_the_same_geometry_id(planning_values) -> None:
    _, catalog, _ = planning_values
    original = next(
        geometry for geometry in catalog.geometries if geometry.representation is PlanningGeometryRepresentation.BOX
    )
    alternative = replace(
        original,
        representation=PlanningGeometryRepresentation.SPHERE,
        inline=PlanningPrimitiveGeometry(PlanningGeometryRepresentation.SPHERE, (0.5,)),
    )
    geometries = tuple(sorted((*catalog.geometries, alternative), key=lambda item: item.geometry_id))
    with pytest.raises(PlanningSceneContractError, match="unique"):
        replace(catalog, content_sha256="", geometries=geometries)


def test_value_state_and_attachment_local_invariants(planning_values) -> None:
    _, _, state = planning_values
    entity = state.entities[0]
    link = state.links[0]
    frame = state.frames[0]
    transform = state.geometry_transforms[0]
    articulation = state.articulations[0]
    attachment = state.attachments[0]
    other_pose = replace(entity.pose, frame_id="frame.other")
    failures = (
        lambda: PlanningEntityState(entity.entity_id, object(), entity.twist),
        lambda: replace(entity, twist=replace(entity.twist, frame_id="frame.other")),
        lambda: PlanningLinkState(link.link_id, object(), link.twist),
        lambda: replace(link, twist=replace(link.twist, frame_id="frame.other")),
        lambda: PlanningFrameState(frame.frame_id, object()),
        lambda: PlanningGeometryTransform(transform.geometry_id, object()),
        lambda: replace(articulation, positions=[]),
        lambda: replace(articulation, positions=()),
        lambda: replace(articulation, position_units=("degree",) * len(articulation.joint_ids)),
        lambda: replace(attachment, parent_T_child=other_pose),
        lambda: replace(attachment, geometry_ids=()),
    )
    for construct in failures:
        with pytest.raises(PlanningSceneContractError):
            construct()
    with pytest.raises(PlanningSceneContractError):
        replace(
            attachment,
            child_frame_id=attachment.parent_frame_id,
            parent_T_child=PlanningPose(
                attachment.parent_frame_id,
                (0.1, 0.0, 0.0),
                (0.0, 0.0, 0.0, 1.0),
            ),
        )


def test_constructor_and_method_generic_exception_boundaries_are_typed(monkeypatch, planning_values) -> None:
    world, catalog, _ = planning_values
    resource_id = next(
        geometry.geometry_id
        for geometry in catalog.geometries
        if geometry.representation is PlanningGeometryRepresentation.TRIANGLE_MESH
    )
    lease = world.resolve_planning_geometry(resource_id)

    def unexpected(*args, **kwargs):
        del args, kwargs
        raise RuntimeError("must not escape")

    monkeypatch.setattr(planning_contract, "_invalid", unexpected)
    with pytest.raises(PlanningSceneContractError, match="value validation failed"):
        PlanningPose("", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    with pytest.raises(PlanningSceneContractError, match="value validation failed"):
        lease.descriptor.read_span(-1)
    lease.close()


def test_resource_mesh_and_grid_schema_negative_matrix(planning_values) -> None:
    world, catalog, _ = planning_values
    geometry_id = next(
        geometry.geometry_id
        for geometry in catalog.geometries
        if geometry.representation is PlanningGeometryRepresentation.TRIANGLE_MESH
    )
    lease = world.resolve_planning_geometry(geometry_id)
    mesh = lease.descriptor
    mesh_failures = (
        {"representation": PlanningGeometryRepresentation.BOX},
        {"units": "cm"},
        {"layout": replace(mesh.layout, representation=PlanningGeometryRepresentation.CONVEX_MESH)},
        {"byte_size": mesh.byte_size - 1},
    )
    for override in mesh_failures:
        with pytest.raises(PlanningSceneContractError):
            replace(mesh, **override)
    for override in (
        {"vertex_dtype": None},
        {"vertex_dtype": PlanningGeometryDType.UINT8},
        {"vertex_shape": (8,)},
        {"vertex_shape": (8, 2)},
        {"grid_dtype": PlanningGeometryDType.FLOAT32},
    ):
        with pytest.raises(PlanningSceneContractError):
            replace(mesh.layout, **override)
    sdf_layout = PlanningGeometryResourceLayout(
        PlanningGeometryRepresentation.SDF,
        PlanningGeometryContentProfile.SDF_DENSE_RAW_LE_V1,
        grid_dtype=PlanningGeometryDType.FLOAT32,
        grid_shape=(2, 2, 2),
        grid_spacing_m=(0.01, 0.01, 0.01),
        grid_origin_m=(0.0, 0.0, 0.0),
    )
    sdf = PlanningGeometryResourceDescriptor(
        "reference.fake",
        "world.grid",
        1,
        0,
        1,
        1,
        "0" * 64,
        "lease.grid",
        "resource.grid",
        "geometry.grid",
        PlanningGeometryRepresentation.SDF,
        PlanningGeometryStorageKind.IMMUTABLE_MEMORY,
        "cache.grid",
        PlanningGeometryContentProfile.SDF_DENSE_RAW_LE_V1,
        "m",
        mesh.axis_convention,
        sdf_layout,
        32,
        "0" * 64,
    )
    with pytest.raises(PlanningSceneContractError):
        replace(sdf.layout, vertex_dtype=PlanningGeometryDType.FLOAT32)
    assert mesh.resolution_key == (mesh.geometry_id, mesh.representation, mesh.sha256)
    with pytest.raises(PlanningSceneContractError):
        mesh.read_span(mesh.byte_size + 1)
    with pytest.raises(PlanningSceneContractError):
        mesh.read_span(0, mesh.byte_size + 1)
    lease.close()


def _rebuild_catalog(catalog: PlanningSceneCatalog, **changes) -> PlanningSceneCatalog:
    return replace(catalog, content_sha256="", **changes)


def _replace_sorted(values: tuple, original, replacement, attribute: str) -> tuple:
    return tuple(
        sorted(
            (replacement if item is original else item for item in values), key=lambda item: getattr(item, attribute)
        )
    )


def test_catalog_budget_schema_path_world_and_ownership_fail_closed(monkeypatch, planning_values) -> None:
    _, catalog, _ = planning_values
    payload = entity_by_path(catalog, "/payload")
    original_items = planning_contract._MAX_ITEMS
    largest_collection = max(
        len(values) for values in (catalog.entities, catalog.links, catalog.joints, catalog.frames, catalog.geometries)
    )
    monkeypatch.setattr(planning_contract, "_MAX_ITEMS", largest_collection)
    with pytest.raises(PlanningSceneContractError, match="node budget"):
        _rebuild_catalog(catalog)
    monkeypatch.setattr(planning_contract, "_MAX_ITEMS", original_items)
    original_relationships = planning_contract._MAX_RELATIONSHIP_REFERENCES
    monkeypatch.setattr(planning_contract, "_MAX_RELATIONSHIP_REFERENCES", 1)
    with pytest.raises(PlanningSceneContractError, match="relationship budget"):
        _rebuild_catalog(catalog)
    monkeypatch.setattr(planning_contract, "_MAX_RELATIONSHIP_REFERENCES", original_relationships)
    original_bytes = planning_contract._MAX_CANONICAL_CATALOG_BYTES
    monkeypatch.setattr(planning_contract, "_MAX_CANONICAL_CATALOG_BYTES", 1)
    with pytest.raises(PlanningSceneContractError, match="byte budget"):
        _rebuild_catalog(catalog)
    monkeypatch.setattr(planning_contract, "_MAX_CANONICAL_CATALOG_BYTES", original_bytes)

    duplicate_path = PlanningEntityDescriptor(
        "entity.clone",
        payload.path,
        PlanningEntityKind.OTHER,
        True,
        "frame.clone",
        (),
        ("frame.clone",),
        (),
    )
    with pytest.raises(PlanningSceneContractError, match="paths"):
        _rebuild_catalog(
            catalog,
            entities=tuple(sorted((*catalog.entities, duplicate_path), key=lambda item: item.entity_id)),
        )
    with pytest.raises(PlanningSceneContractError, match="exactly one world"):
        _rebuild_catalog(
            catalog,
            frames=tuple(frame for frame in catalog.frames if frame.kind is not PlanningFrameKind.WORLD),
        )
    second_world = PlanningFrameDescriptor("frame.world2", PlanningFrameKind.WORLD, None, None, None)
    with pytest.raises(PlanningSceneContractError, match="exactly one world"):
        _rebuild_catalog(
            catalog,
            frames=tuple(sorted((*catalog.frames, second_world), key=lambda item: item.frame_id)),
        )
    duplicate_owner = replace(
        duplicate_path,
        path="/clone",
        root_frame_id=payload.root_frame_id,
        frame_ids=(payload.root_frame_id,),
    )
    with pytest.raises(PlanningSceneContractError, match="multiple entity owners"):
        _rebuild_catalog(
            catalog,
            entities=tuple(sorted((*catalog.entities, duplicate_owner), key=lambda item: item.entity_id)),
        )
    with pytest.raises(PlanningSceneContractError, match="root frame"):
        _rebuild_catalog(
            catalog,
            frames=tuple(frame for frame in catalog.frames if frame.frame_id != payload.root_frame_id),
        )


def test_catalog_link_joint_and_cycle_reference_validation(planning_values) -> None:
    _, catalog, _ = planning_values
    robot = entity_by_path(catalog, "/robot")
    robot_links = tuple(link for link in catalog.links if link.entity_id == robot.entity_id)
    root = next(link for link in robot_links if link.parent_link_id is None)
    child = next(link for link in robot_links if link.parent_link_id == root.link_id)
    invalid_link_cases = (
        replace(root, entity_id="entity.unknown"),
        replace(child, parent_link_id="link.unknown"),
        replace(root, geometry_ids=tuple(sorted((*root.geometry_ids, "geometry.unknown")))),
    )
    for invalid in invalid_link_cases:
        with pytest.raises(PlanningSceneContractError):
            _rebuild_catalog(
                catalog,
                links=_replace_sorted(
                    catalog.links, root if invalid is not invalid_link_cases[1] else child, invalid, "link_id"
                ),
            )
    cycle_root = replace(root, parent_link_id=child.link_id)
    root_frame = next(frame for frame in catalog.frames if frame.frame_id == root.frame_id)
    cycle_root_frame = replace(root_frame, parent_frame_id=child.frame_id)
    with pytest.raises(PlanningSceneContractError, match="cycle"):
        _rebuild_catalog(
            catalog,
            links=_replace_sorted(catalog.links, root, cycle_root, "link_id"),
            frames=_replace_sorted(catalog.frames, root_frame, cycle_root_frame, "frame_id"),
        )
    joint = next(item for item in catalog.joints if item.entity_id == robot.entity_id)
    with pytest.raises(PlanningSceneContractError, match="joint references"):
        _rebuild_catalog(
            catalog,
            joints=_replace_sorted(catalog.joints, joint, replace(joint, axis_frame_id="frame.unknown"), "joint_id"),
        )


def test_catalog_physical_graph_is_bidirectionally_closed(planning_values) -> None:
    _, catalog, _ = planning_values
    robot = entity_by_path(catalog, "/robot")
    robot_links = tuple(link for link in catalog.links if link.entity_id == robot.entity_id)
    root_link = next(link for link in robot_links if link.parent_link_id is None)
    child_link = next(link for link in robot_links if link.parent_link_id == root_link.link_id)
    primary_frame = next(frame for frame in catalog.frames if frame.frame_id == root_link.frame_id)

    for invalid_frame in (
        replace(primary_frame, owner_link_id=None),
        replace(
            primary_frame,
            kind=PlanningFrameKind.NAMED,
            owner_link_id=None,
            role=PlanningFrameRole.EE,
            semantic_key="invalid-root-link",
        ),
    ):
        with pytest.raises(PlanningSceneContractError, match="link ownership"):
            _rebuild_catalog(
                catalog,
                frames=_replace_sorted(catalog.frames, primary_frame, invalid_frame, "frame_id"),
            )

    child_joint = next(joint for joint in catalog.joints if joint.child_link_id == child_link.link_id)
    child_primary_frame = next(frame for frame in catalog.frames if frame.frame_id == child_link.frame_id)
    with pytest.raises(PlanningSceneContractError, match="link parent"):
        _rebuild_catalog(
            catalog,
            frames=_replace_sorted(
                catalog.frames,
                child_primary_frame,
                replace(child_primary_frame, parent_frame_id=catalog.world_frame_id),
                "frame_id",
            ),
        )

    robot_without_joint = replace(
        robot,
        joint_ids=tuple(joint_id for joint_id in robot.joint_ids if joint_id != child_joint.joint_id),
    )
    with pytest.raises(PlanningSceneContractError, match="correspond exactly"):
        _rebuild_catalog(
            catalog,
            entities=_replace_sorted(catalog.entities, robot, robot_without_joint, "entity_id"),
            joints=tuple(joint for joint in catalog.joints if joint.joint_id != child_joint.joint_id),
        )

    geometry = next(item for item in catalog.geometries if item.owner_link_id == root_link.link_id)
    with pytest.raises(PlanningSceneContractError, match="correspond exactly"):
        _rebuild_catalog(
            catalog,
            geometries=_replace_sorted(
                catalog.geometries,
                geometry,
                replace(geometry, owner_link_id=None),
                "geometry_id",
            ),
        )
    with pytest.raises(PlanningSceneContractError, match="correspond exactly"):
        _rebuild_catalog(
            catalog,
            links=_replace_sorted(
                catalog.links,
                root_link,
                replace(root_link, geometry_ids=()),
                "link_id",
            ),
        )
    with pytest.raises(PlanningSceneContractError, match="link ownership"):
        _rebuild_catalog(
            catalog,
            geometries=_replace_sorted(
                catalog.geometries,
                geometry,
                replace(geometry, parent_frame_id=child_link.frame_id),
                "geometry_id",
            ),
        )

    mount = PlanningFrameDescriptor(
        "frame.robot.mount",
        PlanningFrameKind.NAMED,
        root_link.frame_id,
        robot.entity_id,
        root_link.link_id,
        PlanningFrameRole.TOOL,
        "mount",
    )
    robot_with_mount = replace(robot, frame_ids=tuple(sorted((*robot.frame_ids, mount.frame_id))))
    invalid_named_frames = (
        replace(mount, owner_entity_id="entity.unknown", owner_link_id=None),
        replace(mount, owner_link_id="link.unknown"),
        replace(mount, kind=PlanningFrameKind.LINK, role=None, semantic_key=None),
        replace(
            mount,
            kind=PlanningFrameKind.LINK,
            owner_link_id=None,
            role=None,
            semantic_key=None,
        ),
    )
    for invalid_frame in invalid_named_frames:
        with pytest.raises(PlanningSceneContractError):
            _rebuild_catalog(
                catalog,
                entities=_replace_sorted(catalog.entities, robot, robot_with_mount, "entity_id"),
                frames=tuple(sorted((*catalog.frames, invalid_frame), key=lambda item: item.frame_id)),
            )
    mounted_geometry = replace(geometry, parent_frame_id=mount.frame_id)
    with pytest.raises(PlanningSceneContractError, match="geometry link ownership"):
        _rebuild_catalog(
            catalog,
            entities=_replace_sorted(catalog.entities, robot, robot_with_mount, "entity_id"),
            frames=tuple(sorted((*catalog.frames, mount), key=lambda item: item.frame_id)),
            geometries=_replace_sorted(catalog.geometries, geometry, mounted_geometry, "geometry_id"),
        )
    duplicate_joint = replace(child_joint, joint_id="zzjoint.duplicate")
    robot_with_joint = replace(robot, joint_ids=(*robot.joint_ids, duplicate_joint.joint_id))
    with pytest.raises(PlanningSceneContractError, match="unique child"):
        _rebuild_catalog(
            catalog,
            entities=_replace_sorted(catalog.entities, robot, robot_with_joint, "entity_id"),
            joints=tuple(sorted((*catalog.joints, duplicate_joint), key=lambda item: item.joint_id)),
        )


def test_catalog_requires_exact_root_named_geometry_and_joint_frame_ancestry(planning_values) -> None:
    _, catalog, _ = planning_values
    robot = entity_by_path(catalog, "/robot")
    payload = entity_by_path(catalog, "/payload")
    left = entity_by_path(catalog, "/left")
    payload_root = next(frame for frame in catalog.frames if frame.frame_id == payload.root_frame_id)
    with pytest.raises(PlanningSceneContractError, match="root frame ownership"):
        _rebuild_catalog(
            catalog,
            frames=_replace_sorted(
                catalog.frames,
                payload_root,
                replace(payload_root, parent_frame_id=left.root_frame_id),
                "frame_id",
            ),
        )

    robot_links = tuple(link for link in catalog.links if link.entity_id == robot.entity_id)
    root_link = next(link for link in robot_links if link.parent_link_id is None)
    root_link_frame = next(frame for frame in catalog.frames if frame.frame_id == root_link.frame_id)
    with pytest.raises(PlanningSceneContractError, match="root link frame"):
        _rebuild_catalog(
            catalog,
            frames=_replace_sorted(
                catalog.frames,
                root_link_frame,
                replace(root_link_frame, parent_frame_id=catalog.world_frame_id),
                "frame_id",
            ),
        )

    cross_entity_named = PlanningFrameDescriptor(
        "frame.robot.cross_entity",
        PlanningFrameKind.NAMED,
        left.root_frame_id,
        robot.entity_id,
        None,
        PlanningFrameRole.ANNOTATION,
        "annotation.cross_entity",
    )
    robot_with_cross_entity_frame = replace(
        robot,
        frame_ids=tuple(sorted((*robot.frame_ids, cross_entity_named.frame_id))),
    )
    with pytest.raises(PlanningSceneContractError, match="ancestry crosses"):
        _rebuild_catalog(
            catalog,
            entities=_replace_sorted(catalog.entities, robot, robot_with_cross_entity_frame, "entity_id"),
            frames=tuple(sorted((*catalog.frames, cross_entity_named), key=lambda item: item.frame_id)),
        )

    payload_named = PlanningFrameDescriptor(
        "frame.payload.annotation",
        PlanningFrameKind.NAMED,
        payload.root_frame_id,
        payload.entity_id,
        None,
        PlanningFrameRole.ANNOTATION,
        "annotation.payload",
    )
    payload_with_named_frame = replace(
        payload,
        frame_ids=tuple(sorted((*payload.frame_ids, payload_named.frame_id))),
    )
    payload_geometry = next(
        geometry for geometry in catalog.geometries if geometry.owner_entity_id == payload.entity_id
    )
    payload_link = next(link for link in catalog.links if link.entity_id == payload.entity_id)
    with pytest.raises(PlanningSceneContractError, match="exact entity root"):
        _rebuild_catalog(
            catalog,
            entities=_replace_sorted(catalog.entities, payload, payload_with_named_frame, "entity_id"),
            frames=tuple(sorted((*catalog.frames, payload_named), key=lambda item: item.frame_id)),
            links=_replace_sorted(
                catalog.links,
                payload_link,
                replace(
                    payload_link,
                    geometry_ids=tuple(
                        item for item in payload_link.geometry_ids if item != payload_geometry.geometry_id
                    ),
                ),
                "link_id",
            ),
            geometries=_replace_sorted(
                catalog.geometries,
                payload_geometry,
                replace(
                    payload_geometry,
                    owner_link_id=None,
                    parent_frame_id=payload_named.frame_id,
                ),
                "geometry_id",
            ),
        )

    joint = catalog.joints[0]
    joint_entity = next(entity for entity in catalog.entities if entity.entity_id == joint.entity_id)
    unrelated_link = next(
        link
        for link in catalog.links
        if link.entity_id == joint.entity_id and link.link_id not in {joint.parent_link_id, joint.child_link_id}
    )
    unrelated_axis = PlanningFrameDescriptor(
        "frame.robot.unrelated_joint_axis",
        PlanningFrameKind.JOINT,
        unrelated_link.frame_id,
        joint.entity_id,
        joint.parent_link_id,
    )
    joint_entity_with_axis = replace(
        joint_entity,
        frame_ids=tuple(sorted((*joint_entity.frame_ids, unrelated_axis.frame_id))),
    )
    with pytest.raises(PlanningSceneContractError, match="joint topology"):
        _rebuild_catalog(
            catalog,
            entities=_replace_sorted(catalog.entities, joint_entity, joint_entity_with_axis, "entity_id"),
            frames=tuple(sorted((*catalog.frames, unrelated_axis), key=lambda item: item.frame_id)),
            joints=_replace_sorted(
                catalog.joints,
                joint,
                replace(joint, axis_frame_id=unrelated_axis.frame_id),
                "joint_id",
            ),
        )


def test_catalog_frame_geometry_and_completeness_validation(planning_values) -> None:
    _, catalog, _ = planning_values
    robot = entity_by_path(catalog, "/robot")
    payload = entity_by_path(catalog, "/payload")
    left = entity_by_path(catalog, "/left")
    robot_links = tuple(link for link in catalog.links if link.entity_id == robot.entity_id)
    root_link = next(link for link in robot_links if link.parent_link_id is None)
    child_link = next(link for link in robot_links if link.parent_link_id == root_link.link_id)
    root_frame = next(frame for frame in catalog.frames if frame.frame_id == root_link.frame_id)
    child_frame = next(frame for frame in catalog.frames if frame.frame_id == child_link.frame_id)
    frame_cases = (
        replace(root_frame, parent_frame_id="frame.unknown"),
        replace(root_frame, owner_entity_id="entity.unknown"),
        replace(
            root_frame, owner_link_id=next(link for link in catalog.links if link.entity_id == left.entity_id).link_id
        ),
        replace(root_frame, owner_link_id=child_link.link_id),
    )
    for invalid in frame_cases:
        with pytest.raises(PlanningSceneContractError):
            _rebuild_catalog(
                catalog,
                frames=_replace_sorted(catalog.frames, root_frame, invalid, "frame_id"),
            )
    frame_cycle = replace(root_frame, parent_frame_id=child_frame.frame_id)
    with pytest.raises(PlanningSceneContractError, match="root link frame"):
        _rebuild_catalog(catalog, frames=_replace_sorted(catalog.frames, root_frame, frame_cycle, "frame_id"))

    robot_geometry = next(geometry for geometry in catalog.geometries if geometry.owner_entity_id == robot.entity_id)
    duplicate_geometry_link = replace(child_link, geometry_ids=(robot_geometry.geometry_id,))
    with pytest.raises(PlanningSceneContractError, match="multiple link owners"):
        _rebuild_catalog(
            catalog,
            links=_replace_sorted(catalog.links, child_link, duplicate_geometry_link, "link_id"),
        )
    payload_geometry = next(
        geometry for geometry in catalog.geometries if geometry.owner_entity_id == payload.entity_id
    )
    left_frame = next(frame for frame in catalog.frames if frame.frame_id == left.root_frame_id)
    geometry_cases = (
        replace(payload_geometry, owner_entity_id="entity.unknown"),
        replace(payload_geometry, parent_frame_id=left_frame.frame_id),
        replace(
            payload_geometry,
            owner_link_id=next(link for link in catalog.links if link.entity_id == left.entity_id).link_id,
        ),
    )
    for invalid in geometry_cases:
        with pytest.raises(PlanningSceneContractError):
            _rebuild_catalog(
                catalog,
                geometries=_replace_sorted(catalog.geometries, payload_geometry, invalid, "geometry_id"),
            )
    with pytest.raises(PlanningSceneContractError, match="complete"):
        _rebuild_catalog(
            catalog,
            geometries=tuple(item for item in catalog.geometries if item is not payload_geometry),
        )
    rigid_robot = entity_by_path(catalog, "/rigid_robot")
    with pytest.raises(PlanningSceneContractError, match="require physical joints"):
        _rebuild_catalog(
            catalog,
            entities=_replace_sorted(
                catalog.entities,
                rigid_robot,
                replace(rigid_robot, kind=PlanningEntityKind.ARTICULATION),
                "entity_id",
            ),
        )
    with pytest.raises(PlanningSceneContractError, match="cannot declare joints"):
        _rebuild_catalog(
            catalog,
            entities=_replace_sorted(
                catalog.entities,
                robot,
                replace(robot, kind=PlanningEntityKind.RIGID_OBJECT),
                "entity_id",
            ),
        )


def test_state_budget_world_pose_attachment_and_catalog_closure(monkeypatch, planning_values) -> None:
    _, catalog, state = planning_values
    original_items = planning_contract._MAX_ITEMS
    largest_collection = max(
        len(values)
        for values in (
            state.entities,
            state.links,
            state.frames,
            state.articulations,
            state.geometry_transforms,
            state.attachments,
        )
    )
    monkeypatch.setattr(planning_contract, "_MAX_ITEMS", largest_collection)
    with pytest.raises(PlanningSceneContractError, match="node budget"):
        replace(state)
    monkeypatch.setattr(planning_contract, "_MAX_ITEMS", original_items)
    original_relationships = planning_contract._MAX_RELATIONSHIP_REFERENCES
    monkeypatch.setattr(planning_contract, "_MAX_RELATIONSHIP_REFERENCES", 1)
    with pytest.raises(PlanningSceneContractError, match="relationship budget"):
        replace(state)
    monkeypatch.setattr(planning_contract, "_MAX_RELATIONSHIP_REFERENCES", original_relationships)

    entity = state.entities[0]
    foreign_pose = replace(entity.pose, frame_id="frame.foreign")
    foreign_twist = replace(entity.twist, frame_id="frame.foreign")
    with pytest.raises(PlanningSceneContractError, match="world_frame_id"):
        replace(
            state,
            entities=_replace_sorted(
                state.entities,
                entity,
                replace(entity, pose=foreign_pose, twist=foreign_twist),
                "entity_id",
            ),
        )
    world_frame = next(frame for frame in state.frames if frame.frame_id == state.world_frame_id)
    shifted_world = replace(
        world_frame,
        world_pose=replace(world_frame.world_pose, position_m=(0.1, 0.0, 0.0)),
    )
    with pytest.raises(PlanningSceneContractError, match="canonical identity"):
        replace(
            state,
            frames=_replace_sorted(state.frames, world_frame, shifted_world, "frame_id"),
        )
    attachment = state.attachments[0]
    with pytest.raises(PlanningSceneContractError, match="do not close"):
        replace(
            state,
            attachments=_replace_sorted(
                state.attachments,
                attachment,
                replace(attachment, parent_entity_id="entity.unknown"),
                "attachment_id",
            ),
        )
    inconsistent = replace(
        attachment,
        parent_T_child=replace(
            attachment.parent_T_child,
            position_m=(attachment.parent_T_child.position_m[0] + 0.1, 0.0, 0.0),
        ),
    )
    with pytest.raises(PlanningSceneContractError, match="contradicts"):
        replace(
            state,
            attachments=_replace_sorted(state.attachments, attachment, inconsistent, "attachment_id"),
        )

    with pytest.raises(PlanningSceneContractError, match="exact PlanningSceneCatalog"):
        state.validate_against(object())
    with pytest.raises(PlanningSceneContractError, match="envelopes"):
        state.validate_against(replace(catalog, world_id="different", content_sha256=""))
    with pytest.raises(PlanningSceneStaleGenerationError):
        state.validate_against(replace(catalog, generation=catalog.generation + 1, content_sha256=""))
    container = entity_by_path(catalog, "/container")
    container_entity_state = next(item for item in state.entities if item.entity_id == container.entity_id)
    with pytest.raises(PlanningSceneContractError, match="entity set"):
        replace(
            state,
            entities=tuple(item for item in state.entities if item is not container_entity_state),
        ).validate_against(catalog)
    robot = entity_by_path(catalog, "/robot")
    robot_links = tuple(link for link in catalog.links if link.entity_id == robot.entity_id)
    robot_child = next(link for link in robot_links if link.parent_link_id is not None)
    robot_child_state = next(item for item in state.links if item.link_id == robot_child.link_id)
    with pytest.raises(PlanningSceneContractError, match="link set"):
        replace(
            state,
            links=tuple(item for item in state.links if item is not robot_child_state),
        ).validate_against(catalog)
    robot_child_frame = next(item for item in state.frames if item.frame_id == robot_child.frame_id)
    with pytest.raises(PlanningSceneContractError, match="frame set"):
        replace(
            state,
            frames=tuple(item for item in state.frames if item is not robot_child_frame),
        ).validate_against(catalog)
    container_transform = next(
        item for item in state.geometry_transforms if item.geometry_id == container.geometry_ids[0]
    )
    with pytest.raises(PlanningSceneContractError, match="geometry set"):
        replace(
            state,
            geometry_transforms=tuple(item for item in state.geometry_transforms if item is not container_transform),
        ).validate_against(catalog)
    robot_articulation = next(item for item in state.articulations if item.entity_id == robot.entity_id)
    with pytest.raises(PlanningSceneContractError, match="articulation set"):
        replace(
            state,
            articulations=tuple(item for item in state.articulations if item is not robot_articulation),
        ).validate_against(catalog)
    reversed_articulation = replace(
        robot_articulation,
        joint_ids=tuple(reversed(robot_articulation.joint_ids)),
        positions=tuple(reversed(robot_articulation.positions)),
        velocities=tuple(reversed(robot_articulation.velocities)),
        position_units=tuple(reversed(robot_articulation.position_units)),
    )
    with pytest.raises(PlanningSceneContractError, match="order/units"):
        replace(
            state,
            articulations=_replace_sorted(
                state.articulations,
                robot_articulation,
                reversed_articulation,
                "entity_id",
            ),
        ).validate_against(catalog)


def test_state_attachment_catalog_ownership_checks(planning_values) -> None:
    _, catalog, state = planning_values
    attachment = state.attachments[0]
    frame_states = {item.frame_id: item.world_pose for item in state.frames}
    foreign_frame = next(
        frame
        for frame in catalog.frames
        if frame.owner_entity_id is not None
        and frame.owner_entity_id not in {attachment.parent_entity_id, attachment.child_entity_id}
    )
    foreign_parent = replace(
        attachment,
        parent_frame_id=foreign_frame.frame_id,
        parent_link_id=None,
        parent_T_child=planning_contract._relative_pose(
            frame_states[foreign_frame.frame_id],
            frame_states[attachment.child_frame_id],
            foreign_frame.frame_id,
        ),
    )
    with pytest.raises(PlanningSceneContractError, match="parent frame ownership"):
        replace(
            state,
            attachments=_replace_sorted(state.attachments, attachment, foreign_parent, "attachment_id"),
        ).validate_against(catalog)
    foreign_child = replace(
        attachment,
        child_frame_id=foreign_frame.frame_id,
        child_link_id=None,
        parent_T_child=planning_contract._relative_pose(
            frame_states[attachment.parent_frame_id],
            frame_states[foreign_frame.frame_id],
            attachment.parent_frame_id,
        ),
    )
    with pytest.raises(PlanningSceneContractError, match="child frame ownership"):
        replace(
            state,
            attachments=_replace_sorted(state.attachments, attachment, foreign_child, "attachment_id"),
        ).validate_against(catalog)

    wrong_parent_link = next(
        link.link_id
        for link in catalog.links
        if link.entity_id not in {attachment.parent_entity_id, attachment.child_entity_id}
    )
    with pytest.raises(PlanningSceneContractError, match="parent link ownership"):
        replace(
            state,
            attachments=_replace_sorted(
                state.attachments,
                attachment,
                replace(attachment, parent_link_id=wrong_parent_link),
                "attachment_id",
            ),
        ).validate_against(catalog)

    robot_for_frame_check = entity_by_path(catalog, "/robot")
    robot_root_link_for_frame_check = next(
        link for link in catalog.links if link.link_id == robot_for_frame_check.link_ids[0]
    )
    robot_child_link_for_frame_check = next(
        link
        for link in catalog.links
        if link.entity_id == robot_for_frame_check.entity_id and link.link_id != robot_root_link_for_frame_check.link_id
    )
    payload_for_frame_check = entity_by_path(catalog, "/payload")
    payload_link_for_frame_check = next(
        link for link in catalog.links if link.link_id == payload_for_frame_check.link_ids[0]
    )
    same_entity_wrong_link = PlanningAttachment(
        "attachment.same-entity-wrong-link",
        robot_for_frame_check.entity_id,
        payload_for_frame_check.entity_id,
        robot_root_link_for_frame_check.frame_id,
        payload_link_for_frame_check.frame_id,
        planning_contract._relative_pose(
            frame_states[robot_root_link_for_frame_check.frame_id],
            frame_states[payload_link_for_frame_check.frame_id],
            robot_root_link_for_frame_check.frame_id,
        ),
        payload_for_frame_check.geometry_ids,
        robot_child_link_for_frame_check.link_id,
        payload_link_for_frame_check.link_id,
    )
    with pytest.raises(PlanningSceneContractError, match="parent link ownership"):
        replace(
            state,
            attachments=tuple(
                sorted((*state.attachments, same_entity_wrong_link), key=lambda item: item.attachment_id)
            ),
        ).validate_against(catalog)
    with pytest.raises(PlanningSceneContractError, match="child link ownership"):
        replace(
            state,
            attachments=_replace_sorted(
                state.attachments,
                attachment,
                replace(attachment, child_link_id=wrong_parent_link),
                "attachment_id",
            ),
        ).validate_against(catalog)
    for field in ("parent_link_id", "child_link_id"):
        with pytest.raises(PlanningSceneContractError, match=f"{field.removesuffix('_link_id')} link ownership"):
            replace(
                state,
                attachments=_replace_sorted(
                    state.attachments,
                    attachment,
                    replace(attachment, **{field: None}),
                    "attachment_id",
                ),
            ).validate_against(catalog)
    wrong_child = next(
        entity for entity in catalog.entities if entity.entity_id != attachment.child_entity_id and entity.link_ids
    )
    wrong_child_link = next(link for link in catalog.links if link.link_id == wrong_child.link_ids[0])
    parent_frame_state = next(item for item in state.frames if item.frame_id == attachment.parent_frame_id)
    wrong_child_attachment = replace(
        attachment,
        child_entity_id=wrong_child.entity_id,
        child_frame_id=wrong_child_link.frame_id,
        child_link_id=wrong_child_link.link_id,
        parent_T_child=planning_contract._relative_pose(
            parent_frame_state.world_pose,
            frame_states[wrong_child_link.frame_id],
            attachment.parent_frame_id,
        ),
    )
    with pytest.raises(PlanningSceneContractError, match="geometry owner"):
        replace(
            state,
            attachments=_replace_sorted(
                state.attachments,
                attachment,
                wrong_child_attachment,
                "attachment_id",
            ),
        ).validate_against(catalog)

    payload = entity_by_path(catalog, "/payload")
    robot = entity_by_path(catalog, "/robot")
    payload_link = next(item for item in catalog.links if item.link_id == payload.link_ids[0])
    payload_frame = next(item for item in state.frames if item.frame_id == payload_link.frame_id)
    robot_root_link = next(item for item in catalog.links if item.link_id == robot.link_ids[0])
    robot_child_link = next(
        item for item in catalog.links if item.entity_id == robot.entity_id and item.link_id != robot_root_link.link_id
    )
    robot_child_frame = next(item for item in state.frames if item.frame_id == robot_child_link.frame_id)
    wrong_geometry_link = PlanningAttachment(
        "attachment.geometry-link",
        payload.entity_id,
        robot.entity_id,
        payload_link.frame_id,
        robot_child_link.frame_id,
        planning_contract._relative_pose(
            payload_frame.world_pose,
            robot_child_frame.world_pose,
            payload_link.frame_id,
        ),
        robot.geometry_ids,
        payload_link.link_id,
        robot_child_link.link_id,
    )
    with pytest.raises(PlanningSceneContractError, match="geometry link owner"):
        replace(
            state,
            attachments=tuple(sorted((*state.attachments, wrong_geometry_link), key=lambda item: item.attachment_id)),
        ).validate_against(catalog)


def _structural_delta(catalog: PlanningSceneCatalog, state: PlanningSceneState) -> PlanningSceneDelta:
    next_catalog = replace(catalog, catalog_revision=catalog.catalog_revision + 1, content_sha256="")
    next_tick = Tick(state.tick.step_index + 1, state.tick.sim_time_seconds + 0.01)
    next_state = replace(
        state,
        tick=next_tick,
        sequence=state.sequence + 1,
        world_revision=state.world_revision + 1,
        catalog_revision=next_catalog.catalog_revision,
        catalog_content_sha256=next_catalog.content_sha256,
        transform_revision=state.transform_revision + 1,
    )
    next_state.validate_against(next_catalog)
    return PlanningSceneDelta(
        state.provider_id,
        state.world_id,
        state.generation,
        state.environment_index,
        next_tick,
        state.sequence,
        next_state.sequence,
        state.world_revision,
        next_state.world_revision,
        state.catalog_revision,
        next_catalog.catalog_revision,
        state.catalog_content_sha256,
        next_catalog.content_sha256,
        state.geometry_revision,
        next_state.geometry_revision,
        state.transform_revision,
        next_state.transform_revision,
        state.attachment_revision,
        next_state.attachment_revision,
        PlanningSceneDeltaKind.STRUCTURAL,
        catalog=next_catalog,
        state=next_state,
    )


def _attachment_delta(state: PlanningSceneState) -> PlanningSceneDelta:
    tick = Tick(state.tick.step_index + 1, state.tick.sim_time_seconds + 0.01)
    return PlanningSceneDelta(
        state.provider_id,
        state.world_id,
        state.generation,
        state.environment_index,
        tick,
        state.sequence,
        state.sequence + 1,
        state.world_revision,
        state.world_revision + 1,
        state.catalog_revision,
        state.catalog_revision,
        state.catalog_content_sha256,
        state.catalog_content_sha256,
        state.geometry_revision,
        state.geometry_revision,
        state.transform_revision,
        state.transform_revision,
        state.attachment_revision,
        state.attachment_revision + 1,
        PlanningSceneDeltaKind.ATTACHMENT,
        attachments=state.attachments,
    )


def test_resource_layout_changes_are_digest_pinned_and_revisioned(planning_values) -> None:
    world, catalog, state = planning_values
    geometry = next(
        item for item in catalog.geometries if item.representation is PlanningGeometryRepresentation.TRIANGLE_MESH
    )
    assert geometry.resource_layout is not None
    lease = world.resolve_planning_geometry(geometry.geometry_id)
    try:
        prior_descriptor = lease.descriptor
        vertex_shape = geometry.resource_layout.vertex_shape
        index_shape = geometry.resource_layout.index_shape
        assert vertex_shape is not None and index_shape is not None
        changed_layout = replace(
            geometry.resource_layout,
            vertex_shape=(vertex_shape[0] + 1, 3),
            index_shape=(index_shape[0] - 1, 3),
        )
        assert changed_layout.decoded_byte_size == geometry.resource_layout.decoded_byte_size
        changed_geometry = replace(geometry, resource_layout=changed_layout)
        changed_geometries = _replace_sorted(
            catalog.geometries,
            geometry,
            changed_geometry,
            "geometry_id",
        )
        unrevisioned = _rebuild_catalog(catalog, geometries=changed_geometries)
        assert unrevisioned.content_sha256 != catalog.content_sha256
        with pytest.raises(PlanningSceneContractError, match="envelopes"):
            state.validate_against(unrevisioned)
        with pytest.raises(PlanningSceneContractError, match="envelopes"):
            prior_descriptor.validate_against(unrevisioned)
        rebound_descriptor = replace(
            prior_descriptor,
            catalog_content_sha256=unrevisioned.content_sha256,
        )
        with pytest.raises(PlanningSceneContractError, match="metadata"):
            rebound_descriptor.validate_against(unrevisioned)
        inline_geometry = replace(
            geometry,
            representation=PlanningGeometryRepresentation.BOX,
            inline=PlanningPrimitiveGeometry(PlanningGeometryRepresentation.BOX, (0.5, 0.5, 0.5)),
            resource_id=None,
            sha256=None,
            content_profile=None,
            resource_layout=None,
        )
        inline_catalog = _rebuild_catalog(
            catalog,
            geometries=_replace_sorted(catalog.geometries, geometry, inline_geometry, "geometry_id"),
        )
        inline_rebound_descriptor = replace(
            prior_descriptor,
            catalog_content_sha256=inline_catalog.content_sha256,
        )
        with pytest.raises(PlanningSceneContractError, match="absent"):
            inline_rebound_descriptor.validate_against(inline_catalog)
        with pytest.raises(PlanningSceneContractError, match="exact PlanningSceneCatalog"):
            prior_descriptor.validate_against(object())
        stale_catalog = replace(catalog, generation=catalog.generation + 1, content_sha256="")
        with pytest.raises(PlanningSceneStaleGenerationError):
            prior_descriptor.validate_against(stale_catalog)

        def structural_for(next_catalog: PlanningSceneCatalog) -> tuple[PlanningSceneDelta, PlanningSceneState]:
            next_tick = Tick(state.tick.step_index + 1, state.tick.sim_time_seconds + 0.01)
            next_state = replace(
                state,
                tick=next_tick,
                sequence=state.sequence + 1,
                world_revision=state.world_revision + 1,
                catalog_revision=next_catalog.catalog_revision,
                geometry_revision=next_catalog.geometry_revision,
                catalog_content_sha256=next_catalog.content_sha256,
                transform_revision=state.transform_revision + 1,
            )
            return (
                PlanningSceneDelta(
                    state.provider_id,
                    state.world_id,
                    state.generation,
                    state.environment_index,
                    next_tick,
                    state.sequence,
                    next_state.sequence,
                    state.world_revision,
                    next_state.world_revision,
                    state.catalog_revision,
                    next_catalog.catalog_revision,
                    state.catalog_content_sha256,
                    next_catalog.content_sha256,
                    state.geometry_revision,
                    next_catalog.geometry_revision,
                    state.transform_revision,
                    next_state.transform_revision,
                    state.attachment_revision,
                    next_state.attachment_revision,
                    PlanningSceneDeltaKind.STRUCTURAL,
                    catalog=next_catalog,
                    state=next_state,
                ),
                next_state,
            )

        bad_catalog = replace(
            unrevisioned,
            catalog_revision=catalog.catalog_revision + 1,
            content_sha256="",
        )
        bad_delta, _ = structural_for(bad_catalog)
        with pytest.raises(PlanningSceneContractError, match="advance geometry_revision"):
            bad_delta.apply(catalog, state)

        next_catalog = replace(
            unrevisioned,
            catalog_revision=catalog.catalog_revision + 1,
            geometry_revision=catalog.geometry_revision + 1,
            content_sha256="",
        )
        good_delta, next_state = structural_for(next_catalog)
        assert good_delta.apply(catalog, state) == (next_catalog, next_state)
        with pytest.raises(PlanningSceneContractError, match="envelopes"):
            prior_descriptor.validate_against(next_catalog)

        world.step()
        stale_delta = world.planning_scene_delta(state.sequence)
        with pytest.raises(PlanningSceneDeltaContinuityError):
            stale_delta.apply(next_catalog, next_state)
    finally:
        lease.close()


def test_structural_attachment_delta_apply_and_invalid_payload_matrix(planning_values) -> None:
    world, catalog, before = planning_values
    structural = _structural_delta(catalog, before)
    structural_result = structural.apply(catalog, before)
    assert structural_result == (structural.catalog, structural.state)
    attachment = _attachment_delta(before)
    attachment_result = attachment.apply(catalog, before)
    assert attachment_result is not None
    assert attachment_result[1].attachments == before.attachments
    assert attachment_result[1].attachment_revision == before.attachment_revision + 1

    world.step()
    state_delta = world.planning_scene_delta(before.sequence)
    assert state_delta.kind is PlanningSceneDeltaKind.STATE
    for delta in (structural, state_delta, attachment):
        with pytest.raises(PlanningSceneContractError, match="sequence must advance"):
            replace(delta, sequence=delta.base_sequence)
    invalid_state = (
        {"base_sequence": state_delta.sequence + 1},
        {"previous_world_revision": state_delta.world_revision + 1},
        {"resync_required": 1},
        {"resync_required": True},
        {"world_revision": state_delta.previous_world_revision},
        {"state": None},
        {"catalog_revision": state_delta.previous_catalog_revision + 1},
        {"geometry_revision": state_delta.previous_geometry_revision + 1},
        {"attachment_revision": state_delta.previous_attachment_revision + 1},
        {"transform_revision": state_delta.previous_transform_revision},
    )
    for override in invalid_state:
        with pytest.raises(PlanningSceneContractError):
            replace(state_delta, **override)
    with pytest.raises(PlanningSceneContractError, match="catalog content digests"):
        replace(state_delta, kind=PlanningSceneDeltaKind.RESYNC, resync_required=True)
    with pytest.raises(PlanningSceneContractError, match="structural delta"):
        replace(state_delta, kind=PlanningSceneDeltaKind.STRUCTURAL)
    with pytest.raises(PlanningSceneContractError, match="advance catalog_revision"):
        replace(structural, catalog_revision=structural.previous_catalog_revision)
    invalid_attachment = (
        {"state": before},
        {"attachment_revision": attachment.previous_attachment_revision},
        {"catalog_revision": attachment.previous_catalog_revision + 1},
        {"geometry_revision": attachment.previous_geometry_revision + 1},
        {"transform_revision": attachment.previous_transform_revision + 1},
    )
    for override in invalid_attachment:
        with pytest.raises(PlanningSceneContractError):
            replace(attachment, **override)
    with pytest.raises(PlanningSceneContractError, match="payload envelope"):
        replace(structural, provider_id="provider.other")
    with pytest.raises(PlanningSceneContractError, match="catalog identity"):
        replace(structural, catalog_revision=structural.catalog_revision + 1)
    assert state_delta.state is not None
    mismatched_state = replace(
        state_delta.state,
        tick=Tick(state_delta.tick.step_index, state_delta.tick.sim_time_seconds + 1.0),
    )
    with pytest.raises(PlanningSceneContractError, match="state revisions"):
        replace(state_delta, state=mismatched_state)

    resync = PlanningSceneDelta(
        before.provider_id,
        before.world_id,
        before.generation,
        before.environment_index,
        before.tick,
        before.sequence,
        before.sequence,
        before.world_revision,
        before.world_revision,
        before.catalog_revision,
        before.catalog_revision,
        None,
        None,
        before.geometry_revision,
        before.geometry_revision,
        before.transform_revision,
        before.transform_revision,
        before.attachment_revision,
        before.attachment_revision,
        PlanningSceneDeltaKind.RESYNC,
        resync_required=True,
    )
    assert resync.apply(catalog, before) is None
    with pytest.raises(PlanningSceneContractError, match="catalog revision"):
        replace(resync, catalog_revision=resync.previous_catalog_revision + 1)
    with pytest.raises(PlanningSceneContractError, match="geometry revision"):
        replace(resync, geometry_revision=resync.previous_geometry_revision + 1)


def test_delta_apply_rejects_wrong_type_envelope_and_continuity(planning_values) -> None:
    _, catalog, state = planning_values
    delta = _attachment_delta(state)
    with pytest.raises(PlanningSceneContractError, match="exact catalog and state"):
        delta.apply(object(), state)
    with pytest.raises(PlanningSceneContractError, match="envelope"):
        delta.apply(catalog, replace(state, world_id="different"))
    with pytest.raises(PlanningSceneDeltaContinuityError, match="base continuity"):
        delta.apply(catalog, replace(state, world_revision=state.world_revision + 1))
    with pytest.raises(PlanningSceneStaleGenerationError):
        delta.apply(catalog, replace(state, generation=state.generation + 1))
