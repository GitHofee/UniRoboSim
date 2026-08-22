from __future__ import annotations

import gc
import hashlib
import inspect
import sys
import threading
import tracemalloc
import weakref
from dataclasses import replace

import pytest

import unirobosim.api.planning_scene as planning_contract
import unirobosim.testing.fake_backend as fake_contract
from tests.test_soft_matter_specs import fluid_body, surface_body
from unirobosim import (
    PLANNING_FRAME_DECLARATIONS_SCHEMA_VERSION,
    PLANNING_GEOMETRY_READ_LIMIT_BYTES,
    PLANNING_GRID_INDEX_ORDER,
    PLANNING_HEIGHTFIELD_SAMPLE_CONVENTION,
    PLANNING_SDF_SIGN_CONVENTION,
    PLANNING_SYSTEM_ENTITY_ID,
    PLANNING_SYSTEM_ENTITY_PATH,
    PLANNING_VOXEL_OCCUPANCY_CONVENTION,
    BoxGeometrySpec,
    CapabilityId,
    CapabilityRequirement,
    CommandMode,
    EntityKind,
    EntityPath,
    EntitySpec,
    EnvironmentSpec,
    FrozenMap,
    PhysicsSpec,
    PlanningAttachment,
    PlanningCompoundGeometry,
    PlanningEntityDescriptor,
    PlanningEntityKind,
    PlanningFrameDescriptor,
    PlanningFrameKind,
    PlanningFrameRole,
    PlanningGeometryAxisConvention,
    PlanningGeometryContentProfile,
    PlanningGeometryDescriptor,
    PlanningGeometryDType,
    PlanningGeometryLocalPose,
    PlanningGeometryMotionClass,
    PlanningGeometryPurpose,
    PlanningGeometryRepresentation,
    PlanningGeometryResourceDescriptor,
    PlanningGeometryResourceLayout,
    PlanningGeometryResourceRevokedError,
    PlanningGeometryStorageKind,
    PlanningHalfspaceGeometry,
    PlanningSceneCatalog,
    PlanningSceneContractError,
    PlanningSceneDeltaContinuityError,
    PlanningSceneDeltaKind,
    PlanningSceneError,
    PlanningSceneHashMismatchError,
    PlanningSceneIncompleteError,
    PlanningSceneNotFoundError,
    PlanningSceneRepresentationError,
    PlanningSceneWorld,
    Pose,
    ProviderDescriptor,
    SceneCommand,
    SceneCommandKind,
    SceneCommandStatus,
    SessionState,
    ValidationError,
    World,
    WorldSpec,
)
from unirobosim.testing import FAKE_DESCRIPTOR, FakeProvider, FakeSession, FakeWorld


def planning_requirement() -> CapabilityRequirement:
    return CapabilityRequirement(CapabilityId("planning.scene@2"))


def planning_frame_declarations() -> dict[str, object]:
    return {
        "schema": PLANNING_FRAME_DECLARATIONS_SCHEMA_VERSION,
        "component_sha256": "a" * 64,
        "entries": (
            {
                "semantic_key": "annotation.home",
                "role": "annotation",
                "owner_link": "robot",
                "source": {"kind": "link", "name": "robot"},
            },
            {
                "semantic_key": "ee.left",
                "role": "ee",
                "owner_link": "肩关节 child",
                "source": {"kind": "link", "name": "肩关节 child"},
            },
            {
                "semantic_key": "sensor.wrist",
                "role": "sensor",
                "owner_link": "elbow joint child",
                "source": {"kind": "link", "name": "elbow joint child"},
            },
            {
                "semantic_key": "tool.tcp",
                "role": "tool",
                "owner_link": "elbow joint child",
                "source": {"kind": "link", "name": "elbow joint child"},
            },
        ),
    }


def planning_spec(*, environments: int = 2) -> WorldSpec:
    attachments = (
        {
            "attachment_id": "attachment.left_payload",
            "parent_path": "/left",
            "child_path": "/payload",
        },
        {
            "attachment_id": "attachment.right_payload",
            "parent_path": "/right",
            "child_path": "/payload",
        },
        {
            "attachment_id": "attachment.payload_left",
            "parent_path": "/payload",
            "child_path": "/left",
        },
    )
    return WorldSpec(
        "planning-contract",
        (
            EntitySpec(
                EntityPath("/container"),
                EntityKind.RIGID_BODY,
                asset_uri="asset://concave-container",
                metadata=FrozenMap(
                    {
                        "planning_motion_class": "static",
                        "fake_planning_collision_authority": "effective_native",
                    }
                ),
            ),
            EntitySpec(EntityPath("/left"), EntityKind.RIGID_BODY, box=BoxGeometrySpec()),
            EntitySpec(
                EntityPath("/microwave"),
                EntityKind.ARTICULATION,
                joint_names=("门 铰链", "旋钮 α"),
                initial_joint_positions=(0.1, -0.2),
            ),
            EntitySpec(EntityPath("/payload"), EntityKind.RIGID_BODY, box=BoxGeometrySpec()),
            EntitySpec(EntityPath("/right"), EntityKind.RIGID_BODY, box=BoxGeometrySpec()),
            EntitySpec(
                EntityPath("/rigid_robot"),
                EntityKind.RIGID_BODY,
                box=BoxGeometrySpec(),
                metadata=FrozenMap({"planning_entity_kind": "robot"}),
            ),
            EntitySpec(
                EntityPath("/robot"),
                EntityKind.ARTICULATION,
                joint_names=("肩关节", "elbow joint"),
                metadata=FrozenMap(
                    {
                        "planning_entity_kind": "robot",
                        "planning_frame_declarations": planning_frame_declarations(),
                    }
                ),
            ),
        ),
        environments=EnvironmentSpec(environments),
        physics=PhysicsSpec(time_step_seconds=0.01, gravity_m_s2=(0.0, 0.0, 0.0)),
        requirements=(planning_requirement(),),
        metadata=FrozenMap(
            {
                "planning_attachment_authority": "exclusive_registry",
                "planning_attachments": attachments,
            }
        ),
    )


@pytest.fixture
def planning_world():
    session = FakeProvider().open()
    world = session.build(planning_spec())
    try:
        yield world
    finally:
        session.close()


def entity_by_path(catalog: PlanningSceneCatalog, path: str) -> PlanningEntityDescriptor:
    return next(entity for entity in catalog.entities if entity.path == path)


def resource_geometry_id(catalog: PlanningSceneCatalog) -> str:
    return next(
        geometry.geometry_id
        for geometry in catalog.geometries
        if geometry.representation is PlanningGeometryRepresentation.TRIANGLE_MESH
    )


def capture_planning_provenance(
    descriptor: ProviderDescriptor,
    spec: WorldSpec,
) -> tuple[str, dict[str, str]]:
    session = FakeSession(descriptor)
    world = session.build(spec)
    try:
        catalog = world.planning_scene_catalog()
        paths = {entity.entity_id: entity.path for entity in catalog.entities}
        provenance = {paths[geometry.owner_entity_id]: geometry.provenance_sha256 for geometry in catalog.geometries}
        return catalog.content_sha256, provenance
    finally:
        session.close()


def test_protocol_is_separate_and_fake_declares_exact_capability_after_conformance(planning_world) -> None:
    world_members = {name for name in World.__dict__ if not name.startswith("_")}
    extension_members = {name for name in PlanningSceneWorld.__dict__ if not name.startswith("_")}
    assert len(world_members) == 21
    assert extension_members == {
        "planning_scene_catalog",
        "planning_scene_state",
        "planning_scene_delta",
        "resolve_planning_geometry",
    }
    assert isinstance(planning_world, World)
    assert isinstance(planning_world, PlanningSceneWorld)
    declaration = FAKE_DESCRIPTOR.capabilities.get(CapabilityId("planning.scene@2"))
    assert declaration is not None
    assert declaration.properties["representation_fallback"] is False


def test_no_demand_world_allocates_no_catalog_cache_or_resource_storage() -> None:
    session = FakeProvider().open()
    world = session.build(
        WorldSpec(
            "no-planning-demand",
            (EntitySpec(EntityPath("/box"), EntityKind.RIGID_BODY, box=BoxGeometrySpec()),),
        )
    )
    try:
        assert type(world) is FakeWorld
        assert not isinstance(world, PlanningSceneWorld)
        assert not any("planning" in key for key in world.__dict__)
        assert not hasattr(world, "_planning_runtime")
        for method in (
            FakeWorld.__init__,
            FakeWorld.step,
            FakeWorld.reset,
            FakeWorld.apply_scene_command,
            FakeWorld._close,
        ):
            assert "planning" not in inspect.getsource(method).lower()
        planning_calls: list[str] = []

        def profile(frame, event: str, arg) -> None:
            del arg
            if (
                event == "call"
                and frame.f_code.co_filename.endswith("fake_backend.py")
                and "planning" in frame.f_code.co_name.lower()
            ):
                planning_calls.append(frame.f_code.co_name)

        sys.setprofile(profile)
        try:
            world.step(3)
            world.reset()
            result = world.apply_scene_command(
                SceneCommand(
                    "no-demand-pose",
                    "contract-test",
                    "lease",
                    world.generation,
                    SceneCommandKind.SET_POSE,
                    EntityPath("/box"),
                    target_pose=Pose((0.1, 0.2, 0.3)),
                )
            )
            world.close()
        finally:
            sys.setprofile(None)
        assert result.status is SceneCommandStatus.APPLIED
        assert planning_calls == []
        assert not any("planning" in key for key in world.__dict__)
        assert "planning_scene_catalog" not in dir(world)
    finally:
        session.close()


def test_supported_optional_planning_requirement_is_matched_and_enables_provider() -> None:
    requirement = CapabilityRequirement(CapabilityId("planning.scene@2"), required=False)
    spec = WorldSpec(
        "optional-planning-demand",
        (EntitySpec(EntityPath("/box"), EntityKind.RIGID_BODY, box=BoxGeometrySpec()),),
        requirements=(requirement,),
    )
    session = FakeProvider().open()
    try:
        report = session.negotiate(spec.requirements)
        assert report.accepted
        assert CapabilityId("planning.scene@2") in report.matched
        assert not report.optional_issues
        world = session.build(spec)
        assert isinstance(world, PlanningSceneWorld)
        assert hasattr(world, "_planning_runtime")
        assert world.planning_scene_catalog().world_id == spec.world_id
    finally:
        session.close()


def test_missing_optional_requirement_keeps_negotiation_semantics_and_no_provider() -> None:
    requirement = CapabilityRequirement(CapabilityId("planning.missing@1"), required=False)
    spec = WorldSpec(
        "optional-planning-missing",
        (EntitySpec(EntityPath("/box"), EntityKind.RIGID_BODY, box=BoxGeometrySpec()),),
        requirements=(requirement,),
    )
    session = FakeProvider().open()
    try:
        report = session.negotiate(spec.requirements)
        assert report.accepted and not report.required_issues
        assert requirement.capability not in report.matched
        assert len(report.optional_issues) == 1
        assert report.optional_issues[0].capability == requirement.capability
        assert report.optional_issues[0].reason == "missing"
        world = session.build(spec)
        assert type(world) is FakeWorld
        assert not isinstance(world, PlanningSceneWorld)
        assert not hasattr(world, "_planning_runtime")
    finally:
        session.close()


def test_planning_preflight_failures_are_typed_transactional_and_retryable() -> None:
    base = planning_spec(environments=1)
    robot = next(item for item in base.entities if item.path == EntityPath("/robot"))
    robot_metadata = robot.metadata.to_dict()
    robot_metadata["planning_frame_declarations"] = {
        "schema": PLANNING_FRAME_DECLARATIONS_SCHEMA_VERSION,
        "component_sha256": "a" * 64,
        "entries": (
            {
                "semantic_key": "tool.missing",
                "role": "tool",
                "owner_link": "missing link",
                "source": {"kind": "link", "name": "missing link"},
            },
        ),
    }
    bad_frame_robot = replace(robot, metadata=FrozenMap(robot_metadata))
    bad_frame_spec = replace(
        base,
        world_id="planning-bad-frame",
        entities=tuple(bad_frame_robot if item.path == robot.path else item for item in base.entities),
    )

    container = next(item for item in base.entities if item.path == EntityPath("/container"))
    bad_asset_spec = replace(
        base,
        world_id="planning-bad-asset-authority",
        entities=tuple(
            replace(container, metadata=FrozenMap({"planning_motion_class": "static"}))
            if item.path == container.path
            else item
            for item in base.entities
        ),
    )
    incomplete_inventory_metadata = base.metadata.to_dict()
    incomplete_inventory_metadata["fake_planning_unmapped_native_colliders"] = 1
    incomplete_inventory_spec = replace(
        base,
        world_id="planning-unmapped-native",
        metadata=FrozenMap(incomplete_inventory_metadata),
    )
    missing_attachment_authority = base.metadata.to_dict()
    del missing_attachment_authority["planning_attachment_authority"]
    missing_attachment_authority_spec = replace(
        base,
        world_id="planning-untracked-attachments",
        metadata=FrozenMap(missing_attachment_authority),
    )
    spoofed_system_spec = WorldSpec(
        "planning-system-spoof",
        (
            EntitySpec(
                EntityPath(PLANNING_SYSTEM_ENTITY_PATH),
                EntityKind.RIGID_BODY,
                box=BoxGeometrySpec(),
            ),
        ),
        requirements=(planning_requirement(),),
    )

    for invalid in (
        bad_frame_spec,
        bad_asset_spec,
        incomplete_inventory_spec,
        missing_attachment_authority_spec,
        spoofed_system_spec,
    ):
        session = FakeProvider().open()
        try:
            with pytest.raises(PlanningSceneIncompleteError) as caught:
                session.build(invalid)
            assert type(caught.value) is PlanningSceneIncompleteError
            assert caught.value.code == "unirobosim.planning_scene.incomplete"
            assert caught.value.operation == "planning_scene.preflight"
            assert caught.value.__cause__ is None and caught.value.__context__ is None
            assert session.state is SessionState.OPEN and session._active_world is None
            recovered = session.build(
                WorldSpec(
                    f"{invalid.world_id}-recovered",
                    (EntitySpec(EntityPath("/box"), EntityKind.RIGID_BODY, box=BoxGeometrySpec()),),
                    requirements=(planning_requirement(),),
                )
            )
            assert isinstance(recovered, PlanningSceneWorld)
            recovered.close()
        finally:
            session.close()


def test_preflight_failure_detaches_caller_graph_and_closes_provisional_world(monkeypatch) -> None:
    class Sidecar:
        pass

    class AssetText(str):
        def __new__(cls, value: str, sidecar: object):
            instance = super().__new__(cls, value)
            instance.sidecar = sidecar
            return instance

    sidecar = Sidecar()
    reference = weakref.ref(sidecar)
    base = planning_spec(environments=1)
    container = next(item for item in base.entities if item.path == EntityPath("/container"))
    hostile_asset = AssetText(container.asset_uri, sidecar)
    hostile_container = replace(container, asset_uri=hostile_asset)
    hostile_spec = replace(
        base,
        world_id="planning-detached-preflight-failure",
        entities=tuple(hostile_container if item.path == container.path else item for item in base.entities),
    )

    source_failures = [RuntimeError("credential-bearing native failure", sidecar)]

    def unexpected_admission_failure(self) -> None:
        del self
        raise source_failures[0]

    monkeypatch.setattr(
        fake_contract.FakePlanningWorld,
        "_initialize_planning_scene",
        unexpected_admission_failure,
    )
    session = FakeProvider().open()
    with pytest.raises(PlanningSceneIncompleteError) as caught:
        session.build(hostile_spec)
    monkeypatch.undo()
    assert caught.value.message == "planning-scene native admission failed"
    assert caught.value.operation == "planning_scene.preflight"
    assert caught.value.__cause__ is None and caught.value.__context__ is None
    assert session.state is SessionState.OPEN and session._active_world is None
    session.close()
    source_failures.clear()
    del hostile_spec, hostile_container, hostile_asset, container, base, sidecar
    gc.collect()
    assert reference() is None


def test_true_process_interrupt_is_cleaned_up_without_being_masked(monkeypatch) -> None:
    def interrupt(self) -> None:
        del self
        raise KeyboardInterrupt("operator interrupt")

    monkeypatch.setattr(fake_contract.FakePlanningWorld, "_initialize_planning_scene", interrupt)
    session = FakeProvider().open()
    with pytest.raises(KeyboardInterrupt, match="operator interrupt"):
        session.build(planning_spec(environments=1))
    assert session.state is SessionState.OPEN and session._active_world is None
    session.close()


def test_catalog_covers_rigid_generic_articulation_robot_and_unicode(planning_world) -> None:
    catalog = planning_world.planning_scene_catalog()
    assert entity_by_path(catalog, "/payload").kind is PlanningEntityKind.RIGID_OBJECT
    microwave = entity_by_path(catalog, "/microwave")
    robot = entity_by_path(catalog, "/robot")
    rigid_robot = entity_by_path(catalog, "/rigid_robot")
    assert microwave.kind is PlanningEntityKind.ARTICULATION
    assert robot.kind is PlanningEntityKind.ROBOT
    assert rigid_robot.kind is PlanningEntityKind.ROBOT and rigid_robot.joint_ids == ()
    assert {joint.authored_name for joint in catalog.joints}.issuperset({"门 铰链", "旋钮 α", "肩关节"})
    state = planning_world.planning_scene_state()
    assert rigid_robot.entity_id not in {item.entity_id for item in state.articulations}
    state.validate_against(catalog)


def test_v2_system_entity_owns_exact_implicit_halfspace(planning_world) -> None:
    catalog = planning_world.planning_scene_catalog()
    system = next(item for item in catalog.entities if item.entity_id == PLANNING_SYSTEM_ENTITY_ID)
    assert system.path == PLANNING_SYSTEM_ENTITY_PATH
    assert system.kind is PlanningEntityKind.OTHER and system.enabled
    assert system.link_ids == () and system.joint_ids == ()
    assert system.frame_ids == (system.root_frame_id,)
    root = next(item for item in catalog.frames if item.frame_id == system.root_frame_id)
    assert root.kind is PlanningFrameKind.ENTITY
    assert root.parent_frame_id == catalog.world_frame_id
    assert root.owner_entity_id == system.entity_id and root.owner_link_id is None
    ground = next(item for item in catalog.geometries if item.geometry_id == system.geometry_ids[0])
    assert ground.owner_entity_id == system.entity_id and ground.owner_link_id is None
    assert ground.parent_frame_id == system.root_frame_id
    assert ground.purpose is PlanningGeometryPurpose.COLLISION
    assert ground.representation is PlanningGeometryRepresentation.HALFSPACE
    assert type(ground.inline) is PlanningHalfspaceGeometry
    assert ground.scale == (1.0, 1.0, 1.0)
    assert ground.motion_class is PlanningGeometryMotionClass.STATIC
    assert ground.resolution_key is None
    state = planning_world.planning_scene_state()
    system_state = next(item for item in state.entities if item.entity_id == system.entity_id)
    system_frame = next(item for item in state.frames if item.frame_id == system.root_frame_id)
    ground_transform = next(item for item in state.geometry_transforms if item.geometry_id == ground.geometry_id)
    assert system_state.pose == system_frame.world_pose == ground_transform.world_pose
    assert system_state.pose.position_m == (0.0, 0.0, 0.0)
    assert system_state.pose.orientation_xyzw == (0.0, 0.0, 0.0, 1.0)
    assert system_state.twist.linear_m_s == system_state.twist.angular_rad_s == (0.0, 0.0, 0.0)


def test_named_frame_allow_list_comes_only_from_locked_declarations(planning_world) -> None:
    catalog = planning_world.planning_scene_catalog()
    robot = entity_by_path(catalog, "/robot")
    named = tuple(
        sorted(
            (
                item
                for item in catalog.frames
                if item.owner_entity_id == robot.entity_id and item.kind is PlanningFrameKind.NAMED
            ),
            key=lambda item: item.semantic_key or "",
        )
    )
    assert tuple(item.semantic_key for item in named) == (
        "annotation.home",
        "ee.left",
        "sensor.wrist",
        "tool.tcp",
    )
    assert {item.role for item in named} == {
        PlanningFrameRole.ANNOTATION,
        PlanningFrameRole.EE,
        PlanningFrameRole.SENSOR,
        PlanningFrameRole.TOOL,
    }
    link_by_id = {item.link_id: item for item in catalog.links}
    assert all(
        item.owner_link_id is not None and item.parent_frame_id == link_by_id[item.owner_link_id].frame_id
        for item in named
    )
    assert not any("camera" in item.frame_id or "xform" in item.frame_id for item in catalog.frames)


def test_entity_root_named_frame_is_admitted_from_exact_root_link_source() -> None:
    base = planning_spec(environments=1)
    payload = next(item for item in base.entities if item.path == EntityPath("/payload"))
    declaration = {
        "schema": PLANNING_FRAME_DECLARATIONS_SCHEMA_VERSION,
        "component_sha256": "c" * 64,
        "entries": (
            {
                "semantic_key": "annotation.root",
                "role": "annotation",
                "owner_link": None,
                "source": {"kind": "link", "name": "payload"},
            },
        ),
    }
    metadata = payload.metadata.to_dict()
    metadata["planning_frame_declarations"] = declaration
    changed_payload = replace(payload, metadata=FrozenMap(metadata))
    spec = replace(
        base,
        world_id="planning-entity-root-named-frame",
        entities=tuple(changed_payload if item.path == payload.path else item for item in base.entities),
    )
    session = FakeProvider().open()
    world = session.build(spec)
    try:
        catalog = world.planning_scene_catalog()
        descriptor = entity_by_path(catalog, "/payload")
        named = next(item for item in catalog.frames if item.semantic_key == "annotation.root")
        assert named.owner_entity_id == descriptor.entity_id
        assert named.owner_link_id is None
        assert named.parent_frame_id == descriptor.root_frame_id
        world.planning_scene_state().validate_against(catalog)
    finally:
        session.close()


def test_planning_metadata_scalar_and_key_subclasses_are_detached_before_inspection() -> None:
    class SteeringText(str):
        equality_calls = 0

        def __eq__(self, other):
            del other
            type(self).equality_calls += 1
            raise KeyboardInterrupt("metadata equality must not run")

        __hash__ = str.__hash__

    base = planning_spec(environments=1)
    payload = next(item for item in base.entities if item.path == EntityPath("/payload"))
    metadata = payload.metadata.to_dict()
    metadata["planning_motion_class"] = SteeringText("static")
    changed_payload = replace(payload, metadata=FrozenMap(metadata))
    stored = changed_payload.metadata["planning_motion_class"]
    assert type(stored) is str
    spec = replace(
        base,
        world_id="planning-detached-metadata-scalar",
        entities=tuple(changed_payload if item.path == payload.path else item for item in base.entities),
    )
    session = FakeProvider().open()
    world = session.build(spec)
    try:
        catalog = world.planning_scene_catalog()
        payload_descriptor = entity_by_path(catalog, "/payload")
        geometry = next(item for item in catalog.geometries if item.owner_entity_id == payload_descriptor.entity_id)
        assert geometry.motion_class is PlanningGeometryMotionClass.STATIC
        assert SteeringText.equality_calls == 0
    finally:
        session.close()

    class HostileKey(str):
        armed = False
        hash_calls = 0

        def __hash__(self):
            if type(self).armed:
                type(self).hash_calls += 1
                raise KeyboardInterrupt("metadata key hash must not run")
            return str.__hash__(self)

    keys = tuple(HostileKey(value) for value in ("schema", "component_sha256", "entries"))
    declarations = FrozenMap(
        {
            keys[0]: PLANNING_FRAME_DECLARATIONS_SCHEMA_VERSION,
            keys[1]: "a" * 64,
            keys[2]: (),
        }
    )
    HostileKey.armed = True
    parsed = planning_contract.parse_planning_frame_declarations(declarations)
    assert parsed is not None and parsed.entries == ()
    assert HostileKey.hash_calls == 0


def test_attachment_default_endpoint_uses_topology_root_not_opaque_id_order() -> None:
    spec = WorldSpec(
        "planning-attachment-topology-root",
        (
            EntitySpec(EntityPath("/anchor"), EntityKind.RIGID_BODY, box=BoxGeometrySpec()),
            EntitySpec(
                EntityPath("/microwave"),
                EntityKind.ARTICULATION,
                joint_names=("hinge", "dial"),
            ),
        ),
        environments=EnvironmentSpec(1),
        requirements=(planning_requirement(),),
        metadata=FrozenMap(
            {
                "planning_attachment_authority": "exclusive_registry",
                "planning_attachments": (
                    {
                        "attachment_id": "attachment.anchor_microwave",
                        "parent_path": "/anchor",
                        "child_path": "/microwave",
                    },
                ),
            }
        ),
    )
    session = FakeProvider().open()
    world = session.build(spec)
    try:
        catalog = world.planning_scene_catalog()
        target = entity_by_path(catalog, "/microwave")
        root_link = next(
            item for item in catalog.links if item.entity_id == target.entity_id and item.parent_link_id is None
        )
        assert target.link_ids[0] != root_link.link_id
        attachment = world.planning_scene_state().attachments[0]
        assert attachment.child_link_id == root_link.link_id
        assert attachment.child_frame_id == root_link.frame_id
    finally:
        session.close()


def test_locked_joint_frame_source_resolves_and_unsupported_native_named_fails_preflight() -> None:
    base = planning_spec(environments=1)
    robot = next(item for item in base.entities if item.path == EntityPath("/robot"))
    declarations = planning_frame_declarations()
    entries = list(declarations["entries"])
    entries[0] = {
        "semantic_key": "annotation.home",
        "role": "annotation",
        "owner_link": "robot",
        "source": {"kind": "joint", "name": "肩关节"},
    }
    declarations["entries"] = tuple(entries)
    metadata = robot.metadata.to_dict()
    metadata["planning_frame_declarations"] = declarations
    joint_source_robot = replace(robot, metadata=FrozenMap(metadata))
    joint_source_spec = replace(
        base,
        world_id="planning-joint-frame-source",
        entities=tuple(joint_source_robot if item.path == robot.path else item for item in base.entities),
    )
    session = FakeProvider().open()
    world = session.build(joint_source_spec)
    try:
        catalog = world.planning_scene_catalog()
        annotation = next(frame for frame in catalog.frames if frame.semantic_key == "annotation.home")
        shoulder = next(joint for joint in catalog.joints if joint.authored_name == "肩关节")
        assert annotation.parent_frame_id == shoulder.axis_frame_id
        assert annotation.owner_link_id == shoulder.parent_link_id
    finally:
        session.close()

    native_entries = list(entries)
    native_entries[0] = {
        "semantic_key": "annotation.home",
        "role": "annotation",
        "owner_link": "robot",
        "source": {"kind": "native_named", "name": "native_annotation"},
    }
    declarations["entries"] = tuple(native_entries)
    metadata["planning_frame_declarations"] = declarations
    native_robot = replace(robot, metadata=FrozenMap(metadata))
    native_spec = replace(
        base,
        world_id="planning-native-frame-unsupported",
        entities=tuple(native_robot if item.path == robot.path else item for item in base.entities),
    )
    session = FakeProvider().open()
    try:
        with pytest.raises(PlanningSceneIncompleteError) as caught:
            session.build(native_spec)
        assert caught.value.operation == "planning_scene.preflight"
        assert session.state is SessionState.OPEN
    finally:
        session.close()


def test_catalog_ids_are_stable_across_rebuild_and_content_digest_is_verified() -> None:
    session = FakeProvider().open()
    first = session.build(planning_spec(environments=1))
    first_catalog = first.planning_scene_catalog()
    first.close()
    second = session.build(planning_spec(environments=1))
    try:
        second_catalog = second.planning_scene_catalog()
        assert tuple(entity.entity_id for entity in first_catalog.entities) == tuple(
            entity.entity_id for entity in second_catalog.entities
        )
        assert first_catalog.generation != second_catalog.generation
        assert first_catalog.content_sha256 != second_catalog.content_sha256
        with pytest.raises(PlanningSceneContractError, match="content_sha256"):
            replace(second_catalog, content_sha256="0" * 64)
    finally:
        session.close()


def test_asset_text_is_detached_before_hashing_and_never_dispatches_overrides() -> None:
    class HostileAsset(str):
        encode_calls = 0

        def encode(self, *args, **kwargs):
            del args, kwargs
            type(self).encode_calls += 1
            raise KeyboardInterrupt("credential TOKEN")

        def __str__(self) -> str:
            raise RuntimeError("caller string conversion must not run")

    base = planning_spec(environments=1)
    container = next(item for item in base.entities if item.path == EntityPath("/container"))
    assert container.asset_uri is not None
    hostile_asset = HostileAsset(container.asset_uri)
    hostile_container = replace(container, asset_uri=hostile_asset)
    hostile_spec = replace(
        base,
        world_id="planning-hostile-asset-text",
        entities=tuple(hostile_container if item.path == container.path else item for item in base.entities),
    )
    session = FakeProvider().open()
    world = session.build(hostile_spec)
    try:
        catalog = world.planning_scene_catalog()
        owner = entity_by_path(catalog, "/container")
        geometry = next(item for item in catalog.geometries if item.owner_entity_id == owner.entity_id)
        source_only = hashlib.sha256(str.__str__(hostile_asset).encode("utf-8")).hexdigest()
        assert HostileAsset.encode_calls == 0
        assert geometry.provenance_sha256 != source_only
        assert type(geometry.provenance_sha256) is str
    finally:
        session.close()


def test_every_geometry_provenance_binds_provider_version_and_effective_parameters() -> None:
    spec = planning_spec(environments=1)
    original_digest, original = capture_planning_provenance(FAKE_DESCRIPTOR, spec)
    changed_descriptor = replace(FAKE_DESCRIPTOR, version="99.0-remediation-probe")
    changed_digest, changed = capture_planning_provenance(changed_descriptor, spec)
    assert original.keys() == changed.keys()
    assert all(original[path] != changed[path] for path in original)
    assert original_digest != changed_digest

    payload = next(item for item in spec.entities if item.path == EntityPath("/payload"))
    changed_payload = replace(payload, box=BoxGeometrySpec(dimensions_m=(0.7, 0.6, 0.5)))
    effective_spec = replace(
        spec,
        entities=tuple(changed_payload if item.path == payload.path else item for item in spec.entities),
    )
    effective_digest, effective = capture_planning_provenance(FAKE_DESCRIPTOR, effective_spec)
    assert original["/payload"] != effective["/payload"]
    assert original_digest != effective_digest


def test_articulation_geometry_provenance_binds_locked_asset_source() -> None:
    def capture(asset_uri: str) -> tuple[str, str]:
        spec = WorldSpec(
            "planning-articulation-provenance",
            (
                EntitySpec(
                    EntityPath("/machine"),
                    EntityKind.ARTICULATION,
                    asset_uri=asset_uri,
                    joint_names=("hinge",),
                ),
            ),
            environments=EnvironmentSpec(1),
            requirements=(planning_requirement(),),
        )
        session = FakeProvider().open()
        world = session.build(spec)
        try:
            catalog = world.planning_scene_catalog()
            machine = entity_by_path(catalog, "/machine")
            geometry = next(item for item in catalog.geometries if item.owner_entity_id == machine.entity_id)
            return catalog.content_sha256, geometry.provenance_sha256
        finally:
            session.close()

    first = capture("asset://fixture/model-a")
    second = capture("asset://fixture/model-b")
    assert first[0] != second[0]
    assert first[1] != second[1]


def test_logical_path_validation_accepts_absolute_paths_and_rejects_invalid_paths(planning_world) -> None:
    descriptor = entity_by_path(planning_world.planning_scene_catalog(), "/payload")
    assert descriptor.path == "/payload"
    for path in ("payload", "/", "/payload/", "/payload//child", "/payload/../child", "/bad space"):
        with pytest.raises(PlanningSceneContractError):
            replace(descriptor, path=path)


def test_named_frame_may_be_owned_by_link_without_being_primary_link_frame(planning_world) -> None:
    catalog = planning_world.planning_scene_catalog()
    owner = entity_by_path(catalog, "/robot")
    link = next(item for item in catalog.links if item.link_id == owner.link_ids[0])
    frame = PlanningFrameDescriptor(
        "frame.robot.ee",
        PlanningFrameKind.NAMED,
        link.frame_id,
        owner.entity_id,
        link.link_id,
        PlanningFrameRole.EE,
        "tcp",
    )
    updated_owner = replace(owner, frame_ids=tuple(sorted((*owner.frame_ids, frame.frame_id))))
    entities = tuple(updated_owner if item.entity_id == owner.entity_id else item for item in catalog.entities)
    rebuilt = PlanningSceneCatalog.build(
        catalog.provider_id,
        catalog.world_id,
        catalog.generation,
        catalog.environment_index,
        catalog.catalog_revision,
        catalog.geometry_revision,
        entities,
        catalog.links,
        catalog.joints,
        tuple(sorted((*catalog.frames, frame), key=lambda item: item.frame_id)),
        catalog.geometries,
    )
    assert any(item.frame_id == "frame.robot.ee" for item in rebuilt.frames)


def test_compound_parts_keep_nontrivial_geometry_local_poses(planning_world) -> None:
    catalog = planning_world.planning_scene_catalog()
    compound = next(
        geometry
        for geometry in catalog.geometries
        if geometry.representation is PlanningGeometryRepresentation.COMPOUND
    )
    assert type(compound.inline) is PlanningCompoundGeometry
    assert any(part.local_pose.position_m != (0.0, 0.0, 0.0) for part in compound.inline.parts)
    assert any(part.local_pose.orientation_xyzw != (0.0, 0.0, 0.0, 1.0) for part in compound.inline.parts)
    assert compound.geometry_id != compound.parent_frame_id


def test_multi_parent_and_closed_chain_attachments_are_preserved(planning_world) -> None:
    catalog = planning_world.planning_scene_catalog()
    state = planning_world.planning_scene_state()
    payload_id = entity_by_path(catalog, "/payload").entity_id
    child_records = tuple(item for item in state.attachments if item.child_entity_id == payload_id)
    assert len(child_records) == 2
    edges = {(item.parent_entity_id, item.child_entity_id) for item in state.attachments}
    left_id = entity_by_path(catalog, "/left").entity_id
    assert (left_id, payload_id) in edges and (payload_id, left_id) in edges
    assert all(type(item) is PlanningAttachment for item in state.attachments)
    state.validate_against(catalog)


def test_step_yields_coherent_state_delta_and_does_not_materialize_mesh(planning_world) -> None:
    catalog = planning_world.planning_scene_catalog()
    before = planning_world.planning_scene_state()
    runtime = planning_world._planning_runtime
    assert runtime is not None and runtime.geometry_materializations == 0
    planning_world.step(4)
    delta = planning_world.planning_scene_delta(before.sequence)
    assert delta.kind is PlanningSceneDeltaKind.STATE
    result = delta.apply(catalog, before)
    assert result is not None and result[1] == planning_world.planning_scene_state()
    assert result[1].tick.step_index == 4
    assert runtime.geometry_materializations == 0
    with pytest.raises(PlanningSceneDeltaContinuityError, match="no committed planning delta"):
        planning_world.planning_scene_delta(result[1].sequence)


def test_reset_loses_delta_continuity_and_revokes_existing_leases(planning_world) -> None:
    before_catalog = planning_world.planning_scene_catalog()
    before = planning_world.planning_scene_state()
    geometry_id = resource_geometry_id(before_catalog)
    lease = planning_world.resolve_planning_geometry(geometry_id)
    planning_world.reset((0,))
    after_catalog = planning_world.planning_scene_catalog()
    after = planning_world.planning_scene_state()
    assert after_catalog.generation == after.generation == before.generation + 1
    assert after_catalog.content_sha256 != before_catalog.content_sha256
    delta = planning_world.planning_scene_delta(before.sequence)
    assert delta.kind is PlanningSceneDeltaKind.RESYNC and delta.resync_required
    assert delta.previous_catalog_content_sha256 is None and delta.catalog_content_sha256 is None
    assert delta.previous_catalog_revision == delta.catalog_revision
    assert delta.previous_geometry_revision == delta.geometry_revision
    assert delta.catalog is None and delta.state is None and delta.attachments == ()
    assert delta.apply(before_catalog, before) is None
    with pytest.raises(PlanningGeometryResourceRevokedError):
        _ = lease.descriptor
    with pytest.raises(PlanningGeometryResourceRevokedError):
        lease.read()
    lease.close()
    lease.close()


def test_partial_reset_isolates_generation_history_resync_and_lease_authority(planning_world) -> None:
    catalogs_before = tuple(planning_world.planning_scene_catalog(index) for index in (0, 1))
    states_before = tuple(planning_world.planning_scene_state(index) for index in (0, 1))
    leases = tuple(
        planning_world.resolve_planning_geometry(
            resource_geometry_id(catalogs_before[index]),
            environment_index=index,
        )
        for index in (0, 1)
    )

    result = planning_world.reset((0,))
    assert result.environment_indices == (0,)
    catalogs_after = tuple(planning_world.planning_scene_catalog(index) for index in (0, 1))
    assert tuple(item.generation for item in catalogs_after) == (
        catalogs_before[0].generation + 1,
        catalogs_before[1].generation,
    )
    assert planning_world.planning_scene_state(1) is states_before[1]
    env0_delta = planning_world.planning_scene_delta(states_before[0].sequence, 0)
    assert env0_delta.kind is PlanningSceneDeltaKind.RESYNC and env0_delta.resync_required
    with pytest.raises(PlanningSceneDeltaContinuityError, match="no committed planning delta"):
        planning_world.planning_scene_delta(states_before[1].sequence, 1)
    with pytest.raises(PlanningGeometryResourceRevokedError):
        leases[0].read(0, 1)
    assert len(leases[1].read(0, 1)) == 1

    planning_world.reset((1,))
    env1_delta = planning_world.planning_scene_delta(states_before[1].sequence, 1)
    assert env1_delta.kind is PlanningSceneDeltaKind.RESYNC and env1_delta.resync_required
    with pytest.raises(PlanningGeometryResourceRevokedError):
        leases[1].read(0, 1)
    for lease in leases:
        lease.close()


def test_scene_command_commit_advances_only_target_environment(planning_world) -> None:
    states_before = tuple(planning_world.planning_scene_state(index) for index in (0, 1))
    target_pose = Pose((3.0, 2.0, 1.0))
    result = planning_world.apply_scene_command(
        SceneCommand(
            "planning-env-zero-pose",
            "contract-test",
            "lease",
            planning_world.generation,
            SceneCommandKind.SET_POSE,
            EntityPath("/container"),
            environment_index=0,
            target_pose=target_pose,
        )
    )

    assert result.status is SceneCommandStatus.APPLIED
    state_zero = planning_world.planning_scene_state(0)
    assert state_zero.sequence == states_before[0].sequence + 1
    assert (
        next(
            item.pose.position_m
            for item in state_zero.entities
            if item.entity_id == entity_by_path(planning_world.planning_scene_catalog(0), "/container").entity_id
        )
        == target_pose.position
    )
    assert planning_world.planning_scene_state(1) is states_before[1]
    with pytest.raises(PlanningSceneDeltaContinuityError, match="no committed planning delta"):
        planning_world.planning_scene_delta(states_before[1].sequence, 1)


def test_partial_reset_resource_isolation_stress(planning_world) -> None:
    runtime = planning_world._planning_runtime
    assert runtime is not None
    expected_generations = [planning_world.planning_scene_catalog(index).generation for index in (0, 1)]
    for cycle in range(64):
        target = cycle % 2
        untouched = 1 - target
        target_catalog = planning_world.planning_scene_catalog(target)
        untouched_catalog = planning_world.planning_scene_catalog(untouched)
        target_lease = planning_world.resolve_planning_geometry(
            resource_geometry_id(target_catalog),
            environment_index=target,
        )
        untouched_lease = planning_world.resolve_planning_geometry(
            resource_geometry_id(untouched_catalog),
            environment_index=untouched,
        )
        planning_world.reset((target,))
        expected_generations[target] += 1
        assert [planning_world.planning_scene_catalog(index).generation for index in (0, 1)] == expected_generations
        with pytest.raises(PlanningGeometryResourceRevokedError):
            target_lease.read(0, 1)
        assert len(untouched_lease.read(0, 1)) == 1

        states_before_command = tuple(planning_world.planning_scene_state(index) for index in (0, 1))
        untouched_runtime = runtime.environments[untouched]
        untouched_revision_snapshot = (
            untouched_runtime.sequence,
            untouched_runtime.world_revision,
            untouched_runtime.transform_revision,
            tuple(untouched_runtime.history.items()),
            untouched_runtime.force_resync,
        )
        command_result = planning_world.apply_scene_command(
            SceneCommand(
                f"planning-stress-pose-{cycle}",
                "contract-test",
                "lease",
                planning_world.generation,
                SceneCommandKind.SET_POSE,
                EntityPath("/container"),
                environment_index=target,
                target_pose=Pose((float(cycle + 1), float(target), 0.0)),
            )
        )
        assert command_result.status is SceneCommandStatus.APPLIED
        assert planning_world.planning_scene_state(target).sequence == states_before_command[target].sequence + 1
        assert planning_world.planning_scene_state(untouched) is states_before_command[untouched]
        assert (
            untouched_runtime.sequence,
            untouched_runtime.world_revision,
            untouched_runtime.transform_revision,
            tuple(untouched_runtime.history.items()),
            untouched_runtime.force_resync,
        ) == untouched_revision_snapshot
        assert len(untouched_lease.read(0, 1)) == 1

        target_lease.close()
        untouched_lease.close()
        assert len(runtime.storage_cache) == 1
    assert runtime.geometry_materializations == 1


def test_counter_saturation_produces_one_terminal_resync_then_rejects_mutation_atomically(planning_world) -> None:
    maximum = 2**63 - 1
    runtime = planning_world._planning_runtime
    assert runtime is not None
    for environment_index, environment_runtime in runtime.environments.items():
        current = planning_world.planning_scene_state(environment_index)
        saturated = replace(
            current,
            sequence=maximum - 1,
            world_revision=maximum - 1,
            transform_revision=maximum - 1,
        )
        environment_runtime.sequence = maximum - 1
        environment_runtime.world_revision = maximum - 1
        environment_runtime.transform_revision = maximum - 1
        environment_runtime.history = {maximum - 1: saturated}
    planning_world._planning_commit_state()
    delta = planning_world.planning_scene_delta(maximum - 1)
    assert delta.kind is PlanningSceneDeltaKind.RESYNC
    assert delta.resync_required and delta.catalog is None and delta.state is None and not delta.attachments

    payload_runtime = planning_world._rigids[EntityPath("/payload")]
    position_before = tuple(payload_runtime.positions[0])
    state_before = planning_world.planning_scene_state(0)
    scene_sequence_before = planning_world._scene_sequence

    class DerivedSceneCommand(SceneCommand):
        pass

    with pytest.raises(PlanningSceneContractError, match="identity is exhausted") as caught:
        planning_world.apply_scene_command(
            DerivedSceneCommand(
                "saturated-command",
                "test",
                "lease",
                planning_world.generation,
                SceneCommandKind.SET_POSE,
                EntityPath("/payload"),
                target_pose=Pose((10.0, 0.0, 0.0)),
            )
        )
    assert caught.value.__cause__ is None and caught.value.__context__ is None
    assert tuple(payload_runtime.positions[0]) == position_before
    assert planning_world.planning_scene_state(0) is state_before
    assert planning_world._scene_sequence == scene_sequence_before

    step_index_before = planning_world._step_index
    velocity_before = tuple(payload_runtime.linear_velocities[0])
    with pytest.raises(PlanningSceneContractError, match="identity is exhausted"):
        planning_world.step()
    assert planning_world._step_index == step_index_before
    assert tuple(payload_runtime.positions[0]) == position_before
    assert tuple(payload_runtime.linear_velocities[0]) == velocity_before
    assert planning_world.planning_scene_state(0) is state_before


def test_geometry_lease_identity_exhaustion_is_bounded_and_cache_atomic(planning_world) -> None:
    maximum = 2**63 - 1
    catalog = planning_world.planning_scene_catalog()
    geometry_id = resource_geometry_id(catalog)
    runtime = planning_world._planning_runtime
    environment_runtime = runtime.environments[0]
    environment_runtime.lease_serial = maximum
    cache_before = dict(runtime.storage_cache)
    materializations_before = runtime.geometry_materializations

    with pytest.raises(PlanningSceneContractError, match="lease identity is exhausted") as caught:
        planning_world.resolve_planning_geometry(geometry_id)
    assert caught.value.__cause__ is None and caught.value.__context__ is None
    assert environment_runtime.lease_serial == maximum
    assert runtime.storage_cache == cache_before
    assert runtime.geometry_materializations == materializations_before


@pytest.mark.parametrize(
    ("time_step_seconds", "gravity_m_s2", "count", "message"),
    (
        (1.0e308, (0.0, 0.0, 0.0), 2, "simulation time"),
        (1.0e200, (0.0, 0.0, -9.81), 1, "finite"),
    ),
)
def test_failed_numeric_step_rolls_back_physics_clock_and_planning_publication(
    time_step_seconds: float,
    gravity_m_s2: tuple[float, float, float],
    count: int,
    message: str,
) -> None:
    spec = WorldSpec(
        "planning-numeric-step-atomicity",
        (EntitySpec(EntityPath("/payload"), EntityKind.RIGID_BODY, box=BoxGeometrySpec()),),
        environments=EnvironmentSpec(1),
        physics=PhysicsSpec(time_step_seconds=time_step_seconds, gravity_m_s2=gravity_m_s2),
        requirements=(planning_requirement(),),
    )
    session = FakeProvider().open()
    world = session.build(spec)
    try:
        environment_runtime = world._planning_runtime.environments[0]
        state_before = world.planning_scene_state()
        history_before = dict(environment_runtime.history)
        payload_runtime = world._rigids[EntityPath("/payload")]
        physical_before = (
            tuple(payload_runtime.positions[0]),
            tuple(payload_runtime.orientations[0]),
            tuple(payload_runtime.linear_velocities[0]),
            tuple(payload_runtime.angular_velocities[0]),
        )
        counters_before = (
            world._step_index,
            world._scene_sequence,
            environment_runtime.sequence,
            environment_runtime.world_revision,
            environment_runtime.transform_revision,
        )

        with pytest.raises(PlanningSceneContractError, match=message) as caught:
            world.step(count)
        assert caught.value.__cause__ is None and caught.value.__context__ is None
        assert (
            tuple(payload_runtime.positions[0]),
            tuple(payload_runtime.orientations[0]),
            tuple(payload_runtime.linear_velocities[0]),
            tuple(payload_runtime.angular_velocities[0]),
        ) == physical_before
        assert (
            world._step_index,
            world._scene_sequence,
            environment_runtime.sequence,
            environment_runtime.world_revision,
            environment_runtime.transform_revision,
        ) == counters_before
        assert environment_runtime.history == history_before
        assert world.planning_scene_state() is state_before
    finally:
        session.close()


def test_multi_environment_commit_capture_failure_publishes_no_partial_identity(planning_world, monkeypatch) -> None:
    runtime = planning_world._planning_runtime
    before = {
        index: (
            environment.sequence,
            environment.world_revision,
            environment.transform_revision,
            environment.history,
            environment.force_resync,
        )
        for index, environment in runtime.environments.items()
    }
    capture = planning_world._capture_planning_state

    def fail_second_environment(environment_index: int):
        if environment_index == 1:
            raise PlanningSceneContractError(
                "injected staged capture failure",
                operation="planning_scene.commit",
            )
        return capture(environment_index)

    monkeypatch.setattr(planning_world, "_capture_planning_state", fail_second_environment)
    with pytest.raises(PlanningSceneContractError, match="injected staged capture failure"):
        planning_world._planning_commit_state()

    for index, environment in runtime.environments.items():
        previous = before[index]
        assert (
            environment.sequence,
            environment.world_revision,
            environment.transform_revision,
        ) == previous[:3]
        assert environment.history is previous[3]
        assert environment.force_resync is previous[4]


def test_failed_step_restores_articulation_deformable_and_fluid_runtime(monkeypatch) -> None:
    spec = WorldSpec(
        "planning-all-runtime-rollback",
        (
            EntitySpec(EntityPath("/arm"), EntityKind.ARTICULATION, joint_names=("joint",)),
            EntitySpec(EntityPath("/cloth"), EntityKind.SURFACE_DEFORMABLE, deformable=surface_body()),
            EntitySpec(EntityPath("/water"), EntityKind.PARTICLE_FLUID, particle_fluid=fluid_body()),
            EntitySpec(EntityPath("/box"), EntityKind.RIGID_BODY, box=BoxGeometrySpec()),
        ),
        environments=EnvironmentSpec(1),
        physics=PhysicsSpec(time_step_seconds=0.1, gravity_m_s2=(0.0, 0.0, -10.0)),
        requirements=(planning_requirement(),),
    )
    session = FakeProvider().open()
    world = session.build(spec)
    try:
        articulation = world._articulations[EntityPath("/arm")]
        articulation.modes[0][0] = CommandMode.VELOCITY
        articulation.targets[0][0] = 3.0

        def nested(values):
            return tuple(tuple(tuple(vector) for vector in environment) for environment in values)

        before = (
            tuple(tuple(row) for row in articulation.positions),
            tuple(tuple(row) for row in articulation.velocities),
            nested(world._points[EntityPath("/cloth")].positions),
            nested(world._points[EntityPath("/cloth")].velocities),
            nested(world._points[EntityPath("/water")].positions),
            nested(world._points[EntityPath("/water")].velocities),
            world._step_index,
            world._scene_sequence,
        )

        def reject_capture(_environment_index: int):
            raise PlanningSceneContractError("injected rollback probe", operation="planning_scene.commit")

        monkeypatch.setattr(world, "_capture_planning_state", reject_capture)
        with pytest.raises(PlanningSceneContractError, match="injected rollback probe"):
            world.step()
        assert (
            tuple(tuple(row) for row in articulation.positions),
            tuple(tuple(row) for row in articulation.velocities),
            nested(world._points[EntityPath("/cloth")].positions),
            nested(world._points[EntityPath("/cloth")].velocities),
            nested(world._points[EntityPath("/water")].positions),
            nested(world._points[EntityPath("/water")].velocities),
            world._step_index,
            world._scene_sequence,
        ) == before
    finally:
        session.close()


def test_planning_step_and_scene_predictor_reject_defensive_boundaries(planning_world) -> None:
    assert planning_world._planning_scene_command_will_commit(object()) is False
    stale = SceneCommand(
        "stale-predictor",
        "test",
        "lease",
        planning_world.generation + 1,
        SceneCommandKind.SET_POSE,
        EntityPath("/payload"),
        target_pose=Pose((1.0, 2.0, 3.0)),
    )
    assert planning_world._planning_scene_command_will_commit(stale) is False
    articulation_target = replace(
        stale,
        command_id="articulation-predictor",
        expected_generation=planning_world.generation,
        entity_path=EntityPath("/robot"),
    )
    assert planning_world._planning_scene_command_will_commit(articulation_target) is False
    with pytest.raises(ValidationError, match="positive integer"):
        planning_world.step(0)
    planning_world._step_index = 2**63 - 1
    with pytest.raises(PlanningSceneContractError, match="tick identity is exhausted"):
        planning_world.step()


@pytest.mark.parametrize(
    "attachment",
    (
        {
            "attachment_id": "attachment.unsupported",
            "parent_path": "/payload",
            "child_path": "/left",
            "unsupported": True,
        },
        {
            "attachment_id": "attachment.unknown_endpoint",
            "parent_path": "/payload",
            "child_path": "/left",
            "child_link_name": "missing-link",
        },
        {
            "attachment_id": "attachment.child_without_geometry",
            "parent_path": "/payload",
            "child_path": "/microwave",
            "child_link_name": "门 铰链 child",
        },
    ),
)
def test_attachment_registry_rejects_unsupported_or_unresolvable_records(attachment) -> None:
    base = planning_spec(environments=1)
    metadata = base.metadata.to_dict()
    metadata["planning_attachments"] = (attachment,)
    spec = replace(
        base,
        world_id=f"planning-invalid-attachment-{attachment['attachment_id']}",
        metadata=FrozenMap(metadata),
    )
    session = FakeProvider().open()
    try:
        with pytest.raises(PlanningSceneIncompleteError):
            session.build(spec)
    finally:
        session.close()


def test_generation_exhaustion_rejects_reset_before_physics_mutation(planning_world) -> None:
    maximum = 2**63 - 1
    planning_world.apply_scene_command(
        SceneCommand(
            "pose-before-exhausted-reset",
            "test",
            "lease",
            planning_world.generation,
            SceneCommandKind.SET_POSE,
            EntityPath("/payload"),
            environment_index=0,
            target_pose=Pose((9.0, 8.0, 7.0)),
        )
    )
    environment_runtime = planning_world._planning_runtime.environments[0]
    catalog = planning_world.planning_scene_catalog(0)
    state = planning_world.planning_scene_state(0)
    saturated_catalog = replace(catalog, generation=maximum, content_sha256="")
    saturated_state = replace(
        state,
        generation=maximum,
        catalog_content_sha256=saturated_catalog.content_sha256,
    )
    saturated_state.validate_against(saturated_catalog)
    environment_runtime.generation = maximum
    environment_runtime.catalog = saturated_catalog
    environment_runtime.history = {saturated_state.sequence: saturated_state}

    payload_runtime = planning_world._rigids[EntityPath("/payload")]
    physical_before = tuple(payload_runtime.positions[0])
    reset_count_before = planning_world._reset_count
    scene_sequence_before = planning_world._scene_sequence
    with pytest.raises(PlanningSceneContractError, match="generation is exhausted"):
        planning_world.reset((0,))
    assert tuple(payload_runtime.positions[0]) == physical_before
    assert planning_world.planning_scene_state(0) is saturated_state
    assert planning_world._reset_count == reset_count_before
    assert planning_world._scene_sequence == scene_sequence_before


def test_concave_triangle_mesh_resource_metadata_chunk_read_and_cache_reuse(planning_world) -> None:
    catalog = planning_world.planning_scene_catalog()
    geometry_id = resource_geometry_id(catalog)
    geometry = next(item for item in catalog.geometries if item.geometry_id == geometry_id)
    assert geometry.representation is PlanningGeometryRepresentation.TRIANGLE_MESH
    first = planning_world.resolve_planning_geometry(geometry_id)
    descriptor = first.descriptor
    assert geometry.resource_layout is descriptor.resource_layout
    assert descriptor.catalog_revision == catalog.catalog_revision
    assert descriptor.geometry_revision == catalog.geometry_revision
    descriptor.validate_against(catalog)
    assert descriptor.vertex_shape == (8, 3)
    assert descriptor.index_shape == (10, 3)
    assert descriptor.vertex_dtype is PlanningGeometryDType.FLOAT32
    assert descriptor.index_dtype is PlanningGeometryDType.UINT32
    assert descriptor.resource_layout.index_byte_offset == 96
    assert descriptor.byte_size == 216
    content = first.read(0, 100) + first.read(100, descriptor.byte_size - 100)
    assert hashlib.sha256(content).hexdigest() == descriptor.sha256 == geometry.sha256
    second = planning_world.resolve_planning_geometry(
        geometry_id,
        PlanningGeometryRepresentation.TRIANGLE_MESH,
    )
    assert second.descriptor.locator == descriptor.locator
    assert second.descriptor.lease_token != descriptor.lease_token
    runtime = planning_world._planning_runtime
    assert runtime is not None and runtime.geometry_materializations == 1
    with pytest.raises(PlanningSceneContractError):
        first.read(0, PLANNING_GEOMETRY_READ_LIMIT_BYTES + 1)
    first.close()
    second.close()


def test_fake_resource_metadata_is_exactly_bound_to_catalog_layout(planning_world) -> None:
    catalog = planning_world.planning_scene_catalog()
    geometry_id = resource_geometry_id(catalog)
    geometry = next(item for item in catalog.geometries if item.geometry_id == geometry_id)
    runtime = planning_world._planning_runtime
    environment_runtime = runtime.environments[0]
    original = environment_runtime.raw_resources[geometry_id]
    content, representation, resource_id, profile, vertex_shape, index_shape = original
    assert geometry.resource_layout is not None
    assert geometry.resource_layout.decoded_byte_size == len(content)
    same_size_vertex_shape = (vertex_shape[0] + 1, vertex_shape[1])
    same_size_index_shape = (index_shape[0] - 1, index_shape[1])
    contradictions = (
        (content, representation, "resource.forged", profile, vertex_shape, index_shape),
        (
            content,
            representation,
            resource_id,
            profile,
            same_size_vertex_shape,
            same_size_index_shape,
        ),
    )
    try:
        for contradiction in contradictions:
            environment_runtime.raw_resources[geometry_id] = contradiction
            runtime.storage_cache.clear()
            with pytest.raises(PlanningSceneContractError, match="catalog layout"):
                planning_world.resolve_planning_geometry(geometry_id)
    finally:
        environment_runtime.raw_resources[geometry_id] = original
        runtime.storage_cache.clear()


@pytest.mark.parametrize(
    ("representation", "profile", "dtype", "shape", "spacing", "byte_size"),
    (
        (
            PlanningGeometryRepresentation.SDF,
            PlanningGeometryContentProfile.SDF_DENSE_RAW_LE_V1,
            PlanningGeometryDType.FLOAT32,
            (4, 3, 2),
            (0.01, 0.02, 0.03),
            96,
        ),
        (
            PlanningGeometryRepresentation.VOXEL,
            PlanningGeometryContentProfile.VOXEL_OCCUPANCY_UINT8_V1,
            PlanningGeometryDType.UINT8,
            (4, 3, 2),
            (0.01, 0.02, 0.03),
            24,
        ),
        (
            PlanningGeometryRepresentation.HEIGHTFIELD,
            PlanningGeometryContentProfile.HEIGHTFIELD_DENSE_RAW_LE_V1,
            PlanningGeometryDType.FLOAT32,
            (4, 3),
            (0.01, 0.02),
            48,
        ),
    ),
)
def test_grid_resource_representations_have_complete_frozen_consumable_schemas(
    representation,
    profile,
    dtype,
    shape,
    spacing,
    byte_size,
) -> None:
    layout = PlanningGeometryResourceLayout(
        representation,
        profile,
        grid_dtype=dtype,
        grid_shape=shape,
        grid_spacing_m=spacing,
        grid_origin_m=(-0.1, -0.2, 0.05),
    )
    descriptor = PlanningGeometryResourceDescriptor(
        "reference.fake",
        "grid-world",
        1,
        0,
        1,
        1,
        "0" * 64,
        "lease.grid",
        "resource.grid",
        "geometry.grid",
        representation,
        PlanningGeometryStorageKind.IMMUTABLE_MEMORY,
        "cache.grid",
        profile,
        "m",
        PlanningGeometryAxisConvention.RIGHT_HANDED_Z_UP,
        layout,
        byte_size,
        "0" * 64,
    )
    assert descriptor.format is profile
    assert descriptor.grid_dtype is dtype
    assert descriptor.grid_shape == shape
    assert descriptor.grid_spacing_m == spacing
    assert descriptor.grid_origin_m == (-0.1, -0.2, 0.05)
    assert layout.index_byte_offset is None
    assert descriptor.axis_convention is PlanningGeometryAxisConvention.RIGHT_HANDED_Z_UP
    assert tuple(PlanningGeometryAxisConvention) == (PlanningGeometryAxisConvention.RIGHT_HANDED_Z_UP,)
    assert descriptor.read_span() == (0, byte_size)
    assert PLANNING_GRID_INDEX_ORDER == "shape-axes-xy-or-xyz-last-axis-contiguous"
    semantics = {
        PlanningGeometryRepresentation.SDF: (
            PLANNING_SDF_SIGN_CONVENTION,
            "negative-inside",
        ),
        PlanningGeometryRepresentation.VOXEL: (
            PLANNING_VOXEL_OCCUPANCY_CONVENTION,
            "zero-free-one-occupied",
        ),
        PlanningGeometryRepresentation.HEIGHTFIELD: (
            PLANNING_HEIGHTFIELD_SAMPLE_CONVENTION,
            "positive-z-offset-metres",
        ),
    }
    convention, serialized_marker = semantics[representation]
    assert convention and serialized_marker in profile.value
    geometry = PlanningGeometryDescriptor(
        "geometry.grid",
        "entity.grid",
        None,
        "frame.grid",
        PlanningGeometryPurpose.COLLISION,
        representation,
        PlanningGeometryLocalPose(),
        (1.0, 1.0, 1.0),
        PlanningGeometryMotionClass.STATIC,
        1,
        2**32 - 1,
        "1" * 64,
        resource_id="resource.grid",
        sha256="0" * 64,
        content_profile=profile,
        resource_layout=layout,
    )
    assert geometry.content_profile is profile
    with pytest.raises(PlanningSceneContractError):
        replace(
            geometry,
            content_profile=PlanningGeometryContentProfile.MESH_TRIANGLES_RAW_LE_V1,
        )


def test_grid_resource_schema_rejects_profile_dtype_rank_and_byte_contradictions(monkeypatch) -> None:
    layout = PlanningGeometryResourceLayout(
        PlanningGeometryRepresentation.SDF,
        PlanningGeometryContentProfile.SDF_DENSE_RAW_LE_V1,
        grid_dtype=PlanningGeometryDType.FLOAT32,
        grid_shape=(2, 2, 2),
        grid_spacing_m=(0.01, 0.01, 0.01),
        grid_origin_m=(0.0, 0.0, 0.0),
    )
    valid = PlanningGeometryResourceDescriptor(
        "reference.fake",
        "grid-world",
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
        PlanningGeometryAxisConvention.RIGHT_HANDED_Z_UP,
        layout,
        32,
        "0" * 64,
    )
    invalid = (
        {"format": PlanningGeometryContentProfile.VOXEL_OCCUPANCY_UINT8_V1},
        {"byte_size": 31},
        {"axis_convention": "right_handed_y_up"},
    )
    for override in invalid:
        with pytest.raises(PlanningSceneContractError):
            replace(valid, **override)
    invalid_layouts = (
        {"grid_dtype": PlanningGeometryDType.UINT8},
        {"grid_shape": (2, 2)},
        {"grid_spacing_m": (0.01, 0.0, 0.01)},
        {"grid_origin_m": None},
    )
    for override in invalid_layouts:
        with pytest.raises(PlanningSceneContractError):
            replace(layout, **override)
    for inline_representation in (
        PlanningGeometryRepresentation.HALFSPACE,
        PlanningGeometryRepresentation.BOX,
    ):
        with pytest.raises(PlanningSceneContractError, match="inline representations"):
            PlanningGeometryResourceLayout(
                inline_representation,
                PlanningGeometryContentProfile.MESH_TRIANGLES_RAW_LE_V1,
            )
    with pytest.raises(PlanningSceneContractError, match="profile"):
        replace(layout, content_profile=PlanningGeometryContentProfile.MESH_TRIANGLES_RAW_LE_V1)
    monkeypatch.setattr(planning_contract, "_MAX_RESOURCE_BYTES", 1)
    with pytest.raises(PlanningSceneContractError, match="decoded bytes"):
        replace(layout)


def test_schema_v2_has_one_representation_per_geometry_and_resolve_is_an_assertion(planning_world) -> None:
    declaration = FAKE_DESCRIPTOR.capabilities.get(CapabilityId("planning.scene@2"))
    assert declaration is not None
    assert declaration.properties["axis_convention"] == "right_handed_z_up"
    assert declaration.properties["resource_layout"] == "catalog-pinned-v1"
    assert declaration.properties["single_representation_per_geometry"] is True
    assert declaration.properties["representation_fallback"] is False
    geometry_id = resource_geometry_id(planning_world.planning_scene_catalog())
    inferred = planning_world.resolve_planning_geometry(geometry_id)
    asserted = planning_world.resolve_planning_geometry(
        geometry_id,
        PlanningGeometryRepresentation.TRIANGLE_MESH,
    )
    try:
        assert inferred.descriptor.resolution_key == asserted.descriptor.resolution_key
        with pytest.raises(PlanningSceneRepresentationError):
            planning_world.resolve_planning_geometry(
                geometry_id,
                PlanningGeometryRepresentation.CONVEX_MESH,
            )
    finally:
        inferred.close()
        asserted.close()


def test_lease_read_is_worker_safe_but_world_calls_are_authority_thread_only(planning_world) -> None:
    geometry_id = resource_geometry_id(planning_world.planning_scene_catalog())
    lease = planning_world.resolve_planning_geometry(geometry_id)
    results: list[object] = []

    def worker() -> None:
        results.append(lease.read(0, 16))
        try:
            planning_world.planning_scene_state()
        except BaseException as error:
            results.append(error)

    thread = threading.Thread(target=worker)
    thread.start()
    thread.join()
    assert type(results[0]) is bytes and len(results[0]) == 16
    assert type(results[1]) is PlanningSceneContractError
    lease.close()


def test_unknown_representation_and_hash_mismatch_fail_with_typed_errors(planning_world) -> None:
    catalog = planning_world.planning_scene_catalog()
    geometry_id = resource_geometry_id(catalog)
    with pytest.raises(PlanningSceneNotFoundError):
        planning_world.resolve_planning_geometry("geometry.unknown")
    with pytest.raises(PlanningSceneRepresentationError):
        planning_world.resolve_planning_geometry(geometry_id, PlanningGeometryRepresentation.CONVEX_MESH)
    runtime = planning_world._planning_runtime
    assert runtime is not None
    environment_runtime = runtime.environments[0]
    original = environment_runtime.raw_resources[geometry_id]
    environment_runtime.raw_resources[geometry_id] = (b"corrupt", *original[1:])
    with pytest.raises(PlanningSceneHashMismatchError):
        planning_world.resolve_planning_geometry(geometry_id)


def test_public_scalar_subclasses_are_detached_before_lookup_and_hash(planning_world) -> None:
    class HostileMeta(type):
        enabled = False

        def mro(cls):
            if cls.enabled:
                raise RuntimeError("secret from hostile mro")
            return super().mro()

    class HostileText(str, metaclass=HostileMeta):
        def __str__(self):
            raise RuntimeError("secret from hostile str")

        def __hash__(self):
            raise RuntimeError("secret from hostile hash")

        def __eq__(self, other):
            raise RuntimeError("secret from hostile equality")

    HostileMeta.enabled = True
    catalog = planning_world.planning_scene_catalog(0)
    geometry_id = resource_geometry_id(catalog)
    lease = planning_world.resolve_planning_geometry(
        HostileText(geometry_id),
        HostileText("triangle_mesh"),
        0,
    )
    assert type(lease.descriptor.geometry_id) is str
    lease.close()


def test_hostile_metaclass_equality_and_forged_instance_class_are_never_consulted(planning_world) -> None:
    class Sidecar:
        pass

    class HostileMeta(type):
        equality_calls = 0

        def __eq__(cls, other):
            cls.equality_calls += 1
            raise RuntimeError("hostile metaclass equality")

        __hash__ = type.__hash__

    class HostileText(str, metaclass=HostileMeta):
        def __new__(cls, value: str, sidecar: object):
            instance = super().__new__(cls, value)
            instance.sidecar = sidecar
            return instance

        @property
        def __class__(self):
            raise RuntimeError("forged instance class")

        def __str__(self):
            raise RuntimeError("hostile string conversion")

    sidecar = Sidecar()
    reference = weakref.ref(sidecar)
    catalog = planning_world.planning_scene_catalog()
    original = entity_by_path(catalog, "/payload")
    source_id = HostileText(original.entity_id, sidecar)
    source_path = HostileText(original.path, sidecar)
    detached = replace(original, entity_id=source_id, path=source_path)
    error = PlanningSceneContractError(
        HostileText("detached planning failure", sidecar),
        operation=HostileText("planning.validate", sidecar),
    )
    del sidecar, source_id, source_path
    gc.collect()
    assert reference() is None
    assert type(detached.entity_id) is str and type(detached.path) is str
    assert type(error.message) is str and type(error.operation) is str
    assert HostileMeta.equality_calls == 0


@pytest.mark.parametrize(
    "value",
    (
        "bad\x00id",
        "bad\ud800id",
        "é" * 513,
        "四" * 400,
    ),
)
def test_geometry_identity_text_budget_failures_are_typed_and_bounded(planning_world, value: str) -> None:
    with pytest.raises(PlanningSceneContractError) as caught:
        planning_world.resolve_planning_geometry(value)
    assert len(str(caught.value)) <= 1200
    assert caught.value.__cause__ is None
    assert caught.value.__context__ is None


def test_invalid_environment_sequence_and_representation_inputs_are_typed(planning_world) -> None:
    geometry_id = resource_geometry_id(planning_world.planning_scene_catalog())
    for environment in (True, -1, 2, object()):
        with pytest.raises(PlanningSceneContractError):
            planning_world.planning_scene_state(environment)
    for sequence in (True, 0, -1, object()):
        with pytest.raises(PlanningSceneContractError):
            planning_world.planning_scene_delta(sequence)
    for representation in ("triangle_mesh\x00", "四" * 2000, object()):
        with pytest.raises(PlanningSceneRepresentationError):
            planning_world.resolve_planning_geometry(geometry_id, representation)


def test_planning_errors_drop_hostile_details_cause_and_source_graph() -> None:
    class Sidecar:
        pass

    class HostileText(str):
        def __new__(cls, value: str, sidecar: object):
            instance = super().__new__(cls, value)
            instance.sidecar = sidecar
            return instance

        def __str__(self):
            raise RuntimeError("credential-bearing text")

    sidecar = Sidecar()
    reference = weakref.ref(sidecar)
    message = HostileText("public failure", sidecar)
    details = {"source": sidecar}
    cause = RuntimeError("credential-bearing cause", sidecar)
    error = PlanningSceneContractError(
        message,
        operation=HostileText("planning.operation", sidecar),
        details=details,
        cause=cause,
    )
    del sidecar, message, details, cause
    gc.collect()
    assert reference() is None
    assert error.__cause__ is None and error.__context__ is None
    assert error.details.to_dict() == {}
    assert type(error.message) is str and type(error.operation) is str
    assert len(error.message) <= 1024 and len(error.operation) <= 256


def test_planning_error_fields_obey_codepoint_and_utf8_byte_budgets() -> None:
    value = "四" * 5000
    error = PlanningSceneContractError(
        value,
        operation=value,
        backend_id=value,
        world_id=value,
        entity_path=value,
    )
    assert len(error.message) <= 1024 and len(error.message.encode("utf-8")) <= 4096
    assert len(error.operation) <= 256 and len(error.operation.encode("utf-8")) <= 1024
    for optional in (error.backend_id, error.world_id, error.entity_path):
        assert optional is not None
        assert len(optional) <= 512 and len(optional.encode("utf-8")) <= 2048


def test_text_boundaries_reject_huge_subtypes_without_full_detach_or_retention(planning_world) -> None:
    class Sidecar:
        pass

    class HugeText(str):
        def __new__(cls, value: str, sidecar: object):
            instance = super().__new__(cls, value)
            instance.sidecar = sidecar
            return instance

        def __str__(self):
            raise RuntimeError("subclass conversion must not be consulted")

    geometry_id = resource_geometry_id(planning_world.planning_scene_catalog())

    def measured(call):
        sidecar = Sidecar()
        reference = weakref.ref(sidecar)
        source = HugeText("x" * (8 * 1024 * 1024), sidecar)
        tracemalloc.start()
        try:
            try:
                result = call(source)
            except PlanningSceneError as caught:
                result = caught
            _, peak = tracemalloc.get_traced_memory()
        finally:
            tracemalloc.stop()
            source = None
            sidecar = None
            call = None
        return peak, reference, result

    calls = (
        lambda value: planning_contract.PlanningPose(
            value,
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0, 1.0),
        ),
        lambda value: planning_contract.PlanningPrimitiveGeometry(value, (1.0,)),
        lambda value: planning_world.resolve_planning_geometry(value),
        lambda value: planning_world.resolve_planning_geometry(geometry_id, value),
        lambda value: PlanningSceneContractError(
            value,
            operation=value,
            backend_id=value,
            world_id=value,
            entity_path=value,
        ),
    )
    for call in calls:
        peak, reference, result = measured(call)
        assert peak < 512 * 1024
        assert type(result) in {
            PlanningSceneContractError,
            PlanningSceneRepresentationError,
        }
        assert len(result.message.encode("utf-8")) <= 4096
        assert len(result.operation.encode("utf-8")) <= 1024
        traceback = result.__traceback__
        while traceback is not None:
            filename = traceback.tb_frame.f_code.co_filename
            if "/src/unirobosim/" in filename and filename.endswith(
                ("planning_scene.py", "fake_backend.py", "errors.py")
            ):
                assert all(type(value).__name__ != "HugeText" for value in traceback.tb_frame.f_locals.values())
            traceback = traceback.tb_next
        # A caught exception necessarily traces through this test's call site,
        # whose evaluation stack may still hold the argument.  Once that public
        # traceback is released, no library-owned value may retain the source.
        result.__traceback__ = None
        gc.collect()
        assert reference() is None


def test_error_field_four_byte_utf8_boundaries_are_exactly_bounded() -> None:
    value = "𐀀" * 2048
    error = PlanningSceneContractError(
        value,
        operation=value,
        backend_id=value,
        world_id=value,
        entity_path=value,
    )
    assert len(error.message) == 1024 and len(error.message.encode("utf-8")) == 4096
    assert len(error.operation) == 256 and len(error.operation.encode("utf-8")) == 1024
    for optional in (error.backend_id, error.world_id, error.entity_path):
        assert optional is not None
        assert len(optional) == 512 and len(optional.encode("utf-8")) == 2048


def test_failed_value_construction_scrubs_source_objects_from_library_tracebacks() -> None:
    class Sidecar:
        pass

    class InvalidIdentity:
        def __init__(self, sidecar: object) -> None:
            self.sidecar = sidecar

        @property
        def __class__(self):
            raise RuntimeError("credential-bearing forged class")

    sidecar = Sidecar()
    reference = weakref.ref(sidecar)
    source = InvalidIdentity(sidecar)
    with pytest.raises(PlanningSceneContractError) as caught:
        PlanningEntityDescriptor(
            source,
            "/safe",
            PlanningEntityKind.RIGID_OBJECT,
            True,
            "frame.safe",
            (),
            ("frame.safe",),
            (),
        )
    del source, sidecar
    gc.collect()
    assert reference() is None
    assert caught.value.__cause__ is None and caught.value.__context__ is None
    traceback = caught.value.__traceback__
    while traceback is not None:
        if traceback.tb_frame.f_code.co_filename.endswith("planning_scene.py"):
            assert all(type(value).__name__ != "InvalidIdentity" for value in traceback.tb_frame.f_locals.values())
        traceback = traceback.tb_next


def test_failed_world_and_lease_calls_scrub_source_objects_from_library_tracebacks(planning_world) -> None:
    class Sidecar:
        pass

    class InvalidValue:
        def __init__(self, sidecar: object) -> None:
            self.sidecar = sidecar

        @property
        def __class__(self):
            raise RuntimeError("credential-bearing forged class")

    sidecar = Sidecar()
    reference = weakref.ref(sidecar)
    source = InvalidValue(sidecar)
    geometry_id = resource_geometry_id(planning_world.planning_scene_catalog())
    lease = planning_world.resolve_planning_geometry(geometry_id)
    with pytest.raises(PlanningSceneContractError) as world_failure:
        planning_world.planning_scene_state(source)
    with pytest.raises(PlanningSceneContractError) as lease_failure:
        lease.read(source)
    with pytest.raises(PlanningSceneContractError) as descriptor_failure:
        lease.descriptor.read_span(source)
    failures = [world_failure.value, lease_failure.value, descriptor_failure.value]
    del source, sidecar, world_failure, lease_failure, descriptor_failure
    gc.collect()
    assert reference() is None
    for failure in failures:
        assert failure.__cause__ is None and failure.__context__ is None
        traceback = failure.__traceback__
        while traceback is not None:
            if traceback.tb_frame.f_code.co_filename.endswith(("planning_scene.py", "fake_backend.py")):
                assert all(type(value).__name__ != "InvalidValue" for value in traceback.tb_frame.f_locals.values())
            traceback = traceback.tb_next
    lease.close()


@pytest.mark.parametrize(
    "boundary",
    (planning_contract._planning_method_boundary, fake_contract._planning_error_boundary),
)
def test_error_boundaries_never_read_hostile_subclass_properties_or_retain_graph(boundary) -> None:
    class Sidecar:
        pass

    class HostilePlanningError(PlanningSceneError):
        property_reads = 0

        @property
        def message(self):
            type(self).property_reads += 1
            raise RuntimeError("hostile message property")

        @message.setter
        def message(self, value):
            self.__dict__["message"] = value

        @property
        def operation(self):
            type(self).property_reads += 1
            raise RuntimeError("hostile operation property")

        @operation.setter
        def operation(self, value):
            self.__dict__["operation"] = value

    sidecar = Sidecar()
    reference = weakref.ref(sidecar)
    source_errors = [HostilePlanningError("private", operation="private.operation")]
    source_errors[0].payload = sidecar
    source_errors[0].__cause__ = RuntimeError("private cause", sidecar)

    def failing(_value):
        raise source_errors[0]

    wrapped = boundary(failing)
    with pytest.raises(PlanningSceneContractError) as caught:
        wrapped(sidecar)
    assert type(caught.value) is PlanningSceneContractError
    assert caught.value.message == "planning-scene operation failed"
    assert caught.value.operation == "planning_scene"
    assert caught.value.__cause__ is None and caught.value.__context__ is None
    assert HostilePlanningError.property_reads == 0
    source_errors.clear()
    del wrapped, failing, sidecar
    gc.collect()
    assert reference() is None


def test_value_constructor_boundary_replaces_hostile_error_subclass_without_retention(monkeypatch) -> None:
    class Sidecar:
        pass

    class HostilePlanningError(PlanningSceneError):
        property_reads = 0

        def __getattribute__(self, name):
            if name in {"message", "operation", "backend_id", "world_id", "entity_path"}:
                type(self).property_reads += 1
                raise RuntimeError("hostile field access")
            return super().__getattribute__(name)

    sidecar = Sidecar()
    reference = weakref.ref(sidecar)
    source_errors = [HostilePlanningError("private", operation="private.operation")]
    source_errors[0].payload = sidecar

    def hostile_invalid(*_args, **_kwargs):
        raise source_errors[0]

    monkeypatch.setattr(planning_contract, "_invalid", hostile_invalid)
    with pytest.raises(PlanningSceneContractError) as caught:
        planning_contract.PlanningPose("", (), ())
    monkeypatch.undo()
    assert type(caught.value) is PlanningSceneContractError
    assert caught.value.message == "planning-scene operation failed"
    assert caught.value.operation == "planning_scene"
    assert caught.value.__cause__ is None and caught.value.__context__ is None
    assert HostilePlanningError.property_reads == 0
    source_errors.clear()
    del hostile_invalid, sidecar
    gc.collect()
    assert reference() is None


@pytest.mark.parametrize(
    "boundary",
    (planning_contract._planning_method_boundary, fake_contract._planning_error_boundary),
)
def test_error_boundary_detaches_mutated_exact_error_fields_before_new_traceback(boundary) -> None:
    class Sidecar:
        pass

    sidecar = Sidecar()
    reference = weakref.ref(sidecar)
    source_errors = [PlanningSceneContractError("safe", operation="safe.operation")]
    source_errors[0].message = sidecar
    source_errors[0].operation = sidecar
    source_errors[0].backend_id = sidecar
    source_errors[0].world_id = sidecar
    source_errors[0].entity_path = sidecar
    source_errors[0].details = sidecar

    def failing():
        raise source_errors[0]

    wrapped = boundary(failing)
    with pytest.raises(PlanningSceneContractError) as caught:
        wrapped()
    assert type(caught.value) is PlanningSceneContractError
    assert caught.value.message == "planning-scene operation failed"
    assert caught.value.operation == "planning_scene"
    assert caught.value.backend_id in {None, "unavailable"}
    assert caught.value.world_id == caught.value.backend_id
    assert caught.value.entity_path == caught.value.backend_id
    source_errors.clear()
    del wrapped, failing, sidecar
    gc.collect()
    assert reference() is None


def test_protocol_runtime_behavior_is_identical_to_static_member_shape(planning_world) -> None:
    assert inspect.signature(PlanningSceneWorld.planning_scene_catalog) == inspect.signature(
        type(planning_world).planning_scene_catalog
    )
    assert inspect.signature(PlanningSceneWorld.planning_scene_state) == inspect.signature(
        type(planning_world).planning_scene_state
    )
    assert inspect.signature(PlanningSceneWorld.planning_scene_delta) == inspect.signature(
        type(planning_world).planning_scene_delta
    )
    assert inspect.signature(PlanningSceneWorld.resolve_planning_geometry) == inspect.signature(
        type(planning_world).resolve_planning_geometry
    )
