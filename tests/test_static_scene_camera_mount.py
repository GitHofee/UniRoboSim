from __future__ import annotations

import hashlib
import math
from dataclasses import replace
from pathlib import Path

import pytest

import unirobosim
import unirobosim.api as unirobosim_api
from unirobosim import (
    PHYSICAL_WORLD_SCHEMA_VERSION,
    WORLD_SCHEMA_VERSION,
    BoxGeometrySpec,
    BuildInput,
    BuildResourceEntry,
    BuildResourceManifest,
    BuildSourceEntry,
    CameraModality,
    CameraMountSpec,
    CameraSpec,
    CapabilityId,
    CapabilityNegotiationError,
    CapabilityRequirement,
    CapabilitySet,
    EntityKind,
    EntityPath,
    EntitySpec,
    FrozenMap,
    LocalSourceIdentity,
    PlanningEntityKind,
    Pose,
    SceneVisualKind,
    ValidationError,
    WorldSpec,
)
from unirobosim.testing import FAKE_DESCRIPTOR, FakeSession


def _static_scene(path: str = "/scene", *, scale: tuple[float, float, float] = (1.0, 1.0, 1.0)) -> EntitySpec:
    return EntitySpec(
        EntityPath(path),
        EntityKind.STATIC_SCENE,
        asset_uri="asset://workspace",
        scale_xyz=scale,
    )


def _build_input(tmp_path: Path) -> BuildInput:
    payload = b"fake-usd-scene\n"
    source_path = tmp_path / "workspace.usd"
    source_path.write_bytes(payload)
    digest = hashlib.sha256(payload).hexdigest()
    entry = BuildResourceEntry(
        entity_id="entity.scene",
        component_id="component.scene",
        resource_id="resource.scene",
        role="simulation.scene",
        media_type="model/vnd.usd",
        requested_uri="asset://workspace",
        resolved_uri=str(source_path),
        canonical_source_identity=f"sha256:{digest}",
        byte_size=len(payload),
        sha256=digest,
        selected_simulation_input=True,
        purposes=("collision", "planning", "simulation", "visual"),
        relative_bundle_path=source_path.name,
    )
    stat_result = source_path.stat()
    source = BuildSourceEntry(
        resource_id=entry.resource_id,
        source_kind="local-file",
        source_root=str(tmp_path),
        relative_source_path=source_path.name,
        expected_identity=LocalSourceIdentity(
            stat_result.st_dev,
            stat_result.st_ino,
            stat_result.st_mode,
            stat_result.st_size,
            stat_result.st_mtime_ns,
            stat_result.st_ctime_ns,
        ),
        expected_sha256=digest,
    )
    return BuildInput(manifest=BuildResourceManifest((entry,)), sources=(source,))


def test_static_scene_is_physical_asset_only_and_at_most_one() -> None:
    assert unirobosim.CameraMountSpec is CameraMountSpec
    assert unirobosim_api.CameraMountSpec is CameraMountSpec
    assert EntityKind.STATIC_SCENE.value == "static_scene"
    assert CameraModality.NORMALS.value == "normals"

    with pytest.raises(ValidationError, match="asset URI"):
        EntitySpec(EntityPath("/scene"), EntityKind.STATIC_SCENE)

    scene = _static_scene(scale=(0.001, 2.0, 3.0))
    with pytest.raises(ValidationError, match="v0alpha5"):
        WorldSpec("legacy-static", (scene,), schema_version=WORLD_SCHEMA_VERSION)
    with pytest.raises(ValidationError, match="at most one"):
        WorldSpec(
            "duplicate-static",
            (scene, _static_scene("/second")),
            schema_version=PHYSICAL_WORLD_SCHEMA_VERSION,
            build_resource_manifest_sha256="a" * 64,
        )

    with pytest.raises(ValidationError, match="positive"):
        _static_scene(scale=(1.0, 0.0, 1.0))


def test_static_scene_serialization_and_capability_demands_are_additive() -> None:
    scene = _static_scene(scale=(0.001, 2.0, 3.0))
    world = WorldSpec(
        "static-contract",
        (scene,),
        requirements=(
            CapabilityRequirement(CapabilityId("scene.static@1"), required=False),
            CapabilityRequirement(CapabilityId("entity.scale.static_scene@1"), required=False),
        ),
        schema_version=PHYSICAL_WORLD_SCHEMA_VERSION,
        build_resource_manifest_sha256="b" * 64,
    )
    requirements = {requirement.capability.value: requirement for requirement in world.requirements}
    assert requirements["scene.static@1"].required
    assert requirements["entity.scale.static_scene@1"].required
    assert "entity.scale.rigid@1" not in requirements
    assert scene.to_dict(PHYSICAL_WORLD_SCHEMA_VERSION)["kind"] == "static_scene"
    assert scene.to_dict(PHYSICAL_WORLD_SCHEMA_VERSION)["scale_xyz"] == [0.001, 2.0, 3.0]
    assert "mount" not in scene.to_dict(PHYSICAL_WORLD_SCHEMA_VERSION)

    unit_world = replace(
        world,
        world_id="unit-static",
        entities=(_static_scene(),),
        requirements=(),
    )
    unit_requirements = {requirement.capability.value for requirement in unit_world.requirements}
    assert "scene.static@1" in unit_requirements
    assert "entity.scale.static_scene@1" not in unit_requirements


def test_camera_mount_value_and_world_relationship_validation() -> None:
    with pytest.raises(ValidationError, match="EntityPath"):
        CameraMountSpec("/parent")  # type: ignore[arg-type]
    parent = EntitySpec(EntityPath("/parent"), EntityKind.RIGID_BODY, box=BoxGeometrySpec())
    mount = CameraMountSpec(EntityPath("/parent"))
    camera = EntitySpec(
        EntityPath("/camera"),
        EntityKind.CAMERA_SENSOR,
        pose=Pose((0.1, 0.2, 0.3)),
        camera=CameraSpec(),
        mount=mount,
    )
    physical = WorldSpec(
        "mounted-camera",
        (parent, camera),
        schema_version=PHYSICAL_WORLD_SCHEMA_VERSION,
    )
    payload = physical.to_dict()["entities"][0]
    assert payload["path"] == "/camera"
    assert payload["pose"]["position_m"] == [0.1, 0.2, 0.3]
    assert payload["mount"] == {"parent_path": "/parent", "parent_link_name": None}

    with pytest.raises(ValidationError, match="non-empty string"):
        CameraMountSpec(EntityPath("/parent"), "   ")
    with pytest.raises(ValidationError, match="CameraMountSpec"):
        replace(camera, mount=object())
    with pytest.raises(ValidationError, match="only camera"):
        replace(parent, mount=mount)
    with pytest.raises(ValidationError, match="non-unit scale"):
        replace(camera, scale_xyz=(2.0, 2.0, 2.0))
    with pytest.raises(ValidationError, match="v0alpha5"):
        WorldSpec("legacy-mount", (parent, camera), schema_version=WORLD_SCHEMA_VERSION)
    with pytest.raises(ValidationError, match="does not exist"):
        WorldSpec(
            "missing-parent",
            (replace(camera, mount=CameraMountSpec(EntityPath("/missing"))),),
            schema_version=PHYSICAL_WORLD_SCHEMA_VERSION,
        )
    with pytest.raises(ValidationError, match="itself"):
        WorldSpec(
            "self-parent",
            (replace(camera, mount=CameraMountSpec(camera.path)),),
            schema_version=PHYSICAL_WORLD_SCHEMA_VERSION,
        )
    with pytest.raises(ValidationError, match="parent_link_name"):
        WorldSpec(
            "rigid-link-parent",
            (parent, replace(camera, mount=CameraMountSpec(parent.path, "tool"))),
            schema_version=PHYSICAL_WORLD_SCHEMA_VERSION,
        )

    articulation = EntitySpec(
        EntityPath("/robot"),
        EntityKind.ARTICULATION,
        joint_names=("joint",),
    )
    linked_camera = replace(camera, mount=CameraMountSpec(articulation.path, "tool"))
    assert WorldSpec(
        "articulation-link-parent",
        (articulation, linked_camera),
        schema_version=PHYSICAL_WORLD_SCHEMA_VERSION,
    )


def test_camera_mount_rejects_nonphysical_parent_kinds() -> None:
    parent = EntitySpec(EntityPath("/parent-camera"), EntityKind.CAMERA_SENSOR, camera=CameraSpec())
    child = EntitySpec(
        EntityPath("/child-camera"),
        EntityKind.CAMERA_SENSOR,
        camera=CameraSpec(),
        mount=CameraMountSpec(parent.path),
    )
    with pytest.raises(ValidationError, match="rigid body or articulation"):
        WorldSpec(
            "camera-parent",
            (parent, child),
            schema_version=PHYSICAL_WORLD_SCHEMA_VERSION,
        )


def test_normals_contract_and_fake_sample_are_complete() -> None:
    camera = EntitySpec(
        EntityPath("/camera"),
        EntityKind.CAMERA_SENSOR,
        camera=CameraSpec(width_px=3, height_px=2, modalities=(CameraModality.NORMALS,)),
    )
    world_spec = WorldSpec("normals", (camera,), schema_version=PHYSICAL_WORLD_SCHEMA_VERSION)
    requirement_ids = {requirement.capability.value for requirement in world_spec.requirements}
    assert "sensor.camera.normals@1" in requirement_ids
    declaration = FAKE_DESCRIPTOR.capabilities.get(CapabilityId("sensor.camera.normals@1"))
    assert declaration is not None

    world = FakeSession(FAKE_DESCRIPTOR).build(world_spec)
    sample = world.read_sensor(world.resolve(camera.path))
    normals = sample.channel(CameraModality.NORMALS)
    assert normals.shape == (1, 2, 3, 3)
    assert normals.dtype == "float32"
    assert normals.values == (0.0, 0.0, 1.0) * 6


def test_fake_static_scene_snapshot_mount_pose_and_planning_mapping(tmp_path: Path) -> None:
    build_input = _build_input(tmp_path)
    quarter_turn = math.sqrt(0.5)
    scene = replace(
        _static_scene(scale=(2.0, 3.0, 4.0)),
        pose=Pose((10.0, 20.0, 30.0)),
        metadata=FrozenMap({"planning_entity_kind": "robot"}),
    )
    parent = EntitySpec(
        EntityPath("/parent"),
        EntityKind.RIGID_BODY,
        pose=Pose((1.0, 2.0, 3.0), (0.0, 0.0, quarter_turn, quarter_turn)),
        box=BoxGeometrySpec(),
    )
    camera = EntitySpec(
        EntityPath("/camera"),
        EntityKind.CAMERA_SENSOR,
        pose=Pose((1.0, 0.0, 0.0)),
        camera=CameraSpec(width_px=2, height_px=2, modalities=(CameraModality.NORMALS,)),
        mount=CameraMountSpec(parent.path),
    )
    spec = WorldSpec(
        "fake-static-scene",
        (scene, parent, camera),
        requirements=(CapabilityRequirement(CapabilityId("planning.scene@2")),),
        schema_version=PHYSICAL_WORLD_SCHEMA_VERSION,
        build_resource_manifest_sha256=build_input.manifest.sha256,
    )
    assert FAKE_DESCRIPTOR.capabilities.get(CapabilityId("scene.static@1")) is not None
    assert FAKE_DESCRIPTOR.capabilities.get(CapabilityId("entity.scale.static_scene@1")) is not None

    world = FakeSession(FAKE_DESCRIPTOR).build(spec, build_input=build_input)
    snapshot_by_path = {entity.path: entity for entity in world.scene_snapshot().entities}
    static_snapshot = snapshot_by_path[scene.path]
    assert static_snapshot.kind is EntityKind.STATIC_SCENE
    assert static_snapshot.pose == scene.pose
    assert not static_snapshot.draggable
    assert static_snapshot.visuals[0].kind is SceneVisualKind.MESH
    assert static_snapshot.visuals[0].asset_uri == scene.asset_uri
    assert static_snapshot.visuals[0].metadata["scale_xyz"] == scene.scale_xyz

    camera_snapshot = snapshot_by_path[camera.path]
    assert camera_snapshot.pose.position == pytest.approx((1.0, 3.0, 3.0))
    assert camera_snapshot.pose.orientation_xyzw == pytest.approx(parent.pose.orientation_xyzw)

    catalog = world.planning_scene_catalog()
    static_descriptor = next(entity for entity in catalog.entities if entity.path == scene.path.value)
    assert static_descriptor.kind is PlanningEntityKind.OTHER
    camera_descriptor = next(entity for entity in catalog.entities if entity.path == camera.path.value)
    camera_state = next(
        entity for entity in world.planning_scene_state().entities if entity.entity_id == camera_descriptor.entity_id
    )
    assert camera_state.pose.position_m == pytest.approx((1.0, 3.0, 3.0))
    assert camera_state.pose.orientation_xyzw == pytest.approx(parent.pose.orientation_xyzw)


@pytest.mark.parametrize("capability", ("scene.static@1", "entity.scale.static_scene@1"))
def test_fake_static_scene_capabilities_are_negotiated_before_build(
    tmp_path: Path,
    capability: str,
) -> None:
    build_input = _build_input(tmp_path)
    descriptor = replace(
        FAKE_DESCRIPTOR,
        capabilities=CapabilitySet(
            tuple(
                declaration
                for declaration in FAKE_DESCRIPTOR.capabilities
                if declaration.capability != CapabilityId(capability)
            )
        ),
    )
    spec = WorldSpec(
        "missing-static-capability",
        (_static_scene(scale=(2.0, 2.0, 2.0)),),
        schema_version=PHYSICAL_WORLD_SCHEMA_VERSION,
        build_resource_manifest_sha256=build_input.manifest.sha256,
    )
    session = FakeSession(descriptor)
    with pytest.raises(CapabilityNegotiationError, match="requirements"):
        session.build(spec, build_input=build_input)
    assert session.side_effect_snapshot().generation == 0
