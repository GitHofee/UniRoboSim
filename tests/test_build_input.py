from __future__ import annotations

import hashlib
import math
import os
import struct
from dataclasses import replace
from pathlib import Path

import pytest

import unirobosim.testing.fake_backend as fake_backend
from unirobosim import (
    ASSET_DEPENDENCY_INCOMPLETE,
    ASSET_IDENTITY_CHANGED,
    PHYSICAL_WORLD_SCHEMA_VERSION,
    BuildInput,
    BuildLinkDynamics,
    BuildResourceEntry,
    BuildResourceManifest,
    BuildSourceEntry,
    CapabilityId,
    CapabilityRequirement,
    EntityKind,
    EntityPath,
    EntitySpec,
    FrozenMap,
    LocalSourceIdentity,
    PlanningGeometryRepresentation,
    ValidationError,
    WorldSpec,
)
from unirobosim.testing import FakeProvider


def _entry(
    resource_id: str,
    payload: bytes,
    *,
    relative_path: str,
    dependencies: tuple[str, ...] = (),
) -> BuildResourceEntry:
    return BuildResourceEntry(
        entity_id="entity.robot",
        component_id="robot.franka",
        resource_id=resource_id,
        role="simulation",
        media_type="model/vnd.urdf+xml",
        requested_uri=f"asset://{resource_id}",
        resolved_uri=f"cache://{resource_id}",
        canonical_source_identity=f"sha256:{hashlib.sha256(payload).hexdigest()}",
        byte_size=len(payload),
        sha256=hashlib.sha256(payload).hexdigest(),
        selected_simulation_input=True,
        purposes=("collision", "planning", "simulation", "visual"),
        relative_bundle_path=relative_path,
        dependencies=dependencies,
    )


def _identity(path: Path) -> LocalSourceIdentity:
    value = path.stat()
    return LocalSourceIdentity(
        value.st_dev,
        value.st_ino,
        value.st_mode,
        value.st_size,
        value.st_mtime_ns,
        value.st_ctime_ns,
    )


def _input(tmp_path: Path, *, source_kind: str = "local-file") -> tuple[BuildInput, Path]:
    payload = b"<robot name='portable'/>\n"
    path = tmp_path / "robot.urdf"
    path.write_bytes(payload)
    entry = _entry("resource.robot", payload, relative_path="robot/robot.urdf")
    manifest = BuildResourceManifest((entry,))
    source = BuildSourceEntry(
        entry.resource_id,
        source_kind,  # type: ignore[arg-type]
        str(tmp_path),
        path.name,
        _identity(path),
        entry.sha256,
    )
    return BuildInput(manifest=manifest, sources=(source,)), path


def _asset_world(build_input: BuildInput) -> WorldSpec:
    return WorldSpec(
        "asset-world",
        (EntitySpec(EntityPath("/robot"), EntityKind.ARTICULATION, joint_names=("joint",), asset_uri="asset://robot"),),
        schema_version=PHYSICAL_WORLD_SCHEMA_VERSION,
        build_resource_manifest_sha256=build_input.manifest.sha256,
    )


def test_manifest_and_dynamics_are_canonical_and_digest_stable() -> None:
    child = _entry("resource.child", b"mesh", relative_path="mesh/child.obj")
    root = _entry(
        "resource.root",
        b"urdf",
        relative_path="robot/root.urdf",
        dependencies=("resource.child",),
    )
    dynamics = BuildLinkDynamics(
        "link.base",
        2.5,
        (0.2, -0.3, 0.4),
        (1.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 2.5),
        (1.0, 2.0, 2.5),
        (0.0, 0.0, 0.0, 1.0),
        "a" * 64,
    )
    manifest = BuildResourceManifest((child, root), (dynamics,))
    assert manifest.sha256 == hashlib.sha256(manifest.canonical_json.encode()).hexdigest()
    assert manifest.to_dict()["entries"][0]["resource_id"] == "resource.child"  # type: ignore[index]


def test_rotated_link_dynamics_reconstructs_full_tensor_exactly() -> None:
    half_angle = math.pi / 8.0
    dynamics = BuildLinkDynamics(
        "link.rotated",
        3.0,
        (0.1, -0.2, 0.3),
        (1.5, -0.5, 0.0, -0.5, 1.5, 0.0, 0.0, 0.0, 2.5),
        (1.0, 2.0, 2.5),
        (0.0, 0.0, math.sin(half_angle), math.cos(half_angle)),
        "b" * 64,
    )
    assert dynamics.inertia_tensor_com_link_kg_m2[1] == pytest.approx(-0.5)
    assert BuildResourceManifest((_entry("resource.body", b"body", relative_path="body.bin"),), (dynamics,))


@pytest.mark.parametrize(
    ("tensor", "moments", "axes"),
    (
        ((1.0, 0.1, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 2.5), (1.0, 2.0, 2.5), (0.0, 0.0, 0.0, 1.0)),
        ((1.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 2.5), (1.0, 2.0, 2.4), (0.0, 0.0, 0.0, 1.0)),
        ((1.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 4.0), (1.0, 2.0, 4.0), (0.0, 0.0, 0.0, 1.0)),
        ((1.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 2.5), (1.0, 2.0, 2.5), (0.0, 0.0, 0.0, -1.0)),
        ((1.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 2.5), (1.0, 2.0, 2.5), (0.0, 0.0, 0.0, 2.0)),
    ),
)
def test_link_dynamics_rejects_invalid_tensor_eigensystem(
    tensor: tuple[float, ...],
    moments: tuple[float, float, float],
    axes: tuple[float, float, float, float],
) -> None:
    with pytest.raises(ValidationError):
        BuildLinkDynamics("link.bad", 1.0, (0.0, 0.0, 0.0), tensor, moments, axes, "c" * 64)


def test_manifest_rejects_unsorted_duplicate_incomplete_and_cyclic_graphs() -> None:
    child = _entry("resource.child", b"mesh", relative_path="mesh/child.obj")
    root = _entry("resource.root", b"urdf", relative_path="robot/root.urdf")
    with pytest.raises(ValidationError, match="canonical"):
        BuildResourceManifest((root, child))
    with pytest.raises(ValidationError, match="incomplete") as incomplete:
        BuildResourceManifest((replace(child, dependencies=("resource.missing",)), root))
    assert incomplete.value.details["detail_code"] == ASSET_DEPENDENCY_INCOMPLETE
    cyclic_child = replace(child, dependencies=("resource.root",))
    cyclic_root = replace(root, dependencies=("resource.child",))
    with pytest.raises(ValidationError, match="cycle"):
        BuildResourceManifest((cyclic_child, cyclic_root))
    with pytest.raises(ValidationError, match="bundle paths"):
        BuildResourceManifest((child, replace(root, relative_bundle_path=child.relative_bundle_path)))


@pytest.mark.parametrize("path", ("../escape", "/absolute", "a//b", "a/./b", "a\\b"))
def test_manifest_rejects_hostile_bundle_paths(path: str) -> None:
    with pytest.raises(ValidationError):
        _entry("resource.bad", b"bad", relative_path=path)


@pytest.mark.parametrize("path", ("C:/asset.obj", "bundle/e\N{COMBINING ACUTE ACCENT}.obj"))
def test_manifest_rejects_drive_and_non_nfc_paths(path: str) -> None:
    with pytest.raises(ValidationError):
        _entry("resource.bad", b"bad", relative_path=path)


@pytest.mark.parametrize(
    ("first", "second"),
    (("bundle/node", "bundle/node/child.bin"), ("Bundle/Node.bin", "bundle/node.BIN")),
)
def test_manifest_rejects_portable_path_prefix_and_case_collisions(first: str, second: str) -> None:
    with pytest.raises(ValidationError):
        BuildResourceManifest(
            (
                _entry("resource.0", b"zero", relative_path=first),
                _entry("resource.1", b"one", relative_path=second),
            )
        )


def test_manifest_rejects_excessive_dependency_depth_without_recursion_error() -> None:
    count = 1_200
    entries = tuple(
        _entry(
            f"resource.{index:04d}",
            str(index).encode(),
            relative_path=f"resources/{index:04d}.bin",
            dependencies=((f"resource.{index + 1:04d}",) if index + 1 < count else ()),
        )
        for index in range(count)
    )
    with pytest.raises(ValidationError, match="depth budget"):
        BuildResourceManifest(entries)


def test_build_value_exact_validation_edges(tmp_path: Path) -> None:
    entry = _entry("resource.edge", b"edge", relative_path="edge.bin")
    for override in (
        {"entity_id": ""},
        {"resource_id": "bad id"},
        {"media_type": "binary"},
        {"byte_size": -1},
        {"selected_simulation_input": 1},
        {"purposes": ("simulation", "simulation")},
        {"purposes": ("unsupported",)},
        {"dependencies": ("resource.edge",)},
    ):
        with pytest.raises(ValidationError):
            replace(entry, **override)

    dynamics = BuildLinkDynamics(
        "link.edge",
        1.0,
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0),
        (1.0, 1.0, 1.0),
        (0.0, 0.0, 0.0, 1.0),
        "d" * 64,
    )
    for override in (
        {"mass_kg": 10**400},
        {"center_of_mass_xyz_m": (0.0, 0.0)},
        {"inertia_tensor_com_link_kg_m2": (1.0,)},
        {"principal_moments_kg_m2": (1.0,)},
        {"principal_axes_xyzw": (0.0, 0.0, 1.0)},
    ):
        with pytest.raises(ValidationError):
            replace(dynamics, **override)

    with pytest.raises(ValidationError):
        BuildResourceManifest((object(),))  # type: ignore[arg-type]
    with pytest.raises(ValidationError):
        BuildResourceManifest((entry,), (object(),))  # type: ignore[arg-type]
    with pytest.raises(ValidationError):
        BuildResourceManifest((entry,), (dynamics, dynamics))

    build_input, _ = _input(tmp_path)
    source = build_input.sources[0]
    with pytest.raises(ValidationError):
        replace(source, source_kind="unknown")
    with pytest.raises(ValidationError):
        replace(source, expected_identity=object())
    with pytest.raises(ValidationError):
        BuildInput(manifest=object(), sources=())  # type: ignore[arg-type]
    with pytest.raises(ValidationError):
        BuildInput(manifest=build_input.manifest, sources=(object(),))  # type: ignore[arg-type]


def test_build_input_is_keyword_only(tmp_path: Path) -> None:
    build_input, _ = _input(tmp_path)
    with pytest.raises(TypeError):
        BuildInput(build_input.manifest, build_input.sources)  # type: ignore[misc]


def test_build_input_requires_exact_sorted_one_to_one_sources(tmp_path: Path) -> None:
    build_input, _ = _input(tmp_path)
    with pytest.raises(ValidationError) as failure:
        BuildInput(manifest=build_input.manifest, sources=())
    assert failure.value.details["detail_code"] == ASSET_DEPENDENCY_INCOMPLETE
    source = build_input.sources[0]
    with pytest.raises(ValidationError) as mismatch:
        BuildInput(
            manifest=build_input.manifest,
            sources=(replace(source, expected_sha256="0" * 64),),
        )
    assert mismatch.value.details["detail_code"] == ASSET_IDENTITY_CHANGED


@pytest.mark.parametrize("source_kind", ("local-file", "cached-remote"))
def test_fake_build_accepts_local_and_cached_remote_carriers(tmp_path: Path, source_kind: str) -> None:
    build_input, _ = _input(tmp_path, source_kind=source_kind)
    session = FakeProvider().open()
    world = session.build(_asset_world(build_input), build_input=build_input)
    assert world.build_report.world_id == "asset-world"
    world.close()


def test_asset_world_rejects_missing_extra_and_wrong_manifest_before_generation(tmp_path: Path) -> None:
    build_input, _ = _input(tmp_path)
    session = FakeProvider().open()
    with pytest.raises(ValidationError):
        session.build(_asset_world(build_input))
    assert session._generation == 0

    procedural = WorldSpec(
        "procedural",
        (EntitySpec(EntityPath("/box"), EntityKind.RIGID_BODY),),
        schema_version=PHYSICAL_WORLD_SCHEMA_VERSION,
    )
    with pytest.raises(ValidationError):
        session.build(procedural, build_input=build_input)
    assert session._generation == 0


def test_source_mutation_and_symlink_fail_without_path_leak(tmp_path: Path) -> None:
    build_input, path = _input(tmp_path)
    path.write_bytes(b"mutated")
    session = FakeProvider().open()
    with pytest.raises(ValidationError) as failure:
        session.build(_asset_world(build_input), build_input=build_input)
    assert failure.value.details["detail_code"] == ASSET_IDENTITY_CHANGED
    assert str(tmp_path) not in str(failure.value.to_dict())
    assert session._generation == 0

    target = tmp_path / "target.urdf"
    target.write_bytes(b"target")
    link = tmp_path / "link.urdf"
    link.symlink_to(target)
    link_entry = _entry("resource.link", target.read_bytes(), relative_path="link.urdf")
    link_source = BuildSourceEntry(
        link_entry.resource_id,
        "local-file",
        str(tmp_path),
        link.name,
        _identity(target),
        link_entry.sha256,
    )
    link_input = BuildInput(manifest=BuildResourceManifest((link_entry,)), sources=(link_source,))
    with pytest.raises(ValidationError):
        session.build(_asset_world(link_input), build_input=link_input)


def test_symlink_source_root_and_private_failure_are_scrubbed(tmp_path: Path) -> None:
    payload = b"private asset"
    real_root = tmp_path / "real"
    real_root.mkdir()
    source_path = real_root / "asset.bin"
    source_path.write_bytes(payload)
    linked_root = tmp_path / "linked"
    linked_root.symlink_to(real_root, target_is_directory=True)
    entry = _entry("resource.private", payload, relative_path="asset.bin")
    build_input = BuildInput(
        manifest=BuildResourceManifest((entry,)),
        sources=(
            BuildSourceEntry(
                entry.resource_id,
                "local-file",
                str(linked_root),
                source_path.name,
                _identity(source_path),
                entry.sha256,
            ),
        ),
    )
    session = FakeProvider().open()
    before = session.side_effect_snapshot()
    with pytest.raises(ValidationError) as captured:
        session.build(_asset_world(build_input), build_input=build_input)
    assert captured.value.__cause__ is None
    assert captured.value.__context__ is None
    assert captured.value.__traceback__ is None
    assert session.side_effect_snapshot() == before


def test_atomic_path_replace_after_retained_open_uses_opened_inode(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    build_input, path = _input(tmp_path)
    replacement = tmp_path / "replacement.urdf"
    replacement.write_bytes(b"replacement bytes")
    original_read = os.read
    replaced = False

    def replace_after_open(file_descriptor: int, count: int) -> bytes:
        nonlocal replaced
        if not replaced:
            os.replace(replacement, path)
            replaced = True
        return original_read(file_descriptor, count)

    monkeypatch.setattr(fake_backend.os, "read", replace_after_open)
    session = FakeProvider().open()
    world = session.build(_asset_world(build_input), build_input=build_input)
    assert replaced
    assert world.generation == 1
    world.close()
    with pytest.raises(ValidationError) as failure:
        session.build(_asset_world(build_input), build_input=build_input)
    assert failure.value.details["detail_code"] == ASSET_IDENTITY_CHANGED
    assert session._generation == 1


def test_opened_inode_mutation_during_read_fails_before_generation(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    build_input, path = _input(tmp_path)
    original_read = os.read
    mutated = False

    def mutate_open_inode(file_descriptor: int, count: int) -> bytes:
        nonlocal mutated
        result = original_read(file_descriptor, count)
        if not mutated:
            with path.open("ab") as stream:
                stream.write(b"mutation")
                stream.flush()
                os.fsync(stream.fileno())
            mutated = True
        return result

    monkeypatch.setattr(fake_backend.os, "read", mutate_open_inode)
    session = FakeProvider().open()
    with pytest.raises(ValidationError) as failure:
        session.build(_asset_world(build_input), build_input=build_input)
    assert failure.value.details["detail_code"] == ASSET_IDENTITY_CHANGED
    assert session._generation == 0


def test_successful_build_is_detached_from_later_source_mutation(tmp_path: Path) -> None:
    build_input, path = _input(tmp_path)
    session = FakeProvider().open()
    world = session.build(_asset_world(build_input), build_input=build_input)
    path.write_bytes(b"changed after the linearization point")
    world.step()
    assert world.build_report.world_id == "asset-world"
    world.close()
    with pytest.raises(ValidationError) as failure:
        session.build(_asset_world(build_input), build_input=build_input)
    assert failure.value.details["detail_code"] == ASSET_IDENTITY_CHANGED
    assert session._generation == 1


def test_scaled_asset_planning_mesh_contains_final_baked_vertices(tmp_path: Path) -> None:
    build_input, _ = _input(tmp_path)
    spec = WorldSpec(
        "scaled-asset",
        (
            EntitySpec(
                EntityPath("/container"),
                EntityKind.RIGID_BODY,
                asset_uri="asset://container",
                metadata=FrozenMap({"fake_planning_collision_authority": "effective_native"}),
                scale_xyz=(2.0, 3.0, 4.0),
            ),
        ),
        requirements=(CapabilityRequirement(CapabilityId("planning.scene@2")),),
        schema_version=PHYSICAL_WORLD_SCHEMA_VERSION,
        build_resource_manifest_sha256=build_input.manifest.sha256,
    )
    world = FakeProvider().open().build(spec, build_input=build_input)
    catalog = world.planning_scene_catalog()
    container = next(entity for entity in catalog.entities if entity.path == "/container")
    geometry = next(item for item in catalog.geometries if item.geometry_id == container.geometry_ids[0])
    assert geometry.representation is PlanningGeometryRepresentation.TRIANGLE_MESH
    assert geometry.scale == (1.0, 1.0, 1.0)
    lease = world.resolve_planning_geometry(geometry.geometry_id)
    try:
        content = lease.read()
    finally:
        lease.close()
    assert struct.unpack_from("<fff", content, 0) == pytest.approx((-1.0, -1.5, 0.0))
    assert struct.unpack_from("<fff", content, 6 * 12) == pytest.approx((1.0, 1.5, 2.4))


def test_source_root_and_relative_path_are_strict(tmp_path: Path) -> None:
    build_input, _ = _input(tmp_path)
    source = build_input.sources[0]
    with pytest.raises(ValidationError):
        replace(source, source_root="relative")
    with pytest.raises(ValidationError):
        replace(source, relative_source_path="../escape")


def test_local_identity_requires_regular_file(tmp_path: Path) -> None:
    value = os.stat(tmp_path)
    with pytest.raises(ValidationError):
        LocalSourceIdentity(
            value.st_dev,
            value.st_ino,
            value.st_mode,
            value.st_size,
            value.st_mtime_ns,
            value.st_ctime_ns,
        )
