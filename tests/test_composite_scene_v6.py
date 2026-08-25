from __future__ import annotations

import hashlib
import inspect
from dataclasses import replace
from pathlib import Path

import pytest

import unirobosim
import unirobosim.api as unirobosim_api
from unirobosim import (
    COMPOSITE_WORLD_SCHEMA_VERSION,
    PHYSICAL_WORLD_SCHEMA_VERSION,
    ArrayValue,
    ArticulationCommand,
    BoxGeometrySpec,
    BuildInput,
    BuildResourceEntry,
    BuildResourceManifest,
    BuildSourceEntry,
    CapabilityId,
    CapabilityNegotiationError,
    CapabilitySet,
    CommandMode,
    EmbeddedEntityBinding,
    EmbeddedPrimBinding,
    EntityKind,
    EntityPath,
    EntitySpec,
    EnvironmentSpec,
    LocalSourceIdentity,
    ProviderDescriptor,
    Sim,
    ValidationError,
    WorldSpec,
)
from unirobosim.testing import FAKE_DESCRIPTOR, FakeProvider, FakeSession


def _source(path: Path, resource_id: str) -> BuildSourceEntry:
    stat_result = path.stat()
    return BuildSourceEntry(
        resource_id=resource_id,
        source_kind="local-file",
        source_root=str(path.parent),
        relative_source_path=path.name,
        expected_identity=LocalSourceIdentity(
            stat_result.st_dev,
            stat_result.st_ino,
            stat_result.st_mode,
            stat_result.st_size,
            stat_result.st_mtime_ns,
            stat_result.st_ctime_ns,
        ),
        expected_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
    )


def _build_input(tmp_path: Path) -> tuple[BuildInput, Path]:
    layer = tmp_path / "scene.usda"
    texture = tmp_path / "albedo.png"
    layer.write_bytes(b"#usda 1.0\n")
    texture.write_bytes(b"png\n")
    layer_digest = hashlib.sha256(layer.read_bytes()).hexdigest()
    texture_digest = hashlib.sha256(texture.read_bytes()).hexdigest()
    entries = (
        BuildResourceEntry(
            entity_id="entity.scene",
            component_id="component.scene",
            resource_id="resource.scene.layer",
            role="simulation.scene",
            media_type="model/vnd.usd",
            requested_uri=str(layer),
            resolved_uri=str(layer),
            canonical_source_identity=f"sha256:{layer_digest}",
            byte_size=layer.stat().st_size,
            sha256=layer_digest,
            selected_simulation_input=True,
            purposes=("collision", "planning", "simulation", "visual"),
            relative_bundle_path=layer.name,
            dependencies=("resource.scene.texture",),
        ),
        BuildResourceEntry(
            entity_id="entity.scene",
            component_id="component.scene",
            resource_id="resource.scene.texture",
            role="visual.texture",
            media_type="image/png",
            requested_uri=str(texture),
            resolved_uri=str(texture),
            canonical_source_identity=f"sha256:{texture_digest}",
            byte_size=texture.stat().st_size,
            sha256=texture_digest,
            selected_simulation_input=False,
            purposes=("visual",),
            relative_bundle_path=texture.name,
        ),
    )
    sources = tuple(
        sorted(
            (_source(layer, entries[0].resource_id), _source(texture, entries[1].resource_id)),
            key=lambda item: item.resource_id,
        )
    )
    return BuildInput(manifest=BuildResourceManifest(entries), sources=sources), layer


def _binding(container: str = "/scene") -> EmbeddedEntityBinding:
    return EmbeddedEntityBinding(
        EntityPath(container),
        "root/cabinet/base",
        (
            EmbeddedPrimBinding("base", "root/cabinet/base"),
            EmbeddedPrimBinding("door", "root/cabinet/door"),
        ),
        (EmbeddedPrimBinding("hinge", "root/cabinet/hinge"),),
    )


def _entities(asset_uri: str) -> tuple[EntitySpec, EntitySpec]:
    scene = EntitySpec(EntityPath("/scene"), EntityKind.COMPOSITE_SCENE, asset_uri=asset_uri)
    door = EntitySpec(
        EntityPath("/scene/door"),
        EntityKind.ARTICULATION,
        joint_names=("hinge",),
        initial_joint_positions=(0.1,),
        embedded_binding=_binding(),
    )
    return scene, door


def _world(build_input: BuildInput, asset_uri: str, *, environments: int = 1) -> WorldSpec:
    return WorldSpec(
        "composite",
        _entities(asset_uri),
        environments=EnvironmentSpec(environments),
        schema_version=COMPOSITE_WORLD_SCHEMA_VERSION,
        build_resource_manifest_sha256=build_input.manifest.sha256,
    )


def test_v6_public_contract_serialization_and_requirements(tmp_path: Path) -> None:
    build_input, layer = _build_input(tmp_path)
    scene, door = _entities(str(layer))
    world = _world(build_input, str(layer))

    assert unirobosim.COMPOSITE_WORLD_SCHEMA_VERSION == COMPOSITE_WORLD_SCHEMA_VERSION
    assert unirobosim_api.EmbeddedPrimBinding is EmbeddedPrimBinding
    assert EntityKind.COMPOSITE_SCENE.value == "composite_scene"
    assert world.schema_version == "unirobosim.world/v0alpha6"
    requirements = {item.capability.value: item.required for item in world.requirements}
    assert requirements["scene.composite@1"]
    assert requirements["entity.embedded-binding@1"]
    assert requirements["state.articulation.axis-units@1"]
    payload = door.to_dict(COMPOSITE_WORLD_SCHEMA_VERSION)
    assert payload["embedded_binding"] == {
        "container_path": "/scene",
        "root_body_prim_path": "root/cabinet/base",
        "link_prims": [
            {"logical_name": "base", "relative_prim_path": "root/cabinet/base"},
            {"logical_name": "door", "relative_prim_path": "root/cabinet/door"},
        ],
        "joint_prims": [{"logical_name": "hinge", "relative_prim_path": "root/cabinet/hinge"}],
    }
    assert scene.to_dict(COMPOSITE_WORLD_SCHEMA_VERSION)["scale_xyz"] == [1.0, 1.0, 1.0]
    golden = WorldSpec(
        "golden-v6",
        _entities("asset://scene"),
        schema_version=COMPOSITE_WORLD_SCHEMA_VERSION,
        build_resource_manifest_sha256="a" * 64,
    )
    assert len(golden.canonical_json.encode()) == 1496
    assert golden.digest == "076aa79153d10ad26a17a8257f413ce1bb9e0c5097b959bcf4cddf2cb0ec93b9"


@pytest.mark.parametrize(
    "path",
    ("/root/body", "root/body/", "root//body", "root/../body", "root/./body", "root/body-part", ""),
)
def test_embedded_prim_paths_are_container_relative_canonical(path: str) -> None:
    with pytest.raises(ValidationError, match="Prim path|canonical"):
        EmbeddedPrimBinding("body", path)


def test_embedded_binding_and_entity_validation_is_closed() -> None:
    link = EmbeddedPrimBinding("base", "root/base")
    with pytest.raises(ValidationError, match="logical_name"):
        EmbeddedPrimBinding("bad/name", "root/base")
    with pytest.raises(ValidationError, match="non-empty"):
        EmbeddedEntityBinding(EntityPath("/scene"), "root/base", ())
    with pytest.raises(ValidationError, match="declared link"):
        EmbeddedEntityBinding(EntityPath("/scene"), "root/missing", (link,))
    with pytest.raises(ValidationError, match="unique"):
        EmbeddedEntityBinding(EntityPath("/scene"), "root/base", (link, link))
    with pytest.raises(ValidationError, match="must not overlap"):
        EmbeddedEntityBinding(
            EntityPath("/scene"),
            "root/base",
            (link,),
            (EmbeddedPrimBinding("hinge", "root/base"),),
        )

    binding = EmbeddedEntityBinding(EntityPath("/scene"), "root/base", (link,))
    with pytest.raises(ValidationError, match="cannot declare an asset"):
        EntitySpec(
            EntityPath("/scene/body"),
            EntityKind.RIGID_BODY,
            asset_uri="body.usd",
            embedded_binding=binding,
        )
    with pytest.raises(ValidationError, match="procedural geometry"):
        EntitySpec(
            EntityPath("/scene/body"),
            EntityKind.RIGID_BODY,
            box=BoxGeometrySpec(),
            embedded_binding=binding,
        )
    with pytest.raises(ValidationError, match="exactly one link"):
        EntitySpec(
            EntityPath("/scene/body"),
            EntityKind.RIGID_BODY,
            embedded_binding=EmbeddedEntityBinding(
                EntityPath("/scene"),
                "root/base",
                (link, EmbeddedPrimBinding("child", "root/child")),
            ),
        )
    with pytest.raises(ValidationError, match="exactly match"):
        EntitySpec(
            EntityPath("/scene/door"),
            EntityKind.ARTICULATION,
            joint_names=("other",),
            embedded_binding=replace(_binding(), root_body_prim_path="root/cabinet/base"),
        )
    with pytest.raises(ValidationError, match="unit scale"):
        EntitySpec(
            EntityPath("/scene"),
            EntityKind.COMPOSITE_SCENE,
            asset_uri="scene.usd",
            scale_xyz=(2.0, 2.0, 2.0),
        )


def test_world_requires_v6_unique_container_descendants_and_unambiguous_prims(tmp_path: Path) -> None:
    build_input, layer = _build_input(tmp_path)
    scene, door = _entities(str(layer))
    with pytest.raises(ValidationError, match="v0alpha6"):
        WorldSpec(
            "v5-composite",
            (scene, door),
            schema_version=PHYSICAL_WORLD_SCHEMA_VERSION,
            build_resource_manifest_sha256=build_input.manifest.sha256,
        )
    with pytest.raises(ValidationError, match="strictly below"):
        WorldSpec(
            "outside",
            (scene, replace(door, path=EntityPath("/door"))),
            schema_version=COMPOSITE_WORLD_SCHEMA_VERSION,
            build_resource_manifest_sha256=build_input.manifest.sha256,
        )
    with pytest.raises(ValidationError, match="container"):
        WorldSpec(
            "missing-container",
            (replace(door, embedded_binding=_binding("/missing")),),
            schema_version=COMPOSITE_WORLD_SCHEMA_VERSION,
        )
    duplicate = replace(door, path=EntityPath("/scene/door2"))
    with pytest.raises(ValidationError, match="claimed"):
        WorldSpec(
            "ambiguous",
            (scene, door, duplicate),
            schema_version=COMPOSITE_WORLD_SCHEMA_VERSION,
            build_resource_manifest_sha256=build_input.manifest.sha256,
        )
    static = EntitySpec(EntityPath("/static"), EntityKind.STATIC_SCENE, asset_uri=str(layer))
    with pytest.raises(ValidationError, match="at most one"):
        WorldSpec(
            "two-containers",
            (scene, static),
            schema_version=COMPOSITE_WORLD_SCHEMA_VERSION,
            build_resource_manifest_sha256=build_input.manifest.sha256,
        )


@pytest.mark.parametrize("missing_capability", ("scene.composite@1", "entity.embedded-binding@1"))
def test_v6_capability_negotiation_fails_before_side_effects(tmp_path: Path, missing_capability: str) -> None:
    build_input, layer = _build_input(tmp_path)
    descriptor = replace(
        FAKE_DESCRIPTOR,
        capabilities=CapabilitySet(
            tuple(item for item in FAKE_DESCRIPTOR.capabilities if item.capability != CapabilityId(missing_capability))
        ),
    )
    session = FakeSession(descriptor)
    with pytest.raises(CapabilityNegotiationError):
        session.build(_world(build_input, str(layer)), build_input=build_input)
    assert session.side_effect_snapshot().generation == 0
    assert session.side_effect_snapshot().composite_compositions == 0


def test_fake_composes_once_and_embedded_control_state_reset_close(tmp_path: Path) -> None:
    build_input, layer = _build_input(tmp_path)
    session = FakeSession(FAKE_DESCRIPTOR)
    world = session.build(_world(build_input, str(layer), environments=2), build_input=build_input)
    scene_handle = world.resolve(EntityPath("/scene"))
    door_handle = world.resolve(EntityPath("/scene/door"))
    assert scene_handle.entity_kind is EntityKind.COMPOSITE_SCENE
    assert door_handle.entity_kind is EntityKind.ARTICULATION
    assert session.side_effect_snapshot().composite_compositions == 2

    world.apply_articulation_command(
        ArticulationCommand(
            door_handle,
            CommandMode.POSITION,
            ArrayValue.from_rows(((0.8,), (0.8,))),
            target_units=("rad",),
        )
    )
    world.step()
    assert world.read_articulation(door_handle).joint_positions.rows() == ((0.8,), (0.8,))
    world.reset()
    assert world.read_articulation(door_handle).joint_positions.rows() == ((0.1,), (0.1,))
    assert session.side_effect_snapshot().composite_compositions == 2
    assert world.resolve(EntityPath("/scene/door")) == door_handle
    world.close()
    session.close()


def test_easyapi_composite_build_input_and_embedded_articulation(tmp_path: Path) -> None:
    build_input, layer = _build_input(tmp_path)
    sim = Sim(provider=FakeProvider(), num_envs=2)
    scene = sim.add_composite_scene("scene", asset_uri=str(layer))
    door = sim.add_embedded_articulation(
        "door",
        container=scene,
        root_body_prim_path="root/cabinet/base",
        link_prims={"base": "root/cabinet/base", "door": "root/cabinet/door"},
        joint_prims={"hinge": "root/cabinet/hinge"},
        initial_positions=(0.1,),
    )
    assert door.path == EntityPath("/scene/door")
    with pytest.raises(ValidationError, match="BuildInput"):
        sim.start()
    sim.start(build_input=build_input)
    assert sim.world_spec.schema_version == COMPOSITE_WORLD_SCHEMA_VERSION
    door.command((0.6,))
    sim.step()
    assert door.state.joint_positions.rows() == ((0.6,), (0.6,))
    sim.reset()
    assert door.state.joint_positions.rows() == ((0.1,), (0.1,))
    sim.close()


def test_easyapi_composite_accepts_only_an_explicit_asset_uri() -> None:
    signature = inspect.signature(Sim.add_composite_scene)
    assert "asset_uri" in signature.parameters
    assert signature.parameters["asset_uri"].default is inspect.Parameter.empty
    assert "asset" not in signature.parameters
    assert "asset_options" not in signature.parameters

    sim = Sim(provider=FakeProvider())
    with pytest.raises(TypeError, match="asset_uri"):
        sim.add_composite_scene("scene")  # type: ignore[call-arg]
    assert sim.state.value == "configuring"


def test_easyapi_v4_rejects_unexpected_build_input(tmp_path: Path) -> None:
    build_input, _ = _build_input(tmp_path)
    sim = Sim(provider=FakeProvider())
    sim.add_box("box")
    with pytest.raises(ValidationError, match="only accepted"):
        sim.start(build_input=build_input)


def test_provider_descriptor_accepts_v6_and_fake_declares_it() -> None:
    descriptor = ProviderDescriptor(
        "example.v6",
        "Example",
        "1",
        "v0alpha6",
        CapabilitySet(),
        (COMPOSITE_WORLD_SCHEMA_VERSION,),
    )
    assert descriptor.supported_world_schema_versions == (COMPOSITE_WORLD_SCHEMA_VERSION,)
    assert FAKE_DESCRIPTOR.supported_world_schema_versions[-1] == COMPOSITE_WORLD_SCHEMA_VERSION
