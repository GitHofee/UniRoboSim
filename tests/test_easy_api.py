from __future__ import annotations

import hashlib
import json
from pathlib import Path

import pytest

import unirobosim.easy.sim as easy_sim
from unirobosim import (
    ArrayValue,
    AssetBundle,
    BoxGeometrySpec,
    CameraModality,
    CommandMode,
    DebugLifetime,
    DebugPrimitive,
    DebugPrimitiveKind,
    EntityKind,
    EntityPath,
    EntitySpec,
    LifecycleError,
    PointCommandMode,
    ProviderSelectionError,
    Sim,
    SimState,
    UnsupportedCapabilityError,
    ValidationError,
    WorldBuildError,
)
from unirobosim.testing import FakeProvider


def test_easy_api_common_rigid_articulation_camera_debug_and_scene_flow() -> None:
    with Sim(
        provider=FakeProvider(),
        world_id="easy-common",
        num_envs=2,
        time_step_seconds=0.1,
        gravity_m_s2=(0.0, 0.0, 0.0),
    ) as sim:
        box = sim.add_box(
            "box",
            size_m=(0.2, 0.4, 0.6),
            mass_kg=2.0,
            position_m=(0.0, 0.0, 1.0),
        )
        cabinet = sim.add_articulation(
            "cabinet",
            joint_names=("door_hinge", "drawer_slide"),
            initial_positions=(0.1, -0.2),
        )
        camera = sim.add_camera("camera", resolution=(16, 12), outputs=("rgb", "depth"))
        sim.optional("state.fluid.particles@1", reason="use fluid only when native")
        report = sim.start()
        assert report.entity_count == 3 and sim.state is SimState.RUNNING
        assert sim.provider_descriptor.provider_id == "reference.fake"
        assert sim.entity("/box") is box and sim.entity("cabinet") is cabinet
        assert box.state.positions_m.shape == (2, 3)
        assert cabinet.joint_names == ("door_hinge", "drawer_slide") and cabinet.num_joints == 2

        box.apply_wrench((2.0, 0.0, 0.0))
        cabinet.command((0.5,), joints=("door_hinge",), mode="position")
        tick = sim.step()
        assert tick.step_index == 1
        assert box.state.positions_m.rows()[0][0] == pytest.approx(0.01)
        assert cabinet.state.joint_positions.rows() == ((0.5, -0.2), (0.5, -0.2))
        assert camera.read(CameraModality.RGB).shape == (2, 12, 16, 3)
        assert camera.sample().channel(CameraModality.DEPTH).shape == (2, 12, 16)
        assert box.contact().in_contact.shape == (2,)

        primitive = DebugPrimitive(
            "target",
            "easy",
            DebugPrimitiveKind.POINT_SET,
            ArrayValue.from_nested([[[0.0, 0.0, 1.0]], [[0.0, 0.0, 1.0]]]),
            (0, 1),
            lifetime=DebugLifetime.persistent(),
        )
        assert sim.debug.publish(primitive).accepted_count == 1
        assert sim.debug.clear(layer="easy") == 1
        box_scene = next(item for item in sim.scene_snapshot().entities if item.path == EntityPath("/box"))
        assert box_scene.visuals[0].dimensions_m == (0.2, 0.4, 0.6)
        assert sim.reset((1,)).environment_indices == (1,)
    assert sim.state is SimState.CLOSED


def test_easy_api_lifecycle_selection_and_validation_are_actionable() -> None:
    sim = Sim(provider=FakeProvider())
    with pytest.raises(LifecycleError):
        _ = sim.world
    with pytest.raises(LifecycleError):
        _ = sim.world_spec
    with pytest.raises(ValidationError, match="at least one entity"):
        sim.start()
    sim.add_box("box")
    with pytest.raises(ValidationError, match="duplicate"):
        sim.add_box("box")
    articulation = sim.add_articulation("laptop", joint_names=("screen_hinge",))
    sim.start()
    with pytest.raises(LifecycleError):
        sim.add_box("late")
    with pytest.raises(ValidationError, match="unknown joint"):
        articulation.command((0.1,), joints=("missing",))
    with pytest.raises(ValidationError, match="mode"):
        articulation.command((0.1,), mode="magic")
    with pytest.raises(ValidationError, match="shape"):
        articulation.command(((0.1,),), environments=(0, 1))
    sim.close()
    sim.close()
    with pytest.raises(LifecycleError):
        sim.__enter__()


def test_box_geometry_and_capability_reason_are_in_world_fingerprint() -> None:
    box = BoxGeometrySpec((0.1, 0.2, 0.3), 4.0)
    assert box.to_dict() == {
        "kind": "box",
        "dimensions_m": [0.1, 0.2, 0.3],
        "mass_kg": 4.0,
        "color_rgba": [0.15, 0.7, 0.95, 1.0],
        "static_friction": 1.0,
        "dynamic_friction": 1.0,
        "restitution": 0.0,
    }
    with pytest.raises(ValidationError):
        BoxGeometrySpec((1.0, 0.0, 1.0), 1.0)
    with pytest.raises(ValidationError):
        BoxGeometrySpec((1.0, 1.0, 1.0), -1.0)
    with pytest.raises(ValidationError):
        BoxGeometrySpec((1.0, 1.0, 1.0), 1.0, (2.0, 0.0, 0.0, 1.0))
    with pytest.raises(ValidationError):
        BoxGeometrySpec((1.0, 1.0, 1.0), 1.0, static_friction=0.5, dynamic_friction=1.0)
    with pytest.raises(ValidationError):
        BoxGeometrySpec((1.0, 1.0, 1.0), 1.0, restitution=1.1)
    with pytest.raises(ValidationError):
        EntitySpec(
            EntityPath("/bad"),
            EntityKind.ARTICULATION,
            joint_names=("hinge",),
            box=box,
        )

    sim = Sim(provider=FakeProvider())
    sim.add_box("box", size_m=(0.1, 0.2, 0.3), mass_kg=4.0)
    sim.require("render.browser-scene@1", reason="interactive inspection")
    sim.start()
    requirement = next(
        item for item in sim.world_spec.requirements if item.capability.value == "render.browser-scene@1"
    )
    assert requirement.reason == "interactive inspection"
    assert sim.world_spec.to_dict()["entities"][0]["box"] == box.to_dict()


def test_portable_joint_effort_limits_are_validated_and_serialized() -> None:
    articulation = EntitySpec(
        EntityPath("/arm"),
        EntityKind.ARTICULATION,
        joint_names=("shoulder", "wrist"),
        joint_effort_limits=(87.0, 12.0),
    )
    assert articulation.to_dict()["joint_effort_limits"] == [87.0, 12.0]
    for limits in ((1.0,), (1.0, 0.0), (1.0, float("nan"))):
        with pytest.raises(ValidationError):
            EntitySpec(
                EntityPath("/invalid"),
                EntityKind.ARTICULATION,
                joint_names=("shoulder", "wrist"),
                joint_effort_limits=limits,
            )

    sim = Sim(provider=FakeProvider())
    sim.add_articulation(
        "arm",
        joint_names=("shoulder", "wrist"),
        joint_effort_limits=(50.0, 50.0),
    )
    sim.start()
    assert sim.world_spec.to_dict()["entities"][0]["joint_effort_limits"] == [50.0, 50.0]
    sim.close()
    sim.close()

    asset_sim = Sim(provider=FakeProvider())
    body = asset_sim.add_rigid_body("asset", asset_uri="file:///tmp/example.usd", position_m=(1, 2, 3))
    asset_sim.start()
    assert body.state.positions_m.rows()[0] == (1.0, 2.0, 3.0)
    assert asset_sim.world_spec.entities[0].asset_uri == "file:///tmp/example.usd"
    asset_sim.close()


def test_asset_bundle_resolves_after_provider_selection_and_verifies_hash(tmp_path: Path) -> None:
    asset_file = tmp_path / "robot.native"
    asset_file.write_bytes(b"native asset")
    digest = hashlib.sha256(asset_file.read_bytes()).hexdigest()
    manifest = tmp_path / "robot.asset.json"
    manifest.write_text(
        json.dumps(
            {
                "schema_version": "unirobosim.asset-bundle.v1",
                "name": "test_robot",
                "variants": {"fake": {"uri": asset_file.name, "sha256": digest}},
            }
        ),
        encoding="utf-8",
    )
    bundle = AssetBundle.from_manifest(manifest)
    assert bundle.logical_name == "test_robot"
    sim = Sim(provider=FakeProvider())
    sim.add_articulation("robot", joint_names=("joint",), asset=bundle)
    sim.start()
    assert sim.world_spec.entities[0].asset_uri == str(asset_file.resolve())
    assert sim.world_spec.entities[0].metadata["unirobosim_asset"]["media_type"] == "application/octet-stream"
    sim.close()

    missing = Sim(provider=FakeProvider())
    missing.add_articulation(
        "robot",
        joint_names=("joint",),
        asset=AssetBundle("wrong", {"mujoco": "/tmp/robot.xml"}),
    )
    with pytest.raises(ValidationError, match="no variant"):
        missing.start()
    missing.close()


@pytest.mark.parametrize(
    "variants",
    (
        {},
        {1: "robot.usd"},
        {"": "robot.usd"},
        {"fake": 7},
        {"fake": ""},
        {"fake": {"uri": "robot.usd", "sha256": "bad"}},
        {"fake": {"uri": "robot.usd", "media_type": "invalid"}},
    ),
)
def test_asset_bundle_rejects_malformed_variants(variants: object) -> None:
    with pytest.raises(ValidationError):
        AssetBundle("robot", variants)  # type: ignore[arg-type]


def test_asset_bundle_manifest_and_identity_validation(tmp_path: Path) -> None:
    with pytest.raises(ValidationError, match="logical asset name"):
        AssetBundle("", {"fake": "robot.usd"})
    with pytest.raises(ValidationError, match="schema"):
        AssetBundle("robot", {"fake": "robot.usd"}, schema_version="wrong")
    with pytest.raises(ValidationError, match="source_manifest"):
        AssetBundle("robot", {"fake": "robot.usd"}, source_manifest=3)  # type: ignore[arg-type]

    with pytest.raises(ValidationError, match="could not be read"):
        AssetBundle.from_manifest(tmp_path / "missing.json")
    invalid_json = tmp_path / "invalid.json"
    invalid_json.write_text("{", encoding="utf-8")
    with pytest.raises(ValidationError, match="could not be read"):
        AssetBundle.from_manifest(invalid_json)
    root_list = tmp_path / "root-list.json"
    root_list.write_text("[]", encoding="utf-8")
    with pytest.raises(ValidationError, match="root"):
        AssetBundle.from_manifest(root_list)
    wrong_schema = tmp_path / "wrong-schema.json"
    wrong_schema.write_text(json.dumps({"schema_version": "wrong", "name": "robot", "variants": {}}))
    with pytest.raises(ValidationError, match="schema"):
        AssetBundle.from_manifest(wrong_schema)
    wrong_name = tmp_path / "wrong-name.json"
    wrong_name.write_text(
        json.dumps(
            {
                "schema_version": "unirobosim.asset-bundle.v1",
                "name": 7,
                "variants": {"fake": "robot.usd"},
            }
        )
    )
    with pytest.raises(ValidationError, match="name"):
        AssetBundle.from_manifest(wrong_name)


def test_asset_bundle_resolution_fallback_serialization_and_hash_failures(tmp_path: Path) -> None:
    asset = tmp_path / "robot.urdf"
    asset.write_bytes(b"urdf")
    digest = hashlib.sha256(asset.read_bytes()).hexdigest()
    manifest = tmp_path / "robot.json"
    manifest.write_text(
        json.dumps(
            {
                "schema_version": "unirobosim.asset-bundle.v1",
                "name": "robot",
                "variants": {
                    "fake": {"uri": asset.name, "sha256": digest.upper()},
                    "nvidia.isaaclab": {"uri": "https://example.invalid/robot.usdc", "sha256": "0" * 64},
                },
            }
        ),
        encoding="utf-8",
    )
    bundle = AssetBundle.from_manifest(manifest)
    resolved = bundle.resolve(backend="auto", provider_id="reference.fake")
    assert resolved.selector == "fake"
    assert resolved.media_type == "model/vnd.urdf+xml"
    assert resolved.sha256 == digest
    assert resolved.to_dict()["source_manifest"] == str(manifest.resolve())
    assert bundle.to_dict()["source_manifest"] == str(manifest.resolve())

    remote = bundle.resolve(backend="auto", provider_id="nvidia.isaaclab")
    assert remote.uri.startswith("https://") and remote.sha256 == "0" * 64
    assert (
        AssetBundle("xml", {"mujoco": "scene.xml"})
        .resolve(backend="mujoco", provider_id="google-deepmind.mujoco")
        .media_type
        == "application/xml"
    )
    assert "source_manifest" not in AssetBundle("usd", {"fake": "asset.usdc"}).to_dict()

    missing = AssetBundle("missing", {"fake": {"uri": str(tmp_path / "missing.usd"), "sha256": "0" * 64}})
    with pytest.raises(ValidationError, match="could not be hashed"):
        missing.resolve(backend="fake", provider_id="reference.fake")
    mismatched = AssetBundle("mismatch", {"fake": {"uri": str(asset), "sha256": "0" * 64}})
    with pytest.raises(ValidationError, match="does not match"):
        mismatched.resolve(backend="fake", provider_id="reference.fake")


class _EntryPoint:
    def __init__(self, name: str, factory: object) -> None:
        self.name = name
        self._factory = factory

    def load(self) -> object:
        if isinstance(self._factory, Exception):
            raise self._factory
        return self._factory


def test_easy_api_entry_point_discovery_selection_and_diagnostics(monkeypatch: pytest.MonkeyPatch) -> None:
    entries = (
        _EntryPoint("broken", RuntimeError("broken adapter")),
        _EntryPoint("fake", lambda: FakeProvider()),
    )
    monkeypatch.setattr(easy_sim.metadata, "entry_points", lambda **_: entries)
    sim = Sim(backend="auto")
    sim.add_box("box")
    sim.start()
    assert sim.provider_descriptor.provider_id == "reference.fake"
    sim.close()

    by_provider_id = Sim(backend="reference.fake")
    by_provider_id.add_box("box")
    by_provider_id.start()
    assert by_provider_id.provider_descriptor.provider_id == "reference.fake"
    by_provider_id.close()

    monkeypatch.setattr(easy_sim.metadata, "entry_points", lambda **_: ())
    missing = Sim(backend="missing")
    missing.add_box("box")
    with pytest.raises(ProviderSelectionError) as error:
        missing.start()
    assert error.value.details["installed_entry_points"] == ()


def test_easy_api_input_errors_and_build_cleanup() -> None:
    with pytest.raises(ValidationError):
        Sim(backend="")
    with pytest.raises(ValidationError):
        Sim(provider=object())  # type: ignore[arg-type]
    with pytest.raises(ValidationError, match="headless"):
        Sim(provider=FakeProvider(), headless=False)
    with pytest.raises(LifecycleError):
        _ = Sim().provider_descriptor
    before_start = Sim(provider=FakeProvider())
    with pytest.raises(ValidationError):
        before_start.add_box("")
    box = before_start.add_box("box")
    articulation = before_start.add_articulation("cabinet", joint_names=("hinge",))
    camera = before_start.add_camera("camera", resolution=(8, 8), outputs=("rgb",))
    with pytest.raises(ValidationError, match="duplicate capability"):
        before_start.require("state.rigid_body@1").require("state.rigid_body@1")
    before_start.start()
    assert box.kind is EntityKind.RIGID_BODY
    with pytest.raises(ValidationError, match="numeric"):
        box.apply_wrench("bad")
    with pytest.raises(ValidationError, match="iterable"):
        box.apply_wrench(1.0)
    with pytest.raises(ValidationError, match="matrix"):
        box.apply_wrench((("bad", 0.0, 0.0),))
    with pytest.raises(ValidationError, match="joint selection"):
        articulation.command((0.1,), joints=(True,))
    articulation.command((0.2,), joints=(0,), mode=CommandMode.VELOCITY)
    with pytest.raises(ValidationError, match="camera modality"):
        camera.read("normal")
    with pytest.raises(ValidationError, match="unknown entity"):
        before_start.entity("missing")
    before_start.close()

    failed = Sim(provider=FakeProvider(build_failures=1))
    failed.add_box("box")
    with pytest.raises(WorldBuildError):
        failed.start()
    assert failed.state is SimState.CONFIGURING
    failed.close()


def test_easy_api_deformable_and_particle_fluid_flow() -> None:
    with Sim(
        provider=FakeProvider(),
        world_id="easy-soft",
        num_envs=2,
        time_step_seconds=0.1,
        gravity_m_s2=(0.0, 0.0, 0.0),
    ) as sim:
        cloth = sim.add_deformable(
            "cloth",
            rest_positions_m=((0, 0, 1), (1, 0, 1), (1, 1, 1), (0, 1, 1)),
            surface_triangles=((0, 1, 2), (0, 2, 3)),
            kinematic_nodes=(0,),
            node_mass_kg=0.1,
        )
        gel = sim.add_deformable(
            "gel",
            topology="volume",
            rest_positions_m=((0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)),
            tetrahedra=((0, 1, 2, 3),),
        )
        water = sim.add_particle_fluid(
            "water",
            positions_m=((0, 0, 1), (0.02, 0, 1)),
            particle_radius_m=0.01,
        )
        sim.start()
        assert cloth.num_nodes == 4 and gel.num_nodes == 4 and water.num_particles == 2
        assert cloth.state.node_positions_m.shape == (2, 4, 3)
        assert water.state.particle_positions_m.shape == (2, 2, 3)

        cloth.command((2.0, 3.0, 4.0), nodes=(0,), environments=(1,))
        water.command(((1.0, 0.0, 0.0), (2.0, 0.0, 0.0)), mode="velocity")
        sim.step()
        assert cloth.state.node_positions_m.nested()[1][0] == (2.0, 3.0, 4.0)
        assert water.state.particle_positions_m.nested()[0][0][0] == pytest.approx(0.1)
        assert water.state.particle_positions_m.nested()[1][1][0] == pytest.approx(0.22)
        water.command(
            (((3.0, 0.0, 1.0),), ((4.0, 0.0, 1.0),)),
            mode=PointCommandMode.POSITION,
            particles=(0,),
        )
        sim.step()
        assert water.state.particle_positions_m.nested()[0][0] == (3.0, 0.0, 1.0)
        assert water.state.particle_positions_m.nested()[1][0] == (4.0, 0.0, 1.0)

        requirement_ids = {item.capability.value for item in sim.world_spec.requirements}
        assert "control.deformable.points@1" in requirement_ids
        assert "control.fluid.particles@1" in requirement_ids


def test_easy_api_soft_matter_validation_is_actionable() -> None:
    sim = Sim(provider=FakeProvider())
    with pytest.raises(ValidationError, match="topology"):
        sim.add_deformable("bad", topology="cloth", rest_positions_m=((0, 0, 0),))
    cloth = sim.add_deformable(
        "cloth",
        rest_positions_m=((0, 0, 1), (1, 0, 1), (0, 1, 1)),
        surface_triangles=((0, 1, 2),),
    )
    water = sim.add_particle_fluid("water", positions_m=((0, 0, 1),))
    sim.start()
    with pytest.raises(ValidationError, match="position, velocity, or force"):
        cloth.command((0, 0, 0), mode="effort")
    with pytest.raises(ValidationError, match="targets must have shape"):
        water.command(((0, 0, 0), (1, 1, 1), (2, 2, 2)))
    with pytest.raises(ValidationError):
        cloth.command((0, 0, 0), nodes=())
    sim.close()


def test_easy_api_checks_backend_point_modes_before_native_dispatch(monkeypatch: pytest.MonkeyPatch) -> None:
    from dataclasses import replace

    from unirobosim import CapabilityDeclaration, CapabilityId, CapabilitySet, FrozenMap
    from unirobosim.testing import fake_backend

    provider = FakeProvider()
    restricted = tuple(
        CapabilityDeclaration(item.capability, FrozenMap({"modes": ["position"]}), item.limitations)
        if item.capability == CapabilityId("control.fluid.particles@1")
        else item
        for item in provider.descriptor.capabilities
    )
    monkeypatch.setattr(
        fake_backend,
        "FAKE_DESCRIPTOR",
        replace(provider.descriptor, capabilities=CapabilitySet(restricted)),
    )
    sim = Sim(provider=provider)
    water = sim.add_particle_fluid("water", positions_m=((0, 0, 1),))
    sim.start()
    with pytest.raises(UnsupportedCapabilityError, match="velocity") as error:
        water.command((0, 0, 0), mode="velocity")
    assert error.value.details["supported_modes"] == ("position",)
    sim.close()
