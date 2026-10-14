from dataclasses import replace

import pytest

from tests.test_soft_matter_specs import volume_body
from unirobosim import (
    CapabilityId,
    DeformableMaterialSpec,
    DeformableRigidMount,
    EntityKind,
    EntityPath,
    EntitySpec,
    ValidationError,
    WorldSpec,
)


def test_mount_and_material_serialization_and_required_capabilities():
    mount = DeformableRigidMount(EntityPath("/hand"), "finger", (0, 1, 2))
    material = DeformableMaterialSpec(200000, static_friction=0.8, dynamic_friction=0.7)
    body = volume_body(rigid_mount=mount, elastic_material=material)
    pad = EntitySpec(EntityPath("/pad"), EntityKind.VOLUME_DEFORMABLE, deformable=body)
    hand = EntitySpec(EntityPath("/hand"), EntityKind.ARTICULATION, joint_names=("slide",))
    world = WorldSpec("mounted-pad", (pad, hand))
    requirements = {r.capability for r in world.requirements}
    assert CapabilityId("physics.deformable.rigid-mount@1") in requirements
    assert CapabilityId("physics.deformable.elastic-material@1") in requirements
    assert body.to_dict()["rigid_mount"]["parent_path"] == "/hand"
    assert body.to_dict()["elastic_material"]["youngs_modulus_pa"] == 200000
    with pytest.raises(ValidationError):
        replace(world, entities=(pad,))
    with pytest.raises(ValidationError):
        replace(body, kinematic_node_indices=(0,))


@pytest.mark.parametrize(
    "kwargs",
    [
        {"youngs_modulus_pa": 0},
        {"youngs_modulus_pa": float("nan")},
        {"youngs_modulus_pa": True},
        {"youngs_modulus_pa": 100, "poissons_ratio": 0.5},
        {"youngs_modulus_pa": 100, "dynamic_friction": -1},
    ],
)
def test_invalid_material_rejected(kwargs):
    with pytest.raises(ValidationError):
        DeformableMaterialSpec(**kwargs)


def test_invalid_parent_link_and_indices_rejected():
    with pytest.raises(ValidationError):
        DeformableRigidMount(EntityPath("/hand"), "")
    with pytest.raises(ValidationError):
        DeformableRigidMount(EntityPath("/hand"), "finger", (0, 0))
    with pytest.raises(ValidationError):
        volume_body(rigid_mount=DeformableRigidMount(EntityPath("/hand"), "finger", (4,)))


@pytest.mark.parametrize("contact,rest", [(0, 0), (0.001, 0.001), (0.001, -1), (True, 0), (None, 1)])
def test_invalid_contact_offsets_rejected(contact, rest):
    with pytest.raises(ValidationError):
        volume_body(contact_offset_m=contact, rest_offset_m=rest)


def test_contact_offsets_are_optional_and_explicit():
    assert "contact_offset_m" not in volume_body().to_dict()
    body = volume_body(contact_offset_m=0.0005, rest_offset_m=0)
    assert body.to_dict()["contact_offset_m"] == 0.0005


def test_mount_coupling_is_explicit_and_validated():
    mount = DeformableRigidMount(EntityPath("/hand"), "finger", (0,), "kinematic_boundary")
    assert mount.to_dict()["coupling"] == "kinematic_boundary"
    with pytest.raises(ValidationError):
        replace(mount, coupling="silent-fallback")


@pytest.mark.parametrize(
    "field", ["youngs_modulus_pa", "poissons_ratio", "static_friction", "dynamic_friction", "elasticity_damping"]
)
@pytest.mark.parametrize("value", [True, "1", float("inf"), float("nan"), 10**1000])
def test_every_material_field_has_structured_numeric_errors(field, value):
    args = {"youngs_modulus_pa": 100}
    args[field] = value
    with pytest.raises(ValidationError):
        DeformableMaterialSpec(**args)


@pytest.mark.parametrize("contact", [None, 1])
@pytest.mark.parametrize("rest", [False, None, "0", 10**1000, float("nan"), float("inf")])
def test_rest_validated_even_without_contact(contact, rest):
    with pytest.raises(ValidationError):
        volume_body(contact_offset_m=contact, rest_offset_m=rest)


def test_contact_overflow_and_material_conflict():
    with pytest.raises(ValidationError):
        volume_body(contact_offset_m=10**1000)
    with pytest.raises(ValidationError, match="mutually exclusive"):
        volume_body(material_id="legacy", elastic_material=DeformableMaterialSpec(100))
    assert volume_body(material_id="legacy").material_id == "legacy"


def test_default_digest_matches_pre_extension_release():
    world = WorldSpec(
        "default-soft", (EntitySpec(EntityPath("/pad"), EntityKind.VOLUME_DEFORMABLE, deformable=volume_body()),)
    )
    # Generated on origin/main b529ddc, not from the new implementation.
    assert world.digest == "6d58ce806f5483e82078cd9fe7879a50cf5154e02d54190b10e740e8d2340b47"
    assert not {"elastic_material", "rigid_mount", "contact_offset_m", "rest_offset_m"}.intersection(
        volume_body().to_dict()
    )


def mounted_world(mode="solver", **body_fields):
    mount = DeformableRigidMount(EntityPath("/hand"), "finger", (0, 1), mode)
    return WorldSpec(
        "mounted",
        (
            EntitySpec(
                EntityPath("/pad"),
                EntityKind.VOLUME_DEFORMABLE,
                deformable=volume_body(rigid_mount=mount, **body_fields),
            ),
            EntitySpec(EntityPath("/hand"), EntityKind.ARTICULATION, joint_names=("hinge",)),
        ),
    )


@pytest.mark.parametrize("mode,other", [("solver", "kinematic_boundary"), ("kinematic_boundary", "solver")])
def test_exact_mode_negotiation_cannot_silently_fallback(mode, other):
    from unirobosim import CapabilityDeclaration, CapabilitySet

    world = mounted_world(mode)
    declarations = tuple(CapabilityDeclaration(r.capability) for r in world.requirements)
    assert not CapabilitySet(declarations).negotiate(world.requirements).required_issues
    mismatched = mounted_world(other)
    issues = CapabilitySet(declarations).negotiate(mismatched.requirements).required_issues
    assert [i.capability.value for i in issues] == [f"physics.deformable.rigid-mount.{other.replace('_', '-')}@1"]


def test_independent_offsets_gate_and_optional_requirements_preserved():
    from unirobosim import CapabilityNegotiationError, CapabilityRequirement, FrozenMap
    from unirobosim.testing import FAKE_DESCRIPTOR, FakeSession

    cap = CapabilityId("physics.deformable.contact-offsets@1")
    original = CapabilityRequirement(cap, required=False, reason="caller", constraints=FrozenMap({"custom": "exact"}))
    world = WorldSpec(
        "offsets",
        (EntitySpec(EntityPath("/pad"), EntityKind.VOLUME_DEFORMABLE, deformable=volume_body(contact_offset_m=1)),),
        requirements=(original,),
    )
    requirement = next(r for r in world.requirements if r.capability == cap)
    assert (
        requirement.required
        and requirement.reason == original.reason
        and requirement.constraints == original.constraints
    )
    session = FakeSession(FAKE_DESCRIPTOR)
    with pytest.raises(CapabilityNegotiationError):
        session.build(world)
    assert session.side_effect_snapshot().generation == 0


def test_transport_roundtrip_preserves_modes_material_offsets_and_digest():
    import json
    import pickle
    from multiprocessing.reduction import ForkingPickler

    from unirobosim import ArrayValue

    for mode in ("solver", "kinematic_boundary"):
        velocities = ArrayValue.from_rows(((0, 0, 0), (0, 0, 0), (1, 2, 3), (0, 0, 0)))
        world = mounted_world(
            mode,
            contact_offset_m=1,
            rest_offset_m=-0.0,
            elastic_material=DeformableMaterialSpec(100, elasticity_damping=0),
            initial_node_velocities_m_s=velocities,
        )
        # Existing adapter worker transport is trusted multiprocessing pickle, not a new JSON decoder.
        restored = pickle.loads(ForkingPickler.dumps(world))
        assert restored == world
        assert restored.digest == world.digest
        assert json.loads(restored.canonical_json) == json.loads(world.canonical_json)
        body = world.entities[1].deformable
        assert type(body.contact_offset_m) is float and str(body.rest_offset_m) == "0.0"
        numeric_equivalent = replace(
            world,
            entities=(replace(world.entities[1], deformable=replace(body, contact_offset_m=1.0)), world.entities[0]),
        )
        assert numeric_equivalent.digest == world.digest
        assert (
            replace(
                world,
                entities=(
                    replace(world.entities[1], deformable=replace(body, contact_offset_m=2.0)),
                    world.entities[0],
                ),
            ).digest
            != world.digest
        )


@pytest.mark.parametrize("mode", ["solver", "kinematic_boundary"])
def test_backing_velocity_rejected_but_free_node_relative_velocity_preserved(mode):
    from unirobosim import ArrayValue

    with pytest.raises(ValidationError, match="zero relative"):
        mounted_world(mode, initial_node_velocities_m_s=ArrayValue.from_rows(((1, 0, 0),) * 4))
    world = mounted_world(
        mode, initial_node_velocities_m_s=ArrayValue.from_rows(((0, 0, 0), (0, 0, 0), (1, 0, 0), (0, 0, 0)))
    )
    assert world.entities[1].deformable.initial_velocities().rows()[2] == (1, 0, 0)


def test_physical_parent_and_topology_combinations():
    from tests.test_soft_matter_specs import surface_body

    with pytest.raises(ValidationError):
        surface_body(
            rigid_mount=DeformableRigidMount(EntityPath("/parent"), node_indices=(0,)), kinematic_node_indices=()
        )
    world = mounted_world()
    parent = EntitySpec(world.entities[0].path, EntityKind.RIGID_BODY)
    with pytest.raises(ValidationError, match="link name"):
        replace(world, entities=(world.entities[1], parent))
    body = replace(world.entities[1].deformable, rigid_mount=DeformableRigidMount(parent.path, node_indices=(0,)))
    assert replace(world, entities=(replace(world.entities[1], deformable=body), parent))
    with pytest.raises(ValidationError, match="parent link"):
        replace(world, entities=(replace(world.entities[1], deformable=body), world.entities[0]))
