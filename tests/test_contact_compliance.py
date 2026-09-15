"""Compliant contact is explicit, typed, canonical and capability gated."""

import json
from dataclasses import FrozenInstanceError, replace

import pytest

from unirobosim.api import (
    BoxGeometrySpec,
    CapabilityId,
    CapabilityNegotiationError,
    CapabilityRequirement,
    ContactComplianceSpec,
    EntityKind,
    EntityPath,
    EntitySpec,
    Pose,
    ValidationError,
    WorldSpec,
)
from unirobosim.easy import Sim
from unirobosim.testing import FAKE_DESCRIPTOR, FakeProvider, FakeSession


def _body(compliance=None):
    return EntitySpec(
        EntityPath("/target"), EntityKind.RIGID_BODY, box=BoxGeometrySpec(), contact_compliance=compliance
    )


@pytest.mark.parametrize(
    "field,value",
    [
        ("stiffness_n_m", 0),
        ("stiffness_n_m", -1),
        ("stiffness_n_m", True),
        ("stiffness_n_m", "20"),
        ("stiffness_n_m", float("nan")),
        ("stiffness_n_m", float("inf")),
        ("stiffness_n_m", 10**1000),
        ("damping_n_s_m", -1),
        ("damping_n_s_m", False),
        ("damping_n_s_m", float("-inf")),
        ("damping_n_s_m", None),
        ("stiffness_combine_mode", "unsupported"),
        ("damping_combine_mode", []),
    ],
)
def test_invalid_contact_parameters(field, value):
    values = {"stiffness_n_m": 1000, "damping_n_s_m": 2}
    values[field] = value
    with pytest.raises(ValidationError):
        ContactComplianceSpec(**values)


@pytest.mark.parametrize("mode", ["average", "min", "multiply", "max"])
def test_canonical_roundtrip_and_immutable(mode):
    compliance = ContactComplianceSpec(1000, -0.0, mode, mode)
    assert type(compliance.stiffness_n_m) is float
    assert json.loads(json.dumps(compliance.to_dict())) == compliance.to_dict()
    assert ContactComplianceSpec(**compliance.to_dict()) == compliance
    assert str(compliance.damping_n_s_m) == "0.0"
    with pytest.raises(FrozenInstanceError):
        compliance.stiffness_n_m = 2000


def test_default_off_preserves_serialization_and_digest():
    body = _body()
    assert "contact_compliance" not in body.to_dict()
    world = WorldSpec("contact-test", (body,))
    # Generated with pre-compliance specs.py: default-off serialized bytes stay identical.
    assert world.digest == "5d81d9a506f118a2ce3d3c6b6b306dbba640b37312bfb9e8a3d6ac288d56f908"
    assert world == WorldSpec("contact-test", (replace(body, contact_compliance=None),))
    enabled = WorldSpec("contact-test", (replace(body, contact_compliance=ContactComplianceSpec(1000, 2)),))
    assert world.digest != enabled.digest
    assert enabled.to_dict()["entities"][0]["contact_compliance"] == ContactComplianceSpec(1000, 2).to_dict()
    reconstructed = replace(
        enabled,
        entities=(
            replace(
                body,
                contact_compliance=ContactComplianceSpec(
                    **json.loads(enabled.canonical_json)["entities"][0]["contact_compliance"]
                ),
            ),
        ),
    )
    assert reconstructed.digest == enabled.digest
    assert replace(
        enabled.entities[0], pose=Pose(position=(1.0, 0.0, 0.0))
    ).contact_compliance == ContactComplianceSpec(1000, 2)


def test_nonrigid_entity_and_untyped_mapping_rejected():
    with pytest.raises(ValidationError, match="standalone rigid body"):
        EntitySpec(EntityPath("/target"), EntityKind.STATIC_SCENE, contact_compliance=ContactComplianceSpec(1000, 2))
    with pytest.raises(ValidationError, match="ContactComplianceSpec"):
        _body({"stiffness_n_m": 1000, "damping_n_s_m": 2})


def test_capability_cannot_be_optional_and_unsupported_backend_rejects_before_build():
    capability = CapabilityId("physics.contact.compliant@1")
    spec = WorldSpec(
        "contact-test",
        (_body(ContactComplianceSpec(1000, 2)),),
        requirements=(CapabilityRequirement(capability, required=False),),
    )
    assert next(item for item in spec.requirements if item.capability == capability).required
    session = FakeSession(FAKE_DESCRIPTOR)
    with pytest.raises(CapabilityNegotiationError):
        session.build(spec)
    assert session.side_effect_snapshot().generation == 0


@pytest.mark.parametrize("asset", [False, True])
def test_easy_forwards_to_world_and_does_not_silently_emulate(asset):
    sim = Sim(provider=FakeProvider())
    compliance = ContactComplianceSpec(1000, 2)
    if asset:
        body = sim.add_rigid_body("target", asset_uri="file:///tmp/target.usd", contact_compliance=compliance)
    else:
        body = sim.add_box("target", contact_compliance=compliance)
    with pytest.raises(CapabilityNegotiationError):
        sim.start()
    assert body._spec.contact_compliance == compliance
