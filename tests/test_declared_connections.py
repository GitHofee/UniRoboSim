from dataclasses import replace

import pytest

from unirobosim import (
    CompliantConnectionSpec,
    ConnectionProperties,
    EntityKind,
    EntityPath,
    EntitySpec,
    Pose,
    ValidationError,
    WorldSpec,
)


def connection():
    return CompliantConnectionSpec(
        "socket",
        EntityPath("/phone"),
        EntityPath("/plug"),
        Pose((0.0, -0.082, 0.0)),
        ConnectionProperties(300.0, 1.0, 1.0, 0.002, 3.0, 0.01, 0.008, 0.35, 0.0015, 0.02),
    )


def world():
    return WorldSpec(
        "connector", tuple(EntitySpec(EntityPath(path), EntityKind.RIGID_BODY) for path in ("/phone", "/plug"))
    )


def test_default_world_remains_without_connection_data_or_requirement():
    empty = world()
    assert "compliant_connections" not in empty.to_dict()
    assert all(item.capability.value != "physics.compliant_connections@1" for item in empty.requirements)
    assert replace(empty, compliant_connections=()).digest == empty.digest


def test_declaration_is_serialized_hashed_and_requires_backend_support():
    connected = replace(world(), compliant_connections=(connection(),))
    assert connected.to_dict()["compliant_connections"][0]["child_path"] == "/plug"
    assert connected.digest != world().digest
    assert any(
        item.capability.value == "physics.compliant_connections@1" and item.required for item in connected.requirements
    )


@pytest.mark.parametrize(
    "changes",
    [
        {"parent_path": EntityPath("/missing")},
        {"child_path": EntityPath("/phone")},
        {"connection_id": "bad/name"},
        {"properties": None},
    ],
)
def test_invalid_endpoints_and_properties_are_rejected(changes):
    with pytest.raises(ValidationError):
        replace(world(), compliant_connections=(replace(connection(), **changes),))


def test_duplicate_child_and_robot_endpoints_rejected():
    c = connection()
    with pytest.raises(ValidationError):
        replace(world(), compliant_connections=(c, replace(c, connection_id="other")))
    with pytest.raises(ValidationError):
        replace(
            world(),
            entities=(
                EntitySpec(EntityPath("/phone"), EntityKind.ARTICULATION),
                EntitySpec(EntityPath("/plug"), EntityKind.RIGID_BODY),
            ),
            compliant_connections=(c,),
        )


def test_numeric_equivalence_preserves_canonical_digest():
    c = connection()
    original = replace(world(), compliant_connections=(c,))
    integer = replace(c, properties=replace(c.properties, stiffness_n_m=300))
    assert replace(world(), compliant_connections=(integer,)).digest == original.digest


@pytest.mark.parametrize(
    "changes",
    [
        {"admission_position_m": 0.008},
        {"admission_angle_rad": 0.35},
        {"break_angle_rad": 3.2},
    ],
)
def test_invalid_admission_envelopes(changes):
    with pytest.raises(ValidationError):
        replace(connection().properties, **changes)


def test_optional_capability_is_promoted_without_losing_constraints():
    from unirobosim import CapabilityId, CapabilityRequirement, FrozenMap

    capability = CapabilityId("physics.compliant_connections@1")
    requirement = CapabilityRequirement(capability, required=False, reason="caller", constraints=FrozenMap({"x": 1}))
    connected = replace(world(), compliant_connections=(connection(),), requirements=(requirement,))
    selected = [item for item in connected.requirements if item.capability == capability]
    assert len(selected) == 1
    assert selected[0].required
    assert selected[0].reason == requirement.reason
    assert selected[0].constraints == requirement.constraints


def test_connection_order_is_canonical_and_duplicate_ids_are_rejected():
    c = connection()
    other = replace(c, connection_id="another", child_path=EntityPath("/other"))
    base = replace(world(), entities=(*world().entities, EntitySpec(EntityPath("/other"), EntityKind.RIGID_BODY)))
    forward = replace(base, compliant_connections=(c, other))
    backward = replace(base, compliant_connections=(other, c))
    assert forward.digest == backward.digest
    with pytest.raises(ValidationError):
        replace(base, compliant_connections=(c, replace(other, connection_id=c.connection_id)))


@pytest.mark.parametrize("value", [None, (object(),)])
def test_invalid_declaration_collection(value):
    with pytest.raises(ValidationError):
        replace(world(), compliant_connections=value)
