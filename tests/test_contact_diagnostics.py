import pickle

import pytest

from unirobosim import (
    ContactDiagnosticsBatch,
    ContactDiagnosticsRequest,
    ContactDiagnosticsSample,
    ContactForcePoint,
    ContactPairEvidence,
    EntityPath,
    KinematicTarget,
)


def sample(sequence=1):
    p = ContactForcePoint((0.0, 0.0, 0.0), (0.0, 0.0, 2.0))
    pair = ContactPairEvidence(
        "pad", (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0), (p,), (), (0.0, 0.0, 2.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    )
    return ContactDiagnosticsSample(
        sequence, 1, 1, 4, 1 / 240, (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
        (0.0, 0.0, 0.0), (0.0,) * 6, (pair,), 0.18,
        (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 0.0), (0.0,) * 6
    )


def test_contact_request_and_evidence_survive_worker_pickle():
    request = ContactDiagnosticsRequest(
        KinematicTarget("cup", EntityPath("/cup")), (KinematicTarget("pad", EntityPath("/robot"), "finger"),)
    )
    batch = ContactDiagnosticsBatch("world", 1, 1, 1, (sample(),))
    assert pickle.loads(pickle.dumps(request)) == request
    assert pickle.loads(pickle.dumps(batch)) == batch
    assert batch.to_dict()["schema"] == "contact-diagnostics/1"


@pytest.mark.parametrize("value", [True, 0, -1, 16385, 1.5])
def test_invalid_history_capacity_is_rejected(value):
    with pytest.raises(ValueError):
        ContactDiagnosticsRequest(
            KinematicTarget("cup", EntityPath("/cup")),
            (KinematicTarget("pad", EntityPath("/robot"), "finger"),),
            capacity_substeps=value,
        )


def test_same_body_and_sequence_gaps_are_rejected():
    with pytest.raises(ValueError):
        ContactDiagnosticsRequest(KinematicTarget("a", EntityPath("/cup")), (KinematicTarget("b", EntityPath("/cup")),))
    with pytest.raises(ValueError):
        ContactDiagnosticsBatch("world", 1, 1, 3, (sample(1), sample(3)))


@pytest.mark.parametrize("force", [(float("nan"), 0.0, 0.0), (True, 0.0, 0.0), (0.0, 0.0)])
def test_invalid_force_vector_is_rejected(force):
    with pytest.raises((TypeError, ValueError)):
        ContactForcePoint((0.0, 0.0, 0.0), force)


def test_robot_link_and_static_geometry_selection_round_trip():
    from unirobosim import StaticContactTarget

    request = ContactDiagnosticsRequest(
        KinematicTarget("base", EntityPath("/robot"), "base_link"),
        (StaticContactTarget("wall", "geometry.wall123"),),
    )
    assert pickle.loads(pickle.dumps(request)) == request
    with pytest.raises(ValueError, match="unique"):
        ContactDiagnosticsRequest(
            request.subject, (request.partners[0], StaticContactTarget("other", "geometry.wall123")),
        )


@pytest.mark.parametrize("value", ["", "../wall", "/World/wall", "geometry.*", None])
def test_static_contact_rejects_non_catalog_selectors(value):
    from unirobosim import StaticContactTarget

    with pytest.raises(ValueError):
        StaticContactTarget("wall", value)
