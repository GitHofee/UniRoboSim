import pickle

import pytest

from unirobosim import (
    ArticulationDiagnosticsRequest,
    ArticulationDiagnosticsSnapshot,
    ArticulationDriveEvidence,
    EntityPath,
)


def test_native_drive_snapshot_round_trip_and_explicit_unsupported_sleep():
    drive = ArticulationDriveEvidence("x", 0.1, 0.0, 0.5, 0.0, 16000.0, 64.0, 5000.0, 0.0)
    result = ArticulationDiagnosticsSnapshot("world", 1, 7, EntityPath("/robot"), (drive,))
    assert pickle.loads(pickle.dumps(result)) == result
    assert result.to_dict()["unsupported_fields"] == ("sleep_threshold", "sleeping")
    assert result.to_dict()["entity_path"] == "/robot"


@pytest.mark.parametrize("names", [(), ("x", "x"), ("",), ("x\x00",), tuple(str(i) for i in range(33))])
def test_drive_selection_is_bounded_and_unambiguous(names):
    with pytest.raises(ValueError):
        ArticulationDiagnosticsRequest(EntityPath("/robot"), names)


@pytest.mark.parametrize("value", [float("nan"), float("inf"), True])
def test_drive_snapshot_rejects_invalid_numeric_evidence(value):
    with pytest.raises(ValueError):
        ArticulationDriveEvidence("x", value, 0.0, 0.5, 0.0, 1.0, 1.0, 10.0, 0.0)
