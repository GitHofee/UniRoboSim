from __future__ import annotations

import pytest

from unirobosim import (
    SELECTED_KINEMATICS_CAPABILITY,
    EntityPath,
    KinematicState,
    KinematicTarget,
    Pose,
    Tick,
)


def test_selected_kinematics_contract_is_typed_and_capability_gated() -> None:
    target = KinematicTarget("outlet", EntityPath("/articulations/pour_arm"), "kettle_link")
    state = KinematicState(
        "outlet",
        target.entity_path,
        target.link_name,
        Tick(7, 0.11666666666666667),
        Pose((0.5, -0.1, 0.9), (0.0, 0.0, 0.0, 1.0)),
        (0.1, 0.2, 0.3),
        (0.0, 1.0, 0.0),
    )

    assert SELECTED_KINEMATICS_CAPABILITY.value == "state.kinematics.selected@1"
    assert state.target_id == target.target_id
    assert state.link_name == "kettle_link"


def test_selected_kinematics_rejects_invalid_values() -> None:
    with pytest.raises(ValueError, match="target_id"):
        KinematicTarget("", EntityPath("/articulations/pour_arm"), "kettle_link")
    with pytest.raises(ValueError, match="finite"):
        KinematicState(
            "outlet",
            EntityPath("/articulations/pour_arm"),
            "kettle_link",
            Tick(0, 0.0),
            Pose(),
            (float("nan"), 0.0, 0.0),
            (0.0, 0.0, 0.0),
        )
