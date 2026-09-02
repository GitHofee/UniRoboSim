from __future__ import annotations

import pytest

from unirobosim.api import (
    CHECKPOINT_CAPABILITY_ID,
    CheckpointFidelity,
    CheckpointRestoreResult,
    CheckpointWorld,
    Tick,
    WorldCheckpoint,
)


def _checkpoint(**overrides: object) -> WorldCheckpoint:
    values = {
        "provider_id": "reference.fake",
        "world_id": "checkpoint-world",
        "source_generation": 1,
        "source_tick": Tick(12, 0.2),
        "payload_schema": "reference.fake-checkpoint/1",
        "fidelity": CheckpointFidelity.PHYSICAL,
        "payload": b"checkpoint",
        "entity_count": 2,
    }
    values.update(overrides)
    return WorldCheckpoint(**values)  # type: ignore[arg-type]


def test_checkpoint_contract_is_public_and_runtime_checkable() -> None:
    checkpoint = _checkpoint()

    class _World:
        def create_checkpoint(self):
            return checkpoint

        def restore_checkpoint(self, value):
            assert value is checkpoint
            return CheckpointRestoreResult(1, Tick(12, 0.2), 2, 2)

    assert CHECKPOINT_CAPABILITY_ID.value == "checkpoint@1"
    assert isinstance(_World(), CheckpointWorld)
    assert _World().restore_checkpoint(checkpoint).restored_entity_count == 2


@pytest.mark.parametrize(
    ("field", "value", "error"),
    (
        ("provider_id", "", ValueError),
        ("source_generation", 0, ValueError),
        ("source_tick", object(), TypeError),
        ("payload", b"", ValueError),
        ("entity_count", -1, ValueError),
    ),
)
def test_checkpoint_rejects_invalid_values(field: str, value: object, error: type[Exception]) -> None:
    with pytest.raises(error):
        _checkpoint(**{field: value})
