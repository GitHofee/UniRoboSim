"""Capability-gated physical checkpoint capture and restore."""

from __future__ import annotations

import re
from dataclasses import dataclass
from enum import StrEnum
from typing import Protocol, runtime_checkable

from .capabilities import CapabilityId
from .values import Tick

CHECKPOINT_CAPABILITY_ID = CapabilityId("checkpoint@1")
CHECKPOINT_SCHEMA_VERSION = "unirobosim-world-checkpoint/1"
CHECKPOINT_RESTORE_RESULT_SCHEMA_VERSION = "unirobosim-checkpoint-restore-result/1"

_IDENTIFIER = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.:/@+-]{0,255}$")
_MAX_PAYLOAD_BYTES = 4 * 1024**3


class CheckpointFidelity(StrEnum):
    """Backend-declared restoration fidelity for one opaque payload."""

    PHYSICAL = "physical"
    NATIVE = "native"


@dataclass(frozen=True, slots=True)
class WorldCheckpoint:
    """Opaque immutable state owned by one exact provider/world implementation."""

    provider_id: str
    world_id: str
    source_generation: int
    source_tick: Tick
    payload_schema: str
    fidelity: CheckpointFidelity
    payload: bytes
    entity_count: int
    schema_version: str = CHECKPOINT_SCHEMA_VERSION

    def __post_init__(self) -> None:
        if self.schema_version != CHECKPOINT_SCHEMA_VERSION:
            raise ValueError("world checkpoint schema_version is unsupported")
        for label, value in (
            ("provider_id", self.provider_id),
            ("world_id", self.world_id),
            ("payload_schema", self.payload_schema),
        ):
            if type(value) is not str or _IDENTIFIER.fullmatch(value) is None:
                raise ValueError(f"world checkpoint {label} is invalid")
        if type(self.source_generation) is not int or self.source_generation <= 0:
            raise ValueError("world checkpoint source_generation must be positive")
        if not isinstance(self.source_tick, Tick):
            raise TypeError("world checkpoint source_tick must be a Tick")
        if type(self.fidelity) is not CheckpointFidelity:
            raise TypeError("world checkpoint fidelity is invalid")
        if type(self.payload) is not bytes or not self.payload or len(self.payload) > _MAX_PAYLOAD_BYTES:
            raise ValueError("world checkpoint payload is empty or exceeds the 4 GiB v1 limit")
        if type(self.entity_count) is not int or self.entity_count < 0:
            raise ValueError("world checkpoint entity_count must be non-negative")


@dataclass(frozen=True, slots=True)
class CheckpointRestoreResult:
    """Settlement for an atomic restore that does not advance the physics clock."""

    generation: int
    tick: Tick
    state_revision: int
    restored_entity_count: int
    schema_version: str = CHECKPOINT_RESTORE_RESULT_SCHEMA_VERSION

    def __post_init__(self) -> None:
        if self.schema_version != CHECKPOINT_RESTORE_RESULT_SCHEMA_VERSION:
            raise ValueError("checkpoint restore result schema_version is unsupported")
        if type(self.generation) is not int or self.generation <= 0:
            raise ValueError("checkpoint restore generation must be positive")
        if not isinstance(self.tick, Tick):
            raise TypeError("checkpoint restore tick must be a Tick")
        if type(self.state_revision) is not int or self.state_revision <= 0:
            raise ValueError("checkpoint restore state_revision must be positive")
        if type(self.restored_entity_count) is not int or self.restored_entity_count < 0:
            raise ValueError("checkpoint restore entity count must be non-negative")


@runtime_checkable
class CheckpointWorld(Protocol):
    """Optional same-provider physical checkpoint capability."""

    def create_checkpoint(self) -> WorldCheckpoint: ...

    def restore_checkpoint(self, checkpoint: WorldCheckpoint) -> CheckpointRestoreResult: ...


__all__ = (
    "CHECKPOINT_CAPABILITY_ID",
    "CHECKPOINT_RESTORE_RESULT_SCHEMA_VERSION",
    "CHECKPOINT_SCHEMA_VERSION",
    "CheckpointFidelity",
    "CheckpointRestoreResult",
    "CheckpointWorld",
    "WorldCheckpoint",
)
