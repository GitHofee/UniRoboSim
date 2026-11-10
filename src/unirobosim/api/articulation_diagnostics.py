"""Optional on-demand native drive readback; never commands or advances physics."""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass
from typing import Protocol, runtime_checkable

from .values import EntityPath


@dataclass(frozen=True, slots=True)
class ArticulationDiagnosticsRequest:
    entity_path: EntityPath
    joint_names: tuple[str, ...]

    def __post_init__(self) -> None:
        if type(self.entity_path) is not EntityPath:
            raise TypeError("entity_path must be an EntityPath")
        if type(self.joint_names) is not tuple or not 1 <= len(self.joint_names) <= 32:
            raise ValueError("select one to 32 joints")
        if any(type(n) is not str or not n or len(n) > 1024 or "\x00" in n for n in self.joint_names):
            raise ValueError("joint names must be bounded non-empty text")
        if len(set(self.joint_names)) != len(self.joint_names):
            raise ValueError("joint names must be unique")


@dataclass(frozen=True, slots=True)
class ArticulationDriveEvidence:
    joint_name: str
    position: float
    velocity: float
    position_target: float
    velocity_target: float
    stiffness: float
    damping: float
    max_force: float
    actuation_force: float

    def __post_init__(self) -> None:
        if type(self.joint_name) is not str or not self.joint_name or len(self.joint_name) > 1024:
            raise ValueError("invalid joint name")
        for name, value in asdict(self).items():
            if name != "joint_name" and (type(value) not in (float, int) or not math.isfinite(value)):
                raise ValueError("drive evidence must be finite numeric values")
        if min(self.stiffness, self.damping, self.max_force) < 0:
            raise ValueError("drive gains and limits cannot be negative")


@dataclass(frozen=True, slots=True)
class ArticulationDiagnosticsSnapshot:
    world_id: str
    generation: int
    world_step: int
    entity_path: EntityPath
    drives: tuple[ArticulationDriveEvidence, ...]
    sleep_threshold: float | None = None
    sleeping: bool | None = None

    def __post_init__(self) -> None:
        if type(self.world_id) is not str or not self.world_id or len(self.world_id) > 1024:
            raise ValueError("invalid world identity")
        if type(self.generation) is not int or self.generation < 1:
            raise ValueError("invalid generation")
        if type(self.world_step) is not int or self.world_step < 0:
            raise ValueError("invalid world step")
        if type(self.drives) is not tuple or any(type(d) is not ArticulationDriveEvidence for d in self.drives):
            raise TypeError("invalid drive evidence")
        ArticulationDiagnosticsRequest(self.entity_path, tuple(d.joint_name for d in self.drives))
        if self.sleep_threshold is not None and (
            type(self.sleep_threshold) not in (float, int)
            or not math.isfinite(self.sleep_threshold) or self.sleep_threshold < 0
        ):
            raise ValueError("invalid sleep threshold")
        if self.sleeping is not None and type(self.sleeping) is not bool:
            raise TypeError("invalid sleeping state")

    def to_dict(self) -> dict[str, object]:
        result = asdict(self)
        result["entity_path"] = self.entity_path.value
        result["schema"] = "articulation-diagnostics/1"
        result["unsupported_fields"] = tuple(n for n in ("sleep_threshold", "sleeping") if getattr(self, n) is None)
        return result


@runtime_checkable
class ArticulationDiagnosticsWorld(Protocol):
    def articulation_diagnostics(self, request: ArticulationDiagnosticsRequest) -> ArticulationDiagnosticsSnapshot: ...
