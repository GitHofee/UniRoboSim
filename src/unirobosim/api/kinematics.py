"""Pay-for-play selected rigid-link kinematics extension."""

from __future__ import annotations

import math
from collections.abc import Iterable
from dataclasses import dataclass
from typing import Protocol, runtime_checkable

from .capabilities import CapabilityId
from .values import EntityPath, Pose, Tick

SELECTED_KINEMATICS_CAPABILITY = CapabilityId("state.kinematics.selected@1")


def _text(value: object, name: str) -> str:
    if type(value) is not str or not value or len(value) > 1024 or "\x00" in value:
        raise ValueError(f"{name} must be a non-empty bounded string")
    return value


def _vector3(value: object, name: str) -> tuple[float, float, float]:
    if type(value) is not tuple or len(value) != 3:
        raise TypeError(f"{name} must be an exact three-value tuple")
    result = tuple(float(item) for item in value)
    if not all(math.isfinite(item) for item in result):
        raise ValueError(f"{name} must contain only finite values")
    return result  # type: ignore[return-value]


@dataclass(frozen=True, slots=True)
class KinematicTarget:
    """One selected physical body on an entity.

    ``link_name=None`` selects the entity root body. Otherwise ``link_name`` is
    the authored articulation body/link name. ``target_id`` is caller-owned and
    is returned unchanged so callers never need backend path identities.
    """

    target_id: str
    entity_path: EntityPath
    link_name: str | None = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "target_id", _text(self.target_id, "target_id"))
        if type(self.entity_path) is not EntityPath:
            raise TypeError("entity_path must be an exact EntityPath")
        if self.link_name is not None:
            object.__setattr__(self, "link_name", _text(self.link_name, "link_name"))


@dataclass(frozen=True, slots=True)
class KinematicState:
    """Environment-local world pose and twist for one selected target."""

    target_id: str
    entity_path: EntityPath
    link_name: str | None
    tick: Tick
    pose: Pose
    linear_velocity_m_s: tuple[float, float, float]
    angular_velocity_rad_s: tuple[float, float, float]

    def __post_init__(self) -> None:
        object.__setattr__(self, "target_id", _text(self.target_id, "target_id"))
        if type(self.entity_path) is not EntityPath:
            raise TypeError("entity_path must be an exact EntityPath")
        if self.link_name is not None:
            object.__setattr__(self, "link_name", _text(self.link_name, "link_name"))
        if type(self.tick) is not Tick:
            raise TypeError("tick must be an exact Tick")
        if type(self.pose) is not Pose:
            raise TypeError("pose must be an exact Pose")
        object.__setattr__(
            self,
            "linear_velocity_m_s",
            _vector3(self.linear_velocity_m_s, "linear_velocity_m_s"),
        )
        object.__setattr__(
            self,
            "angular_velocity_rad_s",
            _vector3(self.angular_velocity_rad_s, "angular_velocity_rad_s"),
        )


@runtime_checkable
class SelectedKinematicsWorld(Protocol):
    """Optional O(K) state extension that never materializes scene geometry."""

    def read_selected_kinematics(
        self,
        targets: Iterable[KinematicTarget],
        environment_index: int = 0,
    ) -> tuple[KinematicState, ...]: ...


__all__ = [
    "KinematicState",
    "KinematicTarget",
    "SELECTED_KINEMATICS_CAPABILITY",
    "SelectedKinematicsWorld",
]
