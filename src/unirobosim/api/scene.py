"""Portable scene-state and transactional browser-control values."""

from __future__ import annotations

import math
import re
from dataclasses import dataclass, field
from enum import StrEnum
from typing import Any

from .errors import ValidationError
from .frozen import FrozenMap
from .values import EntityKind, EntityPath, Pose, Tick

SCENE_SCHEMA_VERSION = "unirobosim.scene/v1alpha1"
_IDENTIFIER = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.:/-]{0,127}$")


def _invalid(message: str) -> ValidationError:
    return ValidationError(message, operation="scene.validate")


def _finite(values: tuple[float, ...], length: int, name: str) -> tuple[float, ...]:
    try:
        result = tuple(float(value) for value in values)
    except (TypeError, ValueError) as exc:
        raise _invalid(f"{name} must contain finite numbers") from exc
    if len(result) != length or not all(math.isfinite(value) for value in result):
        raise _invalid(f"{name} must contain {length} finite numbers")
    return result


class SceneVisualKind(StrEnum):
    BOX = "box"
    SPHERE = "sphere"
    CAPSULE = "capsule"
    CYLINDER = "cylinder"
    MESH = "mesh"
    POINT_CLOUD = "point_cloud"


class SceneCommandKind(StrEnum):
    SET_POSE = "set_pose"
    DRAG_BEGIN = "drag_begin"
    DRAG_UPDATE = "drag_update"
    DRAG_END = "drag_end"
    DRAG_CANCEL = "drag_cancel"


class SceneDragMode(StrEnum):
    CONSTRAINT = "constraint"
    KINEMATIC = "kinematic"


class SceneCommandStatus(StrEnum):
    APPLIED = "applied"
    DUPLICATE = "duplicate"
    REJECTED = "rejected"


@dataclass(frozen=True)
class SceneVisual:
    """Small render descriptor; dimensions are full XYZ extents in metres."""

    visual_id: str
    kind: SceneVisualKind
    local_pose: Pose = field(default_factory=Pose)
    dimensions_m: tuple[float, float, float] = (1.0, 1.0, 1.0)
    color_rgba: tuple[float, float, float, float] = (0.65, 0.72, 0.82, 1.0)
    asset_uri: str | None = None
    metadata: FrozenMap = field(default_factory=FrozenMap)

    def __post_init__(self) -> None:
        if not isinstance(self.visual_id, str) or not _IDENTIFIER.fullmatch(self.visual_id):
            raise _invalid("scene visual ID is invalid")
        if not isinstance(self.kind, SceneVisualKind) or not isinstance(self.local_pose, Pose):
            raise _invalid("scene visual kind or pose is invalid")
        dimensions = _finite(self.dimensions_m, 3, "dimensions_m")
        color = _finite(self.color_rgba, 4, "color_rgba")
        if any(value <= 0.0 for value in dimensions) or any(not 0.0 <= value <= 1.0 for value in color):
            raise _invalid("scene visual dimensions/color are out of range")
        if self.kind is SceneVisualKind.MESH:
            if not isinstance(self.asset_uri, str) or not self.asset_uri.strip():
                raise _invalid("mesh visuals require an asset URI")
        elif self.asset_uri is not None:
            raise _invalid("only mesh visuals may contain an asset URI")
        if not isinstance(self.metadata, FrozenMap):
            raise _invalid("scene visual metadata must be a FrozenMap")
        object.__setattr__(self, "dimensions_m", dimensions)
        object.__setattr__(self, "color_rgba", color)

    def to_dict(self) -> dict[str, Any]:
        return {
            "visual_id": self.visual_id,
            "kind": self.kind.value,
            "local_pose": _pose_dict(self.local_pose),
            "dimensions_m": list(self.dimensions_m),
            "color_rgba": list(self.color_rgba),
            "asset_uri": self.asset_uri,
            "metadata": self.metadata.to_dict(),
        }


@dataclass(frozen=True)
class SceneEntityState:
    path: EntityPath
    kind: EntityKind
    environment_index: int
    pose: Pose
    linear_velocity_m_s: tuple[float, float, float] = (0.0, 0.0, 0.0)
    angular_velocity_rad_s: tuple[float, float, float] = (0.0, 0.0, 0.0)
    joint_names: tuple[str, ...] = ()
    joint_positions: tuple[float, ...] = ()
    visuals: tuple[SceneVisual, ...] = ()
    selectable: bool = True
    draggable: bool = False
    metadata: FrozenMap = field(default_factory=FrozenMap)

    def __post_init__(self) -> None:
        if (
            not isinstance(self.path, EntityPath)
            or not isinstance(self.kind, EntityKind)
            or not isinstance(self.environment_index, int)
            or isinstance(self.environment_index, bool)
            or self.environment_index < 0
            or not isinstance(self.pose, Pose)
        ):
            raise _invalid("scene entity identity/pose is invalid")
        linear = _finite(self.linear_velocity_m_s, 3, "linear_velocity_m_s")
        angular = _finite(self.angular_velocity_rad_s, 3, "angular_velocity_rad_s")
        try:
            names = tuple(self.joint_names)
            positions = tuple(float(value) for value in self.joint_positions)
            visuals = tuple(self.visuals)
        except (TypeError, ValueError) as exc:
            raise _invalid("scene entity joints/visuals are invalid") from exc
        if (
            len(names) != len(positions)
            or len(names) != len(set(names))
            or any(not isinstance(name, str) or not name for name in names)
            or not all(math.isfinite(value) for value in positions)
            or any(not isinstance(visual, SceneVisual) for visual in visuals)
            or len({visual.visual_id for visual in visuals}) != len(visuals)
            or not isinstance(self.selectable, bool)
            or not isinstance(self.draggable, bool)
            or not isinstance(self.metadata, FrozenMap)
        ):
            raise _invalid("scene entity joints/visuals/flags are invalid")
        object.__setattr__(self, "linear_velocity_m_s", linear)
        object.__setattr__(self, "angular_velocity_rad_s", angular)
        object.__setattr__(self, "joint_names", names)
        object.__setattr__(self, "joint_positions", positions)
        object.__setattr__(self, "visuals", visuals)

    @property
    def key(self) -> tuple[str, int]:
        return (self.path.value, self.environment_index)

    def to_dict(self) -> dict[str, Any]:
        return {
            "path": self.path.value,
            "kind": self.kind.value,
            "environment_index": self.environment_index,
            "pose": _pose_dict(self.pose),
            "linear_velocity_m_s": list(self.linear_velocity_m_s),
            "angular_velocity_rad_s": list(self.angular_velocity_rad_s),
            "joint_names": list(self.joint_names),
            "joint_positions": list(self.joint_positions),
            "visuals": [visual.to_dict() for visual in self.visuals],
            "selectable": self.selectable,
            "draggable": self.draggable,
            "metadata": self.metadata.to_dict(),
        }


@dataclass(frozen=True)
class SceneSnapshot:
    provider_id: str
    world_id: str
    generation: int
    sequence: int
    tick: Tick
    entities: tuple[SceneEntityState, ...]
    schema_version: str = SCENE_SCHEMA_VERSION

    def __post_init__(self) -> None:
        try:
            entities = tuple(self.entities)
        except TypeError as exc:
            raise _invalid("scene snapshot entities must be iterable") from exc
        if (
            self.schema_version != SCENE_SCHEMA_VERSION
            or not isinstance(self.provider_id, str)
            or not self.provider_id
            or not isinstance(self.world_id, str)
            or not self.world_id
            or not isinstance(self.generation, int)
            or isinstance(self.generation, bool)
            or self.generation <= 0
            or not isinstance(self.sequence, int)
            or isinstance(self.sequence, bool)
            or self.sequence < 0
            or not isinstance(self.tick, Tick)
            or any(not isinstance(entity, SceneEntityState) for entity in entities)
            or len({entity.key for entity in entities}) != len(entities)
        ):
            raise _invalid("scene snapshot is invalid")
        object.__setattr__(self, "entities", tuple(sorted(entities, key=lambda item: item.key)))

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "provider_id": self.provider_id,
            "world_id": self.world_id,
            "generation": self.generation,
            "sequence": self.sequence,
            "tick": _tick_dict(self.tick),
            "entities": [entity.to_dict() for entity in self.entities],
        }


@dataclass(frozen=True)
class SceneDelta:
    world_id: str
    generation: int
    base_sequence: int
    sequence: int
    tick: Tick
    upserts: tuple[SceneEntityState, ...] = ()
    removals: tuple[tuple[EntityPath, int], ...] = ()
    schema_version: str = SCENE_SCHEMA_VERSION

    def __post_init__(self) -> None:
        try:
            upserts = tuple(self.upserts)
            removals = tuple(self.removals)
        except TypeError as exc:
            raise _invalid("scene delta collections must be iterable") from exc
        valid_removals = all(
            isinstance(path, EntityPath)
            and isinstance(environment, int)
            and not isinstance(environment, bool)
            and environment >= 0
            for path, environment in removals
        )
        if (
            self.schema_version != SCENE_SCHEMA_VERSION
            or not isinstance(self.world_id, str)
            or not self.world_id
            or not isinstance(self.generation, int)
            or isinstance(self.generation, bool)
            or self.generation <= 0
            or not isinstance(self.base_sequence, int)
            or isinstance(self.base_sequence, bool)
            or self.base_sequence < 0
            or not isinstance(self.sequence, int)
            or isinstance(self.sequence, bool)
            or self.sequence < self.base_sequence
            or not isinstance(self.tick, Tick)
            or any(not isinstance(entity, SceneEntityState) for entity in upserts)
            or not valid_removals
            or len({entity.key for entity in upserts}) != len(upserts)
            or len(set(removals)) != len(removals)
            or {entity.key for entity in upserts} & {(path.value, env) for path, env in removals}
            or self.sequence == self.base_sequence
            and bool(upserts or removals)
        ):
            raise _invalid("scene delta is invalid")
        object.__setattr__(self, "upserts", upserts)
        object.__setattr__(self, "removals", removals)

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "world_id": self.world_id,
            "generation": self.generation,
            "base_sequence": self.base_sequence,
            "sequence": self.sequence,
            "tick": _tick_dict(self.tick),
            "upserts": [entity.to_dict() for entity in self.upserts],
            "removals": [{"path": path.value, "environment_index": environment} for path, environment in self.removals],
        }


@dataclass(frozen=True)
class SceneCommand:
    command_id: str
    client_id: str
    lease_id: str
    expected_generation: int
    kind: SceneCommandKind
    entity_path: EntityPath
    environment_index: int = 0
    target_pose: Pose | None = None
    drag_id: str | None = None
    drag_mode: SceneDragMode | None = None
    grab_point_world_m: tuple[float, float, float] | None = None

    def __post_init__(self) -> None:
        identifiers = (self.command_id, self.client_id, self.lease_id)
        if any(not isinstance(value, str) or not _IDENTIFIER.fullmatch(value) for value in identifiers):
            raise _invalid("scene command identifiers are invalid")
        if (
            not isinstance(self.expected_generation, int)
            or isinstance(self.expected_generation, bool)
            or self.expected_generation <= 0
            or not isinstance(self.kind, SceneCommandKind)
            or not isinstance(self.entity_path, EntityPath)
            or not isinstance(self.environment_index, int)
            or isinstance(self.environment_index, bool)
            or self.environment_index < 0
        ):
            raise _invalid("scene command target is invalid")
        drag_kind = self.kind is not SceneCommandKind.SET_POSE
        if self.kind in {SceneCommandKind.SET_POSE, SceneCommandKind.DRAG_UPDATE} and not isinstance(
            self.target_pose, Pose
        ):
            raise _invalid("pose/set and drag-update commands require target_pose")
        if self.target_pose is not None and not isinstance(self.target_pose, Pose):
            raise _invalid("scene command target_pose is invalid")
        if drag_kind and (not isinstance(self.drag_id, str) or not _IDENTIFIER.fullmatch(self.drag_id)):
            raise _invalid("drag commands require a valid drag ID")
        if not drag_kind and any(
            value is not None for value in (self.drag_id, self.drag_mode, self.grab_point_world_m)
        ):
            raise _invalid("set_pose cannot contain drag fields")
        if self.kind is SceneCommandKind.DRAG_BEGIN:
            if not isinstance(self.drag_mode, SceneDragMode) or self.grab_point_world_m is None:
                raise _invalid("drag_begin requires mode and grab point")
        elif self.drag_mode is not None or self.grab_point_world_m is not None:
            raise _invalid("only drag_begin may contain mode/grab point")
        if self.grab_point_world_m is not None:
            object.__setattr__(
                self,
                "grab_point_world_m",
                _finite(self.grab_point_world_m, 3, "grab_point_world_m"),
            )


@dataclass(frozen=True)
class SceneCommandResult:
    command_id: str
    status: SceneCommandStatus
    generation: int
    scene_sequence: int
    tick: Tick
    error_code: str | None = None
    message: str | None = None

    def __post_init__(self) -> None:
        if (
            not isinstance(self.command_id, str)
            or not _IDENTIFIER.fullmatch(self.command_id)
            or not isinstance(self.status, SceneCommandStatus)
            or not isinstance(self.generation, int)
            or isinstance(self.generation, bool)
            or self.generation <= 0
            or not isinstance(self.scene_sequence, int)
            or isinstance(self.scene_sequence, bool)
            or self.scene_sequence < 0
            or not isinstance(self.tick, Tick)
        ):
            raise _invalid("scene command result is invalid")
        rejected = self.status is SceneCommandStatus.REJECTED
        if rejected != (self.error_code is not None):
            raise _invalid("only rejected results require an error code")
        for value in (self.error_code, self.message):
            if value is not None and (not isinstance(value, str) or not value):
                raise _invalid("scene command result text is invalid")

    def to_dict(self) -> dict[str, Any]:
        return {
            "command_id": self.command_id,
            "status": self.status.value,
            "generation": self.generation,
            "scene_sequence": self.scene_sequence,
            "tick": _tick_dict(self.tick),
            "error_code": self.error_code,
            "message": self.message,
        }


def _pose_dict(pose: Pose) -> dict[str, list[float]]:
    return {
        "position_m": list(pose.position),
        "orientation_xyzw": list(pose.orientation_xyzw),
    }


def _tick_dict(tick: Tick) -> dict[str, float | int]:
    return {"step_index": tick.step_index, "sim_time_seconds": tick.sim_time_seconds}
