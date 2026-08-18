"""Immutable world build and command specifications."""

from __future__ import annotations

import hashlib
import json
import math
import re
from dataclasses import dataclass, field
from typing import Any

from .capabilities import CapabilityId, CapabilityRequirement
from .errors import ValidationError
from .frozen import FrozenMap
from .values import ArrayValue, CommandMode, EntityHandle, EntityKind, EntityPath, Pose

WORLD_SCHEMA_VERSION = "unirobosim.world/v0alpha1"
_WORLD_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]*$")


def _invalid(message: str, operation: str, **details: object) -> ValidationError:
    return ValidationError(message, operation=operation, details=details)


@dataclass(frozen=True)
class PhysicsSpec:
    time_step_seconds: float = 1.0 / 60.0
    substeps: int = 1
    gravity_m_s2: tuple[float, float, float] = (0.0, 0.0, -9.81)

    def __post_init__(self) -> None:
        try:
            time_step = float(self.time_step_seconds)
            gravity = tuple(float(value) for value in self.gravity_m_s2)
        except (TypeError, ValueError) as exc:
            raise _invalid("physics values must be numeric", "physics_spec.validate") from exc
        if not math.isfinite(time_step) or time_step <= 0.0:
            raise _invalid("physics time step must be positive and finite", "physics_spec.validate")
        if not isinstance(self.substeps, int) or isinstance(self.substeps, bool) or self.substeps <= 0:
            raise _invalid("physics substeps must be a positive integer", "physics_spec.validate")
        if len(gravity) != 3 or not all(math.isfinite(value) for value in gravity):
            raise _invalid("gravity must contain three finite values", "physics_spec.validate")
        object.__setattr__(self, "time_step_seconds", time_step)
        object.__setattr__(self, "gravity_m_s2", gravity)


@dataclass(frozen=True)
class EnvironmentSpec:
    count: int = 1

    def __post_init__(self) -> None:
        if not isinstance(self.count, int) or isinstance(self.count, bool) or self.count <= 0:
            raise _invalid("environment count must be a positive integer", "environment_spec.validate")


@dataclass(frozen=True)
class EntitySpec:
    path: EntityPath
    kind: EntityKind
    pose: Pose = field(default_factory=Pose)
    joint_names: tuple[str, ...] = ()
    initial_joint_positions: tuple[float, ...] = ()
    asset_uri: str | None = None
    metadata: FrozenMap = field(default_factory=FrozenMap)

    def __post_init__(self) -> None:
        if not isinstance(self.path, EntityPath) or not isinstance(self.kind, EntityKind):
            raise _invalid("entity path and kind must use canonical value types", "entity_spec.validate")
        if not isinstance(self.pose, Pose):
            raise _invalid("entity pose must be a Pose", "entity_spec.validate", path=str(self.path))
        try:
            names = tuple(self.joint_names)
            positions = tuple(float(value) for value in self.initial_joint_positions)
        except (TypeError, ValueError) as exc:
            raise _invalid("joint names and positions must be valid iterables", "entity_spec.validate") from exc
        if any(not isinstance(name, str) or not name.strip() for name in names) or len(names) != len(set(names)):
            raise _invalid("joint names must be unique non-empty strings", "entity_spec.validate", path=str(self.path))
        if not all(math.isfinite(value) for value in positions):
            raise _invalid("initial joint positions must be finite", "entity_spec.validate", path=str(self.path))
        if self.kind is EntityKind.RIGID_BODY and (names or positions):
            raise _invalid("rigid bodies cannot declare joints", "entity_spec.validate", path=str(self.path))
        if self.kind is EntityKind.ARTICULATION:
            if not names:
                raise _invalid(
                    "articulations must declare at least one joint", "entity_spec.validate", path=str(self.path)
                )
            if not positions:
                positions = (0.0,) * len(names)
            if len(positions) != len(names):
                raise _invalid(
                    "initial position count must equal joint count",
                    "entity_spec.validate",
                    path=str(self.path),
                    joints=len(names),
                    positions=len(positions),
                )
        if self.asset_uri is not None and (not isinstance(self.asset_uri, str) or not self.asset_uri.strip()):
            raise _invalid("asset URI must be a non-empty string", "entity_spec.validate", path=str(self.path))
        if not isinstance(self.metadata, FrozenMap):
            raise _invalid("entity metadata must be a FrozenMap", "entity_spec.validate", path=str(self.path))
        object.__setattr__(self, "joint_names", names)
        object.__setattr__(self, "initial_joint_positions", positions)

    def to_dict(self) -> dict[str, Any]:
        return {
            "path": self.path.value,
            "kind": self.kind.value,
            "pose": {
                "position_m": list(self.pose.position),
                "orientation_xyzw": list(self.pose.orientation_xyzw),
            },
            "joint_names": list(self.joint_names),
            "initial_joint_positions_rad": list(self.initial_joint_positions),
            "asset_uri": self.asset_uri,
            "metadata": self.metadata.to_dict(),
        }


def _default_requirements() -> tuple[CapabilityRequirement, ...]:
    return (CapabilityRequirement(CapabilityId("profile.core-robotics@1")),)


@dataclass(frozen=True)
class WorldSpec:
    world_id: str
    entities: tuple[EntitySpec, ...]
    physics: PhysicsSpec = field(default_factory=PhysicsSpec)
    environments: EnvironmentSpec = field(default_factory=EnvironmentSpec)
    requirements: tuple[CapabilityRequirement, ...] = field(default_factory=_default_requirements)
    metadata: FrozenMap = field(default_factory=FrozenMap)
    schema_version: str = WORLD_SCHEMA_VERSION

    def __post_init__(self) -> None:
        if not isinstance(self.world_id, str) or not _WORLD_ID.fullmatch(self.world_id):
            raise _invalid("world ID is invalid", "world_spec.validate", world_id=self.world_id)
        if self.schema_version != WORLD_SCHEMA_VERSION:
            raise _invalid(
                "unsupported world schema version",
                "world_spec.validate",
                expected=WORLD_SCHEMA_VERSION,
                actual=self.schema_version,
            )
        if not isinstance(self.physics, PhysicsSpec) or not isinstance(self.environments, EnvironmentSpec):
            raise _invalid("world physics/environments use invalid types", "world_spec.validate")
        try:
            raw_entities = tuple(self.entities)
            raw_requirements = tuple(self.requirements)
        except TypeError as exc:
            raise _invalid("world entities and requirements must be iterable", "world_spec.validate") from exc
        if not raw_entities or any(not isinstance(item, EntitySpec) for item in raw_entities):
            raise _invalid("world must contain EntitySpec values", "world_spec.validate")
        entities = tuple(sorted(raw_entities, key=lambda item: item.path.value))
        paths = tuple(item.path for item in entities)
        if len(paths) != len(set(paths)):
            raise _invalid("world entity paths must be unique", "world_spec.validate")
        if any(not isinstance(item, CapabilityRequirement) for item in raw_requirements):
            raise _invalid("world requirements contain an invalid value", "world_spec.validate")
        requirements = tuple(raw_requirements)
        core_id = CapabilityId("profile.core-robotics@1")
        core_requirement = next((item for item in requirements if item.capability == core_id), None)
        if core_requirement is not None and not core_requirement.required:
            raise _invalid("the core robotics profile cannot be optional", "world_spec.validate")
        if core_requirement is None:
            requirements += (CapabilityRequirement(core_id),)
        requirements = tuple(sorted(requirements, key=lambda item: item.capability.value))
        ids = tuple(item.capability for item in requirements)
        if len(ids) != len(set(ids)):
            raise _invalid("world capability requirements must be unique", "world_spec.validate")
        if not isinstance(self.metadata, FrozenMap):
            raise _invalid("world metadata must be a FrozenMap", "world_spec.validate")
        object.__setattr__(self, "entities", entities)
        object.__setattr__(self, "requirements", requirements)

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "world_id": self.world_id,
            "physics": {
                "time_step_seconds": self.physics.time_step_seconds,
                "substeps": self.physics.substeps,
                "gravity_m_s2": list(self.physics.gravity_m_s2),
            },
            "environments": {"count": self.environments.count},
            "entities": [entity.to_dict() for entity in self.entities],
            "requirements": [requirement.to_dict() for requirement in self.requirements],
            "metadata": self.metadata.to_dict(),
        }

    @property
    def canonical_json(self) -> str:
        return json.dumps(self.to_dict(), sort_keys=True, separators=(",", ":"), ensure_ascii=False)

    @property
    def digest(self) -> str:
        return hashlib.sha256(self.canonical_json.encode("utf-8")).hexdigest()


@dataclass(frozen=True)
class ArticulationCommand:
    handle: EntityHandle
    mode: CommandMode
    targets: ArrayValue
    environment_indices: tuple[int, ...] | None = None
    degree_of_freedom_indices: tuple[int, ...] | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.handle, EntityHandle) or self.handle.entity_kind is not EntityKind.ARTICULATION:
            raise _invalid("articulation command requires an articulation handle", "command.validate")
        if not isinstance(self.mode, CommandMode) or not isinstance(self.targets, ArrayValue):
            raise _invalid("articulation command mode/targets use invalid types", "command.validate")
        if len(self.targets.shape) != 2 or not self.targets.dtype.startswith("float"):
            raise _invalid("articulation targets must be a rank-2 floating array", "command.validate")
        for field_name in ("environment_indices", "degree_of_freedom_indices"):
            value = getattr(self, field_name)
            if value is not None:
                try:
                    indices = tuple(value)
                except TypeError as exc:
                    raise _invalid(f"{field_name} must be iterable", "command.validate") from exc
                if not indices or any(
                    not isinstance(index, int) or isinstance(index, bool) or index < 0 for index in indices
                ):
                    raise _invalid(f"{field_name} must contain non-negative integers", "command.validate")
                if len(indices) != len(set(indices)):
                    raise _invalid(f"{field_name} must be unique", "command.validate")
                object.__setattr__(self, field_name, indices)
