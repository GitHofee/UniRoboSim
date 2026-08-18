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
from .values import (
    ArrayValue,
    CommandMode,
    DeformableTopology,
    EntityHandle,
    EntityKind,
    EntityPath,
    PointCommandMode,
    Pose,
)

LEGACY_WORLD_SCHEMA_VERSION = "unirobosim.world/v0alpha1"
SOFT_MATTER_WORLD_SCHEMA_VERSION = "unirobosim.world/v0alpha2"
WORLD_SCHEMA_VERSION = "unirobosim.world/v0alpha3"
SUPPORTED_WORLD_SCHEMA_VERSIONS = (
    LEGACY_WORLD_SCHEMA_VERSION,
    SOFT_MATTER_WORLD_SCHEMA_VERSION,
    WORLD_SCHEMA_VERSION,
)
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


def _validate_point_array(value: ArrayValue, name: str, *, first_dimension_minimum: int = 1) -> None:
    if (
        not isinstance(value, ArrayValue)
        or not value.dtype.startswith("float")
        or len(value.shape) != 2
        or value.shape[0] < first_dimension_minimum
        or value.shape[1] != 3
    ):
        raise _invalid(
            f"{name} must be a floating [point, xyz] array",
            "soft_entity_spec.validate",
            name=name,
        )


def _validate_topology_array(
    value: ArrayValue | None,
    name: str,
    *,
    width: int,
    point_count: int,
    required: bool,
) -> None:
    if value is None:
        if required:
            raise _invalid(f"{name} is required", "deformable_spec.validate")
        return
    if (
        not isinstance(value, ArrayValue)
        or not value.dtype.startswith("int")
        or len(value.shape) != 2
        or value.shape[1] != width
    ):
        raise _invalid(
            f"{name} must be an integer [element, {width}] array",
            "deformable_spec.validate",
        )
    for element in value.rows():
        indices = tuple(int(index) for index in element)
        if len(indices) != len(set(indices)) or any(index < 0 or index >= point_count for index in indices):
            raise _invalid(
                f"{name} contains a repeated or out-of-range point index",
                "deformable_spec.validate",
                element=list(indices),
                point_count=point_count,
            )


@dataclass(frozen=True)
class DeformableBodySpec:
    """Entity-local deformable authoring geometry and portable physical intent."""

    topology: DeformableTopology
    rest_positions_m: ArrayValue
    surface_triangles: ArrayValue | None = None
    tetrahedra: ArrayValue | None = None
    initial_node_velocities_m_s: ArrayValue | None = None
    kinematic_node_indices: tuple[int, ...] = ()
    node_mass_kg: float = 1.0
    linear_damping_per_s: float = 0.0
    self_collision: bool = False
    material_id: str | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.topology, DeformableTopology):
            raise _invalid("deformable topology is invalid", "deformable_spec.validate")
        minimum = 3 if self.topology is DeformableTopology.SURFACE else 4
        _validate_point_array(self.rest_positions_m, "rest_positions_m", first_dimension_minimum=minimum)
        point_count = self.rest_positions_m.shape[0]
        _validate_topology_array(
            self.surface_triangles,
            "surface_triangles",
            width=3,
            point_count=point_count,
            required=self.topology is DeformableTopology.SURFACE,
        )
        _validate_topology_array(
            self.tetrahedra,
            "tetrahedra",
            width=4,
            point_count=point_count,
            required=self.topology is DeformableTopology.VOLUME,
        )
        if self.topology is DeformableTopology.SURFACE and self.tetrahedra is not None:
            raise _invalid("surface deformables cannot contain tetrahedra", "deformable_spec.validate")
        if self.initial_node_velocities_m_s is not None:
            _validate_point_array(self.initial_node_velocities_m_s, "initial_node_velocities_m_s")
            if self.initial_node_velocities_m_s.shape != self.rest_positions_m.shape:
                raise _invalid(
                    "initial node velocity shape must match rest positions",
                    "deformable_spec.validate",
                )
        try:
            kinematic = tuple(self.kinematic_node_indices)
        except TypeError as exc:
            raise _invalid("kinematic node indices must be iterable", "deformable_spec.validate") from exc
        if any(
            not isinstance(index, int) or isinstance(index, bool) or index < 0 or index >= point_count
            for index in kinematic
        ) or len(kinematic) != len(set(kinematic)):
            raise _invalid("kinematic node indices must be unique and in range", "deformable_spec.validate")
        if isinstance(self.node_mass_kg, bool) or isinstance(self.linear_damping_per_s, bool):
            raise _invalid("deformable mass and damping must be numeric", "deformable_spec.validate")
        try:
            mass = float(self.node_mass_kg)
            damping = float(self.linear_damping_per_s)
        except (TypeError, ValueError) as exc:
            raise _invalid("deformable mass and damping must be numeric", "deformable_spec.validate") from exc
        if not math.isfinite(mass) or mass <= 0.0 or not math.isfinite(damping) or damping < 0.0:
            raise _invalid("deformable mass/damping is out of range", "deformable_spec.validate")
        if not isinstance(self.self_collision, bool):
            raise _invalid("self_collision must be boolean", "deformable_spec.validate")
        if self.material_id is not None and (not isinstance(self.material_id, str) or not self.material_id.strip()):
            raise _invalid("material ID must be a non-empty string", "deformable_spec.validate")
        object.__setattr__(self, "kinematic_node_indices", kinematic)
        object.__setattr__(self, "node_mass_kg", mass)
        object.__setattr__(self, "linear_damping_per_s", damping)

    @property
    def node_count(self) -> int:
        return self.rest_positions_m.shape[0]

    def initial_velocities(self) -> ArrayValue:
        if self.initial_node_velocities_m_s is not None:
            return self.initial_node_velocities_m_s
        return ArrayValue(shape=self.rest_positions_m.shape, values=(0.0,) * (self.node_count * 3))

    def to_dict(self) -> dict[str, Any]:
        result: dict[str, Any] = {
            "topology": self.topology.value,
            "rest_positions_m": self.rest_positions_m.nested(),
            "kinematic_node_indices": list(self.kinematic_node_indices),
            "node_mass_kg": self.node_mass_kg,
            "linear_damping_per_s": self.linear_damping_per_s,
            "self_collision": self.self_collision,
            "material_id": self.material_id,
        }
        if self.surface_triangles is not None:
            result["surface_triangles"] = self.surface_triangles.nested()
        if self.tetrahedra is not None:
            result["tetrahedra"] = self.tetrahedra.nested()
        if self.initial_node_velocities_m_s is not None:
            result["initial_node_velocities_m_s"] = self.initial_node_velocities_m_s.nested()
        return result


@dataclass(frozen=True)
class ParticleFluidSpec:
    """Entity-local fixed-count particle-fluid authoring state and physical intent."""

    initial_particle_positions_m: ArrayValue
    initial_particle_velocities_m_s: ArrayValue | None = None
    particle_radius_m: float = 0.01
    rest_density_kg_m3: float = 1000.0
    particle_mass_kg: float | None = None
    dynamic_viscosity_pa_s: float = 0.001
    surface_tension_n_m: float = 0.072
    material_id: str | None = None

    def __post_init__(self) -> None:
        _validate_point_array(self.initial_particle_positions_m, "initial_particle_positions_m")
        if self.initial_particle_velocities_m_s is not None:
            _validate_point_array(self.initial_particle_velocities_m_s, "initial_particle_velocities_m_s")
            if self.initial_particle_velocities_m_s.shape != self.initial_particle_positions_m.shape:
                raise _invalid(
                    "initial particle velocity shape must match particle positions",
                    "particle_fluid_spec.validate",
                )
        numeric_properties = (
            self.particle_radius_m,
            self.rest_density_kg_m3,
            self.dynamic_viscosity_pa_s,
            self.surface_tension_n_m,
        )
        if any(isinstance(value, bool) for value in numeric_properties) or isinstance(self.particle_mass_kg, bool):
            raise _invalid("particle fluid properties must be numeric", "particle_fluid_spec.validate")
        try:
            radius = float(self.particle_radius_m)
            density = float(self.rest_density_kg_m3)
            mass = None if self.particle_mass_kg is None else float(self.particle_mass_kg)
            viscosity = float(self.dynamic_viscosity_pa_s)
            tension = float(self.surface_tension_n_m)
        except (TypeError, ValueError) as exc:
            raise _invalid("particle fluid properties must be numeric", "particle_fluid_spec.validate") from exc
        if (
            not math.isfinite(radius)
            or radius <= 0.0
            or not math.isfinite(density)
            or density <= 0.0
            or mass is not None
            and (not math.isfinite(mass) or mass <= 0.0)
            or not math.isfinite(viscosity)
            or viscosity < 0.0
            or not math.isfinite(tension)
            or tension < 0.0
        ):
            raise _invalid("particle fluid properties are out of range", "particle_fluid_spec.validate")
        if self.material_id is not None and (not isinstance(self.material_id, str) or not self.material_id.strip()):
            raise _invalid("material ID must be a non-empty string", "particle_fluid_spec.validate")
        object.__setattr__(self, "particle_radius_m", radius)
        object.__setattr__(self, "rest_density_kg_m3", density)
        object.__setattr__(self, "particle_mass_kg", mass)
        object.__setattr__(self, "dynamic_viscosity_pa_s", viscosity)
        object.__setattr__(self, "surface_tension_n_m", tension)

    @property
    def particle_count(self) -> int:
        return self.initial_particle_positions_m.shape[0]

    @property
    def resolved_particle_mass_kg(self) -> float:
        if self.particle_mass_kg is not None:
            return self.particle_mass_kg
        return self.rest_density_kg_m3 * (4.0 / 3.0) * math.pi * self.particle_radius_m**3

    def initial_velocities(self) -> ArrayValue:
        if self.initial_particle_velocities_m_s is not None:
            return self.initial_particle_velocities_m_s
        return ArrayValue(
            shape=self.initial_particle_positions_m.shape,
            values=(0.0,) * (self.particle_count * 3),
        )

    def to_dict(self) -> dict[str, Any]:
        result: dict[str, Any] = {
            "initial_particle_positions_m": self.initial_particle_positions_m.nested(),
            "particle_radius_m": self.particle_radius_m,
            "rest_density_kg_m3": self.rest_density_kg_m3,
            "particle_mass_kg": self.particle_mass_kg,
            "dynamic_viscosity_pa_s": self.dynamic_viscosity_pa_s,
            "surface_tension_n_m": self.surface_tension_n_m,
            "material_id": self.material_id,
        }
        if self.initial_particle_velocities_m_s is not None:
            result["initial_particle_velocities_m_s"] = self.initial_particle_velocities_m_s.nested()
        return result


@dataclass(frozen=True)
class EntitySpec:
    path: EntityPath
    kind: EntityKind
    pose: Pose = field(default_factory=Pose)
    joint_names: tuple[str, ...] = ()
    initial_joint_positions: tuple[float, ...] = ()
    asset_uri: str | None = None
    metadata: FrozenMap = field(default_factory=FrozenMap)
    deformable: DeformableBodySpec | None = None
    particle_fluid: ParticleFluidSpec | None = None

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
        if self.kind is not EntityKind.ARTICULATION and (names or positions):
            raise _invalid("only articulations can declare joints", "entity_spec.validate", path=str(self.path))
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
        soft_kinds = {EntityKind.SURFACE_DEFORMABLE, EntityKind.VOLUME_DEFORMABLE}
        if self.kind in soft_kinds:
            if not isinstance(self.deformable, DeformableBodySpec) or self.particle_fluid is not None:
                raise _invalid(
                    "deformable entities require only a DeformableBodySpec",
                    "entity_spec.validate",
                    path=str(self.path),
                )
            expected_topology = (
                DeformableTopology.SURFACE if self.kind is EntityKind.SURFACE_DEFORMABLE else DeformableTopology.VOLUME
            )
            if self.deformable.topology is not expected_topology:
                raise _invalid(
                    "entity kind and deformable topology do not match",
                    "entity_spec.validate",
                    path=str(self.path),
                )
        elif self.kind is EntityKind.PARTICLE_FLUID:
            if not isinstance(self.particle_fluid, ParticleFluidSpec) or self.deformable is not None:
                raise _invalid(
                    "particle-fluid entities require only a ParticleFluidSpec",
                    "entity_spec.validate",
                    path=str(self.path),
                )
        elif self.deformable is not None or self.particle_fluid is not None:
            raise _invalid(
                "rigid/articulation entities cannot contain soft-matter specs",
                "entity_spec.validate",
                path=str(self.path),
            )
        if self.asset_uri is not None and (not isinstance(self.asset_uri, str) or not self.asset_uri.strip()):
            raise _invalid("asset URI must be a non-empty string", "entity_spec.validate", path=str(self.path))
        if not isinstance(self.metadata, FrozenMap):
            raise _invalid("entity metadata must be a FrozenMap", "entity_spec.validate", path=str(self.path))
        object.__setattr__(self, "joint_names", names)
        object.__setattr__(self, "initial_joint_positions", positions)

    def to_dict(self) -> dict[str, Any]:
        result: dict[str, Any] = {
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
        if self.deformable is not None:
            result["deformable"] = self.deformable.to_dict()
        if self.particle_fluid is not None:
            result["particle_fluid"] = self.particle_fluid.to_dict()
        return result


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
        if self.schema_version not in SUPPORTED_WORLD_SCHEMA_VERSIONS:
            raise _invalid(
                "unsupported world schema version",
                "world_spec.validate",
                expected=list(SUPPORTED_WORLD_SCHEMA_VERSIONS),
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
        soft_kinds = {
            EntityKind.SURFACE_DEFORMABLE,
            EntityKind.VOLUME_DEFORMABLE,
            EntityKind.PARTICLE_FLUID,
        }
        if self.schema_version == LEGACY_WORLD_SCHEMA_VERSION and any(entity.kind in soft_kinds for entity in entities):
            raise _invalid("v0alpha1 worlds cannot contain soft-matter entities", "world_spec.validate")
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
        kind_requirements = {
            EntityKind.SURFACE_DEFORMABLE: CapabilityId("state.deformable.surface@1"),
            EntityKind.VOLUME_DEFORMABLE: CapabilityId("state.deformable.volume@1"),
            EntityKind.PARTICLE_FLUID: CapabilityId("state.fluid.particles@1"),
        }
        if self.schema_version == WORLD_SCHEMA_VERSION:
            kind_requirements[EntityKind.RIGID_BODY] = CapabilityId("state.rigid_body@1")
        existing_ids = {item.capability for item in requirements}
        for entity in entities:
            capability = kind_requirements.get(entity.kind)
            if capability is not None and capability not in existing_ids:
                requirements += (CapabilityRequirement(capability),)
                existing_ids.add(capability)
            if (
                entity.deformable is not None
                and entity.deformable.self_collision
                and CapabilityId("physics.deformable.self-collision@1") not in existing_ids
            ):
                self_collision_capability = CapabilityId("physics.deformable.self-collision@1")
                requirements += (CapabilityRequirement(self_collision_capability),)
                existing_ids.add(self_collision_capability)
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


@dataclass(frozen=True)
class RigidBodyCommand:
    """Persistent world-frame force and free-torque command for rigid bodies."""

    handle: EntityHandle
    forces_n: ArrayValue
    torques_n_m: ArrayValue
    environment_indices: tuple[int, ...] | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.handle, EntityHandle) or self.handle.entity_kind is not EntityKind.RIGID_BODY:
            raise _invalid("rigid-body command requires a rigid-body handle", "rigid_body_command.validate")
        for name, value in (("forces_n", self.forces_n), ("torques_n_m", self.torques_n_m)):
            if (
                not isinstance(value, ArrayValue)
                or not value.dtype.startswith("float")
                or len(value.shape) != 2
                or value.shape[1] != 3
            ):
                raise _invalid(
                    f"{name} must be a floating [environment, xyz] array",
                    "rigid_body_command.validate",
                )
        if self.forces_n.shape != self.torques_n_m.shape:
            raise _invalid("rigid-body force and torque shapes must match", "rigid_body_command.validate")
        if self.environment_indices is not None:
            try:
                indices = tuple(self.environment_indices)
            except TypeError as exc:
                raise _invalid("environment_indices must be iterable", "rigid_body_command.validate") from exc
            if (
                not indices
                or any(not isinstance(index, int) or isinstance(index, bool) or index < 0 for index in indices)
                or len(indices) != len(set(indices))
            ):
                raise _invalid(
                    "environment_indices must contain unique non-negative integers",
                    "rigid_body_command.validate",
                )
            object.__setattr__(self, "environment_indices", indices)


def _validate_point_command(
    handle: EntityHandle,
    accepted_kinds: set[EntityKind],
    mode: PointCommandMode,
    targets: ArrayValue,
    environment_indices: tuple[int, ...] | None,
    point_indices: tuple[int, ...] | None,
    operation: str,
) -> tuple[tuple[int, ...] | None, tuple[int, ...] | None]:
    if not isinstance(handle, EntityHandle) or handle.entity_kind not in accepted_kinds:
        raise _invalid("point command handle has the wrong entity kind", operation)
    if not isinstance(mode, PointCommandMode) or not isinstance(targets, ArrayValue):
        raise _invalid("point command mode/targets use invalid types", operation)
    if not targets.dtype.startswith("float") or len(targets.shape) != 3 or targets.shape[2] != 3:
        raise _invalid("point command targets must be floating [environment, point, xyz]", operation)
    normalized: list[tuple[int, ...] | None] = []
    for field_name, value in (
        ("environment_indices", environment_indices),
        ("point_indices", point_indices),
    ):
        if value is None:
            normalized.append(None)
            continue
        try:
            indices = tuple(value)
        except TypeError as exc:
            raise _invalid(f"{field_name} must be iterable", operation) from exc
        if (
            not indices
            or any(not isinstance(index, int) or isinstance(index, bool) or index < 0 for index in indices)
            or len(indices) != len(set(indices))
        ):
            raise _invalid(f"{field_name} must contain unique non-negative integers", operation)
        normalized.append(indices)
    return normalized[0], normalized[1]


@dataclass(frozen=True)
class DeformableCommand:
    """Strict world-frame command for selected deformable nodes and environments."""

    handle: EntityHandle
    mode: PointCommandMode
    targets: ArrayValue
    environment_indices: tuple[int, ...] | None = None
    node_indices: tuple[int, ...] | None = None

    def __post_init__(self) -> None:
        environments, points = _validate_point_command(
            self.handle,
            {EntityKind.SURFACE_DEFORMABLE, EntityKind.VOLUME_DEFORMABLE},
            self.mode,
            self.targets,
            self.environment_indices,
            self.node_indices,
            "deformable_command.validate",
        )
        object.__setattr__(self, "environment_indices", environments)
        object.__setattr__(self, "node_indices", points)


@dataclass(frozen=True)
class ParticleFluidCommand:
    """Strict world-frame command for selected fluid particles and environments."""

    handle: EntityHandle
    mode: PointCommandMode
    targets: ArrayValue
    environment_indices: tuple[int, ...] | None = None
    particle_indices: tuple[int, ...] | None = None

    def __post_init__(self) -> None:
        environments, points = _validate_point_command(
            self.handle,
            {EntityKind.PARTICLE_FLUID},
            self.mode,
            self.targets,
            self.environment_indices,
            self.particle_indices,
            "particle_fluid_command.validate",
        )
        object.__setattr__(self, "environment_indices", environments)
        object.__setattr__(self, "particle_indices", points)
