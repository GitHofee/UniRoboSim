"""Portable runtime reports and state snapshots."""

from __future__ import annotations

import math
import re
from dataclasses import dataclass, field

from .capabilities import CapabilitySet
from .errors import ValidationError
from .frozen import FrozenMap
from .values import ArrayValue, Tick

_PROVIDER_ID = re.compile(r"^[a-z][a-z0-9]*(?:[._-][a-z0-9]+)*$")
_SHA256 = re.compile(r"^[0-9a-f]{64}$")


@dataclass(frozen=True)
class ProviderDescriptor:
    provider_id: str
    display_name: str
    version: str
    contract_version: str
    capabilities: CapabilitySet
    metadata: FrozenMap = field(default_factory=FrozenMap)

    def __post_init__(self) -> None:
        if not isinstance(self.provider_id, str) or not _PROVIDER_ID.fullmatch(self.provider_id):
            raise ValidationError("invalid provider ID", operation="provider_descriptor.validate")
        if any(
            not isinstance(value, str) or not value.strip()
            for value in (self.display_name, self.version, self.contract_version)
        ):
            raise ValidationError(
                "provider descriptor strings must be non-empty", operation="provider_descriptor.validate"
            )
        if not isinstance(self.capabilities, CapabilitySet):
            raise ValidationError(
                "provider capabilities must be a CapabilitySet", operation="provider_descriptor.validate"
            )
        if not isinstance(self.metadata, FrozenMap):
            raise ValidationError("provider metadata must be a FrozenMap", operation="provider_descriptor.validate")


@dataclass(frozen=True)
class ProbeReport:
    descriptor: ProviderDescriptor
    available: bool
    reason: str | None = None
    details: FrozenMap = field(default_factory=FrozenMap)

    def __post_init__(self) -> None:
        if not isinstance(self.descriptor, ProviderDescriptor) or not isinstance(self.available, bool):
            raise ValidationError("probe descriptor/availability is invalid", operation="probe_report.validate")
        if self.reason is not None and (not isinstance(self.reason, str) or not self.reason.strip()):
            raise ValidationError("probe reason must be a non-empty string", operation="probe_report.validate")
        if not isinstance(self.details, FrozenMap):
            raise ValidationError("probe details must be a FrozenMap", operation="probe_report.validate")


@dataclass(frozen=True)
class BuildFingerprint:
    provider_id: str
    provider_version: str
    contract_version: str
    world_digest: str
    capability_digest: str

    def __post_init__(self) -> None:
        identity = (self.provider_id, self.provider_version, self.contract_version)
        if any(not isinstance(value, str) or not value.strip() for value in identity):
            raise ValidationError("fingerprint identity must be non-empty", operation="build_fingerprint.validate")
        if not isinstance(self.world_digest, str) or not _SHA256.fullmatch(self.world_digest):
            raise ValidationError("world digest must be lowercase SHA-256", operation="build_fingerprint.validate")
        if not isinstance(self.capability_digest, str) or not _SHA256.fullmatch(self.capability_digest):
            raise ValidationError("capability digest must be lowercase SHA-256", operation="build_fingerprint.validate")

    def to_dict(self) -> dict[str, str]:
        return {
            "provider_id": self.provider_id,
            "provider_version": self.provider_version,
            "contract_version": self.contract_version,
            "world_digest": self.world_digest,
            "capability_digest": self.capability_digest,
        }


@dataclass(frozen=True)
class BuildReport:
    fingerprint: BuildFingerprint
    world_id: str
    generation: int
    environment_count: int
    entity_count: int

    def __post_init__(self) -> None:
        if (
            not isinstance(self.fingerprint, BuildFingerprint)
            or not isinstance(self.world_id, str)
            or not self.world_id
        ):
            raise ValidationError("build report identity is invalid", operation="build_report.validate")
        counts = (self.generation, self.environment_count, self.entity_count)
        if any(not isinstance(value, int) or isinstance(value, bool) for value in counts):
            raise ValidationError("build report counts must be integers", operation="build_report.validate")
        if self.generation <= 0 or self.environment_count <= 0 or self.entity_count < 0:
            raise ValidationError("build report counts are out of range", operation="build_report.validate")


@dataclass(frozen=True)
class ResetResult:
    environment_indices: tuple[int, ...]
    reset_count: int
    tick: Tick

    def __post_init__(self) -> None:
        indices = tuple(self.environment_indices)
        if (
            not indices
            or any(not isinstance(index, int) or isinstance(index, bool) or index < 0 for index in indices)
            or len(indices) != len(set(indices))
            or not isinstance(self.reset_count, int)
            or isinstance(self.reset_count, bool)
            or self.reset_count <= 0
            or not isinstance(self.tick, Tick)
        ):
            raise ValidationError("reset result is invalid", operation="reset_result.validate")
        object.__setattr__(self, "environment_indices", indices)


@dataclass(frozen=True)
class ArticulationState:
    joint_positions: ArrayValue
    joint_velocities: ArrayValue
    tick: Tick

    def __post_init__(self) -> None:
        if (
            not isinstance(self.joint_positions, ArrayValue)
            or not isinstance(self.joint_velocities, ArrayValue)
            or not isinstance(self.tick, Tick)
        ):
            raise ValidationError("articulation state values are invalid", operation="articulation_state.validate")
        if self.joint_positions.shape != self.joint_velocities.shape:
            raise ValidationError("articulation state shapes must match", operation="articulation_state.validate")


def _validate_environment_vectors(value: ArrayValue, width: int, name: str, operation: str) -> None:
    if (
        not isinstance(value, ArrayValue)
        or not value.dtype.startswith("float")
        or len(value.shape) != 2
        or value.shape[1] != width
    ):
        raise ValidationError(f"{name} must be a floating [environment, {width}] array", operation=operation)


@dataclass(frozen=True)
class RigidBodyState:
    """Root-link pose and twist in the environment-local world frame."""

    positions_m: ArrayValue
    orientations_xyzw: ArrayValue
    linear_velocities_m_s: ArrayValue
    angular_velocities_rad_s: ArrayValue
    tick: Tick

    def __post_init__(self) -> None:
        operation = "rigid_body_state.validate"
        _validate_environment_vectors(self.positions_m, 3, "positions_m", operation)
        _validate_environment_vectors(self.orientations_xyzw, 4, "orientations_xyzw", operation)
        _validate_environment_vectors(self.linear_velocities_m_s, 3, "linear_velocities_m_s", operation)
        _validate_environment_vectors(self.angular_velocities_rad_s, 3, "angular_velocities_rad_s", operation)
        environment_count = self.positions_m.shape[0]
        if (
            self.orientations_xyzw.shape[0] != environment_count
            or self.linear_velocities_m_s.shape[0] != environment_count
            or self.angular_velocities_rad_s.shape[0] != environment_count
            or not isinstance(self.tick, Tick)
        ):
            raise ValidationError("rigid-body state batches/tick must match", operation=operation)
        for orientation in self.orientations_xyzw.rows():
            norm = math.sqrt(sum(float(value) ** 2 for value in orientation))
            if not math.isclose(norm, 1.0, rel_tol=0.0, abs_tol=1.0e-5):
                raise ValidationError("rigid-body orientations must be unit XYZW quaternions", operation=operation)


@dataclass(frozen=True)
class ContactState:
    """Aggregated normal-contact state for one rigid body."""

    net_normal_forces_n: ArrayValue
    in_contact: ArrayValue
    tick: Tick

    def __post_init__(self) -> None:
        operation = "contact_state.validate"
        _validate_environment_vectors(self.net_normal_forces_n, 3, "net_normal_forces_n", operation)
        if (
            not isinstance(self.in_contact, ArrayValue)
            or self.in_contact.dtype != "bool"
            or len(self.in_contact.shape) != 1
            or self.in_contact.shape[0] != self.net_normal_forces_n.shape[0]
            or not isinstance(self.tick, Tick)
        ):
            raise ValidationError(
                "contact state must use boolean [environment] flags matching force batches",
                operation=operation,
            )


def _validate_point_state(positions: ArrayValue, velocities: ArrayValue, tick: Tick, operation: str) -> None:
    if not isinstance(positions, ArrayValue) or not isinstance(velocities, ArrayValue) or not isinstance(tick, Tick):
        raise ValidationError("point state values are invalid", operation=operation)
    if (
        positions.shape != velocities.shape
        or len(positions.shape) != 3
        or positions.shape[2] != 3
        or not positions.dtype.startswith("float")
        or not velocities.dtype.startswith("float")
    ):
        raise ValidationError(
            "point state must use matching floating [environment, point, xyz] arrays", operation=operation
        )


@dataclass(frozen=True)
class DeformableState:
    """World-frame batched deformable-node state."""

    node_positions_m: ArrayValue
    node_velocities_m_s: ArrayValue
    tick: Tick

    def __post_init__(self) -> None:
        _validate_point_state(
            self.node_positions_m,
            self.node_velocities_m_s,
            self.tick,
            "deformable_state.validate",
        )


@dataclass(frozen=True)
class ParticleFluidState:
    """World-frame batched particle-fluid state."""

    particle_positions_m: ArrayValue
    particle_velocities_m_s: ArrayValue
    tick: Tick

    def __post_init__(self) -> None:
        _validate_point_state(
            self.particle_positions_m,
            self.particle_velocities_m_s,
            self.tick,
            "particle_fluid_state.validate",
        )
