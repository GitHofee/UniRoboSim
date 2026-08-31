"""Capability-gated state injection for render-only replay."""

from __future__ import annotations

import math
from dataclasses import dataclass
from functools import reduce
from operator import mul
from typing import Protocol, runtime_checkable

from .capabilities import CapabilityId
from .values import ArrayValue, EntityHandle, EntityKind, Tick

RENDER_STATE_CAPABILITY_ID = CapabilityId("render.state.apply@1")


@dataclass(frozen=True, slots=True)
class PackedFloat32Array:
    """Immutable contiguous little-endian float32 storage for high-volume state."""

    shape: tuple[int, ...]
    data: bytes

    def __post_init__(self) -> None:
        try:
            shape = tuple(self.shape)
        except TypeError as exc:
            raise TypeError("packed float32 shape must be iterable") from exc
        if not shape or any(type(size) is not int or size <= 0 for size in shape):
            raise ValueError("packed float32 shape must contain positive integers")
        if type(self.data) is not bytes:
            raise TypeError("packed float32 data must be immutable bytes")
        expected = reduce(mul, shape, 1) * 4
        if len(self.data) != expected:
            raise ValueError("packed float32 byte length does not match its shape")
        object.__setattr__(self, "shape", shape)

    @property
    def dtype(self) -> str:
        return "float32-le"

    @property
    def nbytes(self) -> int:
        return len(self.data)


def _selection(value: tuple[int, ...] | None, field: str) -> tuple[int, ...] | None:
    if value is None:
        return None
    try:
        result = tuple(value)
    except TypeError as exc:
        raise ValueError(f"{field} must be iterable") from exc
    if (
        not result
        or any(type(index) is not int or index < 0 for index in result)
        or len(result) != len(set(result))
    ):
        raise ValueError(f"{field} must contain unique non-negative integers")
    return result


def _float_array(value: object, field: str, rank: int, width: int | None = None) -> ArrayValue:
    if (
        not isinstance(value, ArrayValue)
        or not value.dtype.startswith("float")
        or len(value.shape) != rank
        or (width is not None and value.shape[-1] != width)
    ):
        suffix = "" if width is None else f" with final width {width}"
        raise TypeError(f"{field} must be a floating rank-{rank} array{suffix}")
    return value


def _particle_array(value: object, field: str) -> ArrayValue | PackedFloat32Array:
    if isinstance(value, PackedFloat32Array):
        if len(value.shape) != 3 or value.shape[-1] != 3:
            raise TypeError(f"{field} must be a packed rank-3 xyz array")
        return value
    return _float_array(value, field, 3, 3)


@dataclass(frozen=True, slots=True)
class RenderArticulationState:
    """Selected articulation joint state in declared joint order and SI units."""

    handle: EntityHandle
    joint_positions: ArrayValue
    joint_velocities: ArrayValue
    environment_indices: tuple[int, ...] | None = None
    degree_of_freedom_indices: tuple[int, ...] | None = None
    root_positions_m: ArrayValue | None = None
    root_orientations_xyzw: ArrayValue | None = None
    root_linear_velocities_m_s: ArrayValue | None = None
    root_angular_velocities_rad_s: ArrayValue | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.handle, EntityHandle) or self.handle.entity_kind is not EntityKind.ARTICULATION:
            raise TypeError("render articulation state requires an articulation handle")
        positions = _float_array(self.joint_positions, "joint_positions", 2)
        velocities = _float_array(self.joint_velocities, "joint_velocities", 2)
        if positions.shape != velocities.shape:
            raise ValueError("joint position and velocity shapes must match")
        environments = _selection(self.environment_indices, "environment_indices")
        degrees = _selection(self.degree_of_freedom_indices, "degree_of_freedom_indices")
        if environments is not None and len(environments) != positions.shape[0]:
            raise ValueError("environment_indices must match the articulation state rows")
        if degrees is not None and len(degrees) != positions.shape[1]:
            raise ValueError("degree_of_freedom_indices must match the articulation state columns")
        root_values = (
            self.root_positions_m,
            self.root_orientations_xyzw,
            self.root_linear_velocities_m_s,
            self.root_angular_velocities_rad_s,
        )
        if any(value is not None for value in root_values):
            if any(value is None for value in root_values):
                raise ValueError("articulation root pose and velocity arrays must be supplied together")
            assert self.root_positions_m is not None
            assert self.root_orientations_xyzw is not None
            assert self.root_linear_velocities_m_s is not None
            assert self.root_angular_velocities_rad_s is not None
            root_positions = _float_array(self.root_positions_m, "root_positions_m", 2, 3)
            root_orientations = _float_array(self.root_orientations_xyzw, "root_orientations_xyzw", 2, 4)
            root_linear = _float_array(self.root_linear_velocities_m_s, "root_linear_velocities_m_s", 2, 3)
            root_angular = _float_array(self.root_angular_velocities_rad_s, "root_angular_velocities_rad_s", 2, 3)
            row_count = positions.shape[0]
            if any(
                array.shape[0] != row_count
                for array in (root_positions, root_orientations, root_linear, root_angular)
            ):
                raise ValueError("articulation root arrays must match the joint-state environment rows")
            for row in root_orientations.rows():
                norm = math.sqrt(sum(float(value) * float(value) for value in row))
                if not math.isclose(norm, 1.0, rel_tol=0.0, abs_tol=1.0e-6):
                    raise ValueError("root_orientations_xyzw rows must be unit quaternions")
        object.__setattr__(self, "environment_indices", environments)
        object.__setattr__(self, "degree_of_freedom_indices", degrees)


@dataclass(frozen=True, slots=True)
class RenderRigidBodyState:
    """Selected dynamic rigid-body root state in environment-local world coordinates."""

    handle: EntityHandle
    positions_m: ArrayValue
    orientations_xyzw: ArrayValue
    linear_velocities_m_s: ArrayValue
    angular_velocities_rad_s: ArrayValue
    environment_indices: tuple[int, ...] | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.handle, EntityHandle) or self.handle.entity_kind is not EntityKind.RIGID_BODY:
            raise TypeError("render rigid-body state requires a rigid-body handle")
        positions = _float_array(self.positions_m, "positions_m", 2, 3)
        orientations = _float_array(self.orientations_xyzw, "orientations_xyzw", 2, 4)
        linear = _float_array(self.linear_velocities_m_s, "linear_velocities_m_s", 2, 3)
        angular = _float_array(self.angular_velocities_rad_s, "angular_velocities_rad_s", 2, 3)
        row_count = positions.shape[0]
        if any(array.shape[0] != row_count for array in (orientations, linear, angular)):
            raise ValueError("rigid-body state arrays must have the same environment row count")
        for row in orientations.rows():
            norm = math.sqrt(sum(float(value) * float(value) for value in row))
            if not math.isclose(norm, 1.0, rel_tol=0.0, abs_tol=1.0e-6):
                raise ValueError("orientations_xyzw rows must be unit quaternions")
        environments = _selection(self.environment_indices, "environment_indices")
        if environments is not None and len(environments) != row_count:
            raise ValueError("environment_indices must match the rigid-body state rows")
        object.__setattr__(self, "environment_indices", environments)


@dataclass(frozen=True, slots=True)
class RenderParticleFluidState:
    """One contiguous active particle range in environment-local world coordinates."""

    handle: EntityHandle
    positions_m: ArrayValue | PackedFloat32Array
    velocities_m_s: ArrayValue | PackedFloat32Array | None = None
    environment_indices: tuple[int, ...] | None = None
    first_particle_index: int = 0

    def __post_init__(self) -> None:
        if not isinstance(self.handle, EntityHandle) or self.handle.entity_kind is not EntityKind.PARTICLE_FLUID:
            raise TypeError("render particle-fluid state requires a particle-fluid handle")
        positions = _particle_array(self.positions_m, "positions_m")
        if self.velocities_m_s is not None:
            velocities = _particle_array(self.velocities_m_s, "velocities_m_s")
            if velocities.shape != positions.shape:
                raise ValueError("particle position and velocity shapes must match")
        environments = _selection(self.environment_indices, "environment_indices")
        if environments is not None and len(environments) != positions.shape[0]:
            raise ValueError("environment_indices must match the particle state rows")
        if type(self.first_particle_index) is not int or self.first_particle_index < 0:
            raise ValueError("first_particle_index must be a non-negative integer")
        object.__setattr__(self, "environment_indices", environments)


@dataclass(frozen=True, slots=True)
class RenderStateFrame:
    """One all-or-nothing render state update that never advances physics."""

    articulations: tuple[RenderArticulationState, ...] = ()
    rigid_bodies: tuple[RenderRigidBodyState, ...] = ()
    particle_fluids: tuple[RenderParticleFluidState, ...] = ()

    def __post_init__(self) -> None:
        try:
            articulations = tuple(self.articulations)
            rigid_bodies = tuple(self.rigid_bodies)
            particle_fluids = tuple(self.particle_fluids)
        except TypeError as exc:
            raise TypeError("render state frame groups must be iterable") from exc
        if any(type(value) is not RenderArticulationState for value in articulations):
            raise TypeError("articulations contains an invalid render state entry")
        if any(type(value) is not RenderRigidBodyState for value in rigid_bodies):
            raise TypeError("rigid_bodies contains an invalid render state entry")
        if any(type(value) is not RenderParticleFluidState for value in particle_fluids):
            raise TypeError("particle_fluids contains an invalid render state entry")
        object.__setattr__(self, "articulations", articulations)
        object.__setattr__(self, "rigid_bodies", rigid_bodies)
        object.__setattr__(self, "particle_fluids", particle_fluids)

        seen: set[tuple[str, str, int, str]] = set()
        for handle in (
            *(value.handle for value in articulations),
            *(value.handle for value in rigid_bodies),
            *(value.handle for value in particle_fluids),
        ):
            identity = (handle.provider_id, handle.world_id, handle.generation, handle.path.value)
            if identity in seen:
                raise ValueError("a render state frame may update each entity at most once")
            seen.add(identity)
        if not seen:
            raise ValueError("a render state frame must contain at least one entity update")


@dataclass(frozen=True, slots=True)
class RenderStateResult:
    """Identity of one committed render-state update at an unchanged physics tick."""

    generation: int
    tick: Tick
    state_revision: int
    articulation_count: int
    rigid_body_count: int
    particle_fluid_count: int

    def __post_init__(self) -> None:
        if type(self.generation) is not int or self.generation <= 0:
            raise ValueError("render state generation must be a positive integer")
        if not isinstance(self.tick, Tick):
            raise TypeError("render state result tick must be a Tick")
        if type(self.state_revision) is not int or self.state_revision <= 0:
            raise ValueError("render state revision must be a positive integer")
        if any(
            type(value) is not int or value < 0
            for value in (self.articulation_count, self.rigid_body_count, self.particle_fluid_count)
        ):
            raise ValueError("render state result counts must be non-negative integers")


@runtime_checkable
class RenderStateWorld(Protocol):
    """Optional capability-gated world surface for render-only state replay."""

    def apply_render_state(self, frame: RenderStateFrame) -> RenderStateResult: ...


__all__ = (
    "PackedFloat32Array",
    "RENDER_STATE_CAPABILITY_ID",
    "RenderArticulationState",
    "RenderParticleFluidState",
    "RenderRigidBodyState",
    "RenderStateFrame",
    "RenderStateResult",
    "RenderStateWorld",
)
