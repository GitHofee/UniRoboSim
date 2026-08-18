"""Structural interfaces implemented by backend adapters."""

from __future__ import annotations

from collections.abc import Iterable
from typing import Protocol, runtime_checkable

from .capabilities import CapabilityRequirement, NegotiationReport
from .debug import DebugBatch, DebugPublishReport
from .reports import (
    ArticulationState,
    BuildReport,
    ContactState,
    DeformableState,
    ParticleFluidState,
    ProbeReport,
    ProviderDescriptor,
    ResetResult,
    RigidBodyState,
    SensorSample,
)
from .specs import ArticulationCommand, DeformableCommand, ParticleFluidCommand, RigidBodyCommand, WorldSpec
from .values import EntityHandle, EntityPath, SessionState, Tick, WorldState


@runtime_checkable
class World(Protocol):
    @property
    def world_id(self) -> str: ...

    @property
    def generation(self) -> int: ...

    @property
    def state(self) -> WorldState: ...

    @property
    def tick(self) -> Tick: ...

    @property
    def build_report(self) -> BuildReport: ...

    def resolve(self, path: EntityPath) -> EntityHandle: ...

    def reset(self, environment_indices: Iterable[int] | None = None) -> ResetResult: ...

    def apply_articulation_command(self, command: ArticulationCommand) -> None: ...

    def read_articulation(self, handle: EntityHandle) -> ArticulationState: ...

    def apply_rigid_body_command(self, command: RigidBodyCommand) -> None: ...

    def read_rigid_body(self, handle: EntityHandle) -> RigidBodyState: ...

    def read_contact(self, handle: EntityHandle, force_threshold_n: float = 1.0e-6) -> ContactState: ...

    def apply_deformable_command(self, command: DeformableCommand) -> None: ...

    def read_deformable(self, handle: EntityHandle) -> DeformableState: ...

    def apply_particle_fluid_command(self, command: ParticleFluidCommand) -> None: ...

    def read_particle_fluid(self, handle: EntityHandle) -> ParticleFluidState: ...

    def read_sensor(self, handle: EntityHandle) -> SensorSample: ...

    def publish_debug(self, batch: DebugBatch) -> DebugPublishReport: ...

    def clear_debug(self, *, layer: str | None = None, primitive_id: str | None = None) -> int: ...

    def step(self, count: int = 1) -> Tick: ...

    def close(self) -> None: ...


@runtime_checkable
class Session(Protocol):
    @property
    def session_id(self) -> str: ...

    @property
    def state(self) -> SessionState: ...

    @property
    def descriptor(self) -> ProviderDescriptor: ...

    def negotiate(self, requirements: Iterable[CapabilityRequirement]) -> NegotiationReport: ...

    def build(self, spec: WorldSpec) -> World: ...

    def close(self) -> None: ...


@runtime_checkable
class Provider(Protocol):
    @property
    def descriptor(self) -> ProviderDescriptor: ...

    def probe(self) -> ProbeReport: ...

    def open(self) -> Session: ...
