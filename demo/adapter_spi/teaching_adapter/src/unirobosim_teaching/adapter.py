"""A small articulation-only Adapter implemented from public UniRoboSim contracts.

This is a teaching implementation, not a physics simulator.  Replace the in-memory
joint table with native SDK objects when adapting a real backend, while preserving
the same lifecycle, capability, handle, command, and state boundaries.
"""

from __future__ import annotations

import hashlib
from collections.abc import Iterable
from uuid import uuid4

from unirobosim import (
    WORLD_SCHEMA_VERSION,
    ArrayValue,
    ArticulationCommand,
    ArticulationState,
    BuildFingerprint,
    BuildInput,
    BuildReport,
    CapabilityDeclaration,
    CapabilityId,
    CapabilityNegotiationError,
    CapabilityRequirement,
    CapabilitySet,
    CommandError,
    CommandMode,
    ContactState,
    DebugBatch,
    DebugPublishReport,
    DeformableCommand,
    DeformableState,
    EntityHandle,
    EntityKind,
    EntityNotFoundError,
    EntityPath,
    EntitySpec,
    FrozenMap,
    LifecycleError,
    NegotiationReport,
    ParticleFluidCommand,
    ParticleFluidState,
    ProbeReport,
    ProviderDescriptor,
    ResetResult,
    RigidBodyCommand,
    RigidBodyState,
    SensorSample,
    SessionState,
    StaleHandleError,
    Tick,
    UnsupportedCapabilityError,
    WorldBuildError,
    WorldSpec,
    WorldState,
)

CAPABILITIES = CapabilitySet(
    (
        CapabilityDeclaration(
            CapabilityId("profile.core-robotics@1"),
            FrozenMap(
                {
                    "coordinate_system": "right-handed-z-up",
                    "quaternion_order": "xyzw",
                    "array_layout": "batch-first",
                }
            ),
        ),
        CapabilityDeclaration(CapabilityId("world.multi-environment@1")),
        CapabilityDeclaration(CapabilityId("state.articulation@1")),
        CapabilityDeclaration(CapabilityId("control.articulation.position@1")),
    )
)

DESCRIPTOR = ProviderDescriptor(
    provider_id="example.teaching",
    display_name="UniRoboSim Teaching Adapter",
    version="0.1.0",
    contract_version="v0alpha4",
    capabilities=CAPABILITIES,
    supported_world_schema_versions=(WORLD_SCHEMA_VERSION,),
    metadata=FrozenMap({"implementation": "in-memory-articulation-table"}),
)


def _lifecycle(message: str, operation: str, *, world_id: str | None = None) -> LifecycleError:
    return LifecycleError(
        message,
        operation=operation,
        backend_id=DESCRIPTOR.provider_id,
        world_id=world_id,
    )


def _unsupported(capability: str, operation: str, world_id: str) -> UnsupportedCapabilityError:
    return UnsupportedCapabilityError(
        f"the teaching Adapter does not implement {capability}",
        operation=operation,
        backend_id=DESCRIPTOR.provider_id,
        world_id=world_id,
        details={"capability": capability},
    )


class TeachingProvider:
    """Cheap discovery object; constructing and probing it starts no simulator."""

    def __init__(self, *, available: bool = True, build_failures: int = 0) -> None:
        if not isinstance(available, bool):
            raise TypeError("available must be bool")
        if not isinstance(build_failures, int) or isinstance(build_failures, bool) or build_failures < 0:
            raise ValueError("build_failures must be a non-negative integer")
        self._available = available
        self._build_failures = build_failures
        self._open_count = 0

    @property
    def descriptor(self) -> ProviderDescriptor:
        return DESCRIPTOR

    @property
    def open_count(self) -> int:
        """Adapter-specific teaching metric proving that probe is side-effect free."""

        return self._open_count

    def probe(self) -> ProbeReport:
        reason = None if self._available else "teaching backend disabled by constructor"
        return ProbeReport(DESCRIPTOR, self._available, reason=reason)

    def open(self) -> TeachingSession:
        if not self._available:
            raise _lifecycle("provider is unavailable", "teaching.provider.open")
        self._open_count += 1
        return TeachingSession(self, build_failures=self._build_failures)


class TeachingSession:
    """Owns one live teaching World and makes build failure transactional."""

    def __init__(self, provider: TeachingProvider, *, build_failures: int = 0) -> None:
        self._provider = provider
        self._session_id = f"teaching-{uuid4().hex}"
        self._state = SessionState.OPEN
        self._world: TeachingWorld | None = None
        self._generation = 0
        self._build_failures = build_failures

    @property
    def descriptor(self) -> ProviderDescriptor:
        return DESCRIPTOR

    @property
    def session_id(self) -> str:
        return self._session_id

    @property
    def state(self) -> SessionState:
        return self._state

    def _ensure_open(self, operation: str, *, ready_allowed: bool = False) -> None:
        allowed = {SessionState.OPEN}
        if ready_allowed:
            allowed.add(SessionState.READY)
        if self._state not in allowed:
            raise _lifecycle("session is not open for this operation", operation)

    def negotiate(self, requirements: Iterable[CapabilityRequirement]) -> NegotiationReport:
        self._ensure_open("teaching.session.negotiate", ready_allowed=True)
        return CAPABILITIES.negotiate(requirements)

    def build(self, spec: WorldSpec, *, build_input: BuildInput | None = None) -> TeachingWorld:
        self._ensure_open("teaching.session.build")
        if type(spec) is not WorldSpec:
            raise WorldBuildError(
                "build requires an immutable WorldSpec",
                operation="teaching.session.build",
                backend_id=DESCRIPTOR.provider_id,
            )
        if build_input is not None:
            raise WorldBuildError(
                "the teaching Adapter has no asset ingestion path",
                operation="teaching.session.build",
                backend_id=DESCRIPTOR.provider_id,
                world_id=spec.world_id,
            )
        if spec.schema_version not in DESCRIPTOR.supported_world_schema_versions:
            raise WorldBuildError(
                "World schema is not supported",
                operation="teaching.session.build",
                backend_id=DESCRIPTOR.provider_id,
                world_id=spec.world_id,
                details={"schema_version": spec.schema_version},
            )
        negotiation = CAPABILITIES.negotiate(spec.requirements)
        if not negotiation.accepted:
            raise CapabilityNegotiationError(
                "required capabilities are not supported",
                operation="teaching.session.build",
                backend_id=DESCRIPTOR.provider_id,
                world_id=spec.world_id,
                details={"negotiation": negotiation.to_dict()},
            )
        unsupported = tuple(entity.path.value for entity in spec.entities if entity.kind is not EntityKind.ARTICULATION)
        if unsupported:
            raise WorldBuildError(
                "the teaching Adapter only builds articulation entities",
                operation="teaching.session.build",
                backend_id=DESCRIPTOR.provider_id,
                world_id=spec.world_id,
                details={"unsupported_entities": unsupported},
            )
        if any(entity.asset_uri is not None for entity in spec.entities):
            raise WorldBuildError(
                "the teaching Adapter accepts programmatic articulations only",
                operation="teaching.session.build",
                backend_id=DESCRIPTOR.provider_id,
                world_id=spec.world_id,
            )
        if self._build_failures:
            self._build_failures -= 1
            raise WorldBuildError(
                "injected failure before native resource commit",
                operation="teaching.session.build",
                backend_id=DESCRIPTOR.provider_id,
                world_id=spec.world_id,
            )

        next_generation = self._generation + 1
        candidate = TeachingWorld(self, spec, next_generation)
        self._generation = next_generation
        self._world = candidate
        self._state = SessionState.READY
        return candidate

    def _world_closed(self, world: TeachingWorld) -> None:
        if self._world is world:
            self._world = None
            if self._state is not SessionState.CLOSED:
                self._state = SessionState.OPEN

    def close(self) -> None:
        if self._state is SessionState.CLOSED:
            return
        world = self._world
        self._world = None
        self._state = SessionState.CLOSED
        if world is not None:
            world._close(notify_session=False)


class TeachingWorld:
    """In-memory articulation table illustrating the complete base World shape."""

    def __init__(self, session: TeachingSession, spec: WorldSpec, generation: int) -> None:
        self._session = session
        self._spec = spec
        self._generation = generation
        self._state = WorldState.READY
        self._tick = Tick(0, 0.0)
        self._reset_count = 0
        self._entities = {entity.path: entity for entity in spec.entities}
        self._initial = {entity.path: tuple(entity.initial_joint_positions) for entity in spec.entities}
        self._positions = {
            entity.path: [list(entity.initial_joint_positions) for _ in range(spec.environments.count)]
            for entity in spec.entities
        }
        self._velocities = {
            entity.path: [[0.0] * len(entity.joint_names) for _ in range(spec.environments.count)]
            for entity in spec.entities
        }
        self._pending: list[tuple[EntityPath, tuple[int, ...], tuple[int, ...], tuple[tuple[float, ...], ...]]] = []
        self._handles = {path: self._new_handle(path) for path in self._entities}
        self._build_report = BuildReport(
            BuildFingerprint(
                DESCRIPTOR.provider_id,
                DESCRIPTOR.version,
                DESCRIPTOR.contract_version,
                spec.digest,
                CAPABILITIES.digest,
            ),
            spec.world_id,
            generation,
            spec.environments.count,
            len(spec.entities),
        )

    @property
    def world_id(self) -> str:
        return self._spec.world_id

    @property
    def generation(self) -> int:
        return self._generation

    @property
    def state(self) -> WorldState:
        return self._state

    @property
    def tick(self) -> Tick:
        return self._tick

    @property
    def build_report(self) -> BuildReport:
        return self._build_report

    def _ensure_ready(self, operation: str) -> None:
        if self._state is not WorldState.READY:
            raise _lifecycle("world is closed", operation, world_id=self.world_id)

    def _new_handle(self, path: EntityPath) -> EntityHandle:
        payload = f"{self._session.session_id}:{self.world_id}:{self.generation}:{path.value}"
        token = hashlib.sha256(payload.encode("utf-8")).hexdigest()
        return EntityHandle(
            DESCRIPTOR.provider_id,
            self._session.session_id,
            self.world_id,
            self.generation,
            path,
            EntityKind.ARTICULATION,
            token,
        )

    def resolve(self, path: EntityPath) -> EntityHandle:
        self._ensure_ready("teaching.world.resolve")
        handle = self._handles.get(path)
        if handle is None:
            raise EntityNotFoundError(
                "entity path does not exist",
                operation="teaching.world.resolve",
                backend_id=DESCRIPTOR.provider_id,
                world_id=self.world_id,
                entity_path=str(path),
            )
        return handle

    def _entity_for(self, handle: EntityHandle, operation: str) -> EntitySpec:
        self._ensure_ready(operation)
        current = self._handles.get(handle.path)
        if current != handle:
            raise StaleHandleError(
                "entity handle does not belong to this live World generation",
                operation=operation,
                backend_id=DESCRIPTOR.provider_id,
                world_id=self.world_id,
                entity_path=handle.path.value,
            )
        return self._entities[handle.path]

    def _environment_indices(self, values: Iterable[int] | None, operation: str) -> tuple[int, ...]:
        indices = tuple(range(self._spec.environments.count)) if values is None else tuple(values)
        if (
            not indices
            or len(indices) != len(set(indices))
            or any(index < 0 or index >= self._spec.environments.count for index in indices)
        ):
            raise CommandError(
                "environment selection is out of range",
                operation=operation,
                backend_id=DESCRIPTOR.provider_id,
                world_id=self.world_id,
                details={"environment_indices": indices},
            )
        return indices

    def reset(self, environment_indices: Iterable[int] | None = None) -> ResetResult:
        self._ensure_ready("teaching.world.reset")
        selected = self._environment_indices(environment_indices, "teaching.world.reset")
        for path, initial in self._initial.items():
            for environment in selected:
                self._positions[path][environment] = list(initial)
                self._velocities[path][environment] = [0.0] * len(initial)
        reset_environments = set(selected)
        remaining_commands = []
        for path, environments, degrees, targets in self._pending:
            remaining_rows = tuple(
                (environment, targets[row])
                for row, environment in enumerate(environments)
                if environment not in reset_environments
            )
            if remaining_rows:
                remaining_commands.append(
                    (
                        path,
                        tuple(environment for environment, _ in remaining_rows),
                        degrees,
                        tuple(target for _, target in remaining_rows),
                    )
                )
        self._pending = remaining_commands
        self._reset_count += 1
        return ResetResult(selected, self._reset_count, self._tick)

    def apply_articulation_command(self, command: ArticulationCommand) -> None:
        operation = "teaching.world.apply_articulation_command"
        entity = self._entity_for(command.handle, operation)
        if command.mode is not CommandMode.POSITION:
            raise _unsupported(f"control.articulation.{command.mode.value}@1", operation, self.world_id)
        environments = self._environment_indices(command.environment_indices, operation)
        degrees = (
            tuple(range(len(entity.joint_names)))
            if command.degree_of_freedom_indices is None
            else tuple(command.degree_of_freedom_indices)
        )
        if (
            not degrees
            or len(degrees) != len(set(degrees))
            or any(index >= len(entity.joint_names) for index in degrees)
        ):
            raise CommandError(
                "joint selection is out of range",
                operation=operation,
                backend_id=DESCRIPTOR.provider_id,
                world_id=self.world_id,
            )
        expected_shape = (len(environments), len(degrees))
        if command.targets.shape != expected_shape:
            raise CommandError(
                "target shape does not match selected environments and joints",
                operation=operation,
                backend_id=DESCRIPTOR.provider_id,
                world_id=self.world_id,
                details={"expected_shape": expected_shape, "actual_shape": command.targets.shape},
            )
        expected_units = tuple(entity.joint_position_units[index] for index in degrees)
        if command.target_units and command.target_units != expected_units:
            raise CommandError(
                "target units do not match articulation axes",
                operation=operation,
                backend_id=DESCRIPTOR.provider_id,
                world_id=self.world_id,
                details={"expected_units": expected_units, "actual_units": command.target_units},
            )
        targets = tuple(tuple(float(value) for value in row) for row in command.targets.rows())
        self._pending.append((entity.path, environments, degrees, targets))

    def read_articulation(self, handle: EntityHandle) -> ArticulationState:
        entity = self._entity_for(handle, "teaching.world.read_articulation")
        return ArticulationState(
            entity.path.value,
            self.generation,
            self.tick,
            entity.joint_names,
            ArrayValue.from_rows(self._positions[entity.path]),
            ArrayValue.from_rows(self._velocities[entity.path]),
            entity.joint_position_units,
            tuple("rad/s" if unit == "rad" else "m/s" for unit in entity.joint_position_units),
        )

    def apply_rigid_body_command(self, command: RigidBodyCommand) -> None:
        del command
        raise _unsupported("control.rigid_body.wrench@1", "teaching.world.apply_rigid_body_command", self.world_id)

    def read_rigid_body(self, handle: EntityHandle) -> RigidBodyState:
        del handle
        raise _unsupported("state.rigid_body@1", "teaching.world.read_rigid_body", self.world_id)

    def read_contact(self, handle: EntityHandle, force_threshold_n: float = 1.0e-6) -> ContactState:
        del handle, force_threshold_n
        raise _unsupported("contact.binary@1", "teaching.world.read_contact", self.world_id)

    def apply_deformable_command(self, command: DeformableCommand) -> None:
        del command
        raise _unsupported("control.deformable.points@1", "teaching.world.apply_deformable_command", self.world_id)

    def read_deformable(self, handle: EntityHandle) -> DeformableState:
        del handle
        raise _unsupported("state.deformable.surface@1", "teaching.world.read_deformable", self.world_id)

    def apply_particle_fluid_command(self, command: ParticleFluidCommand) -> None:
        del command
        raise _unsupported("control.fluid.particles@1", "teaching.world.apply_particle_fluid_command", self.world_id)

    def read_particle_fluid(self, handle: EntityHandle) -> ParticleFluidState:
        del handle
        raise _unsupported("state.fluid.particles@1", "teaching.world.read_particle_fluid", self.world_id)

    def read_sensor(self, handle: EntityHandle) -> SensorSample:
        del handle
        raise _unsupported("sensor.camera@1", "teaching.world.read_sensor", self.world_id)

    def publish_debug(self, batch: DebugBatch) -> DebugPublishReport:
        del batch
        raise _unsupported("debug.sink.native_overlay@1", "teaching.world.publish_debug", self.world_id)

    def clear_debug(
        self,
        *,
        layer: str | None = None,
        group: str | None = None,
        primitive_id: str | None = None,
    ) -> int:
        del layer, group, primitive_id
        raise _unsupported("debug.sink.native_overlay@1", "teaching.world.clear_debug", self.world_id)

    def step(self, count: int = 1) -> Tick:
        self._ensure_ready("teaching.world.step")
        if not isinstance(count, int) or isinstance(count, bool) or count <= 0:
            raise CommandError(
                "step count must be a positive integer",
                operation="teaching.world.step",
                backend_id=DESCRIPTOR.provider_id,
                world_id=self.world_id,
            )
        for path, environments, degrees, targets in self._pending:
            for row, environment in enumerate(environments):
                for column, degree in enumerate(degrees):
                    self._positions[path][environment][degree] = targets[row][column]
                    self._velocities[path][environment][degree] = 0.0
        self._pending.clear()
        step_index = self._tick.step_index + count
        sim_time = self._tick.sim_time_seconds + count * self._spec.physics.time_step_seconds
        self._tick = Tick(step_index, sim_time)
        return self._tick

    def _close(self, *, notify_session: bool) -> None:
        if self._state is WorldState.CLOSED:
            return
        self._state = WorldState.CLOSED
        self._pending.clear()
        if notify_session:
            self._session._world_closed(self)

    def close(self) -> None:
        self._close(notify_session=True)


def create_provider() -> TeachingProvider:
    """Entry-point factory. It must stay argument-free and discovery-safe."""

    return TeachingProvider()
