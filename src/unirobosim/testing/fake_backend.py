"""Deterministic reference implementation for contract tests.

This module is deliberately explicit and never selected by production runtime code. Its state update
rules test the API; they are not a physics-fidelity claim.
"""

from __future__ import annotations

import hashlib
import itertools
from collections.abc import Iterable
from dataclasses import dataclass
from types import TracebackType

from unirobosim.api.capabilities import (
    CapabilityDeclaration,
    CapabilityId,
    CapabilityRequirement,
    CapabilitySet,
    NegotiationReport,
)
from unirobosim.api.errors import (
    CapabilityNegotiationError,
    CommandError,
    EntityNotFoundError,
    LifecycleError,
    ProviderSelectionError,
    StaleHandleError,
    ValidationError,
    WorldBuildError,
)
from unirobosim.api.frozen import FrozenMap
from unirobosim.api.reports import (
    ArticulationState,
    BuildFingerprint,
    BuildReport,
    ProbeReport,
    ProviderDescriptor,
    ResetResult,
)
from unirobosim.api.specs import ArticulationCommand, EntitySpec, WorldSpec
from unirobosim.api.values import (
    ArrayValue,
    CommandMode,
    EntityHandle,
    EntityKind,
    EntityPath,
    SessionState,
    Tick,
    WorldState,
)

FAKE_CAPABILITIES = CapabilitySet(
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
        CapabilityDeclaration(CapabilityId("control.articulation.velocity@1")),
        CapabilityDeclaration(
            CapabilityId("control.articulation.effort@1"),
            limitations=("unit-mass deterministic test integration; not physical simulation",),
        ),
    )
)

FAKE_DESCRIPTOR = ProviderDescriptor(
    provider_id="reference.fake",
    display_name="UniRoboSim Fake Reference Backend",
    version="0.1.0a0",
    contract_version="v0alpha1",
    capabilities=FAKE_CAPABILITIES,
    metadata=FrozenMap({"purpose": "contract-testing-only"}),
)

_SESSION_IDS = itertools.count(1)


@dataclass
class _ArticulationRuntime:
    spec: EntitySpec
    positions: list[list[float]]
    velocities: list[list[float]]
    modes: list[list[CommandMode]]
    targets: list[list[float]]


class FakeProvider:
    """Provider for the deterministic protocol oracle."""

    def __init__(self, *, available: bool = True, build_failures: int = 0) -> None:
        if not isinstance(available, bool):
            raise ValidationError("available must be boolean", operation="fake_provider.init")
        if not isinstance(build_failures, int) or isinstance(build_failures, bool) or build_failures < 0:
            raise ValidationError("build_failures must be non-negative", operation="fake_provider.init")
        self._available = available
        self._build_failures = build_failures

    @property
    def descriptor(self) -> ProviderDescriptor:
        return FAKE_DESCRIPTOR

    def probe(self) -> ProbeReport:
        return ProbeReport(
            descriptor=self.descriptor,
            available=self._available,
            reason=None if self._available else "disabled by FakeProvider configuration",
        )

    def open(self) -> FakeSession:
        probe = self.probe()
        if not probe.available:
            raise ProviderSelectionError(
                "fake provider is unavailable",
                operation="provider.open",
                backend_id=self.descriptor.provider_id,
                details={"reason": probe.reason},
            )
        return FakeSession(self.descriptor, build_failures=self._build_failures)


class FakeSession:
    def __init__(self, descriptor: ProviderDescriptor, *, build_failures: int = 0) -> None:
        self._descriptor = descriptor
        self._session_id = f"fake-session-{next(_SESSION_IDS)}"
        self._state = SessionState.OPEN
        self._generation = 0
        self._active_world: FakeWorld | None = None
        self._build_failures = build_failures

    @property
    def descriptor(self) -> ProviderDescriptor:
        return self._descriptor

    @property
    def session_id(self) -> str:
        return self._session_id

    @property
    def state(self) -> SessionState:
        return self._state

    def _ensure_open(self, operation: str, *, allow_ready: bool = False) -> None:
        accepted = {SessionState.OPEN, SessionState.READY} if allow_ready else {SessionState.OPEN}
        if self._state not in accepted:
            raise LifecycleError(
                "session is not in a valid state for this operation",
                operation=operation,
                backend_id=self.descriptor.provider_id,
                details={"state": self._state.value, "accepted": sorted(item.value for item in accepted)},
            )

    def negotiate(self, requirements: Iterable[CapabilityRequirement]) -> NegotiationReport:
        self._ensure_open("session.negotiate", allow_ready=True)
        return self.descriptor.capabilities.negotiate(tuple(requirements))

    def build(self, spec: WorldSpec) -> FakeWorld:
        self._ensure_open("session.build")
        if not isinstance(spec, WorldSpec):
            raise ValidationError("build requires a WorldSpec", operation="session.build")
        negotiation = self.negotiate(spec.requirements)
        if not negotiation.accepted:
            raise CapabilityNegotiationError(
                "world requirements are not satisfied",
                operation="session.build",
                backend_id=self.descriptor.provider_id,
                world_id=spec.world_id,
                details={"negotiation": negotiation.to_dict()},
            )
        if self._build_failures:
            self._build_failures -= 1
            raise WorldBuildError(
                "injected fake build failure",
                operation="session.build",
                backend_id=self.descriptor.provider_id,
                world_id=spec.world_id,
                details={"remaining_injected_failures": self._build_failures},
            )
        self._generation += 1
        world = FakeWorld(self, spec, self._generation)
        self._active_world = world
        self._state = SessionState.READY
        return world

    def _world_closed(self, world: FakeWorld) -> None:
        if self._active_world is world:
            self._active_world = None
            if self._state is not SessionState.CLOSED:
                self._state = SessionState.OPEN

    def close(self) -> None:
        if self._state is SessionState.CLOSED:
            return
        world = self._active_world
        self._state = SessionState.CLOSED
        self._active_world = None
        if world is not None:
            world._close(notify_session=False)

    def __enter__(self) -> FakeSession:
        self._ensure_open("session.enter", allow_ready=True)
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        self.close()


class FakeWorld:
    def __init__(self, session: FakeSession, spec: WorldSpec, generation: int) -> None:
        self._session = session
        self._spec = spec
        self._generation = generation
        self._state = WorldState.READY
        self._step_index = 0
        self._reset_count = 0
        self._entities = {entity.path: entity for entity in spec.entities}
        self._articulations: dict[EntityPath, _ArticulationRuntime] = {}
        for entity in spec.entities:
            if entity.kind is EntityKind.ARTICULATION:
                initial = list(entity.initial_joint_positions)
                positions = [initial.copy() for _ in range(spec.environments.count)]
                self._articulations[entity.path] = _ArticulationRuntime(
                    spec=entity,
                    positions=positions,
                    velocities=[[0.0] * len(initial) for _ in range(spec.environments.count)],
                    modes=[[CommandMode.POSITION] * len(initial) for _ in range(spec.environments.count)],
                    targets=[initial.copy() for _ in range(spec.environments.count)],
                )
        fingerprint = BuildFingerprint(
            provider_id=session.descriptor.provider_id,
            provider_version=session.descriptor.version,
            contract_version=session.descriptor.contract_version,
            world_digest=spec.digest,
            capability_digest=session.descriptor.capabilities.digest,
        )
        self._build_report = BuildReport(
            fingerprint=fingerprint,
            world_id=spec.world_id,
            generation=generation,
            environment_count=spec.environments.count,
            entity_count=len(spec.entities),
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
        return Tick(self._step_index, self._step_index * self._spec.physics.time_step_seconds)

    @property
    def build_report(self) -> BuildReport:
        return self._build_report

    def _ensure_ready(self, operation: str) -> None:
        if self._state is not WorldState.READY:
            raise LifecycleError(
                "world is closed",
                operation=operation,
                backend_id=self._session.descriptor.provider_id,
                world_id=self.world_id,
                details={"state": self._state.value},
            )

    def _handle_token(self, path: EntityPath) -> str:
        raw = f"{self._session.session_id}|{self.world_id}|{self.generation}|{path.value}"
        return hashlib.sha256(raw.encode("utf-8")).hexdigest()

    def resolve(self, path: EntityPath) -> EntityHandle:
        self._ensure_ready("world.resolve")
        if not isinstance(path, EntityPath):
            raise ValidationError("resolve requires an EntityPath", operation="world.resolve")
        entity = self._entities.get(path)
        if entity is None:
            raise EntityNotFoundError(
                "logical entity path does not exist",
                operation="world.resolve",
                backend_id=self._session.descriptor.provider_id,
                world_id=self.world_id,
                entity_path=path.value,
            )
        return EntityHandle(
            provider_id=self._session.descriptor.provider_id,
            session_id=self._session.session_id,
            world_id=self.world_id,
            generation=self.generation,
            path=path,
            entity_kind=entity.kind,
            token=self._handle_token(path),
        )

    def _validate_handle(self, handle: EntityHandle, operation: str) -> EntitySpec:
        if not isinstance(handle, EntityHandle):
            raise StaleHandleError("operation requires an EntityHandle", operation=operation, world_id=self.world_id)
        expected = (
            self._session.descriptor.provider_id,
            self._session.session_id,
            self.world_id,
            self.generation,
            self._handle_token(handle.path),
        )
        actual = (handle.provider_id, handle.session_id, handle.world_id, handle.generation, handle.token)
        entity = self._entities.get(handle.path)
        if actual != expected or entity is None or handle.entity_kind is not entity.kind:
            raise StaleHandleError(
                "entity handle does not belong to this live world generation",
                operation=operation,
                backend_id=self._session.descriptor.provider_id,
                world_id=self.world_id,
                entity_path=handle.path.value,
                details={"expected_generation": self.generation, "actual_generation": handle.generation},
            )
        return entity

    @staticmethod
    def _indices(
        values: Iterable[int] | None,
        size: int,
        name: str,
        *,
        operation: str,
    ) -> tuple[int, ...]:
        if values is None:
            return tuple(range(size))
        result = tuple(values)
        if (
            not result
            or any(
                not isinstance(index, int) or isinstance(index, bool) or index < 0 or index >= size for index in result
            )
            or len(result) != len(set(result))
        ):
            raise ValidationError(
                f"{name} must be a non-empty unique in-range selection",
                operation=operation,
                details={"selection": list(result), "size": size},
            )
        return result

    def reset(self, environment_indices: Iterable[int] | None = None) -> ResetResult:
        self._ensure_ready("world.reset")
        environments = self._indices(
            environment_indices,
            self._spec.environments.count,
            "environment_indices",
            operation="world.reset",
        )
        for runtime in self._articulations.values():
            initial = runtime.spec.initial_joint_positions
            for environment in environments:
                runtime.positions[environment] = list(initial)
                runtime.velocities[environment] = [0.0] * len(initial)
                runtime.modes[environment] = [CommandMode.POSITION] * len(initial)
                runtime.targets[environment] = list(initial)
        self._reset_count += 1
        return ResetResult(environments, self._reset_count, self.tick)

    def apply_articulation_command(self, command: ArticulationCommand) -> None:
        self._ensure_ready("world.apply_articulation_command")
        if not isinstance(command, ArticulationCommand):
            raise CommandError(
                "operation requires an ArticulationCommand", operation="world.apply_articulation_command"
            )
        entity = self._validate_handle(command.handle, "world.apply_articulation_command")
        if entity.kind is not EntityKind.ARTICULATION:
            raise CommandError(
                "entity is not an articulation",
                operation="world.apply_articulation_command",
                entity_path=entity.path.value,
            )
        runtime = self._articulations[entity.path]
        environments = self._indices(
            command.environment_indices,
            self._spec.environments.count,
            "environment_indices",
            operation="world.apply_articulation_command",
        )
        degrees = self._indices(
            command.degree_of_freedom_indices,
            len(entity.joint_names),
            "degree_of_freedom_indices",
            operation="world.apply_articulation_command",
        )
        expected_shape = (len(environments), len(degrees))
        if command.targets.shape != expected_shape:
            raise CommandError(
                "command target shape must exactly match selected environments and degrees of freedom",
                operation="world.apply_articulation_command",
                backend_id=self._session.descriptor.provider_id,
                world_id=self.world_id,
                entity_path=entity.path.value,
                details={"expected_shape": list(expected_shape), "actual_shape": list(command.targets.shape)},
            )
        rows = command.targets.rows()
        for row_index, environment in enumerate(environments):
            for column_index, degree in enumerate(degrees):
                runtime.modes[environment][degree] = command.mode
                runtime.targets[environment][degree] = float(rows[row_index][column_index])

    def read_articulation(self, handle: EntityHandle) -> ArticulationState:
        self._ensure_ready("world.read_articulation")
        entity = self._validate_handle(handle, "world.read_articulation")
        if entity.kind is not EntityKind.ARTICULATION:
            raise CommandError(
                "entity is not an articulation",
                operation="world.read_articulation",
                entity_path=entity.path.value,
            )
        runtime = self._articulations[entity.path]
        return ArticulationState(
            joint_positions=ArrayValue.from_rows(runtime.positions),
            joint_velocities=ArrayValue.from_rows(runtime.velocities),
            tick=self.tick,
        )

    def step(self, count: int = 1) -> Tick:
        self._ensure_ready("world.step")
        if not isinstance(count, int) or isinstance(count, bool) or count <= 0:
            raise ValidationError("step count must be a positive integer", operation="world.step")
        time_step = self._spec.physics.time_step_seconds
        for _ in range(count):
            for runtime in self._articulations.values():
                for environment in range(self._spec.environments.count):
                    for degree in range(len(runtime.spec.joint_names)):
                        mode = runtime.modes[environment][degree]
                        target = runtime.targets[environment][degree]
                        if mode is CommandMode.POSITION:
                            previous = runtime.positions[environment][degree]
                            runtime.positions[environment][degree] = target
                            runtime.velocities[environment][degree] = (target - previous) / time_step
                        elif mode is CommandMode.VELOCITY:
                            runtime.velocities[environment][degree] = target
                            runtime.positions[environment][degree] += target * time_step
                        else:
                            runtime.velocities[environment][degree] += target * time_step
                            runtime.positions[environment][degree] += (
                                runtime.velocities[environment][degree] * time_step
                            )
            self._step_index += 1
        return self.tick

    def _close(self, *, notify_session: bool) -> None:
        if self._state is WorldState.CLOSED:
            return
        self._state = WorldState.CLOSED
        self._entities.clear()
        self._articulations.clear()
        if notify_session:
            self._session._world_closed(self)

    def close(self) -> None:
        self._close(notify_session=True)

    def __enter__(self) -> FakeWorld:
        self._ensure_ready("world.enter")
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        self.close()
