"""Deterministic reference implementation for contract tests.

This module is deliberately explicit and never selected by production runtime code. Its state update
rules test the API; they are not a physics-fidelity claim.
"""

from __future__ import annotations

import hashlib
import itertools
import math
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
from unirobosim.api.debug import DebugBatch, DebugLifetimeMode, DebugPrimitive, DebugPublishReport
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
    ContactState,
    DeformableState,
    ParticleFluidState,
    ProbeReport,
    ProviderDescriptor,
    ResetResult,
    RigidBodyState,
    SensorChannel,
    SensorSample,
)
from unirobosim.api.scene import (
    SceneCommand,
    SceneCommandKind,
    SceneCommandResult,
    SceneCommandStatus,
    SceneDelta,
    SceneDragMode,
    SceneEntityState,
    SceneSnapshot,
    SceneVisual,
    SceneVisualKind,
)
from unirobosim.api.specs import (
    ArticulationCommand,
    DeformableCommand,
    EntitySpec,
    ParticleFluidCommand,
    RigidBodyCommand,
    WorldSpec,
)
from unirobosim.api.values import (
    ArrayValue,
    CameraModality,
    CommandMode,
    EntityHandle,
    EntityKind,
    EntityPath,
    PointCommandMode,
    Pose,
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
        CapabilityDeclaration(CapabilityId("state.rigid_body@1")),
        CapabilityDeclaration(
            CapabilityId("control.rigid_body.wrench@1"),
            FrozenMap({"frame": "environment-local-world", "persistence": "until-overwrite-or-reset"}),
            limitations=("unit mass and unit diagonal inertia; not physical simulation",),
        ),
        CapabilityDeclaration(
            CapabilityId("contact.binary@1"),
            limitations=("fake reference backend has no collision model and therefore reports false",),
        ),
        CapabilityDeclaration(
            CapabilityId("contact.net_normal_force@1"),
            FrozenMap({"aggregation": "all-partners", "frame": "environment-local-world"}),
            limitations=("fake reference backend has no collision model and therefore reports zero",),
        ),
        CapabilityDeclaration(CapabilityId("state.articulation@1")),
        CapabilityDeclaration(CapabilityId("control.articulation.position@1")),
        CapabilityDeclaration(CapabilityId("control.articulation.velocity@1")),
        CapabilityDeclaration(
            CapabilityId("control.articulation.effort@1"),
            limitations=("unit-mass deterministic test integration; not physical simulation",),
        ),
        CapabilityDeclaration(
            CapabilityId("profile.soft-matter@1"),
            FrozenMap(
                {
                    "state_layout": "batch-point-xyz",
                    "point_count": "fixed",
                    "dynamics": "independent-point-mass-reference-only",
                }
            ),
            limitations=("no elasticity, incompressibility, collision, or self-collision",),
        ),
        CapabilityDeclaration(
            CapabilityId("state.deformable.surface@1"),
            FrozenMap({"topology": "triangles", "point_count": "fixed"}),
            limitations=("topology does not affect fake point-mass dynamics",),
        ),
        CapabilityDeclaration(
            CapabilityId("state.deformable.volume@1"),
            FrozenMap({"topology": "tetrahedra", "point_count": "fixed"}),
            limitations=("no FEM constitutive model or collision",),
        ),
        CapabilityDeclaration(
            CapabilityId("control.deformable.points@1"),
            FrozenMap({"modes": ["position", "velocity", "force"], "frame": "world"}),
            limitations=("independent point control only",),
        ),
        CapabilityDeclaration(
            CapabilityId("state.fluid.particles@1"),
            FrozenMap({"representation": "particles", "point_count": "fixed"}),
            limitations=("no density constraint, viscosity, surface tension, or collision",),
        ),
        CapabilityDeclaration(
            CapabilityId("control.fluid.particles@1"),
            FrozenMap({"modes": ["position", "velocity", "force"], "frame": "world"}),
            limitations=("independent particle control only",),
        ),
        CapabilityDeclaration(
            CapabilityId("sensor.camera@1"),
            FrozenMap({"schedule": "synchronous", "pose_frame": "environment-local-world"}),
            limitations=("deterministic test pattern; not a rendered image",),
        ),
        CapabilityDeclaration(
            CapabilityId("sensor.camera.rgb@1"),
            FrozenMap({"dtype": "uint8", "layout": "environment-height-width-rgb"}),
            limitations=("deterministic test pattern; not a rendered image",),
        ),
        CapabilityDeclaration(
            CapabilityId("sensor.camera.depth@1"),
            FrozenMap({"dtype": "float32", "unit": "metre", "no_hit": 0.0}),
            limitations=("deterministic test pattern; not ray-cast depth",),
        ),
        CapabilityDeclaration(
            CapabilityId("debug.sink.native_overlay@1"),
            FrozenMap({"primitives": ["point_set", "line_list"], "stable_ids": True}),
            limitations=("in-memory reference endpoint; not a renderer overlay",),
        ),
        CapabilityDeclaration(CapabilityId("scene.snapshot@1")),
        CapabilityDeclaration(CapabilityId("scene.delta@1")),
        CapabilityDeclaration(CapabilityId("scene.command.pose@1")),
        CapabilityDeclaration(
            CapabilityId("scene.command.drag@1"),
            FrozenMap({"entity_kinds": ["rigid_body"], "modes": ["kinematic"]}),
        ),
        CapabilityDeclaration(CapabilityId("render.browser-scene@1")),
    )
)

FAKE_DESCRIPTOR = ProviderDescriptor(
    provider_id="reference.fake",
    display_name="UniRoboSim Fake Reference Backend",
    version="0.7.0",
    contract_version="v0alpha4",
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


@dataclass
class _RigidRuntime:
    initial_position: list[float]
    initial_orientation: list[float]
    positions: list[list[float]]
    orientations: list[list[float]]
    linear_velocities: list[list[float]]
    angular_velocities: list[list[float]]
    forces: list[list[float]]
    torques: list[list[float]]
    mass_kg: float


@dataclass
class _PointRuntime:
    spec: EntitySpec
    initial_positions: list[list[float]]
    initial_velocities: list[list[float]]
    point_mass_kg: float
    linear_damping_per_s: float
    kinematic_indices: frozenset[int]
    positions: list[list[list[float]]]
    velocities: list[list[list[float]]]
    modes: list[list[PointCommandMode]]
    targets: list[list[list[float]]]


def _vectors(value: ArrayValue) -> list[list[float]]:
    return [
        [float(value.values[offset]), float(value.values[offset + 1]), float(value.values[offset + 2])]
        for offset in range(0, len(value.values), 3)
    ]


def _copy_vectors(values: list[list[float]]) -> list[list[float]]:
    return [vector.copy() for vector in values]


def _rotate_vector_xyzw(vector: list[float], quaternion: tuple[float, float, float, float]) -> list[float]:
    x, y, z, w = quaternion
    vx, vy, vz = vector
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return [
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    ]


def _integrate_orientation_xyzw(orientation: list[float], angular_velocity_w: list[float], dt: float) -> list[float]:
    x, y, z, w = orientation
    wx, wy, wz = angular_velocity_w
    integrated = [
        x + 0.5 * dt * (wx * w + wy * z - wz * y),
        y + 0.5 * dt * (-wx * z + wy * w + wz * x),
        z + 0.5 * dt * (wx * y - wy * x + wz * w),
        w + 0.5 * dt * (-wx * x - wy * y - wz * z),
    ]
    norm = math.sqrt(sum(value * value for value in integrated))
    return [value / norm for value in integrated]


def _entity_frame_vectors(entity: EntitySpec, value: ArrayValue, *, translate: bool) -> list[list[float]]:
    result = []
    for vector in _vectors(value):
        transformed = _rotate_vector_xyzw(vector, entity.pose.orientation_xyzw)
        if translate:
            transformed = [transformed[axis] + entity.pose.position[axis] for axis in range(3)]
        result.append([0.0 if math.isclose(component, 0.0, abs_tol=1e-15) else component for component in transformed])
    return result


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
        self._scene_sequence = 0
        self._scene_results: dict[str, SceneCommandResult] = {}
        self._active_drags: dict[str, tuple[EntityPath, int, Pose]] = {}
        self._entities = {entity.path: entity for entity in spec.entities}
        self._articulations: dict[EntityPath, _ArticulationRuntime] = {}
        self._rigids: dict[EntityPath, _RigidRuntime] = {}
        self._points: dict[EntityPath, _PointRuntime] = {}
        self._debug_primitives: dict[tuple[str, str, str], DebugPrimitive] = {}
        self._debug_expirations: dict[tuple[str, str, str], int | None] = {}
        for entity in spec.entities:
            if entity.kind is EntityKind.RIGID_BODY:
                position = list(entity.pose.position)
                orientation = list(entity.pose.orientation_xyzw)
                environment_count = spec.environments.count
                self._rigids[entity.path] = _RigidRuntime(
                    initial_position=position,
                    initial_orientation=orientation,
                    positions=[position.copy() for _ in range(environment_count)],
                    orientations=[orientation.copy() for _ in range(environment_count)],
                    linear_velocities=[[0.0, 0.0, 0.0] for _ in range(environment_count)],
                    angular_velocities=[[0.0, 0.0, 0.0] for _ in range(environment_count)],
                    forces=[[0.0, 0.0, 0.0] for _ in range(environment_count)],
                    torques=[[0.0, 0.0, 0.0] for _ in range(environment_count)],
                    mass_kg=1.0 if entity.box is None else entity.box.mass_kg,
                )
            elif entity.kind is EntityKind.ARTICULATION:
                initial = list(entity.initial_joint_positions)
                positions = [initial.copy() for _ in range(spec.environments.count)]
                self._articulations[entity.path] = _ArticulationRuntime(
                    spec=entity,
                    positions=positions,
                    velocities=[[0.0] * len(initial) for _ in range(spec.environments.count)],
                    modes=[[CommandMode.POSITION] * len(initial) for _ in range(spec.environments.count)],
                    targets=[initial.copy() for _ in range(spec.environments.count)],
                )
            elif entity.deformable is not None:
                initial_positions = _entity_frame_vectors(entity, entity.deformable.rest_positions_m, translate=True)
                initial_velocities = _entity_frame_vectors(
                    entity, entity.deformable.initial_velocities(), translate=False
                )
                self._points[entity.path] = self._make_point_runtime(
                    entity,
                    initial_positions,
                    initial_velocities,
                    point_mass_kg=entity.deformable.node_mass_kg,
                    linear_damping_per_s=entity.deformable.linear_damping_per_s,
                    kinematic_indices=frozenset(entity.deformable.kinematic_node_indices),
                )
            elif entity.particle_fluid is not None:
                initial_positions = _entity_frame_vectors(
                    entity, entity.particle_fluid.initial_particle_positions_m, translate=True
                )
                initial_velocities = _entity_frame_vectors(
                    entity, entity.particle_fluid.initial_velocities(), translate=False
                )
                self._points[entity.path] = self._make_point_runtime(
                    entity,
                    initial_positions,
                    initial_velocities,
                    point_mass_kg=entity.particle_fluid.resolved_particle_mass_kg,
                    linear_damping_per_s=0.0,
                    kinematic_indices=frozenset(),
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

    def _make_point_runtime(
        self,
        entity: EntitySpec,
        initial_positions: list[list[float]],
        initial_velocities: list[list[float]],
        *,
        point_mass_kg: float,
        linear_damping_per_s: float,
        kinematic_indices: frozenset[int],
    ) -> _PointRuntime:
        environment_count = self._spec.environments.count
        point_count = len(initial_positions)
        positions = [_copy_vectors(initial_positions) for _ in range(environment_count)]
        velocities = [_copy_vectors(initial_velocities) for _ in range(environment_count)]
        modes = [[PointCommandMode.FORCE] * point_count for _ in range(environment_count)]
        targets = [[([0.0, 0.0, 0.0]) for _ in range(point_count)] for _ in range(environment_count)]
        for environment in range(environment_count):
            for point in kinematic_indices:
                modes[environment][point] = PointCommandMode.POSITION
                targets[environment][point] = initial_positions[point].copy()
                velocities[environment][point] = [0.0, 0.0, 0.0]
        return _PointRuntime(
            spec=entity,
            initial_positions=initial_positions,
            initial_velocities=initial_velocities,
            point_mass_kg=point_mass_kg,
            linear_damping_per_s=linear_damping_per_s,
            kinematic_indices=kinematic_indices,
            positions=positions,
            velocities=velocities,
            modes=modes,
            targets=targets,
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
        for articulation_runtime in self._articulations.values():
            initial = articulation_runtime.spec.initial_joint_positions
            for environment in environments:
                articulation_runtime.positions[environment] = list(initial)
                articulation_runtime.velocities[environment] = [0.0] * len(initial)
                articulation_runtime.modes[environment] = [CommandMode.POSITION] * len(initial)
                articulation_runtime.targets[environment] = list(initial)
        for rigid_runtime in self._rigids.values():
            for environment in environments:
                rigid_runtime.positions[environment] = rigid_runtime.initial_position.copy()
                rigid_runtime.orientations[environment] = rigid_runtime.initial_orientation.copy()
                rigid_runtime.linear_velocities[environment] = [0.0, 0.0, 0.0]
                rigid_runtime.angular_velocities[environment] = [0.0, 0.0, 0.0]
                rigid_runtime.forces[environment] = [0.0, 0.0, 0.0]
                rigid_runtime.torques[environment] = [0.0, 0.0, 0.0]
        for point_runtime in self._points.values():
            for environment in environments:
                point_runtime.positions[environment] = _copy_vectors(point_runtime.initial_positions)
                point_runtime.velocities[environment] = _copy_vectors(point_runtime.initial_velocities)
                point_runtime.modes[environment] = [PointCommandMode.FORCE] * len(point_runtime.initial_positions)
                point_runtime.targets[environment] = [
                    [0.0, 0.0, 0.0] for _ in range(len(point_runtime.initial_positions))
                ]
                for point in point_runtime.kinematic_indices:
                    point_runtime.modes[environment][point] = PointCommandMode.POSITION
                    point_runtime.targets[environment][point] = point_runtime.initial_positions[point].copy()
                    point_runtime.velocities[environment][point] = [0.0, 0.0, 0.0]
        reset_debug_keys = tuple(
            key
            for key, primitive in self._debug_primitives.items()
            if primitive.lifetime.mode is not DebugLifetimeMode.MANUAL
        )
        for key in reset_debug_keys:
            del self._debug_primitives[key]
            del self._debug_expirations[key]
        self._reset_count += 1
        self._scene_sequence += 1
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

    def apply_rigid_body_command(self, command: RigidBodyCommand) -> None:
        operation = "world.apply_rigid_body_command"
        self._ensure_ready(operation)
        if not isinstance(command, RigidBodyCommand):
            raise CommandError("operation requires a RigidBodyCommand", operation=operation)
        entity = self._validate_handle(command.handle, operation)
        if entity.kind is not EntityKind.RIGID_BODY:
            raise CommandError("entity is not a rigid body", operation=operation, entity_path=entity.path.value)
        environments = self._indices(
            command.environment_indices,
            self._spec.environments.count,
            "environment_indices",
            operation=operation,
        )
        expected_shape = (len(environments), 3)
        if command.forces_n.shape != expected_shape or command.torques_n_m.shape != expected_shape:
            raise CommandError(
                "rigid-body command shapes must exactly match selected environments and xyz",
                operation=operation,
                backend_id=self._session.descriptor.provider_id,
                world_id=self.world_id,
                entity_path=entity.path.value,
                details={
                    "expected_shape": list(expected_shape),
                    "force_shape": list(command.forces_n.shape),
                    "torque_shape": list(command.torques_n_m.shape),
                },
            )
        runtime = self._rigids[entity.path]
        forces = command.forces_n.rows()
        torques = command.torques_n_m.rows()
        for row, environment in enumerate(environments):
            runtime.forces[environment] = [float(value) for value in forces[row]]
            runtime.torques[environment] = [float(value) for value in torques[row]]

    def read_rigid_body(self, handle: EntityHandle) -> RigidBodyState:
        operation = "world.read_rigid_body"
        self._ensure_ready(operation)
        entity = self._validate_handle(handle, operation)
        if entity.kind is not EntityKind.RIGID_BODY:
            raise CommandError("entity is not a rigid body", operation=operation, entity_path=entity.path.value)
        runtime = self._rigids[entity.path]
        return RigidBodyState(
            positions_m=ArrayValue.from_rows(runtime.positions),
            orientations_xyzw=ArrayValue.from_rows(runtime.orientations),
            linear_velocities_m_s=ArrayValue.from_rows(runtime.linear_velocities),
            angular_velocities_rad_s=ArrayValue.from_rows(runtime.angular_velocities),
            tick=self.tick,
        )

    def read_contact(self, handle: EntityHandle, force_threshold_n: float = 1.0e-6) -> ContactState:
        operation = "world.read_contact"
        self._ensure_ready(operation)
        entity = self._validate_handle(handle, operation)
        if entity.kind is not EntityKind.RIGID_BODY:
            raise CommandError("entity is not a rigid body", operation=operation, entity_path=entity.path.value)
        try:
            threshold = float(force_threshold_n)
        except (TypeError, ValueError) as exc:
            raise ValidationError("force threshold must be numeric", operation=operation) from exc
        if not math.isfinite(threshold) or threshold < 0.0:
            raise ValidationError("force threshold must be finite and non-negative", operation=operation)
        environment_count = self._spec.environments.count
        return ContactState(
            net_normal_forces_n=ArrayValue.from_rows(((0.0, 0.0, 0.0),) * environment_count),
            in_contact=ArrayValue((environment_count,), (False,) * environment_count, dtype="bool"),
            tick=self.tick,
        )

    def _apply_point_command(
        self,
        *,
        handle: EntityHandle,
        mode: PointCommandMode,
        targets: ArrayValue,
        environment_indices: tuple[int, ...] | None,
        point_indices: tuple[int, ...] | None,
        accepted_kinds: frozenset[EntityKind],
        operation: str,
    ) -> None:
        entity = self._validate_handle(handle, operation)
        if entity.kind not in accepted_kinds:
            raise CommandError(
                "entity does not support this point operation",
                operation=operation,
                entity_path=entity.path.value,
                details={"entity_kind": entity.kind.value},
            )
        runtime = self._points[entity.path]
        environments = self._indices(
            environment_indices,
            self._spec.environments.count,
            "environment_indices",
            operation=operation,
        )
        points = self._indices(
            point_indices,
            len(runtime.initial_positions),
            "point_indices",
            operation=operation,
        )
        expected_shape = (len(environments), len(points), 3)
        if targets.shape != expected_shape:
            raise CommandError(
                "point target shape must exactly match selected environments and points",
                operation=operation,
                backend_id=self._session.descriptor.provider_id,
                world_id=self.world_id,
                entity_path=entity.path.value,
                details={"expected_shape": list(expected_shape), "actual_shape": list(targets.shape)},
            )
        selected_kinematic = runtime.kinematic_indices.intersection(points)
        if mode is not PointCommandMode.POSITION and selected_kinematic:
            raise CommandError(
                "kinematic deformable points only accept position commands",
                operation=operation,
                backend_id=self._session.descriptor.provider_id,
                world_id=self.world_id,
                entity_path=entity.path.value,
                details={"kinematic_indices": sorted(selected_kinematic), "mode": mode.value},
            )
        point_count = len(points)
        for row_index, environment in enumerate(environments):
            for column_index, point in enumerate(points):
                offset = (row_index * point_count + column_index) * 3
                runtime.modes[environment][point] = mode
                runtime.targets[environment][point] = [
                    float(targets.values[offset]),
                    float(targets.values[offset + 1]),
                    float(targets.values[offset + 2]),
                ]

    def apply_deformable_command(self, command: DeformableCommand) -> None:
        operation = "world.apply_deformable_command"
        self._ensure_ready(operation)
        if not isinstance(command, DeformableCommand):
            raise CommandError("operation requires a DeformableCommand", operation=operation)
        self._apply_point_command(
            handle=command.handle,
            mode=command.mode,
            targets=command.targets,
            environment_indices=command.environment_indices,
            point_indices=command.node_indices,
            accepted_kinds=frozenset({EntityKind.SURFACE_DEFORMABLE, EntityKind.VOLUME_DEFORMABLE}),
            operation=operation,
        )

    def read_deformable(self, handle: EntityHandle) -> DeformableState:
        operation = "world.read_deformable"
        self._ensure_ready(operation)
        entity = self._validate_handle(handle, operation)
        if entity.kind not in {EntityKind.SURFACE_DEFORMABLE, EntityKind.VOLUME_DEFORMABLE}:
            raise CommandError("entity is not a deformable", operation=operation, entity_path=entity.path.value)
        runtime = self._points[entity.path]
        return DeformableState(
            node_positions_m=ArrayValue.from_nested(runtime.positions),
            node_velocities_m_s=ArrayValue.from_nested(runtime.velocities),
            tick=self.tick,
        )

    def apply_particle_fluid_command(self, command: ParticleFluidCommand) -> None:
        operation = "world.apply_particle_fluid_command"
        self._ensure_ready(operation)
        if not isinstance(command, ParticleFluidCommand):
            raise CommandError("operation requires a ParticleFluidCommand", operation=operation)
        self._apply_point_command(
            handle=command.handle,
            mode=command.mode,
            targets=command.targets,
            environment_indices=command.environment_indices,
            point_indices=command.particle_indices,
            accepted_kinds=frozenset({EntityKind.PARTICLE_FLUID}),
            operation=operation,
        )

    def read_particle_fluid(self, handle: EntityHandle) -> ParticleFluidState:
        operation = "world.read_particle_fluid"
        self._ensure_ready(operation)
        entity = self._validate_handle(handle, operation)
        if entity.kind is not EntityKind.PARTICLE_FLUID:
            raise CommandError("entity is not a particle fluid", operation=operation, entity_path=entity.path.value)
        runtime = self._points[entity.path]
        return ParticleFluidState(
            particle_positions_m=ArrayValue.from_nested(runtime.positions),
            particle_velocities_m_s=ArrayValue.from_nested(runtime.velocities),
            tick=self.tick,
        )

    def read_sensor(self, handle: EntityHandle) -> SensorSample:
        operation = "world.read_sensor"
        self._ensure_ready(operation)
        entity = self._validate_handle(handle, operation)
        if entity.kind is not EntityKind.CAMERA_SENSOR or entity.camera is None:
            raise CommandError("entity is not a camera sensor", operation=operation, entity_path=entity.path.value)
        camera = entity.camera
        environment_count = self._spec.environments.count
        channels: list[SensorChannel] = []
        for modality in camera.modalities:
            if modality is CameraModality.RGB:
                rgb: list[int] = []
                for environment in range(environment_count):
                    for row in range(camera.height_px):
                        for column in range(camera.width_px):
                            rgb.extend(
                                (
                                    (column * 17 + environment * 31 + self._step_index) % 256,
                                    (row * 29 + environment * 13 + self._step_index * 3) % 256,
                                    (column * 7 + row * 11 + self._step_index * 5) % 256,
                                )
                            )
                data = ArrayValue(
                    (environment_count, camera.height_px, camera.width_px, 3),
                    tuple(rgb),
                    dtype="uint8",
                )
            else:
                depth = tuple(
                    min(
                        camera.far_plane_m,
                        max(
                            camera.near_plane_m,
                            1.0 + environment * 0.1 + row * 0.002 + column * 0.001,
                        ),
                    )
                    for environment in range(environment_count)
                    for row in range(camera.height_px)
                    for column in range(camera.width_px)
                )
                data = ArrayValue(
                    (environment_count, camera.height_px, camera.width_px),
                    depth,
                    dtype="float32",
                )
            channels.append(SensorChannel(modality, data))
        return SensorSample(handle=handle, channels=tuple(channels), tick=self.tick)

    def publish_debug(self, batch: DebugBatch) -> DebugPublishReport:
        operation = "world.publish_debug"
        self._ensure_ready(operation)
        if not isinstance(batch, DebugBatch):
            raise ValidationError("publish requires a DebugBatch", operation=operation)
        for primitive in batch.primitives:
            if any(index >= self._spec.environments.count for index in primitive.environment_indices):
                raise ValidationError(
                    "debug primitive contains an out-of-range environment",
                    operation=operation,
                    details={"environment_indices": list(primitive.environment_indices)},
                )
        for primitive in batch.primitives:
            self._debug_primitives[primitive.key] = primitive
            if primitive.lifetime.mode is DebugLifetimeMode.FRAME:
                expiration = self._step_index + 1
            elif primitive.lifetime.mode is DebugLifetimeMode.STEPS:
                assert primitive.lifetime.step_count is not None
                expiration = self._step_index + primitive.lifetime.step_count
            else:
                expiration = None
            self._debug_expirations[primitive.key] = expiration
        return DebugPublishReport(len(batch.primitives), 0, len(self._debug_primitives))

    def clear_debug(
        self,
        *,
        layer: str | None = None,
        group: str | None = None,
        primitive_id: str | None = None,
    ) -> int:
        operation = "world.clear_debug"
        self._ensure_ready(operation)
        for name, value in (("layer", layer), ("group", group), ("primitive_id", primitive_id)):
            if value is not None and (not isinstance(value, str) or not value):
                raise ValidationError(f"debug {name} must be a non-empty string", operation=operation)
        keys = tuple(
            key
            for key in self._debug_primitives
            if (layer is None or key[0] == layer)
            and (group is None or key[1] == group)
            and (primitive_id is None or key[2] == primitive_id)
        )
        for key in keys:
            del self._debug_primitives[key]
            del self._debug_expirations[key]
        return len(keys)

    def step(self, count: int = 1) -> Tick:
        self._ensure_ready("world.step")
        if not isinstance(count, int) or isinstance(count, bool) or count <= 0:
            raise ValidationError("step count must be a positive integer", operation="world.step")
        time_step = self._spec.physics.time_step_seconds
        for _ in range(count):
            for rigid_runtime in self._rigids.values():
                for environment in range(self._spec.environments.count):
                    linear_acceleration = [
                        rigid_runtime.forces[environment][axis] / rigid_runtime.mass_kg
                        + self._spec.physics.gravity_m_s2[axis]
                        for axis in range(3)
                    ]
                    angular_acceleration = [
                        value / rigid_runtime.mass_kg for value in rigid_runtime.torques[environment]
                    ]
                    rigid_runtime.linear_velocities[environment] = [
                        rigid_runtime.linear_velocities[environment][axis] + linear_acceleration[axis] * time_step
                        for axis in range(3)
                    ]
                    rigid_runtime.angular_velocities[environment] = [
                        rigid_runtime.angular_velocities[environment][axis] + angular_acceleration[axis] * time_step
                        for axis in range(3)
                    ]
                    rigid_runtime.positions[environment] = [
                        rigid_runtime.positions[environment][axis]
                        + rigid_runtime.linear_velocities[environment][axis] * time_step
                        for axis in range(3)
                    ]
                    rigid_runtime.orientations[environment] = _integrate_orientation_xyzw(
                        rigid_runtime.orientations[environment],
                        rigid_runtime.angular_velocities[environment],
                        time_step,
                    )
            for articulation_runtime in self._articulations.values():
                for environment in range(self._spec.environments.count):
                    for degree in range(len(articulation_runtime.spec.joint_names)):
                        articulation_mode = articulation_runtime.modes[environment][degree]
                        articulation_target = articulation_runtime.targets[environment][degree]
                        if articulation_mode is CommandMode.POSITION:
                            previous = articulation_runtime.positions[environment][degree]
                            articulation_runtime.positions[environment][degree] = articulation_target
                            articulation_runtime.velocities[environment][degree] = (
                                articulation_target - previous
                            ) / time_step
                        elif articulation_mode is CommandMode.VELOCITY:
                            articulation_runtime.velocities[environment][degree] = articulation_target
                            articulation_runtime.positions[environment][degree] += articulation_target * time_step
                        else:
                            articulation_runtime.velocities[environment][degree] += articulation_target * time_step
                            articulation_runtime.positions[environment][degree] += (
                                articulation_runtime.velocities[environment][degree] * time_step
                            )
            for point_runtime in self._points.values():
                damping_factor = max(0.0, 1.0 - point_runtime.linear_damping_per_s * time_step)
                for environment in range(self._spec.environments.count):
                    for point in range(len(point_runtime.initial_positions)):
                        point_mode = point_runtime.modes[environment][point]
                        point_target = point_runtime.targets[environment][point]
                        position = point_runtime.positions[environment][point]
                        velocity = point_runtime.velocities[environment][point]
                        if point_mode is PointCommandMode.POSITION:
                            point_runtime.positions[environment][point] = point_target.copy()
                            point_runtime.velocities[environment][point] = [
                                (point_target[axis] - position[axis]) / time_step for axis in range(3)
                            ]
                        elif point_mode is PointCommandMode.VELOCITY:
                            point_runtime.velocities[environment][point] = point_target.copy()
                            point_runtime.positions[environment][point] = [
                                position[axis] + point_target[axis] * time_step for axis in range(3)
                            ]
                        else:
                            acceleration = [
                                point_target[axis] / point_runtime.point_mass_kg + self._spec.physics.gravity_m_s2[axis]
                                for axis in range(3)
                            ]
                            next_velocity = [
                                (velocity[axis] + acceleration[axis] * time_step) * damping_factor for axis in range(3)
                            ]
                            point_runtime.velocities[environment][point] = next_velocity
                            point_runtime.positions[environment][point] = [
                                position[axis] + next_velocity[axis] * time_step for axis in range(3)
                            ]
            self._step_index += 1
            expired = tuple(
                key
                for key, expiration in self._debug_expirations.items()
                if expiration is not None and expiration <= self._step_index
            )
            for key in expired:
                del self._debug_primitives[key]
                del self._debug_expirations[key]
        self._scene_sequence += count
        return self.tick

    def _scene_visual(self, entity: EntitySpec) -> tuple[SceneVisual, ...]:
        if entity.kind is EntityKind.CAMERA_SENSOR:
            return (
                SceneVisual(
                    "camera",
                    SceneVisualKind.BOX,
                    dimensions_m=(0.18, 0.12, 0.1),
                    color_rgba=(0.2, 0.25, 0.32, 1.0),
                ),
            )
        if entity.kind in {
            EntityKind.SURFACE_DEFORMABLE,
            EntityKind.VOLUME_DEFORMABLE,
            EntityKind.PARTICLE_FLUID,
        }:
            return (
                SceneVisual(
                    "points",
                    SceneVisualKind.POINT_CLOUD,
                    dimensions_m=(0.6, 0.6, 0.6),
                    color_rgba=(0.15, 0.65, 1.0, 0.8),
                ),
            )
        if entity.kind is EntityKind.ARTICULATION:
            return (
                SceneVisual(
                    "body",
                    SceneVisualKind.BOX,
                    dimensions_m=(0.55, 0.45, 0.7),
                    color_rgba=(0.92, 0.49, 0.16, 1.0),
                ),
            )
        dimensions = (0.5, 0.5, 0.5) if entity.box is None else entity.box.dimensions_m
        color = (0.24, 0.72, 0.92, 1.0) if entity.box is None else entity.box.color_rgba
        return (
            SceneVisual(
                "body",
                SceneVisualKind.BOX,
                dimensions_m=dimensions,
                color_rgba=color,
            ),
        )

    def _scene_entities(self) -> tuple[SceneEntityState, ...]:
        entities: list[SceneEntityState] = []
        for entity in self._spec.entities:
            for environment in range(self._spec.environments.count):
                if entity.kind is EntityKind.RIGID_BODY:
                    runtime = self._rigids[entity.path]
                    position = runtime.positions[environment]
                    orientation = runtime.orientations[environment]
                    pose = Pose(
                        (position[0], position[1], position[2]),
                        (orientation[0], orientation[1], orientation[2], orientation[3]),
                    )
                    linear_values = runtime.linear_velocities[environment]
                    angular_values = runtime.angular_velocities[environment]
                    linear = (linear_values[0], linear_values[1], linear_values[2])
                    angular = (angular_values[0], angular_values[1], angular_values[2])
                    joints: tuple[float, ...] = ()
                elif entity.kind is EntityKind.ARTICULATION:
                    runtime_articulation = self._articulations[entity.path]
                    pose = entity.pose
                    linear = (0.0, 0.0, 0.0)
                    angular = (0.0, 0.0, 0.0)
                    joints = tuple(runtime_articulation.positions[environment])
                else:
                    pose = entity.pose
                    linear = (0.0, 0.0, 0.0)
                    angular = (0.0, 0.0, 0.0)
                    joints = ()
                entities.append(
                    SceneEntityState(
                        entity.path,
                        entity.kind,
                        environment,
                        pose,
                        linear,
                        angular,
                        entity.joint_names,
                        joints,
                        self._scene_visual(entity),
                        draggable=entity.kind is EntityKind.RIGID_BODY,
                    )
                )
        return tuple(entities)

    def scene_snapshot(self) -> SceneSnapshot:
        self._ensure_ready("world.scene_snapshot")
        return SceneSnapshot(
            self._session.descriptor.provider_id,
            self.world_id,
            self.generation,
            self._scene_sequence,
            self.tick,
            self._scene_entities(),
        )

    def scene_delta(self, base_sequence: int) -> SceneDelta:
        self._ensure_ready("world.scene_delta")
        if (
            not isinstance(base_sequence, int)
            or isinstance(base_sequence, bool)
            or base_sequence < 0
            or base_sequence > self._scene_sequence
        ):
            raise ValidationError("scene delta base sequence is invalid", operation="world.scene_delta")
        return SceneDelta(
            self.world_id,
            self.generation,
            base_sequence,
            self._scene_sequence,
            self.tick,
            () if base_sequence == self._scene_sequence else self._scene_entities(),
        )

    def _scene_result(
        self,
        command: SceneCommand,
        status: SceneCommandStatus,
        *,
        error_code: str | None = None,
        message: str | None = None,
    ) -> SceneCommandResult:
        result = SceneCommandResult(
            command.command_id,
            status,
            self.generation,
            self._scene_sequence,
            self.tick,
            error_code,
            message,
        )
        self._scene_results[command.command_id] = result
        if len(self._scene_results) > 4096:
            del self._scene_results[next(iter(self._scene_results))]
        return result

    def apply_scene_command(self, command: SceneCommand) -> SceneCommandResult:
        self._ensure_ready("world.apply_scene_command")
        if not isinstance(command, SceneCommand):
            raise ValidationError("operation requires a SceneCommand", operation="world.apply_scene_command")
        previous = self._scene_results.get(command.command_id)
        if previous is not None:
            return SceneCommandResult(
                command.command_id,
                SceneCommandStatus.DUPLICATE,
                previous.generation,
                previous.scene_sequence,
                previous.tick,
                message="original command result already recorded",
            )
        if command.expected_generation != self.generation:
            return self._scene_result(
                command,
                SceneCommandStatus.REJECTED,
                error_code="stale_generation",
                message="command generation does not match the live world",
            )
        entity = self._entities.get(command.entity_path)
        if entity is None or command.environment_index >= self._spec.environments.count:
            return self._scene_result(
                command,
                SceneCommandStatus.REJECTED,
                error_code="target_not_found",
                message="entity or environment does not exist",
            )
        if entity.kind is not EntityKind.RIGID_BODY:
            return self._scene_result(
                command,
                SceneCommandStatus.REJECTED,
                error_code="unsupported_entity_kind",
                message="the fake adapter exposes scene manipulation only for rigid bodies",
            )
        runtime = self._rigids[entity.path]
        environment = command.environment_index
        if command.kind is SceneCommandKind.SET_POSE:
            assert command.target_pose is not None
            self._set_rigid_pose(runtime, environment, command.target_pose)
        elif command.kind is SceneCommandKind.DRAG_BEGIN:
            assert command.drag_id is not None
            if command.drag_mode is not SceneDragMode.KINEMATIC:
                return self._scene_result(
                    command,
                    SceneCommandStatus.REJECTED,
                    error_code="unsupported_drag_mode",
                    message="the fake adapter supports only explicit kinematic drag",
                )
            if command.drag_id in self._active_drags:
                return self._scene_result(
                    command,
                    SceneCommandStatus.REJECTED,
                    error_code="drag_exists",
                    message="drag ID is already active",
                )
            self._active_drags[command.drag_id] = (
                entity.path,
                environment,
                Pose(
                    (
                        runtime.positions[environment][0],
                        runtime.positions[environment][1],
                        runtime.positions[environment][2],
                    ),
                    (
                        runtime.orientations[environment][0],
                        runtime.orientations[environment][1],
                        runtime.orientations[environment][2],
                        runtime.orientations[environment][3],
                    ),
                ),
            )
        else:
            assert command.drag_id is not None
            active = self._active_drags.get(command.drag_id)
            if active is None or active[:2] != (entity.path, environment):
                return self._scene_result(
                    command,
                    SceneCommandStatus.REJECTED,
                    error_code="drag_not_active",
                    message="drag transaction is missing or targets a different entity",
                )
            if command.kind is SceneCommandKind.DRAG_UPDATE:
                assert command.target_pose is not None
                self._set_rigid_pose(runtime, environment, command.target_pose)
            elif command.kind is SceneCommandKind.DRAG_CANCEL:
                self._set_rigid_pose(runtime, environment, active[2])
                del self._active_drags[command.drag_id]
            else:
                del self._active_drags[command.drag_id]
        self._scene_sequence += 1
        return self._scene_result(command, SceneCommandStatus.APPLIED)

    @staticmethod
    def _set_rigid_pose(runtime: _RigidRuntime, environment: int, pose: Pose) -> None:
        runtime.positions[environment] = list(pose.position)
        runtime.orientations[environment] = list(pose.orientation_xyzw)
        runtime.linear_velocities[environment] = [0.0, 0.0, 0.0]
        runtime.angular_velocities[environment] = [0.0, 0.0, 0.0]

    def _close(self, *, notify_session: bool) -> None:
        if self._state is WorldState.CLOSED:
            return
        self._state = WorldState.CLOSED
        self._entities.clear()
        self._articulations.clear()
        self._rigids.clear()
        self._points.clear()
        self._debug_primitives.clear()
        self._debug_expirations.clear()
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
