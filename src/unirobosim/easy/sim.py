"""Backend-neutral Easy API without a second lifecycle or hidden native behavior."""

from __future__ import annotations

from collections.abc import Iterable, Mapping, Sequence
from dataclasses import replace
from enum import StrEnum
from importlib import metadata
from os import PathLike
from typing import Any, cast

from unirobosim.api.capabilities import CapabilityId, CapabilityRequirement
from unirobosim.api.debug import DebugBatch, DebugPrimitive, DebugPublishReport
from unirobosim.api.errors import (
    AssetConversionError,
    LifecycleError,
    ProviderSelectionError,
    UnsupportedCapabilityError,
    ValidationError,
)
from unirobosim.api.frozen import FrozenMap
from unirobosim.api.protocols import Provider, SceneControlWorld, Session, World
from unirobosim.api.reports import (
    ArticulationState,
    BuildReport,
    ContactState,
    DeformableState,
    ParticleFluidState,
    ProviderDescriptor,
    ResetResult,
    RigidBodyState,
    SensorSample,
)
from unirobosim.api.scene import SceneSnapshot
from unirobosim.api.specs import (
    ArticulationCommand,
    BoxGeometrySpec,
    CameraSpec,
    DeformableBodySpec,
    DeformableCommand,
    EntitySpec,
    EnvironmentSpec,
    ParticleFluidCommand,
    ParticleFluidSpec,
    PhysicsSpec,
    RigidBodyCommand,
    WorldSpec,
)
from unirobosim.api.values import (
    ArrayValue,
    CameraModality,
    CommandMode,
    DeformableTopology,
    EntityHandle,
    EntityKind,
    EntityPath,
    PointCommandMode,
    Pose,
    Tick,
)

from .assets import AssetBundle, ResolvedAsset, infer_media_type
from .conversion import (
    AssetConversionRequest,
    AssetConverter,
    AssetPolicy,
    default_asset_cache_directory,
    discover_asset_converters,
    select_asset_converter,
)


class SimState(StrEnum):
    CONFIGURING = "configuring"
    RUNNING = "running"
    CLOSED = "closed"


def _invalid(message: str, operation: str) -> ValidationError:
    return ValidationError(message, operation=operation)


def _path(name: str) -> EntityPath:
    if not isinstance(name, str) or not name:
        raise _invalid("entity name must be a non-empty string", "easy.entity.path")
    return EntityPath(name if name.startswith("/") else f"/{name}")


def _pose(position: Sequence[float], orientation_xyzw: Sequence[float]) -> Pose:
    return Pose(tuple(position), tuple(orientation_xyzw))  # type: ignore[arg-type]


def _rows(values: object, width: int, rows: int, operation: str) -> ArrayValue:
    if isinstance(values, (str, bytes)):
        raise _invalid("targets must be numeric values", operation)
    try:
        sequence = tuple(cast(Iterable[Any], values))
    except TypeError as exc:
        raise _invalid("targets must be iterable", operation) from exc
    if len(sequence) == width and all(
        isinstance(value, (int, float)) and not isinstance(value, bool) for value in sequence
    ):
        normalized = tuple(tuple(float(value) for value in sequence) for _ in range(rows))
    else:
        try:
            normalized = tuple(tuple(float(value) for value in cast(Iterable[Any], row)) for row in sequence)
        except (TypeError, ValueError) as exc:
            raise _invalid("targets must be a vector or matrix", operation) from exc
    if len(normalized) != rows or any(len(row) != width for row in normalized):
        raise _invalid(f"targets must have shape [{rows}, {width}]", operation)
    return ArrayValue.from_rows(normalized)


def _point_targets(values: object, points: int, environments: int, operation: str) -> ArrayValue:
    """Normalize vec3, point×xyz, or environment×point×xyz targets."""

    if isinstance(values, (str, bytes)):
        raise _invalid("targets must be numeric values", operation)
    try:
        candidate = ArrayValue.from_nested(values)
    except (TypeError, ValueError, ValidationError) as exc:
        raise _invalid("targets must be a vector, point matrix, or environment batch", operation) from exc
    if candidate.shape == (3,):
        nested = tuple(tuple(tuple(candidate.values) for _ in range(points)) for _ in range(environments))
    elif candidate.shape == (points, 3):
        point_rows = candidate.nested()
        nested = tuple(point_rows for _ in range(environments))
    elif candidate.shape == (environments, points, 3):
        return candidate
    else:
        raise _invalid(f"targets must have shape [3], [{points}, 3], or [{environments}, {points}, 3]", operation)
    return ArrayValue.from_nested(nested)


def _entry_points() -> tuple[metadata.EntryPoint, ...]:
    return tuple(sorted(metadata.entry_points(group="unirobosim.backends"), key=lambda item: item.name))


def _load_provider(backend: str, requirements: tuple[CapabilityRequirement, ...]) -> Provider:
    candidates = _entry_points()
    if backend != "auto":
        named = tuple(item for item in candidates if item.name == backend)
        candidates = named or candidates
    attempts: list[dict[str, object]] = []
    for entry_point in candidates:
        try:
            factory = entry_point.load()
            provider = factory()
            if not isinstance(provider, Provider):
                raise TypeError("entry point did not return a Provider")
            if backend != "auto" and entry_point.name != backend and provider.descriptor.provider_id != backend:
                continue
            probe = provider.probe()
            negotiation = provider.descriptor.capabilities.negotiate(requirements)
            attempts.append(
                {
                    "entry_point": entry_point.name,
                    "provider_id": provider.descriptor.provider_id,
                    "available": probe.available,
                    "reason": probe.reason,
                    "negotiation": negotiation.to_dict(),
                }
            )
            if probe.available and negotiation.accepted:
                return provider
        except Exception as exc:
            attempts.append({"entry_point": entry_point.name, "error": f"{type(exc).__name__}: {exc}"})
    raise ProviderSelectionError(
        "no installed backend satisfies this Easy API scene",
        operation="easy.sim.select_provider",
        backend_id=None if backend == "auto" else backend,
        details={"installed_entry_points": [item.name for item in _entry_points()], "attempts": attempts},
    )


class Entity:
    def __init__(self, sim: Sim, spec: EntitySpec) -> None:
        self._sim = sim
        self._spec = spec

    @property
    def path(self) -> EntityPath:
        return self._spec.path

    @property
    def kind(self) -> EntityKind:
        return self._spec.kind

    @property
    def handle(self) -> EntityHandle:
        return self._sim.world.resolve(self.path)


class RigidBody(Entity):
    @property
    def state(self) -> RigidBodyState:
        return self._sim.world.read_rigid_body(self.handle)

    def apply_wrench(
        self,
        force_n: object,
        torque_n_m: object = (0.0, 0.0, 0.0),
        *,
        environments: Iterable[int] | None = None,
    ) -> None:
        selected = None if environments is None else tuple(environments)
        row_count = self._sim.num_envs if selected is None else len(selected)
        self._sim.world.apply_rigid_body_command(
            RigidBodyCommand(
                self.handle,
                _rows(force_n, 3, row_count, "easy.rigid.apply_wrench"),
                _rows(torque_n_m, 3, row_count, "easy.rigid.apply_wrench"),
                selected,
            )
        )

    def contact(self, *, force_threshold_n: float = 1.0e-6) -> ContactState:
        return self._sim.world.read_contact(self.handle, force_threshold_n)


class Articulation(Entity):
    @property
    def joint_names(self) -> tuple[str, ...]:
        return self._spec.joint_names

    @property
    def num_joints(self) -> int:
        return len(self.joint_names)

    @property
    def state(self) -> ArticulationState:
        return self._sim.world.read_articulation(self.handle)

    def command(
        self,
        targets: object,
        *,
        mode: CommandMode | str = CommandMode.POSITION,
        joints: Iterable[str | int] | None = None,
        environments: Iterable[int] | None = None,
    ) -> None:
        try:
            resolved_mode = mode if isinstance(mode, CommandMode) else CommandMode(mode)
        except ValueError as exc:
            raise _invalid("mode must be position, velocity, or effort", "easy.articulation.command") from exc
        degrees = (
            tuple(range(self.num_joints)) if joints is None else tuple(self._joint_index(value) for value in joints)
        )
        selected = None if environments is None else tuple(environments)
        row_count = self._sim.num_envs if selected is None else len(selected)
        self._sim.world.apply_articulation_command(
            ArticulationCommand(
                self.handle,
                resolved_mode,
                _rows(targets, len(degrees), row_count, "easy.articulation.command"),
                selected,
                degrees,
            )
        )

    def _joint_index(self, value: str | int) -> int:
        if isinstance(value, bool):
            raise _invalid("joint selection is invalid", "easy.articulation.command")
        if isinstance(value, int):
            if 0 <= value < self.num_joints:
                return value
        elif isinstance(value, str) and value in self.joint_names:
            return self.joint_names.index(value)
        raise _invalid(f"unknown joint: {value}", "easy.articulation.command")


class Camera(Entity):
    def sample(self) -> SensorSample:
        return self._sim.world.read_sensor(self.handle)

    def read(self, modality: CameraModality | str) -> ArrayValue:
        try:
            resolved = modality if isinstance(modality, CameraModality) else CameraModality(modality)
        except ValueError as exc:
            raise _invalid("camera modality must be rgb or depth", "easy.camera.read") from exc
        return self.sample().channel(resolved)


class Deformable(Entity):
    @property
    def num_nodes(self) -> int:
        assert self._spec.deformable is not None
        return self._spec.deformable.node_count

    @property
    def state(self) -> DeformableState:
        return self._sim.world.read_deformable(self.handle)

    def command(
        self,
        targets: object,
        *,
        mode: PointCommandMode | str = PointCommandMode.POSITION,
        nodes: Iterable[int] | None = None,
        environments: Iterable[int] | None = None,
    ) -> None:
        resolved_mode = _point_mode(mode, "easy.deformable.command")
        self._sim._require_command_mode("control.deformable.points@1", resolved_mode.value, "easy.deformable.command")
        selected_nodes = tuple(range(self.num_nodes)) if nodes is None else tuple(nodes)
        selected_envs = None if environments is None else tuple(environments)
        environment_count = self._sim.num_envs if selected_envs is None else len(selected_envs)
        self._sim.world.apply_deformable_command(
            DeformableCommand(
                self.handle,
                resolved_mode,
                _point_targets(targets, len(selected_nodes), environment_count, "easy.deformable.command"),
                selected_envs,
                selected_nodes,
            )
        )


class ParticleFluid(Entity):
    @property
    def num_particles(self) -> int:
        assert self._spec.particle_fluid is not None
        return self._spec.particle_fluid.particle_count

    @property
    def state(self) -> ParticleFluidState:
        return self._sim.world.read_particle_fluid(self.handle)

    def command(
        self,
        targets: object,
        *,
        mode: PointCommandMode | str = PointCommandMode.POSITION,
        particles: Iterable[int] | None = None,
        environments: Iterable[int] | None = None,
    ) -> None:
        resolved_mode = _point_mode(mode, "easy.particle_fluid.command")
        self._sim._require_command_mode("control.fluid.particles@1", resolved_mode.value, "easy.particle_fluid.command")
        selected_particles = tuple(range(self.num_particles)) if particles is None else tuple(particles)
        selected_envs = None if environments is None else tuple(environments)
        environment_count = self._sim.num_envs if selected_envs is None else len(selected_envs)
        self._sim.world.apply_particle_fluid_command(
            ParticleFluidCommand(
                self.handle,
                resolved_mode,
                _point_targets(
                    targets,
                    len(selected_particles),
                    environment_count,
                    "easy.particle_fluid.command",
                ),
                selected_envs,
                selected_particles,
            )
        )


def _point_mode(mode: PointCommandMode | str, operation: str) -> PointCommandMode:
    try:
        return mode if isinstance(mode, PointCommandMode) else PointCommandMode(mode)
    except ValueError as exc:
        raise _invalid("mode must be position, velocity, or force", operation) from exc


class EasyDebug:
    def __init__(self, sim: Sim) -> None:
        self._sim = sim

    def publish(self, *primitives: DebugPrimitive) -> DebugPublishReport:
        return self._sim.world.publish_debug(DebugBatch(tuple(primitives)))

    def clear(
        self,
        *,
        layer: str | None = None,
        group: str | None = None,
        primitive_id: str | None = None,
    ) -> int:
        return self._sim.world.clear_debug(layer=layer, group=group, primitive_id=primitive_id)


class Sim:
    """Concise scene builder and view factory backed by one strict Runtime world."""

    def __init__(
        self,
        *,
        backend: str = "auto",
        provider: Provider | None = None,
        world_id: str = "easy",
        num_envs: int = 1,
        time_step_seconds: float = 1.0 / 60.0,
        gravity_m_s2: Sequence[float] = (0.0, 0.0, -9.81),
        headless: bool = True,
        asset_policy: AssetPolicy | str = AssetPolicy.CONVERT_IF_NEEDED,
        asset_cache_directory: str | PathLike[str] | None = None,
        asset_converters: Iterable[AssetConverter] | None = None,
    ) -> None:
        if not isinstance(backend, str) or not backend:
            raise _invalid("backend must be a non-empty string", "easy.sim.init")
        if not isinstance(provider, Provider | type(None)):
            raise _invalid("provider must satisfy the Provider protocol", "easy.sim.init")
        if headless is not True:
            raise _invalid("the current Easy API compatibility profile is headless", "easy.sim.init")
        try:
            resolved_policy = asset_policy if isinstance(asset_policy, AssetPolicy) else AssetPolicy(asset_policy)
        except ValueError as exc:
            raise _invalid("asset_policy must be prebuilt_only or convert_if_needed", "easy.sim.init") from exc
        if asset_cache_directory is not None and not isinstance(asset_cache_directory, (str, PathLike)):
            raise _invalid("asset_cache_directory must be a non-empty path", "easy.sim.init")
        asset_cache_path = None if asset_cache_directory is None else str(asset_cache_directory)
        if asset_cache_path is not None and not asset_cache_path.strip():
            raise _invalid("asset_cache_directory must be a non-empty path", "easy.sim.init")
        configured_converters = None if asset_converters is None else tuple(asset_converters)
        if configured_converters is not None and any(
            not isinstance(converter, AssetConverter) for converter in configured_converters
        ):
            raise _invalid("asset_converters must satisfy AssetConverter", "easy.sim.init")
        self._backend = backend
        self._provider = provider
        self._world_id = world_id
        self._environments = EnvironmentSpec(num_envs)
        gravity = cast(tuple[float, float, float], tuple(gravity_m_s2))
        self._physics = PhysicsSpec(time_step_seconds, gravity_m_s2=gravity)
        self._state = SimState.CONFIGURING
        self._entities: dict[EntityPath, Entity] = {}
        self._asset_bundles: dict[EntityPath, AssetBundle] = {}
        self._asset_conversion_options: dict[EntityPath, FrozenMap] = {}
        self._asset_policy = resolved_policy
        self._asset_cache_directory = asset_cache_path or default_asset_cache_directory()
        self._asset_converters = configured_converters
        self._requirements: dict[CapabilityId, CapabilityRequirement] = {}
        self._session: Session | None = None
        self._world: World | None = None
        self._world_spec: WorldSpec | None = None
        self.debug = EasyDebug(self)

    @property
    def state(self) -> SimState:
        return self._state

    @property
    def num_envs(self) -> int:
        return self._environments.count

    @property
    def world(self) -> World:
        if self._state is not SimState.RUNNING or self._world is None:
            raise LifecycleError("simulation has not been started", operation="easy.sim.world")
        return self._world

    @property
    def world_spec(self) -> WorldSpec:
        if self._world_spec is None:
            raise LifecycleError("world spec is not compiled until start", operation="easy.sim.world_spec")
        return self._world_spec

    @property
    def provider_descriptor(self) -> ProviderDescriptor:
        if self._provider is None:
            raise LifecycleError("provider is not selected until start", operation="easy.sim.provider_descriptor")
        return self._provider.descriptor

    @property
    def build_report(self) -> BuildReport:
        return self.world.build_report

    def _configuring(self, operation: str) -> None:
        if self._state is not SimState.CONFIGURING:
            raise LifecycleError("scene can only be edited before start", operation=operation)

    def _add(self, entity: Entity) -> Entity:
        self._configuring("easy.sim.add")
        if entity.path in self._entities:
            raise _invalid(f"duplicate entity path: {entity.path}", "easy.sim.add")
        self._entities[entity.path] = entity
        return entity

    def add_box(
        self,
        name: str,
        *,
        size_m: float | Sequence[float] = 0.5,
        mass_kg: float = 1.0,
        color_rgba: Sequence[float] = (0.15, 0.7, 0.95, 1.0),
        static_friction: float = 1.0,
        dynamic_friction: float = 1.0,
        restitution: float = 0.0,
        position_m: Sequence[float] = (0.0, 0.0, 0.5),
        orientation_xyzw: Sequence[float] = (0.0, 0.0, 0.0, 1.0),
    ) -> RigidBody:
        dimensions = (float(size_m),) * 3 if isinstance(size_m, (int, float)) else tuple(size_m)
        spec = EntitySpec(
            _path(name),
            EntityKind.RIGID_BODY,
            pose=_pose(position_m, orientation_xyzw),
            box=BoxGeometrySpec(
                dimensions_m=dimensions,  # type: ignore[arg-type]
                mass_kg=mass_kg,
                color_rgba=tuple(color_rgba),  # type: ignore[arg-type]
                static_friction=static_friction,
                dynamic_friction=dynamic_friction,
                restitution=restitution,
            ),
        )
        return cast(RigidBody, self._add(RigidBody(self, spec)))

    def add_rigid_body(
        self,
        name: str,
        *,
        asset_uri: str | None = None,
        asset: AssetBundle | None = None,
        conversion_options: Mapping[str, object] | None = None,
        position_m: Sequence[float] = (0.0, 0.0, 0.0),
        orientation_xyzw: Sequence[float] = (0.0, 0.0, 0.0, 1.0),
    ) -> RigidBody:
        """Add a backend-native rigid asset while preserving the portable entity view."""

        path = _path(name)
        if (asset_uri is None) == (asset is None):
            raise _invalid("provide exactly one of asset_uri or asset", "easy.sim.add_rigid_body")
        spec = EntitySpec(
            path,
            EntityKind.RIGID_BODY,
            pose=_pose(position_m, orientation_xyzw),
            asset_uri=asset_uri,
        )
        body = cast(RigidBody, self._add(RigidBody(self, spec)))
        if asset is not None:
            self._asset_bundles[path] = asset
        if conversion_options is not None:
            self._asset_conversion_options[path] = FrozenMap(conversion_options)
        return body

    def add_articulation(
        self,
        name: str,
        *,
        joint_names: Iterable[str],
        initial_positions: Iterable[float] = (),
        joint_effort_limits: Iterable[float] = (),
        position_m: Sequence[float] = (0.0, 0.0, 0.0),
        orientation_xyzw: Sequence[float] = (0.0, 0.0, 0.0, 1.0),
        asset_uri: str | None = None,
        asset: AssetBundle | None = None,
    ) -> Articulation:
        if asset_uri is not None and asset is not None:
            raise _invalid("asset_uri and asset are mutually exclusive", "easy.sim.add_articulation")
        path = _path(name)
        spec = EntitySpec(
            path,
            EntityKind.ARTICULATION,
            pose=_pose(position_m, orientation_xyzw),
            joint_names=tuple(joint_names),
            initial_joint_positions=tuple(initial_positions),
            joint_effort_limits=tuple(joint_effort_limits),
            asset_uri=asset_uri,
        )
        articulation = cast(Articulation, self._add(Articulation(self, spec)))
        if asset is not None:
            self._asset_bundles[path] = asset
        return articulation

    def add_camera(
        self,
        name: str,
        *,
        resolution: tuple[int, int] = (640, 480),
        outputs: Iterable[CameraModality | str] = (CameraModality.RGB, CameraModality.DEPTH),
        position_m: Sequence[float] = (2.0, 0.0, 1.5),
        orientation_xyzw: Sequence[float] = (0.0, 0.7071067811865475, 0.0, 0.7071067811865475),
    ) -> Camera:
        try:
            modalities = tuple(
                value if isinstance(value, CameraModality) else CameraModality(value) for value in outputs
            )
        except ValueError as exc:
            raise _invalid("camera outputs must contain rgb or depth", "easy.sim.add_camera") from exc
        spec = EntitySpec(
            _path(name),
            EntityKind.CAMERA_SENSOR,
            pose=_pose(position_m, orientation_xyzw),
            camera=CameraSpec(resolution[0], resolution[1], modalities),
        )
        return cast(Camera, self._add(Camera(self, spec)))

    def add_deformable(
        self,
        name: str,
        *,
        rest_positions_m: object,
        topology: DeformableTopology | str = DeformableTopology.SURFACE,
        surface_triangles: object | None = None,
        tetrahedra: object | None = None,
        initial_velocities_m_s: object | None = None,
        kinematic_nodes: Iterable[int] = (),
        node_mass_kg: float = 1.0,
        linear_damping_per_s: float = 0.0,
        self_collision: bool = False,
        position_m: Sequence[float] = (0.0, 0.0, 0.0),
        orientation_xyzw: Sequence[float] = (0.0, 0.0, 0.0, 1.0),
    ) -> Deformable:
        try:
            resolved_topology = topology if isinstance(topology, DeformableTopology) else DeformableTopology(topology)
        except ValueError as exc:
            raise _invalid("topology must be surface or volume", "easy.sim.add_deformable") from exc
        deformable = DeformableBodySpec(
            resolved_topology,
            ArrayValue.from_nested(rest_positions_m),
            None if surface_triangles is None else ArrayValue.from_nested(surface_triangles, dtype="int64"),
            None if tetrahedra is None else ArrayValue.from_nested(tetrahedra, dtype="int64"),
            None if initial_velocities_m_s is None else ArrayValue.from_nested(initial_velocities_m_s),
            tuple(kinematic_nodes),
            node_mass_kg,
            linear_damping_per_s,
            self_collision,
        )
        kind = (
            EntityKind.SURFACE_DEFORMABLE
            if resolved_topology is DeformableTopology.SURFACE
            else EntityKind.VOLUME_DEFORMABLE
        )
        spec = EntitySpec(
            _path(name),
            kind,
            pose=_pose(position_m, orientation_xyzw),
            deformable=deformable,
        )
        self._ensure_required(
            "control.deformable.points@1",
            "Easy API deformable point commands",
        )
        return cast(Deformable, self._add(Deformable(self, spec)))

    def add_particle_fluid(
        self,
        name: str,
        *,
        positions_m: object,
        initial_velocities_m_s: object | None = None,
        particle_radius_m: float = 0.01,
        rest_density_kg_m3: float = 1000.0,
        particle_mass_kg: float | None = None,
        dynamic_viscosity_pa_s: float = 0.001,
        surface_tension_n_m: float = 0.072,
        position_m: Sequence[float] = (0.0, 0.0, 0.0),
        orientation_xyzw: Sequence[float] = (0.0, 0.0, 0.0, 1.0),
    ) -> ParticleFluid:
        fluid = ParticleFluidSpec(
            ArrayValue.from_nested(positions_m),
            None if initial_velocities_m_s is None else ArrayValue.from_nested(initial_velocities_m_s),
            particle_radius_m,
            rest_density_kg_m3,
            particle_mass_kg,
            dynamic_viscosity_pa_s,
            surface_tension_n_m,
        )
        spec = EntitySpec(
            _path(name),
            EntityKind.PARTICLE_FLUID,
            pose=_pose(position_m, orientation_xyzw),
            particle_fluid=fluid,
        )
        self._ensure_required(
            "control.fluid.particles@1",
            "Easy API particle-fluid point commands",
        )
        return cast(ParticleFluid, self._add(ParticleFluid(self, spec)))

    def require(self, capability: str, *, reason: str | None = None) -> Sim:
        return self._require(capability, required=True, reason=reason)

    def optional(self, capability: str, *, reason: str | None = None) -> Sim:
        return self._require(capability, required=False, reason=reason)

    def _require(self, capability: str, *, required: bool, reason: str | None) -> Sim:
        self._configuring("easy.sim.require")
        requirement = CapabilityRequirement(CapabilityId(capability), required=required, reason=reason)
        if requirement.capability in self._requirements:
            raise _invalid(f"duplicate capability: {capability}", "easy.sim.require")
        self._requirements[requirement.capability] = requirement
        return self

    def _ensure_required(self, capability: str, reason: str) -> None:
        capability_id = CapabilityId(capability)
        if capability_id not in self._requirements:
            self._requirements[capability_id] = CapabilityRequirement(capability_id, reason=reason)

    def _require_command_mode(self, capability: str, mode: str, operation: str) -> None:
        declaration = self.provider_descriptor.capabilities.get(CapabilityId(capability))
        supported = None if declaration is None else declaration.properties.get("modes")
        if supported is not None and mode not in supported:
            raise UnsupportedCapabilityError(
                f"backend does not support {mode!r} for {capability}",
                operation=operation,
                backend_id=self.provider_descriptor.provider_id,
                world_id=self.world_spec.world_id,
                details={"requested_mode": mode, "supported_modes": supported},
            )

    def start(self) -> BuildReport:
        self._configuring("easy.sim.start")
        if not self._entities:
            raise _invalid("add at least one entity before start", "easy.sim.start")
        requirements = tuple(self._requirements.values())
        provider = self._provider or _load_provider(self._backend, requirements)
        resolved_entities: list[EntitySpec] = []
        for entity in self._entities.values():
            bundle = self._asset_bundles.get(entity.path)
            if bundle is None and entity._spec.asset_uri is None:
                resolved_entities.append(entity._spec)
                continue
            resolved = self._resolve_asset(entity._spec, bundle, provider)
            metadata_values = entity._spec.metadata.to_dict()
            metadata_values["unirobosim_asset"] = resolved.to_dict()
            resolved_entities.append(
                replace(
                    entity._spec,
                    asset_uri=resolved.uri,
                    metadata=FrozenMap(metadata_values),
                )
            )
        spec = WorldSpec(
            self._world_id,
            tuple(resolved_entities),
            physics=self._physics,
            environments=self._environments,
            requirements=requirements,
        )
        session = provider.open()
        try:
            world = session.build(spec)
        except Exception:
            session.close()
            raise
        self._provider = provider
        self._session = session
        self._world = world
        self._world_spec = spec
        self._state = SimState.RUNNING
        return world.build_report

    @staticmethod
    def _supported_asset_media_types(provider: Provider, kind: EntityKind) -> tuple[str, ...] | None:
        declaration = provider.descriptor.capabilities.get(CapabilityId("asset.formats@1"))
        if declaration is None:
            return None
        raw = declaration.properties.get(kind.value)
        if raw is None:
            return ()
        if not isinstance(raw, tuple) or any(not isinstance(value, str) for value in raw):
            raise AssetConversionError(
                "provider asset format declaration is invalid",
                operation="easy.asset.formats",
                backend_id=provider.descriptor.provider_id,
                details={"entity_kind": kind.value, "declared": raw},
            )
        return raw

    def _resolve_asset(
        self,
        entity: EntitySpec,
        bundle: AssetBundle | None,
        provider: Provider,
    ) -> ResolvedAsset:
        if bundle is None:
            assert entity.asset_uri is not None
            source = ResolvedAsset(
                logical_name=entity.path.value,
                selector="direct",
                uri=entity.asset_uri,
                media_type=infer_media_type(entity.asset_uri),
            )
        else:
            try:
                source = bundle.resolve(backend=self._backend, provider_id=provider.descriptor.provider_id)
            except ValidationError as native_resolution_error:
                if self._asset_policy is AssetPolicy.PREBUILT_ONLY:
                    raise
                try:
                    source = bundle.source_for_conversion()
                except ValidationError:
                    # Preserve the primary error when the bundle contains neither
                    # a native variant nor a canonical USD conversion source.
                    raise native_resolution_error from None

        supported = self._supported_asset_media_types(provider, entity.kind)
        if supported is None or source.media_type in supported:
            return source
        if self._asset_policy is AssetPolicy.PREBUILT_ONLY:
            raise AssetConversionError(
                "selected provider does not accept this asset format and conversion is disabled",
                operation="easy.asset.resolve",
                backend_id=provider.descriptor.provider_id,
                entity_path=entity.path.value,
                details={"source": source.to_dict(), "supported_media_types": supported},
            )

        request = AssetConversionRequest(
            source_uri=source.uri,
            source_media_type=source.media_type,
            target_backend=self._backend,
            provider_id=provider.descriptor.provider_id,
            entity_kind=entity.kind,
            cache_directory=self._asset_cache_directory,
            options=self._asset_conversion_options.get(entity.path, FrozenMap()),
        )
        converters = self._asset_converters
        if converters is None:
            converters = discover_asset_converters()
            self._asset_converters = converters
        converter = select_asset_converter(request, converters)
        try:
            converted = converter.convert(request)
        except AssetConversionError:
            raise
        except Exception as exc:
            raise AssetConversionError(
                "asset converter failed",
                operation="easy.asset.convert",
                backend_id=provider.descriptor.provider_id,
                entity_path=entity.path.value,
                details={"converter_id": converter.converter_id, "request": request.to_dict()},
                cause=exc,
            ) from exc
        if converted.media_type not in supported:
            raise AssetConversionError(
                "asset converter returned a format unsupported by the provider",
                operation="easy.asset.convert",
                backend_id=provider.descriptor.provider_id,
                entity_path=entity.path.value,
                details={"conversion": converted.to_dict(), "supported_media_types": supported},
            )
        return ResolvedAsset(
            logical_name=source.logical_name,
            selector=f"converted:{converted.converter_id}",
            uri=converted.uri,
            media_type=converted.media_type,
            sha256=converted.output_sha256,
            source_manifest=source.source_manifest,
            conversion=FrozenMap(converted.to_dict()),
        )

    def reset(self, environments: Iterable[int] | None = None) -> ResetResult:
        return self.world.reset(environments)

    def step(self, count: int = 1) -> Tick:
        return self.world.step(count)

    def entity(self, name: str) -> Entity:
        path = _path(name)
        entity = self._entities.get(path)
        if entity is None:
            raise _invalid(f"unknown entity: {path}", "easy.sim.entity")
        return entity

    def scene_snapshot(self) -> SceneSnapshot:
        world = self.world
        if not isinstance(world, SceneControlWorld):
            raise LifecycleError("backend has no scene-control extension", operation="easy.sim.scene_snapshot")
        return world.scene_snapshot()

    def close(self) -> None:
        if self._state is SimState.CLOSED:
            return
        if self._session is not None:
            self._session.close()
        self._session = None
        self._world = None
        self._state = SimState.CLOSED

    def __enter__(self) -> Sim:
        if self._state is SimState.CLOSED:
            raise LifecycleError("closed simulation cannot be re-entered", operation="easy.sim.enter")
        return self

    def __exit__(self, *_: object) -> None:
        self.close()
