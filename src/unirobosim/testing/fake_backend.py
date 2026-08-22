"""Deterministic reference implementation for contract tests.

This module is deliberately explicit and never selected by production runtime code. Its state update
rules test the API; they are not a physics-fidelity claim.
"""

from __future__ import annotations

import hashlib
import itertools
import json
import math
import re
import struct
import threading
from collections.abc import Callable, Iterable
from dataclasses import dataclass, field
from functools import wraps
from types import TracebackType
from typing import TypeVar, cast

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
    PlanningGeometryResourceRevokedError,
    PlanningSceneContractError,
    PlanningSceneDeltaContinuityError,
    PlanningSceneError,
    PlanningSceneHashMismatchError,
    PlanningSceneIncompleteError,
    PlanningSceneNotFoundError,
    PlanningSceneRepresentationError,
    ProviderSelectionError,
    StaleHandleError,
    ValidationError,
    WorldBuildError,
    _snapshot_and_scrub_planning_error,
)
from unirobosim.api.frozen import FrozenMap
from unirobosim.api.planning_scene import (
    PLANNING_SCENE_CAPABILITY_ID,
    PLANNING_SCENE_SCHEMA_VERSION,
    PLANNING_SYSTEM_ENTITY_ID,
    PLANNING_SYSTEM_ENTITY_PATH,
    PlanningArticulationState,
    PlanningAttachment,
    PlanningCompoundGeometry,
    PlanningCompoundPart,
    PlanningEntityDescriptor,
    PlanningEntityKind,
    PlanningEntityState,
    PlanningFrameDeclaration,
    PlanningFrameDescriptor,
    PlanningFrameKind,
    PlanningFrameSourceKind,
    PlanningFrameState,
    PlanningGeometryAxisConvention,
    PlanningGeometryContentProfile,
    PlanningGeometryDescriptor,
    PlanningGeometryDType,
    PlanningGeometryLease,
    PlanningGeometryLocalPose,
    PlanningGeometryMotionClass,
    PlanningGeometryPurpose,
    PlanningGeometryRepresentation,
    PlanningGeometryResourceDescriptor,
    PlanningGeometryResourceLayout,
    PlanningGeometryStorageKind,
    PlanningGeometryTransform,
    PlanningHalfspaceGeometry,
    PlanningJointDescriptor,
    PlanningJointType,
    PlanningLinkDescriptor,
    PlanningLinkState,
    PlanningPose,
    PlanningPrimitiveGeometry,
    PlanningSceneCatalog,
    PlanningSceneDelta,
    PlanningSceneDeltaKind,
    PlanningSceneState,
    PlanningTwist,
    parse_planning_frame_declarations,
)
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
        CapabilityDeclaration(
            CapabilityId("planning.scene@2"),
            FrozenMap(
                {
                    "authority_thread": "synchronous",
                    "axis_convention": "right_handed_z_up",
                    "geometry_read_limit_bytes": 64 * 1024 * 1024,
                    "resource_layout": "catalog-pinned-v1",
                    "single_representation_per_geometry": True,
                    "representation_fallback": False,
                }
            ),
            limitations=("deterministic contract reference; not a physics-fidelity planning world",),
        ),
    )
)

FAKE_DESCRIPTOR = ProviderDescriptor(
    provider_id="reference.fake",
    display_name="UniRoboSim Fake Reference Backend",
    version="0.7.1",
    contract_version="v0alpha4",
    capabilities=FAKE_CAPABILITIES,
    metadata=FrozenMap({"purpose": "contract-testing-only"}),
)

_SESSION_IDS = itertools.count(1)

_FAKE_PLANNING_PROVENANCE_SCHEMA = "unirobosim.planning-geometry-provenance/v1"
_FAKE_PLANNING_ADAPTER_ID = "unirobosim.reference.fake"
_FAKE_PLANNING_ADAPTER_VERSION = "planning-scene-v2"
_FAKE_PLANNING_NATIVE_PROFILE = "fake-native-collision/v1"
_FAKE_PLANNING_COMPILER_PROFILE = "python-struct-little-endian/v1"
_FAKE_PLANNING_CANONICALIZATION_ALGORITHM = "json-utf8-sort-keys-compact-sha256/v1"


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


@dataclass
class _PlanningLeaseEpoch:
    live: bool = True


@dataclass
class _FakePlanningEnvironmentRuntime:
    generation: int
    sequence: int = 1
    world_revision: int = 1
    catalog_revision: int = 1
    geometry_revision: int = 1
    transform_revision: int = 1
    attachment_revision: int = 1
    force_resync: bool = False
    lease_serial: int = 0
    lease_epoch: _PlanningLeaseEpoch = field(default_factory=_PlanningLeaseEpoch)
    raw_resources: dict[
        str,
        tuple[
            bytes,
            PlanningGeometryRepresentation,
            str,
            PlanningGeometryContentProfile,
            tuple[int, int],
            tuple[int, int],
        ],
    ] = field(default_factory=dict)
    catalog: PlanningSceneCatalog | None = None
    history: dict[int, PlanningSceneState] = field(default_factory=dict)


@dataclass
class _FakePlanningRuntime:
    authority_thread_id: int
    environments: dict[int, _FakePlanningEnvironmentRuntime]
    storage_cache: dict[tuple[str, PlanningGeometryRepresentation, str], tuple[bytes, str]] = field(
        default_factory=dict
    )
    geometry_materializations: int = 0


_PlanningResultT = TypeVar("_PlanningResultT")

_PLANNING_ERROR_TYPES: tuple[type[PlanningSceneError], ...] = (
    PlanningSceneContractError,
    PlanningSceneDeltaContinuityError,
    PlanningSceneNotFoundError,
    PlanningSceneIncompleteError,
    PlanningSceneRepresentationError,
    PlanningSceneHashMismatchError,
    PlanningGeometryResourceRevokedError,
    PlanningSceneError,
)


def _planning_error_boundary(
    function: Callable[..., _PlanningResultT],
) -> Callable[..., _PlanningResultT]:
    """Detach public planning failures from caller-controlled arguments."""

    @wraps(function)
    def wrapped(*args: object, **kwargs: object) -> _PlanningResultT:
        failure: tuple[type[PlanningSceneError], str, str, str | None, str | None, str | None] | None = None
        try:
            return function(*args, **kwargs)
        except PlanningSceneError as caught:
            failure = _snapshot_and_scrub_planning_error(caught, _PLANNING_ERROR_TYPES)
        args = ()
        kwargs = {}
        assert failure is not None
        error_type, message, operation, backend_id, world_id, entity_path = failure
        raise error_type(
            message,
            operation=operation,
            backend_id=backend_id,
            world_id=world_id,
            entity_path=entity_path,
        ) from None

    return cast(Callable[..., _PlanningResultT], wrapped)


class _FakePlanningGeometryLease:
    """Worker-safe read-only lease over validated immutable bytes."""

    __slots__ = ("_closed", "_content", "_descriptor", "_epoch", "_lock")

    def __init__(
        self,
        descriptor: PlanningGeometryResourceDescriptor,
        content: bytes,
        epoch: _PlanningLeaseEpoch,
    ) -> None:
        self._descriptor = descriptor
        self._content = content
        self._epoch = epoch
        self._closed = False
        self._lock = threading.RLock()

    def _ensure_live(self) -> None:
        if self._closed or not self._epoch.live:
            raise PlanningGeometryResourceRevokedError(
                "planning geometry resource lease is revoked",
                operation="planning_geometry.read",
            ) from None

    @property
    def descriptor(self) -> PlanningGeometryResourceDescriptor:
        with self._lock:
            self._ensure_live()
            return self._descriptor

    @property
    def closed(self) -> bool:
        with self._lock:
            return self._closed or not self._epoch.live

    @_planning_error_boundary
    def read(self, offset: int = 0, length: int | None = None) -> bytes:
        with self._lock:
            self._ensure_live()
            start, count = self._descriptor.read_span(offset, length)
            result = self._content[start : start + count]
            if type(result) is not bytes or len(result) != count:
                raise PlanningSceneContractError(
                    "planning geometry storage returned an invalid byte span",
                    operation="planning_geometry.read",
                ) from None
            return result

    def close(self) -> None:
        with self._lock:
            self._closed = True


def _planning_id(prefix: str, *parts: str) -> str:
    digest = hashlib.sha256("\x1f".join(parts).encode("utf-8")).hexdigest()[:20]
    return f"{prefix}.{digest}"


def _planning_actual_base(value: object, allowed: tuple[type, ...]) -> type | None:
    try:
        mro = type.mro(type(value))
    except BaseException:
        return None
    if type(mro) is not list or list.__len__(mro) > 256:
        return None
    for allowed_base in allowed:
        for index in range(list.__len__(mro)):
            if list.__getitem__(mro, index) is allowed_base:
                return allowed_base
    return None


def _planning_exact_text(value: object, label: str) -> str:
    """Detach accepted text without invoking caller-provided methods."""

    if _planning_actual_base(value, (str,)) is not str:
        raise PlanningSceneIncompleteError(
            f"fake planning {label} is not portable text",
            operation="planning_scene.preflight",
        ) from None
    source_length = str.__len__(value)  # type: ignore[arg-type]
    if source_length == 0 or source_length > 512:
        raise PlanningSceneIncompleteError(
            f"fake planning {label} exceeds its text budget",
            operation="planning_scene.preflight",
        ) from None
    canonical = str.__str__(value)
    if (
        type(canonical) is not str
        or "\x00" in canonical
        or any(0xD800 <= ord(character) <= 0xDFFF for character in canonical)
        or len(canonical.encode("utf-8")) > 4096
    ):
        raise PlanningSceneIncompleteError(
            f"fake planning {label} is not bounded portable text",
            operation="planning_scene.preflight",
        ) from None
    return canonical


def _planning_record_sha256(value: dict[str, object]) -> str:
    canonical = json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()


def _planning_quaternion_multiply(
    left: tuple[float, float, float, float],
    right: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def _planning_rotate(
    vector: tuple[float, float, float], quaternion: tuple[float, float, float, float]
) -> tuple[float, float, float]:
    conjugate = (-quaternion[0], -quaternion[1], -quaternion[2], quaternion[3])
    pure = (vector[0], vector[1], vector[2], 0.0)
    result = _planning_quaternion_multiply(_planning_quaternion_multiply(quaternion, pure), conjugate)
    return result[0], result[1], result[2]


def _planning_compose(world_pose: PlanningPose, local_pose: PlanningGeometryLocalPose) -> PlanningPose:
    offset = _planning_rotate(local_pose.position_m, world_pose.orientation_xyzw)
    return PlanningPose(
        world_pose.frame_id,
        (
            world_pose.position_m[0] + offset[0],
            world_pose.position_m[1] + offset[1],
            world_pose.position_m[2] + offset[2],
        ),
        _planning_quaternion_multiply(world_pose.orientation_xyzw, local_pose.orientation_xyzw),
    )


def _planning_relative(parent: PlanningPose, child: PlanningPose, parent_frame_id: str) -> PlanningPose:
    inverse = (
        -parent.orientation_xyzw[0],
        -parent.orientation_xyzw[1],
        -parent.orientation_xyzw[2],
        parent.orientation_xyzw[3],
    )
    displacement = (
        child.position_m[0] - parent.position_m[0],
        child.position_m[1] - parent.position_m[1],
        child.position_m[2] - parent.position_m[2],
    )
    return PlanningPose(
        parent_frame_id,
        _planning_rotate(displacement, inverse),
        _planning_quaternion_multiply(inverse, child.orientation_xyzw),
    )


def _concave_container_mesh_bytes() -> tuple[bytes, tuple[int, int], tuple[int, int]]:
    """Open box surface: a triangle mesh that preserves a concave container interior."""

    vertices = (
        (-0.5, -0.5, 0.0),
        (0.5, -0.5, 0.0),
        (0.5, 0.5, 0.0),
        (-0.5, 0.5, 0.0),
        (-0.5, -0.5, 0.6),
        (0.5, -0.5, 0.6),
        (0.5, 0.5, 0.6),
        (-0.5, 0.5, 0.6),
    )
    triangles = (
        (0, 2, 1),
        (0, 3, 2),
        (0, 1, 5),
        (0, 5, 4),
        (1, 2, 6),
        (1, 6, 5),
        (2, 3, 7),
        (2, 7, 6),
        (3, 0, 4),
        (3, 4, 7),
    )
    content = b"".join(struct.pack("<fff", *vertex) for vertex in vertices) + b"".join(
        struct.pack("<III", *triangle) for triangle in triangles
    )
    return content, (len(vertices), 3), (len(triangles), 3)


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
        planning_demanded = CapabilityId("planning.scene@2") in negotiation.matched
        world_type = FakePlanningWorld if planning_demanded else FakeWorld
        planning_failure: (
            tuple[
                type[PlanningSceneError],
                str,
                str,
                str | None,
                str | None,
                str | None,
            ]
            | None
        ) = None
        try:
            world = world_type(self, spec, self._generation)
        except PlanningSceneError as caught:
            planning_failure = _snapshot_and_scrub_planning_error(caught, _PLANNING_ERROR_TYPES)
        if planning_failure is not None:
            del spec
            error_type, message, operation, backend_id, world_id, entity_path = planning_failure
            raise error_type(
                message,
                operation=operation,
                backend_id=backend_id,
                world_id=world_id,
                entity_path=entity_path,
            ) from None
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
    # Installed only by ``FakePlanningWorld``.  A type annotation creates no
    # instance field and keeps the no-demand object layout unchanged.
    _planning_runtime: _FakePlanningRuntime

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

    @staticmethod
    def _planning_entity_id(path: str) -> str:
        return _planning_id("entity", path)

    @staticmethod
    def _planning_link_id(path: str, index: int) -> str:
        return _planning_id("link", path, str(index))

    @staticmethod
    def _planning_frame_id(path: str, index: int) -> str:
        return _planning_id("frame", path, str(index))

    @staticmethod
    def _planning_entity_frame_id(path: str) -> str:
        return _planning_id("frame", path, "entity")

    @staticmethod
    def _planning_joint_frame_id(path: str, index: int) -> str:
        return _planning_id("frame", path, "joint", str(index))

    @staticmethod
    def _planning_joint_id(path: str, index: int) -> str:
        return _planning_id("joint", path, str(index))

    @staticmethod
    def _planning_geometry_id(path: str) -> str:
        return _planning_id("geometry", path, "root")

    def _planning_kind(self, entity: EntitySpec) -> PlanningEntityKind:
        explicit = entity.metadata.get("planning_entity_kind")
        if explicit == "robot":
            return PlanningEntityKind.ROBOT
        if entity.kind is EntityKind.ARTICULATION:
            return PlanningEntityKind.ARTICULATION
        if entity.kind is EntityKind.RIGID_BODY:
            return PlanningEntityKind.RIGID_OBJECT
        return PlanningEntityKind.OTHER

    def _planning_motion_class(self, entity: EntitySpec) -> PlanningGeometryMotionClass:
        explicit = entity.metadata.get("planning_motion_class")
        if explicit == "static":
            return PlanningGeometryMotionClass.STATIC
        if explicit == "kinematic":
            return PlanningGeometryMotionClass.KINEMATIC
        return PlanningGeometryMotionClass.DYNAMIC

    def _planning_provenance_sha256(
        self,
        *,
        source_kind: str,
        source_parameters: dict[str, object],
        representation: PlanningGeometryRepresentation,
        cooking_profile: str,
        effective_parameters: dict[str, object],
        canonical_content_profile: str,
    ) -> str:
        provider = self._session.descriptor
        record: dict[str, object] = {
            "schema": _FAKE_PLANNING_PROVENANCE_SCHEMA,
            "source": {
                "kind": source_kind,
                "sha256": _planning_record_sha256(source_parameters),
            },
            "provider": {
                "id": _planning_exact_text(provider.provider_id, "provider ID"),
                "version": _planning_exact_text(provider.version, "provider version"),
            },
            "adapter": {
                "id": _FAKE_PLANNING_ADAPTER_ID,
                "version": _FAKE_PLANNING_ADAPTER_VERSION,
            },
            "contract": {
                "provider_contract_version": _planning_exact_text(
                    provider.contract_version,
                    "provider contract version",
                ),
                "capability": PLANNING_SCENE_CAPABILITY_ID,
                "catalog_schema": PLANNING_SCENE_SCHEMA_VERSION,
            },
            "native": {
                "profile": _FAKE_PLANNING_NATIVE_PROFILE,
                "compiler_profile": _FAKE_PLANNING_COMPILER_PROFILE,
                "cooking_profile": cooking_profile,
            },
            "effective_parameters": {
                "representation": representation.value,
                **effective_parameters,
            },
            "canonicalization": {
                "algorithm": _FAKE_PLANNING_CANONICALIZATION_ALGORITHM,
                "content_profile": canonical_content_profile,
            },
        }
        return _planning_record_sha256(record)

    def _planning_geometry(
        self,
        entity: EntitySpec,
        entity_id: str,
        root_link_id: str,
        root_frame_id: str,
        environment_runtime: _FakePlanningEnvironmentRuntime,
    ) -> PlanningGeometryDescriptor | None:
        if entity.kind not in {EntityKind.RIGID_BODY, EntityKind.ARTICULATION}:
            return None
        path = _planning_exact_text(entity.path.value, "entity path")
        geometry_id = self._planning_geometry_id(path)
        motion_class = self._planning_motion_class(entity)

        def descriptor(
            representation: PlanningGeometryRepresentation,
            *,
            source_kind: str,
            source_parameters: dict[str, object],
            cooking_profile: str,
            effective_shape: dict[str, object],
            inline: PlanningHalfspaceGeometry | PlanningPrimitiveGeometry | PlanningCompoundGeometry | None = None,
            resource_id: str | None = None,
            digest: str | None = None,
            content_profile: PlanningGeometryContentProfile | None = None,
            resource_layout: PlanningGeometryResourceLayout | None = None,
        ) -> PlanningGeometryDescriptor:
            canonical_content_profile = "inline-portable-values/v1"
            if content_profile is not None:
                canonical_content_profile = content_profile.value
            provenance = self._planning_provenance_sha256(
                source_kind=source_kind,
                source_parameters=source_parameters,
                representation=representation,
                cooking_profile=cooking_profile,
                effective_parameters={
                    "parent_frame_T_geometry": {
                        "position_m": (0.0, 0.0, 0.0),
                        "orientation_xyzw": (0.0, 0.0, 0.0, 1.0),
                    },
                    "scale": (1.0, 1.0, 1.0),
                    "motion_class": motion_class.value,
                    "collision_group": 1,
                    "collision_mask": 2**32 - 1,
                    "shape": effective_shape,
                },
                canonical_content_profile=canonical_content_profile,
            )
            return PlanningGeometryDescriptor(
                geometry_id=geometry_id,
                owner_entity_id=entity_id,
                owner_link_id=root_link_id,
                parent_frame_id=root_frame_id,
                purpose=PlanningGeometryPurpose.COLLISION,
                representation=representation,
                parent_frame_T_geometry=PlanningGeometryLocalPose(),
                scale=(1.0, 1.0, 1.0),
                motion_class=motion_class,
                collision_group=1,
                collision_mask=2**32 - 1,
                provenance_sha256=provenance,
                inline=inline,
                resource_id=resource_id,
                sha256=digest,
                content_profile=content_profile,
                resource_layout=resource_layout,
            )

        if entity.kind is EntityKind.ARTICULATION:
            part_a = PlanningCompoundPart(
                _planning_id("part", entity.path.value, "body"),
                PlanningGeometryLocalPose((0.04, -0.03, 0.08), (0.0, 0.0, 0.0, 1.0)),
                PlanningPrimitiveGeometry(PlanningGeometryRepresentation.BOX, (0.3, 0.2, 0.16)),
            )
            half_angle = math.pi / 8.0
            part_b = PlanningCompoundPart(
                _planning_id("part", entity.path.value, "column"),
                PlanningGeometryLocalPose(
                    (-0.02, 0.05, 0.27),
                    (0.0, math.sin(half_angle), 0.0, math.cos(half_angle)),
                ),
                PlanningPrimitiveGeometry(PlanningGeometryRepresentation.CYLINDER, (0.05, 0.35)),
            )
            return descriptor(
                PlanningGeometryRepresentation.COMPOUND,
                source_kind="authored-procedural-articulation",
                source_parameters={
                    "entity_path": path,
                    "profile": "fake-articulation-compound/v1",
                },
                cooking_profile="fake-inline-compound-cooking/v1",
                effective_shape={
                    "parts": tuple(
                        {
                            "part_id": part.part_id,
                            "position_m": part.local_pose.position_m,
                            "orientation_xyzw": part.local_pose.orientation_xyzw,
                            "representation": part.primitive.representation.value,
                            "dimensions_m": part.primitive.dimensions_m,
                        }
                        for part in (part_a, part_b)
                    )
                },
                inline=PlanningCompoundGeometry(tuple(sorted((part_a, part_b), key=lambda item: item.part_id))),
            )
        if entity.asset_uri is not None:
            if entity.metadata.get("fake_planning_collision_authority") != "effective_native":
                raise PlanningSceneIncompleteError(
                    "asset-backed fake geometry lacks effective-native collision authority",
                    operation="planning_scene.preflight",
                    entity_path=entity.path.value,
                ) from None
            content, vertex_shape, index_shape = _concave_container_mesh_bytes()
            digest = hashlib.sha256(content).hexdigest()
            resource_id = _planning_id("resource", entity.path.value, digest)
            layout = PlanningGeometryResourceLayout(
                PlanningGeometryRepresentation.TRIANGLE_MESH,
                PlanningGeometryContentProfile.MESH_TRIANGLES_RAW_LE_V1,
                PlanningGeometryDType.FLOAT32,
                vertex_shape,
                PlanningGeometryDType.UINT32,
                index_shape,
            )
            environment_runtime.raw_resources[geometry_id] = (
                content,
                PlanningGeometryRepresentation.TRIANGLE_MESH,
                resource_id,
                PlanningGeometryContentProfile.MESH_TRIANGLES_RAW_LE_V1,
                vertex_shape,
                index_shape,
            )
            return descriptor(
                PlanningGeometryRepresentation.TRIANGLE_MESH,
                source_kind="locked-asset-uri",
                source_parameters={
                    "asset_uri": _planning_exact_text(entity.asset_uri, "asset URI"),
                },
                cooking_profile="fake-concave-container-triangle-mesh-cooking/v1",
                effective_shape={
                    "resource_sha256": digest,
                    "vertex_dtype": PlanningGeometryDType.FLOAT32.value,
                    "vertex_shape": vertex_shape,
                    "index_dtype": PlanningGeometryDType.UINT32.value,
                    "index_shape": index_shape,
                },
                resource_id=resource_id,
                digest=digest,
                content_profile=PlanningGeometryContentProfile.MESH_TRIANGLES_RAW_LE_V1,
                resource_layout=layout,
            )
        dimensions = (0.5, 0.5, 0.5) if entity.box is None else entity.box.dimensions_m
        return descriptor(
            PlanningGeometryRepresentation.BOX,
            source_kind="authored-procedural-box",
            source_parameters={
                "entity_path": path,
                "dimensions_m": dimensions,
            },
            cooking_profile="fake-inline-box-cooking/v1",
            effective_shape={"dimensions_m": dimensions},
            inline=PlanningPrimitiveGeometry(PlanningGeometryRepresentation.BOX, dimensions),
        )

    @staticmethod
    def _planning_named_frame(
        entity: EntitySpec,
        entity_id: str,
        declaration: PlanningFrameDeclaration,
        links: tuple[PlanningLinkDescriptor, ...],
        joints: tuple[PlanningJointDescriptor, ...],
        frames: tuple[PlanningFrameDescriptor, ...],
    ) -> PlanningFrameDescriptor:
        link_by_id = {item.link_id: item for item in links}
        frame_by_id = {item.frame_id: item for item in frames}
        owner_matches = (
            ()
            if declaration.owner_link_name is None
            else tuple(item for item in links if item.authored_name == declaration.owner_link_name)
        )
        if declaration.owner_link_name is None or len(owner_matches) != 1:
            raise PlanningSceneIncompleteError(
                "planning frame owner link does not resolve uniquely",
                operation="planning_scene.preflight",
                entity_path=entity.path.value,
            ) from None
        owner_link = owner_matches[0]

        if declaration.source.kind is PlanningFrameSourceKind.LINK:
            link_source_matches = tuple(item for item in links if item.authored_name == declaration.source.name)
            if len(link_source_matches) != 1 or link_source_matches[0].link_id != owner_link.link_id:
                raise PlanningSceneIncompleteError(
                    "planning frame link source does not match its locked owner",
                    operation="planning_scene.preflight",
                    entity_path=entity.path.value,
                ) from None
            parent_frame_id = link_source_matches[0].frame_id
        elif declaration.source.kind is PlanningFrameSourceKind.JOINT:
            joint_source_matches = tuple(item for item in joints if item.authored_name == declaration.source.name)
            if len(joint_source_matches) != 1:
                raise PlanningSceneIncompleteError(
                    "planning frame joint source does not resolve uniquely",
                    operation="planning_scene.preflight",
                    entity_path=entity.path.value,
                ) from None
            axis_frame = frame_by_id[joint_source_matches[0].axis_frame_id]
            if axis_frame.owner_link_id != owner_link.link_id:
                raise PlanningSceneIncompleteError(
                    "planning frame joint source does not match its locked owner",
                    operation="planning_scene.preflight",
                    entity_path=entity.path.value,
                ) from None
            parent_frame_id = axis_frame.frame_id
        else:
            raise PlanningSceneIncompleteError(
                "fake planning profile has no authoritative native-named frame registry",
                operation="planning_scene.preflight",
                entity_path=entity.path.value,
            ) from None

        if link_by_id[owner_link.link_id].entity_id != entity_id:
            raise PlanningSceneIncompleteError(
                "planning frame owner does not belong to its entity",
                operation="planning_scene.preflight",
                entity_path=entity.path.value,
            ) from None
        return PlanningFrameDescriptor(
            _planning_id("frame", entity.path.value, "named", declaration.semantic_key),
            PlanningFrameKind.NAMED,
            parent_frame_id,
            entity_id,
            owner_link.link_id,
            declaration.role,
            declaration.semantic_key,
        )

    def _build_planning_catalog(self, environment_index: int) -> PlanningSceneCatalog:
        runtime = self._planning_runtime
        assert runtime is not None
        environment_runtime = runtime.environments[environment_index]
        system_frame_id = "frame.system.simulator_effective"
        system_geometry_id = "geometry.system.simulator_effective.ground"
        system_provenance = self._planning_provenance_sha256(
            source_kind="provider-owned-effective-collider",
            source_parameters={"identity": "implicit-ground"},
            representation=PlanningGeometryRepresentation.HALFSPACE,
            cooking_profile="fake-inline-halfspace-cooking/v1",
            effective_parameters={
                "parent_frame_T_geometry": {
                    "position_m": (0.0, 0.0, 0.0),
                    "orientation_xyzw": (0.0, 0.0, 0.0, 1.0),
                },
                "scale": (1.0, 1.0, 1.0),
                "motion_class": PlanningGeometryMotionClass.STATIC.value,
                "collision_group": 1,
                "collision_mask": 2**32 - 1,
                "shape": {"occupied_region": "local-z-less-than-or-equal-zero"},
            },
            canonical_content_profile="inline-halfspace/v1",
        )
        entities: list[PlanningEntityDescriptor] = [
            PlanningEntityDescriptor(
                PLANNING_SYSTEM_ENTITY_ID,
                PLANNING_SYSTEM_ENTITY_PATH,
                PlanningEntityKind.OTHER,
                True,
                system_frame_id,
                (),
                (system_frame_id,),
                (system_geometry_id,),
                (),
            )
        ]
        links: list[PlanningLinkDescriptor] = []
        joints: list[PlanningJointDescriptor] = []
        frames: list[PlanningFrameDescriptor] = [
            PlanningFrameDescriptor("frame.world", PlanningFrameKind.WORLD, None, None, None),
            PlanningFrameDescriptor(
                system_frame_id,
                PlanningFrameKind.ENTITY,
                "frame.world",
                PLANNING_SYSTEM_ENTITY_ID,
                None,
            ),
        ]
        geometries: list[PlanningGeometryDescriptor] = [
            PlanningGeometryDescriptor(
                system_geometry_id,
                PLANNING_SYSTEM_ENTITY_ID,
                None,
                system_frame_id,
                PlanningGeometryPurpose.COLLISION,
                PlanningGeometryRepresentation.HALFSPACE,
                PlanningGeometryLocalPose(),
                (1.0, 1.0, 1.0),
                PlanningGeometryMotionClass.STATIC,
                1,
                2**32 - 1,
                system_provenance,
                PlanningHalfspaceGeometry(),
            )
        ]
        for entity in self._spec.entities:
            path = entity.path.value
            entity_id = self._planning_entity_id(path)
            entity_frame_id = self._planning_entity_frame_id(path)
            joint_count = len(entity.joint_names) if entity.kind is EntityKind.ARTICULATION else 0
            link_count = joint_count + 1
            link_ids = tuple(self._planning_link_id(path, index) for index in range(link_count))
            frame_ids = tuple(self._planning_frame_id(path, index) for index in range(link_count))
            joint_ids = tuple(self._planning_joint_id(path, index) for index in range(joint_count))
            geometry = self._planning_geometry(
                entity,
                entity_id,
                link_ids[0],
                frame_ids[0],
                environment_runtime,
            )
            geometry_ids = () if geometry is None else (geometry.geometry_id,)
            entity_frames: list[PlanningFrameDescriptor] = [
                PlanningFrameDescriptor(
                    entity_frame_id,
                    PlanningFrameKind.ENTITY,
                    "frame.world",
                    entity_id,
                    None,
                )
            ]
            entity_links: list[PlanningLinkDescriptor] = []
            for index, (link_id, frame_id) in enumerate(zip(link_ids, frame_ids, strict=True)):
                parent_link_id = None if index == 0 else link_ids[index - 1]
                authored_name = entity.path.name if index == 0 else f"{entity.joint_names[index - 1]} child"
                link_geometry_ids = geometry_ids if index == 0 else ()
                entity_links.append(
                    PlanningLinkDescriptor(
                        link_id,
                        entity_id,
                        authored_name,
                        frame_id,
                        parent_link_id,
                        link_geometry_ids,
                    )
                )
                entity_frames.append(
                    PlanningFrameDescriptor(
                        frame_id,
                        PlanningFrameKind.LINK,
                        entity_frame_id if index == 0 else frame_ids[index - 1],
                        entity_id,
                        link_id,
                    )
                )
            entity_joints: list[PlanningJointDescriptor] = []
            for index, (joint_name, joint_id) in enumerate(zip(entity.joint_names, joint_ids, strict=True)):
                max_effort = entity.joint_effort_limits[index] if entity.joint_effort_limits else None
                entity_joints.append(
                    PlanningJointDescriptor(
                        joint_id,
                        entity_id,
                        joint_name,
                        link_ids[index],
                        link_ids[index + 1],
                        PlanningJointType.REVOLUTE,
                        frame_ids[index],
                        (0.0, 0.0, 1.0),
                        "rad",
                        None,
                        None,
                        None,
                        max_effort,
                    )
                )
            declarations = parse_planning_frame_declarations(entity.metadata.get("planning_frame_declarations"))
            if declarations is not None:
                physical_frames = tuple(entity_frames)
                for declaration in declarations.entries:
                    entity_frames.append(
                        self._planning_named_frame(
                            entity,
                            entity_id,
                            declaration,
                            tuple(entity_links),
                            tuple(entity_joints),
                            physical_frames,
                        )
                    )
            entities.append(
                PlanningEntityDescriptor(
                    entity_id,
                    path,
                    self._planning_kind(entity),
                    True,
                    entity_frame_id,
                    tuple(sorted(link_ids)),
                    tuple(sorted(frame.frame_id for frame in entity_frames)),
                    geometry_ids,
                    joint_ids,
                )
            )
            links.extend(entity_links)
            joints.extend(entity_joints)
            frames.extend(entity_frames)
            if geometry is not None:
                geometries.append(geometry)
        return PlanningSceneCatalog.build(
            self._session.descriptor.provider_id,
            self.world_id,
            environment_runtime.generation,
            environment_index,
            environment_runtime.catalog_revision,
            environment_runtime.geometry_revision,
            tuple(sorted(entities, key=lambda item: item.entity_id)),
            tuple(sorted(links, key=lambda item: item.link_id)),
            tuple(sorted(joints, key=lambda item: item.joint_id)),
            tuple(sorted(frames, key=lambda item: item.frame_id)),
            tuple(sorted(geometries, key=lambda item: item.geometry_id)),
        )

    def _planning_entity_pose_and_twist(
        self,
        entity: EntitySpec,
        environment_index: int,
        world_frame_id: str,
    ) -> tuple[PlanningPose, PlanningTwist]:
        if entity.kind is EntityKind.RIGID_BODY:
            runtime = self._rigids[entity.path]
            position = runtime.positions[environment_index]
            orientation = runtime.orientations[environment_index]
            linear = runtime.linear_velocities[environment_index]
            angular = runtime.angular_velocities[environment_index]
            pose = PlanningPose(
                world_frame_id,
                (position[0], position[1], position[2]),
                (orientation[0], orientation[1], orientation[2], orientation[3]),
            )
            twist = PlanningTwist(
                world_frame_id,
                (linear[0], linear[1], linear[2]),
                (angular[0], angular[1], angular[2]),
            )
            return pose, twist
        return (
            PlanningPose(world_frame_id, entity.pose.position, entity.pose.orientation_xyzw),
            PlanningTwist(world_frame_id),
        )

    def _planning_attachment_values(
        self,
        catalog: PlanningSceneCatalog,
        frame_states: tuple[PlanningFrameState, ...],
    ) -> tuple[PlanningAttachment, ...]:
        raw = self._spec.metadata.get("planning_attachments", ())
        if type(raw) is not tuple:
            raise PlanningSceneContractError(
                "fake planning attachment metadata must be an immutable tuple",
                operation="fake.planning_scene.build",
            ) from None
        if raw and self._spec.metadata.get("planning_attachment_authority") != "exclusive_registry":
            raise PlanningSceneIncompleteError(
                "fake planning attachments lack an exclusive authoritative registry",
                operation="planning_scene.preflight",
            ) from None
        entity_by_path = {entity.path: entity for entity in catalog.entities}
        frame_pose = {frame.frame_id: frame.world_pose for frame in frame_states}
        result: list[PlanningAttachment] = []
        for index, item in enumerate(raw):
            if type(item) is not FrozenMap:
                raise PlanningSceneContractError(
                    "fake planning attachment metadata contains an invalid record",
                    operation="fake.planning_scene.build",
                ) from None
            parent_path = item.get("parent_path")
            child_path = item.get("child_path")
            if type(parent_path) is not str or type(child_path) is not str:
                raise PlanningSceneContractError(
                    "fake planning attachment paths must be exact strings",
                    operation="fake.planning_scene.build",
                ) from None
            parent = entity_by_path.get(parent_path)
            child = entity_by_path.get(child_path)
            if parent is None or child is None:
                raise PlanningSceneContractError(
                    "fake planning attachment references an unknown entity path",
                    operation="fake.planning_scene.build",
                ) from None
            attachment_id_value = item.get("attachment_id")
            attachment_id = (
                _planning_id("attachment", parent_path, child_path, str(index))
                if attachment_id_value is None
                else attachment_id_value
            )
            if type(attachment_id) is not str:
                raise PlanningSceneContractError(
                    "fake planning attachment ID must be an exact string",
                    operation="fake.planning_scene.build",
                ) from None
            parent_link = next(link for link in catalog.links if link.link_id == parent.link_ids[0])
            child_link = next(link for link in catalog.links if link.link_id == child.link_ids[0])
            parent_frame = parent_link.frame_id
            child_frame = child_link.frame_id
            if not child.geometry_ids:
                raise PlanningSceneContractError(
                    "fake planning attachment child has no planning geometry",
                    operation="fake.planning_scene.build",
                ) from None
            result.append(
                PlanningAttachment(
                    attachment_id,
                    parent.entity_id,
                    child.entity_id,
                    parent_frame,
                    child_frame,
                    _planning_relative(frame_pose[parent_frame], frame_pose[child_frame], parent_frame),
                    child.geometry_ids,
                    parent_link.link_id,
                    child_link.link_id,
                )
            )
        return tuple(sorted(result, key=lambda item: item.attachment_id))

    def _capture_planning_state(self, environment_index: int) -> PlanningSceneState:
        runtime = self._planning_runtime
        assert runtime is not None
        environment_runtime = runtime.environments[environment_index]
        catalog = environment_runtime.catalog
        assert catalog is not None
        spec_by_path = {entity.path.value: entity for entity in self._spec.entities}
        entity_states: list[PlanningEntityState] = []
        link_states: list[PlanningLinkState] = []
        frame_states: list[PlanningFrameState] = [
            PlanningFrameState("frame.world", PlanningPose("frame.world", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)))
        ]
        pose_by_entity: dict[str, PlanningPose] = {}
        twist_by_entity: dict[str, PlanningTwist] = {}
        for descriptor in catalog.entities:
            if descriptor.entity_id == PLANNING_SYSTEM_ENTITY_ID:
                pose = PlanningPose(
                    catalog.world_frame_id,
                    (0.0, 0.0, 0.0),
                    (0.0, 0.0, 0.0, 1.0),
                )
                twist = PlanningTwist(catalog.world_frame_id)
            else:
                spec = spec_by_path[descriptor.path]
                pose, twist = self._planning_entity_pose_and_twist(spec, environment_index, catalog.world_frame_id)
            pose_by_entity[descriptor.entity_id] = pose
            twist_by_entity[descriptor.entity_id] = twist
            entity_states.append(PlanningEntityState(descriptor.entity_id, pose, twist))
        for link in catalog.links:
            pose = pose_by_entity[link.entity_id]
            twist = twist_by_entity[link.entity_id]
            link_states.append(PlanningLinkState(link.link_id, pose, twist))
        for frame in catalog.frames:
            if frame.kind is PlanningFrameKind.WORLD:
                continue
            assert frame.owner_entity_id is not None
            frame_states.append(PlanningFrameState(frame.frame_id, pose_by_entity[frame.owner_entity_id]))
        sorted_frames = tuple(sorted(frame_states, key=lambda item: item.frame_id))
        frame_pose = {frame.frame_id: frame.world_pose for frame in sorted_frames}
        articulation_states: list[PlanningArticulationState] = []
        joint_by_id = {joint.joint_id: joint for joint in catalog.joints}
        for descriptor in catalog.entities:
            if not descriptor.joint_ids:
                continue
            spec = spec_by_path[descriptor.path]
            articulation_runtime = self._articulations[spec.path]
            articulation_states.append(
                PlanningArticulationState(
                    descriptor.entity_id,
                    descriptor.joint_ids,
                    tuple(articulation_runtime.positions[environment_index]),
                    tuple(articulation_runtime.velocities[environment_index]),
                    tuple(joint_by_id[joint_id].position_unit for joint_id in descriptor.joint_ids),
                )
            )
        transforms = tuple(
            sorted(
                (
                    PlanningGeometryTransform(
                        geometry.geometry_id,
                        _planning_compose(frame_pose[geometry.parent_frame_id], geometry.parent_frame_T_geometry),
                    )
                    for geometry in catalog.geometries
                ),
                key=lambda item: item.geometry_id,
            )
        )
        attachments = self._planning_attachment_values(catalog, sorted_frames)
        state = PlanningSceneState(
            self._session.descriptor.provider_id,
            self.world_id,
            environment_runtime.generation,
            environment_index,
            self.tick,
            environment_runtime.sequence,
            environment_runtime.world_revision,
            environment_runtime.catalog_revision,
            environment_runtime.geometry_revision,
            catalog.content_sha256,
            environment_runtime.transform_revision,
            environment_runtime.attachment_revision,
            catalog.world_frame_id,
            tuple(sorted(entity_states, key=lambda item: item.entity_id)),
            tuple(sorted(link_states, key=lambda item: item.link_id)),
            sorted_frames,
            tuple(sorted(articulation_states, key=lambda item: item.entity_id)),
            transforms,
            attachments,
        )
        state.validate_against(catalog)
        return state

    def _initialize_planning_scene(self, environment_indices: tuple[int, ...] | None = None) -> None:
        runtime = self._planning_runtime
        assert runtime is not None
        for key, label in (
            ("fake_planning_unmapped_native_colliders", "native collision inventory"),
            ("fake_planning_untracked_constraints", "persistent constraint inventory"),
        ):
            value = self._spec.metadata.get(key, 0)
            if type(value) is not int or value != 0:
                raise PlanningSceneIncompleteError(
                    f"fake {label} is incomplete",
                    operation="planning_scene.preflight",
                ) from None
        selected = tuple(range(self._spec.environments.count)) if environment_indices is None else environment_indices
        for environment_index in selected:
            runtime.environments[environment_index].catalog = self._build_planning_catalog(environment_index)
        for environment_index in selected:
            state = self._capture_planning_state(environment_index)
            runtime.environments[environment_index].history = {state.sequence: state}

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

    def _planning_require_authority(self, operation: str) -> _FakePlanningRuntime:
        runtime = self._planning_runtime
        if threading.get_ident() != runtime.authority_thread_id:
            raise PlanningSceneContractError(
                "planning-scene world calls require the authority thread",
                operation=operation,
                world_id=self.world_id,
            ) from None
        if self._state is not WorldState.READY:
            raise PlanningSceneContractError(
                "planning-scene world is no longer live",
                operation=operation,
                world_id=self.world_id,
            ) from None
        return runtime

    def _planning_environment(self, value: object, operation: str) -> int:
        if _planning_actual_base(value, (bool, int)) is not int:
            raise PlanningSceneContractError(
                "planning environment index must be an integer",
                operation=operation,
                world_id=self.world_id,
            ) from None
        try:
            result = int.__int__(value)  # type: ignore[arg-type]
        except BaseException:
            result = -1
        if type(result) is not int or not 0 <= result < self._spec.environments.count:
            raise PlanningSceneContractError(
                "planning environment index is out of range",
                operation=operation,
                world_id=self.world_id,
            ) from None
        return result

    def _planning_scene_catalog_impl(self, environment_index: int = 0) -> PlanningSceneCatalog:
        operation = "world.planning_scene_catalog"
        runtime = self._planning_require_authority(operation)
        environment = self._planning_environment(environment_index, operation)
        catalog = runtime.environments[environment].catalog
        assert catalog is not None
        return catalog

    def _planning_scene_state_impl(self, environment_index: int = 0) -> PlanningSceneState:
        operation = "world.planning_scene_state"
        runtime = self._planning_require_authority(operation)
        environment = self._planning_environment(environment_index, operation)
        environment_runtime = runtime.environments[environment]
        return environment_runtime.history[environment_runtime.sequence]

    @staticmethod
    def _planning_sequence_value(value: object) -> int:
        result: object = None
        if _planning_actual_base(value, (bool, int)) is int:
            try:
                result = int.__int__(value)  # type: ignore[arg-type]
            except BaseException:
                result = None
        if type(result) is not int or not 1 <= result <= 2**63 - 1:
            raise PlanningSceneContractError(
                "planning delta base_sequence must be a positive bounded integer",
                operation="world.planning_scene_delta",
            ) from None
        return result

    @staticmethod
    def _planning_delta_value(
        current: PlanningSceneState,
        previous: PlanningSceneState,
        base_sequence: int,
        kind: PlanningSceneDeltaKind,
        *,
        catalog: PlanningSceneCatalog | None = None,
        state: PlanningSceneState | None = None,
        attachments: tuple[PlanningAttachment, ...] = (),
        resync_required: bool = False,
    ) -> PlanningSceneDelta:
        return PlanningSceneDelta(
            provider_id=current.provider_id,
            world_id=current.world_id,
            generation=current.generation,
            environment_index=current.environment_index,
            tick=current.tick,
            base_sequence=base_sequence,
            sequence=current.sequence,
            previous_world_revision=previous.world_revision,
            world_revision=current.world_revision,
            previous_catalog_revision=previous.catalog_revision,
            catalog_revision=current.catalog_revision,
            previous_catalog_content_sha256=(
                None if kind is PlanningSceneDeltaKind.RESYNC else previous.catalog_content_sha256
            ),
            catalog_content_sha256=None if kind is PlanningSceneDeltaKind.RESYNC else current.catalog_content_sha256,
            previous_geometry_revision=previous.geometry_revision,
            geometry_revision=current.geometry_revision,
            previous_transform_revision=previous.transform_revision,
            transform_revision=current.transform_revision,
            previous_attachment_revision=previous.attachment_revision,
            attachment_revision=current.attachment_revision,
            kind=kind,
            catalog=catalog,
            state=state,
            attachments=attachments,
            resync_required=resync_required,
        )

    def _planning_scene_delta_impl(self, base_sequence: int, environment_index: int = 0) -> PlanningSceneDelta:
        operation = "world.planning_scene_delta"
        runtime = self._planning_require_authority(operation)
        environment = self._planning_environment(environment_index, operation)
        environment_runtime = runtime.environments[environment]
        base_sequence = self._planning_sequence_value(base_sequence)
        current = environment_runtime.history[environment_runtime.sequence]
        base = environment_runtime.history.get(base_sequence)
        if environment_runtime.force_resync or base is None:
            delta = self._planning_delta_value(
                current,
                current,
                base_sequence,
                PlanningSceneDeltaKind.RESYNC,
                resync_required=True,
            )
            environment_runtime.force_resync = False
            return delta
        if base.sequence == current.sequence:
            raise PlanningSceneDeltaContinuityError(
                "no committed planning delta exists after base_sequence",
                operation=operation,
                world_id=self.world_id,
            ) from None
        if current.catalog_revision != base.catalog_revision:
            return self._planning_delta_value(
                current,
                base,
                base_sequence,
                PlanningSceneDeltaKind.STRUCTURAL,
                catalog=environment_runtime.catalog,
                state=current,
            )
        if (
            current.attachment_revision != base.attachment_revision
            and current.transform_revision == base.transform_revision
        ):
            return self._planning_delta_value(
                current,
                base,
                base_sequence,
                PlanningSceneDeltaKind.ATTACHMENT,
                attachments=current.attachments,
            )
        return self._planning_delta_value(
            current,
            base,
            base_sequence,
            PlanningSceneDeltaKind.STATE,
            state=current,
        )

    @staticmethod
    def _planning_requested_representation(value: object) -> PlanningGeometryRepresentation:
        if type(value) is PlanningGeometryRepresentation:
            return value
        canonical: object = None
        if _planning_actual_base(value, (str,)) is str:
            try:
                source_length = str.__len__(value)  # type: ignore[arg-type]
                if source_length <= 512:
                    canonical = str.__str__(value)
            except BaseException:
                canonical = None
        if (
            type(canonical) is str
            and 0 < len(canonical) <= 512
            and "\x00" not in canonical
            and not any(0xD800 <= ord(character) <= 0xDFFF for character in canonical)
            and len(canonical.encode("utf-8")) <= 4096
        ):
            for representation in PlanningGeometryRepresentation:
                if representation.value == canonical:
                    return representation
        raise PlanningSceneRepresentationError(
            "requested planning geometry representation is unavailable",
            operation="world.resolve_planning_geometry",
        ) from None

    @staticmethod
    def _planning_geometry_identity(value: object) -> str:
        canonical: object = None
        if _planning_actual_base(value, (str,)) is str:
            try:
                source_length = str.__len__(value)  # type: ignore[arg-type]
                if source_length <= 512:
                    canonical = str.__str__(value)
            except BaseException:
                canonical = None
        valid_text = (
            type(canonical) is str
            and 0 < len(canonical) <= 512
            and "\x00" not in canonical
            and not any(0xD800 <= ord(character) <= 0xDFFF for character in canonical)
        )
        if not valid_text:
            raise PlanningSceneContractError(
                "planning geometry ID is invalid",
                operation="world.resolve_planning_geometry",
            ) from None
        assert type(canonical) is str
        if len(canonical.encode("utf-8")) > 1024 or re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.:/-]*", canonical) is None:
            raise PlanningSceneContractError(
                "planning geometry ID is invalid",
                operation="world.resolve_planning_geometry",
            ) from None
        return canonical

    def _resolve_planning_geometry_impl(
        self,
        geometry_id: str,
        representation: PlanningGeometryRepresentation | None = None,
        environment_index: int = 0,
    ) -> PlanningGeometryLease:
        operation = "world.resolve_planning_geometry"
        runtime = self._planning_require_authority(operation)
        environment = self._planning_environment(environment_index, operation)
        environment_runtime = runtime.environments[environment]
        identity = self._planning_geometry_identity(geometry_id)
        catalog = environment_runtime.catalog
        assert catalog is not None
        geometry = next((item for item in catalog.geometries if item.geometry_id == identity), None)
        if geometry is None:
            raise PlanningSceneNotFoundError(
                "planning geometry ID does not exist",
                operation=operation,
                world_id=self.world_id,
            ) from None
        requested = (
            geometry.representation
            if representation is None
            else self._planning_requested_representation(representation)
        )
        if requested is not geometry.representation or geometry.resolution_key is None:
            raise PlanningSceneRepresentationError(
                "requested planning geometry representation is unavailable",
                operation=operation,
                world_id=self.world_id,
            ) from None
        raw = environment_runtime.raw_resources.get(identity)
        if raw is None:
            raise PlanningSceneRepresentationError(
                "planning geometry has no materializable resource",
                operation=operation,
                world_id=self.world_id,
            ) from None
        if type(raw) is not tuple or len(raw) != 6:
            raise PlanningSceneContractError(
                "planning geometry resource metadata is invalid",
                operation=operation,
                world_id=self.world_id,
            ) from None
        content, raw_representation, resource_id, profile, vertex_shape, index_shape = raw
        layout = geometry.resource_layout
        assert type(layout) is PlanningGeometryResourceLayout
        if type(content) is not bytes or raw_representation is not requested:
            raise PlanningSceneHashMismatchError(
                "planning geometry content hash does not match the catalog",
                operation=operation,
                world_id=self.world_id,
            ) from None
        digest = hashlib.sha256(content).hexdigest()
        if digest != geometry.sha256:
            raise PlanningSceneHashMismatchError(
                "planning geometry content hash does not match the catalog",
                operation=operation,
                world_id=self.world_id,
            ) from None
        exact_mesh_shapes = (
            type(vertex_shape) is tuple
            and len(vertex_shape) == 2
            and all(type(value) is int for value in vertex_shape)
            and type(index_shape) is tuple
            and len(index_shape) == 2
            and all(type(value) is int for value in index_shape)
        )
        if (
            type(resource_id) is not str
            or resource_id != geometry.resource_id
            or profile is not geometry.content_profile
            or not exact_mesh_shapes
            or vertex_shape != layout.vertex_shape
            or index_shape != layout.index_shape
        ):
            raise PlanningSceneContractError(
                "planning geometry resource metadata does not match the catalog layout",
                operation=operation,
                world_id=self.world_id,
            ) from None
        key = geometry.resolution_key
        assert key is not None
        cached = runtime.storage_cache.get(key)
        if cached is None:
            locator = _planning_id("cache", identity, requested.value, digest)
            cached = bytes(content), locator
            if hashlib.sha256(cached[0]).hexdigest() != digest:
                raise PlanningSceneHashMismatchError(
                    "planning geometry cache verification failed",
                    operation=operation,
                    world_id=self.world_id,
                ) from None
            runtime.storage_cache[key] = cached
            runtime.geometry_materializations += 1
        immutable_content, locator = cached
        environment_runtime.lease_serial += 1
        lease_token = _planning_id(
            "lease",
            self.world_id,
            str(environment_runtime.generation),
            str(environment),
            identity,
            str(environment_runtime.lease_serial),
        )
        descriptor = PlanningGeometryResourceDescriptor(
            self._session.descriptor.provider_id,
            self.world_id,
            environment_runtime.generation,
            environment,
            catalog.catalog_revision,
            catalog.geometry_revision,
            catalog.content_sha256,
            lease_token,
            resource_id,
            identity,
            requested,
            PlanningGeometryStorageKind.IMMUTABLE_MEMORY,
            locator,
            profile,
            "m",
            PlanningGeometryAxisConvention.RIGHT_HANDED_Z_UP,
            layout,
            len(immutable_content),
            digest,
        )
        descriptor.validate_against(catalog)
        return _FakePlanningGeometryLease(descriptor, immutable_content, environment_runtime.lease_epoch)

    def _planning_commit_state(self, environment_indices: tuple[int, ...] | None = None) -> None:
        runtime = self._planning_runtime
        selected = tuple(runtime.environments) if environment_indices is None else environment_indices
        for environment_index in selected:
            environment_runtime = runtime.environments[environment_index]
            counters = (
                environment_runtime.sequence,
                environment_runtime.world_revision,
                environment_runtime.transform_revision,
            )
            if any(counter >= 2**63 - 1 for counter in counters):
                environment_runtime.force_resync = True
                state = self._capture_planning_state(environment_index)
                environment_runtime.history = {state.sequence: state}
                continue
            environment_runtime.sequence += 1
            environment_runtime.world_revision += 1
            environment_runtime.transform_revision += 1
            state = self._capture_planning_state(environment_index)
            history = environment_runtime.history
            history[state.sequence] = state
            while len(history) > 128:
                del history[next(iter(history))]

    def _planning_reset(self, environment_indices: tuple[int, ...]) -> None:
        runtime = self._planning_runtime
        if any(runtime.environments[index].generation >= 2**63 - 1 for index in environment_indices):
            raise PlanningSceneContractError(
                "planning generation is exhausted",
                operation="world.reset",
                world_id=self.world_id,
            ) from None
        for index in environment_indices:
            environment_runtime = runtime.environments[index]
            environment_runtime.lease_epoch.live = False
            environment_runtime.lease_epoch = _PlanningLeaseEpoch()
            environment_runtime.raw_resources.clear()
            environment_runtime.catalog = None
            environment_runtime.history.clear()
            environment_runtime.generation += 1
            environment_runtime.sequence = 1
            environment_runtime.world_revision = 1
            environment_runtime.catalog_revision = 1
            environment_runtime.geometry_revision = 1
            environment_runtime.transform_revision = 1
            environment_runtime.attachment_revision = 1
            environment_runtime.force_resync = True
            environment_runtime.lease_serial = 0
        self._initialize_planning_scene(environment_indices)

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


class FakePlanningWorld(FakeWorld):
    """Demand-only FakeWorld variant implementing ``planning.scene@2``.

    Selecting a separate concrete type keeps the ordinary ``FakeWorld``
    object layout and its reset/step/scene-command paths free of planning
    state, callbacks, lookups and branches.
    """

    def __init__(self, session: FakeSession, spec: WorldSpec, generation: int) -> None:
        super().__init__(session, spec, generation)
        self._planning_runtime = _FakePlanningRuntime(
            threading.get_ident(),
            {index: _FakePlanningEnvironmentRuntime(generation) for index in range(spec.environments.count)},
        )
        admission_failed = False
        try:
            self._initialize_planning_scene()
        except PlanningSceneError as caught:
            _snapshot_and_scrub_planning_error(caught, _PLANNING_ERROR_TYPES)
            admission_failed = True
        except Exception as caught:
            try:
                BaseException.__setattr__(caught, "__traceback__", None)
                BaseException.__setattr__(caught, "__cause__", None)
                BaseException.__setattr__(caught, "__context__", None)
            except BaseException:
                pass
            admission_failed = True
        except BaseException:
            self._close(notify_session=False)
            raise
        if admission_failed:
            self._close(notify_session=False)
            raise PlanningSceneIncompleteError(
                "planning-scene native admission failed",
                operation="planning_scene.preflight",
                backend_id=session.descriptor.provider_id,
                world_id=spec.world_id,
            ) from None

    @_planning_error_boundary
    def planning_scene_catalog(self, environment_index: int = 0) -> PlanningSceneCatalog:
        return self._planning_scene_catalog_impl(environment_index)

    @_planning_error_boundary
    def planning_scene_state(self, environment_index: int = 0) -> PlanningSceneState:
        return self._planning_scene_state_impl(environment_index)

    @_planning_error_boundary
    def planning_scene_delta(self, base_sequence: int, environment_index: int = 0) -> PlanningSceneDelta:
        return self._planning_scene_delta_impl(base_sequence, environment_index)

    @_planning_error_boundary
    def resolve_planning_geometry(
        self,
        geometry_id: str,
        representation: PlanningGeometryRepresentation | None = None,
        environment_index: int = 0,
    ) -> PlanningGeometryLease:
        return self._resolve_planning_geometry_impl(geometry_id, representation, environment_index)

    def reset(self, environment_indices: Iterable[int] | None = None) -> ResetResult:
        result = super().reset(environment_indices)
        self._planning_reset(result.environment_indices)
        return result

    def step(self, count: int = 1) -> Tick:
        result = super().step(count)
        self._planning_commit_state()
        return result

    def apply_scene_command(self, command: SceneCommand) -> SceneCommandResult:
        result = super().apply_scene_command(command)
        if result.status is SceneCommandStatus.APPLIED:
            self._planning_commit_state((command.environment_index,))
        return result

    def _close(self, *, notify_session: bool) -> None:
        if self._state is not WorldState.CLOSED:
            runtime = self._planning_runtime
            for environment_runtime in runtime.environments.values():
                environment_runtime.lease_epoch.live = False
                environment_runtime.history.clear()
                environment_runtime.catalog = None
                environment_runtime.raw_resources.clear()
            runtime.storage_cache.clear()
        super()._close(notify_session=notify_session)
