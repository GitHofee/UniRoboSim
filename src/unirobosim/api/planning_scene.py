"""Portable, capability-gated planning-scene contracts.

``planning.scene@2`` is deliberately separate from :class:`World`.  A backend
that does not implement and validate this protocol simply omits the capability.
The values in this module contain no backend object, native handle, filesystem
path, planner policy, or browser reconstruction data.
"""

from __future__ import annotations

import hashlib
import json
import math
import re
from collections.abc import Callable
from dataclasses import dataclass, fields, replace
from enum import StrEnum
from functools import wraps
from typing import Any, Protocol, TypeVar, cast, runtime_checkable

from .errors import (
    PlanningSceneContractError,
    PlanningSceneDeltaContinuityError,
    PlanningSceneError,
    PlanningSceneIncompleteError,
    PlanningSceneStaleGenerationError,
    _snapshot_and_scrub_planning_error,
)
from .frozen import FrozenMap
from .values import Tick

PLANNING_SCENE_CAPABILITY_ID = "planning.scene@2"
PLANNING_SCENE_SCHEMA_VERSION = "unirobosim.planning-scene/v2"
PLANNING_FRAME_DECLARATIONS_SCHEMA_VERSION = "unirobosim.planning-frame-declarations/v1"
PLANNING_SYSTEM_ENTITY_ID = "system.simulator_effective"
PLANNING_SYSTEM_ENTITY_PATH = "/system/simulator_effective"
PLANNING_GEOMETRY_READ_LIMIT_BYTES = 64 * 1024 * 1024

_MAX_COUNTER = 2**63 - 1
_MAX_RESOURCE_BYTES = 8 * 1024 * 1024 * 1024
_MAX_ITEMS = 100_000
_MAX_RELATIONSHIP_REFERENCES = 500_000
_MAX_CANONICAL_CATALOG_BYTES = 64 * 1024 * 1024
_MAX_TEXT_CODEPOINTS = 512
_MAX_TEXT_BYTES = 4096
_MAX_ID_BYTES = 1024
_QUATERNION_TOLERANCE = 1.0e-6
_TRANSFORM_TOLERANCE = 1.0e-6
_IDENTIFIER = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.:/-]*$")
_OPAQUE_IDENTIFIER = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]*$")
_SHA256 = re.compile(r"^[0-9a-f]{64}$")
_TYPE_INSPECTION_FAILED = object()
_ValueT = TypeVar("_ValueT")
_EnumT = TypeVar("_EnumT", bound=StrEnum)
_MethodResultT = TypeVar("_MethodResultT")

_PLANNING_VALUE_ERROR_TYPES: tuple[type[PlanningSceneError], ...] = (
    PlanningSceneContractError,
    PlanningSceneDeltaContinuityError,
    PlanningSceneStaleGenerationError,
    PlanningSceneError,
    PlanningSceneIncompleteError,
)


def _invalid(message: str, *, operation: str = "planning_scene.validate") -> PlanningSceneContractError:
    return PlanningSceneContractError(message, operation=operation)


def _preflight_invalid(message: str) -> PlanningSceneIncompleteError:
    return PlanningSceneIncompleteError(message, operation="planning_scene.preflight")


def _actual_base(value: object, allowed: tuple[type, ...]) -> object:
    """Classify by the real type MRO without consulting ``value.__class__``."""

    actual_type = type(value)
    try:
        mro = type.mro(actual_type)
    except BaseException:
        return _TYPE_INSPECTION_FAILED
    if type(mro) is not list or list.__len__(mro) > 256:
        return _TYPE_INSPECTION_FAILED
    for allowed_base in allowed:
        for index in range(list.__len__(mro)):
            if list.__getitem__(mro, index) is allowed_base:
                return allowed_base
    return None


def _text(value: object, label: str, *, identifier: bool = False, opaque: bool = False) -> str:
    if _actual_base(value, (str,)) is not str:
        raise _invalid(f"{label} must be a string") from None
    byte_limit = _MAX_ID_BYTES if identifier or opaque else _MAX_TEXT_BYTES
    try:
        source_length = str.__len__(value)  # type: ignore[arg-type]
    except BaseException:
        source_length = _MAX_TEXT_CODEPOINTS + 1
    if source_length > min(_MAX_TEXT_CODEPOINTS, byte_limit):
        raise _invalid(f"{label} exceeds its text budget") from None
    try:
        canonical = str.__str__(value)
    except BaseException:
        canonical = None
    if type(canonical) is not str or not canonical or len(canonical) > _MAX_TEXT_CODEPOINTS or "\x00" in canonical:
        raise _invalid(f"{label} must be bounded non-empty portable text") from None
    if any(0xD800 <= ord(character) <= 0xDFFF for character in canonical):
        raise _invalid(f"{label} must contain valid Unicode") from None
    encoded = canonical.encode("utf-8")
    if len(encoded) > byte_limit:
        raise _invalid(f"{label} exceeds its UTF-8 byte budget") from None
    if identifier and _IDENTIFIER.fullmatch(canonical) is None:
        raise _invalid(f"{label} must be a stable ASCII identifier") from None
    if opaque and _OPAQUE_IDENTIFIER.fullmatch(canonical) is None:
        raise _invalid(f"{label} must be an opaque scoped identifier") from None
    return canonical


def _sha256(value: object, label: str) -> str:
    result = _text(value, label, opaque=True)
    if _SHA256.fullmatch(result) is None:
        raise _invalid(f"{label} must be lowercase SHA-256") from None
    return result


def _logical_path(value: object, label: str) -> str:
    result = _text(value, label)
    if result == "/" or not result.startswith("/") or result.endswith("/") or "//" in result:
        raise _invalid(f"{label} must be an absolute non-root logical path") from None
    segments = result[1:].split("/")
    if any(
        not segment or segment in {".", ".."} or _OPAQUE_IDENTIFIER.fullmatch(segment) is None for segment in segments
    ):
        raise _invalid(f"{label} contains an invalid path segment") from None
    return result


def _integer(
    value: object,
    label: str,
    *,
    minimum: int = 0,
    maximum: int = _MAX_COUNTER,
) -> int:
    base = _actual_base(value, (bool, int))
    if base is not int:
        raise _invalid(f"{label} must be an integer") from None
    try:
        result = int.__int__(value)  # type: ignore[arg-type]
    except BaseException:
        result = None
    if type(result) is not int or not minimum <= result <= maximum:
        raise _invalid(f"{label} is outside its bounded integer range") from None
    return result


def _finite(value: object, label: str, *, positive: bool = False, non_negative: bool = False) -> float:
    base = _actual_base(value, (bool, int, float))
    if base not in (int, float):
        raise _invalid(f"{label} must be numeric") from None
    try:
        result = float.__float__(value) if base is float else float(int.__int__(value))  # type: ignore[arg-type]
    except BaseException:
        result = math.nan
    if not math.isfinite(result):
        raise _invalid(f"{label} must be finite") from None
    if positive and result <= 0.0:
        raise _invalid(f"{label} must be positive") from None
    if non_negative and result < 0.0:
        raise _invalid(f"{label} must be non-negative") from None
    return 0.0 if result == 0.0 else result


def _vector(value: object, size: int, label: str) -> tuple[float, ...]:
    if type(value) is not tuple or tuple.__len__(value) != size:
        raise _invalid(f"{label} must be an immutable {size}-element tuple") from None
    return tuple(_finite(tuple.__getitem__(value, index), f"{label}[{index}]") for index in range(size))


def _typed_tuple(
    value: object,
    item_type: type[_ValueT],
    label: str,
    *,
    allow_empty: bool = True,
) -> tuple[_ValueT, ...]:
    if type(value) is not tuple or tuple.__len__(value) > _MAX_ITEMS or (not allow_empty and not value):
        raise _invalid(f"{label} must be a bounded immutable tuple") from None
    for index in range(tuple.__len__(value)):
        if type(tuple.__getitem__(value, index)) is not item_type:
            raise _invalid(f"{label} contains an invalid value") from None
    return cast(tuple[_ValueT, ...], value)


def _identifier_tuple(value: object, label: str, *, allow_empty: bool = True, ordered: bool = False) -> tuple[str, ...]:
    if type(value) is not tuple or tuple.__len__(value) > _MAX_ITEMS or (not allow_empty and not value):
        raise _invalid(f"{label} must be a bounded immutable tuple") from None
    result = tuple(
        _text(tuple.__getitem__(value, index), f"{label} item", identifier=True) for index in range(len(value))
    )
    if len(set(result)) != len(result):
        raise _invalid(f"{label} must not contain duplicates") from None
    if not ordered and result != tuple(sorted(result)):
        raise _invalid(f"{label} must use deterministic identifier order") from None
    return result


def _enum(value: object, enum_type: type[_EnumT], label: str) -> _EnumT:
    canonical = _text(value, label)
    for member in enum_type:
        if member.value == canonical:
            return member
    raise _invalid(f"{label} is unsupported") from None


def _unique_sorted(values: tuple[object, ...], attribute: str, label: str) -> None:
    identities = tuple(getattr(value, attribute) for value in values)
    if len(set(identities)) != len(identities):
        raise _invalid(f"{label} must use unique identifiers") from None
    if identities != tuple(sorted(identities)):
        raise _invalid(f"{label} must use deterministic identifier order") from None


def _reject_cycles(parent_by_child: dict[str, str], label: str) -> None:
    complete = frozenset(parent_by_child)
    finished: set[str] = set()
    for start in sorted(complete):
        current = start
        trail: list[str] = []
        positions: set[str] = set()
        while current in complete and current not in finished:
            if current in positions:
                raise _invalid(f"{label} must not contain a cycle") from None
            positions.add(current)
            trail.append(current)
            current = parent_by_child[current]
        finished.update(trail)


def _tick(value: object, label: str) -> Tick:
    if type(value) is not Tick:
        raise _invalid(f"{label} must be an exact Tick") from None
    step_index = _integer(value.step_index, f"{label} step_index")
    sim_time_seconds = _finite(value.sim_time_seconds, f"{label} sim_time_seconds", non_negative=True)
    return Tick(step_index, sim_time_seconds)


class PlanningEntityKind(StrEnum):
    RIGID_OBJECT = "rigid_object"
    ARTICULATION = "articulation"
    ROBOT = "robot"
    OTHER = "other"


class PlanningFrameKind(StrEnum):
    WORLD = "world"
    ENTITY = "entity"
    LINK = "link"
    JOINT = "joint"
    NAMED = "named"


class PlanningFrameRole(StrEnum):
    EE = "ee"
    TOOL = "tool"
    SENSOR = "sensor"
    ANNOTATION = "annotation"


class PlanningFrameSourceKind(StrEnum):
    LINK = "link"
    JOINT = "joint"
    NATIVE_NAMED = "native_named"


class PlanningJointType(StrEnum):
    REVOLUTE = "revolute"
    CONTINUOUS = "continuous"
    PRISMATIC = "prismatic"
    FIXED = "fixed"


class PlanningGeometryPurpose(StrEnum):
    COLLISION = "collision"
    PLANNING_PROXY = "planning_proxy"


class PlanningGeometryRepresentation(StrEnum):
    HALFSPACE = "halfspace"
    BOX = "box"
    SPHERE = "sphere"
    CAPSULE = "capsule"
    CYLINDER = "cylinder"
    COMPOUND = "compound"
    CONVEX_MESH = "convex_mesh"
    TRIANGLE_MESH = "triangle_mesh"
    SDF = "sdf"
    VOXEL = "voxel"
    HEIGHTFIELD = "heightfield"


class PlanningGeometryMotionClass(StrEnum):
    STATIC = "static"
    KINEMATIC = "kinematic"
    DYNAMIC = "dynamic"


class PlanningGeometryStorageKind(StrEnum):
    IMMUTABLE_MEMORY = "immutable_memory"
    READ_ONLY_FILE = "read_only_file"
    SHARED_MEMORY = "shared_memory"


class PlanningGeometryAxisConvention(StrEnum):
    """Canonical axis convention for every ``planning.scene@2`` value."""

    RIGHT_HANDED_Z_UP = "right_handed_z_up"


class PlanningGeometryDType(StrEnum):
    UINT8 = "uint8"
    FLOAT32 = "float32"
    FLOAT64 = "float64"
    INT32 = "int32"
    UINT32 = "uint32"


class PlanningGeometryContentProfile(StrEnum):
    """Frozen byte layouts and sample semantics for resource geometry.

    SDF samples are signed metres: negative is inside, zero is the surface,
    and positive is outside. Occupancy samples accept only ``0`` (free) and
    ``1`` (occupied). A heightfield sample is a signed metre offset along the
    positive local Z/up axis from ``grid_origin_m.z`` at its local X/Y sample.
    All resources are canonical right-handed Z-up; adapters for a Y-up backend
    rotate data and transforms at their boundary.
    """

    MESH_TRIANGLES_RAW_LE_V1 = "mesh.triangles-raw-le/v1"
    SDF_DENSE_RAW_LE_V1 = "grid.sdf-negative-inside-metres-raw-le/v1"
    VOXEL_OCCUPANCY_UINT8_V1 = "grid.occupancy-u8-zero-free-one-occupied/v1"
    HEIGHTFIELD_DENSE_RAW_LE_V1 = "grid.heightfield-positive-z-offset-metres-raw-le/v1"


PLANNING_SDF_SIGN_CONVENTION = "negative-inside-zero-surface-positive-outside"
PLANNING_VOXEL_OCCUPANCY_CONVENTION = "uint8-zero-free-one-occupied"
PLANNING_HEIGHTFIELD_SAMPLE_CONVENTION = "signed-metre-offset-along-positive-local-z-from-origin"
PLANNING_GRID_INDEX_ORDER = "shape-axes-xy-or-xyz-last-axis-contiguous"


_RESOURCE_PROFILE_BY_REPRESENTATION = {
    PlanningGeometryRepresentation.CONVEX_MESH: PlanningGeometryContentProfile.MESH_TRIANGLES_RAW_LE_V1,
    PlanningGeometryRepresentation.TRIANGLE_MESH: PlanningGeometryContentProfile.MESH_TRIANGLES_RAW_LE_V1,
    PlanningGeometryRepresentation.SDF: PlanningGeometryContentProfile.SDF_DENSE_RAW_LE_V1,
    PlanningGeometryRepresentation.VOXEL: PlanningGeometryContentProfile.VOXEL_OCCUPANCY_UINT8_V1,
    PlanningGeometryRepresentation.HEIGHTFIELD: PlanningGeometryContentProfile.HEIGHTFIELD_DENSE_RAW_LE_V1,
}

_DTYPE_BYTES = {
    PlanningGeometryDType.UINT8: 1,
    PlanningGeometryDType.FLOAT32: 4,
    PlanningGeometryDType.FLOAT64: 8,
    PlanningGeometryDType.INT32: 4,
    PlanningGeometryDType.UINT32: 4,
}


class PlanningSceneDeltaKind(StrEnum):
    STRUCTURAL = "structural"
    STATE = "state"
    ATTACHMENT = "attachment"
    RESYNC = "resync"


def _planning_method_boundary(
    function: Callable[..., _MethodResultT],
) -> Callable[..., _MethodResultT]:
    """Detach method-call arguments when validation fails."""

    @wraps(function)
    def wrapped(*args: object, **kwargs: object) -> _MethodResultT:
        failure: tuple[type[PlanningSceneError], str, str] | None = None
        try:
            return function(*args, **kwargs)
        except PlanningSceneError as caught:
            snapshot = _snapshot_and_scrub_planning_error(caught, _PLANNING_VALUE_ERROR_TYPES)
            failure = snapshot[0], snapshot[1], snapshot[2]
        except Exception as caught:
            failure = PlanningSceneContractError, "planning-scene value validation failed", "planning_scene.validate"
            try:
                BaseException.__setattr__(caught, "__traceback__", None)
                BaseException.__setattr__(caught, "__cause__", None)
                BaseException.__setattr__(caught, "__context__", None)
            except BaseException:
                pass
        args = ()
        kwargs = {}
        assert failure is not None
        error_type, message, operation = failure
        raise error_type(message, operation=operation) from None

    return cast(Callable[..., _MethodResultT], wrapped)


class _PlanningValueMeta(type):
    """Scrub failed constructor frames before exposing a public error."""

    def __call__(cls, *args: object, **kwargs: object) -> object:
        failure: tuple[type[PlanningSceneError], str, str] | None = None
        try:
            return super().__call__(*args, **kwargs)
        except PlanningSceneError as caught:
            snapshot = _snapshot_and_scrub_planning_error(caught, _PLANNING_VALUE_ERROR_TYPES)
            failure = snapshot[0], snapshot[1], snapshot[2]
        except Exception as caught:
            failure = PlanningSceneContractError, "planning-scene value validation failed", "planning_scene.validate"
            try:
                BaseException.__setattr__(caught, "__traceback__", None)
                BaseException.__setattr__(caught, "__cause__", None)
                BaseException.__setattr__(caught, "__context__", None)
            except BaseException:
                pass
        args = ()
        kwargs = {}
        assert failure is not None
        error_type, message, operation = failure
        raise error_type(message, operation=operation) from None


class _PlanningValue(metaclass=_PlanningValueMeta):
    __slots__ = ()


@dataclass(frozen=True, slots=True)
class PlanningFrameSource(_PlanningValue):
    kind: PlanningFrameSourceKind
    name: str

    def __post_init__(self) -> None:
        object.__setattr__(self, "kind", _enum(self.kind, PlanningFrameSourceKind, "planning frame source kind"))
        object.__setattr__(self, "name", _text(self.name, "planning frame source name"))


@dataclass(frozen=True, slots=True)
class PlanningFrameDeclaration(_PlanningValue):
    semantic_key: str
    role: PlanningFrameRole
    owner_link_name: str | None
    source: PlanningFrameSource

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "semantic_key",
            _text(self.semantic_key, "planning frame semantic_key", identifier=True),
        )
        object.__setattr__(self, "role", _enum(self.role, PlanningFrameRole, "planning frame role"))
        if self.owner_link_name is not None:
            object.__setattr__(
                self,
                "owner_link_name",
                _text(self.owner_link_name, "planning frame owner_link"),
            )
        if type(self.source) is not PlanningFrameSource:
            raise _invalid("planning frame declaration requires an exact source") from None


@dataclass(frozen=True, slots=True)
class PlanningFrameDeclarations(_PlanningValue):
    component_sha256: str
    entries: tuple[PlanningFrameDeclaration, ...]
    schema_version: str = PLANNING_FRAME_DECLARATIONS_SCHEMA_VERSION

    def __post_init__(self) -> None:
        schema = _text(self.schema_version, "planning frame declarations schema")
        if schema != PLANNING_FRAME_DECLARATIONS_SCHEMA_VERSION:
            raise _invalid("planning frame declarations schema is unsupported") from None
        object.__setattr__(self, "schema_version", schema)
        object.__setattr__(
            self,
            "component_sha256",
            _sha256(self.component_sha256, "planning frame component_sha256"),
        )
        entries = _typed_tuple(self.entries, PlanningFrameDeclaration, "planning frame declarations")
        _unique_sorted(entries, "semantic_key", "planning frame declarations")


@_planning_method_boundary
def parse_planning_frame_declarations(value: object) -> PlanningFrameDeclarations | None:
    """Validate FastSim's immutable, canonical planning-frame projection.

    The metadata record is intentionally narrow.  Adapters receive only locked
    semantic intent and an exact source selector; they never infer frames from
    native names or enumerate arbitrary SDK transforms.
    """

    if value is None:
        return None
    try:
        if type(value) is not FrozenMap or frozenset(value) != {
            "schema",
            "component_sha256",
            "entries",
        }:
            raise _invalid("planning frame declarations must use the canonical mapping shape") from None
        schema = value["schema"]
        component_sha256 = value["component_sha256"]
        raw_entries = value["entries"]
        if type(raw_entries) is not tuple or len(raw_entries) > _MAX_ITEMS:
            raise _invalid("planning frame declaration entries must be a bounded immutable tuple") from None
        entries: list[PlanningFrameDeclaration] = []
        for index in range(tuple.__len__(raw_entries)):
            raw_entry = tuple.__getitem__(raw_entries, index)
            if type(raw_entry) is not FrozenMap or frozenset(raw_entry) != {
                "semantic_key",
                "role",
                "owner_link",
                "source",
            }:
                raise _invalid("planning frame declaration entry has an invalid mapping shape") from None
            raw_source = raw_entry["source"]
            if type(raw_source) is not FrozenMap or frozenset(raw_source) != {"kind", "name"}:
                raise _invalid("planning frame declaration source has an invalid mapping shape") from None
            entries.append(
                PlanningFrameDeclaration(
                    raw_entry["semantic_key"],
                    raw_entry["role"],
                    raw_entry["owner_link"],
                    PlanningFrameSource(raw_source["kind"], raw_source["name"]),
                )
            )
        return PlanningFrameDeclarations(component_sha256, tuple(entries), schema)
    except PlanningSceneError:
        raise _preflight_invalid("planning frame declarations are invalid") from None


@dataclass(frozen=True, slots=True)
class PlanningGeometryResourceLayout(_PlanningValue):
    """Catalog-pinned decoding layout for one raw resource payload.

    The layout is part of the catalog's canonical content digest.  A resolver
    must return this exact layout, so the headerless mesh vertex/index split or
    dense-grid interpretation cannot change while retaining the same resource
    bytes and resolution key.
    """

    representation: PlanningGeometryRepresentation
    content_profile: PlanningGeometryContentProfile
    vertex_dtype: PlanningGeometryDType | None = None
    vertex_shape: tuple[int, int] | None = None
    index_dtype: PlanningGeometryDType | None = None
    index_shape: tuple[int, int] | None = None
    grid_dtype: PlanningGeometryDType | None = None
    grid_shape: tuple[int, ...] | None = None
    grid_spacing_m: tuple[float, ...] | None = None
    grid_origin_m: tuple[float, float, float] | None = None

    def __post_init__(self) -> None:
        representation = _enum(self.representation, PlanningGeometryRepresentation, "resource representation")
        if representation not in _RESOURCE_PROFILE_BY_REPRESENTATION:
            raise _invalid("inline representations cannot declare resource layouts") from None
        profile = _enum(self.content_profile, PlanningGeometryContentProfile, "resource content_profile")
        if profile is not _RESOURCE_PROFILE_BY_REPRESENTATION[representation]:
            raise _invalid("resource layout profile does not match its representation") from None
        object.__setattr__(self, "representation", representation)
        object.__setattr__(self, "content_profile", profile)

        if representation in {
            PlanningGeometryRepresentation.TRIANGLE_MESH,
            PlanningGeometryRepresentation.CONVEX_MESH,
        }:
            if (
                self.vertex_dtype is None
                or self.vertex_shape is None
                or self.index_dtype is None
                or self.index_shape is None
            ):
                raise _invalid("mesh resource layouts require vertex and index schemas") from None
            vertex_dtype = _enum(self.vertex_dtype, PlanningGeometryDType, "resource vertex_dtype")
            index_dtype = _enum(self.index_dtype, PlanningGeometryDType, "resource index_dtype")
            if vertex_dtype not in {
                PlanningGeometryDType.FLOAT32,
                PlanningGeometryDType.FLOAT64,
            } or index_dtype not in {
                PlanningGeometryDType.INT32,
                PlanningGeometryDType.UINT32,
            }:
                raise _invalid("mesh resource layout dtypes are unsupported") from None
            object.__setattr__(self, "vertex_dtype", vertex_dtype)
            object.__setattr__(self, "vertex_shape", _shape(self.vertex_shape, "resource vertex_shape", width=3))
            object.__setattr__(self, "index_dtype", index_dtype)
            object.__setattr__(self, "index_shape", _shape(self.index_shape, "resource index_shape", width=3))
            if any(
                value is not None
                for value in (self.grid_dtype, self.grid_shape, self.grid_spacing_m, self.grid_origin_m)
            ):
                raise _invalid("mesh resource layouts cannot declare grid schemas") from None
            _ = self.decoded_byte_size
            return

        if any(
            value is not None for value in (self.vertex_dtype, self.vertex_shape, self.index_dtype, self.index_shape)
        ):
            raise _invalid("grid resource layouts cannot declare mesh schemas") from None
        if any(value is None for value in (self.grid_dtype, self.grid_shape, self.grid_spacing_m, self.grid_origin_m)):
            raise _invalid("grid resource layouts require dtype, shape, spacing and local origin") from None
        rank = 2 if representation is PlanningGeometryRepresentation.HEIGHTFIELD else 3
        dtype = _enum(self.grid_dtype, PlanningGeometryDType, "resource grid_dtype")
        allowed_dtypes = (
            {PlanningGeometryDType.UINT8}
            if representation is PlanningGeometryRepresentation.VOXEL
            else {PlanningGeometryDType.FLOAT32, PlanningGeometryDType.FLOAT64}
        )
        if dtype not in allowed_dtypes:
            raise _invalid("grid resource layout dtype does not match its representation") from None
        shape = _grid_shape(self.grid_shape, "resource grid_shape", rank=rank)
        spacing = _vector(self.grid_spacing_m, rank, "resource grid_spacing_m")
        if any(value <= 0.0 for value in spacing):
            raise _invalid("resource grid_spacing_m must be positive") from None
        object.__setattr__(self, "grid_dtype", dtype)
        object.__setattr__(self, "grid_shape", shape)
        object.__setattr__(self, "grid_spacing_m", spacing)
        object.__setattr__(self, "grid_origin_m", _vector(self.grid_origin_m, 3, "resource grid_origin_m"))
        _ = self.decoded_byte_size

    @property
    def decoded_byte_size(self) -> int:
        if self.vertex_dtype is not None:
            assert self.vertex_shape is not None and self.index_dtype is not None and self.index_shape is not None
            vertex_bytes = self.vertex_shape[0] * self.vertex_shape[1] * _DTYPE_BYTES[self.vertex_dtype]
            index_bytes = self.index_shape[0] * self.index_shape[1] * _DTYPE_BYTES[self.index_dtype]
            decoded = vertex_bytes + index_bytes
        else:
            assert self.grid_dtype is not None and self.grid_shape is not None
            decoded = math.prod(self.grid_shape) * _DTYPE_BYTES[self.grid_dtype]
        if decoded > _MAX_RESOURCE_BYTES:
            raise _invalid("resource layout decoded bytes exceed the bounded resource size") from None
        return decoded

    @property
    def index_byte_offset(self) -> int | None:
        if self.vertex_dtype is None or self.vertex_shape is None:
            return None
        return self.vertex_shape[0] * self.vertex_shape[1] * _DTYPE_BYTES[self.vertex_dtype]


@dataclass(frozen=True, slots=True)
class PlanningGeometryLocalPose(_PlanningValue):
    """A geometry-local pose with no synthetic frame identity."""

    position_m: tuple[float, float, float] = (0.0, 0.0, 0.0)
    orientation_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)

    def __post_init__(self) -> None:
        object.__setattr__(self, "position_m", _vector(self.position_m, 3, "local pose position_m"))
        object.__setattr__(
            self,
            "orientation_xyzw",
            _canonical_quaternion(self.orientation_xyzw, "local pose orientation_xyzw"),
        )


@dataclass(frozen=True, slots=True)
class PlanningPose(_PlanningValue):
    frame_id: str
    position_m: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]

    def __post_init__(self) -> None:
        object.__setattr__(self, "frame_id", _text(self.frame_id, "pose frame_id", identifier=True))
        object.__setattr__(self, "position_m", _vector(self.position_m, 3, "pose position_m"))
        object.__setattr__(self, "orientation_xyzw", _canonical_quaternion(self.orientation_xyzw, "pose orientation"))


@dataclass(frozen=True, slots=True)
class PlanningHalfspaceGeometry(_PlanningValue):
    """Exact infinite half-space: local boundary ``z=0`` and solid ``z<=0``."""


def _canonical_quaternion(value: object, label: str) -> tuple[float, float, float, float]:
    quaternion = _vector(value, 4, label)
    norm = math.sqrt(sum(component * component for component in quaternion))
    if norm <= 1.0e-12 or abs(norm - 1.0) > _QUATERNION_TOLERANCE:
        raise _invalid(f"{label} must be a unit quaternion") from None
    normalized = tuple(component / norm for component in quaternion)
    sign = 1.0
    if normalized[3] < 0.0:
        sign = -1.0
    elif normalized[3] == 0.0:
        first = next((component for component in normalized[:3] if component != 0.0), 0.0)
        if first < 0.0:
            sign = -1.0
    return tuple(0.0 if component * sign == 0.0 else component * sign for component in normalized)  # type: ignore[return-value]


@dataclass(frozen=True, slots=True)
class PlanningTwist(_PlanningValue):
    frame_id: str
    linear_m_s: tuple[float, float, float] = (0.0, 0.0, 0.0)
    angular_rad_s: tuple[float, float, float] = (0.0, 0.0, 0.0)

    def __post_init__(self) -> None:
        object.__setattr__(self, "frame_id", _text(self.frame_id, "twist frame_id", identifier=True))
        object.__setattr__(self, "linear_m_s", _vector(self.linear_m_s, 3, "twist linear_m_s"))
        object.__setattr__(self, "angular_rad_s", _vector(self.angular_rad_s, 3, "twist angular_rad_s"))


@dataclass(frozen=True, slots=True)
class PlanningPrimitiveGeometry(_PlanningValue):
    representation: PlanningGeometryRepresentation
    dimensions_m: tuple[float, ...]

    def __post_init__(self) -> None:
        representation = _enum(self.representation, PlanningGeometryRepresentation, "primitive representation")
        expected = {
            PlanningGeometryRepresentation.BOX: 3,
            PlanningGeometryRepresentation.SPHERE: 1,
            PlanningGeometryRepresentation.CAPSULE: 2,
            PlanningGeometryRepresentation.CYLINDER: 2,
        }
        if representation not in expected:
            raise _invalid("primitive representation must be box, sphere, capsule, or cylinder") from None
        dimensions = _vector(self.dimensions_m, expected[representation], "primitive dimensions_m")
        if any(value <= 0.0 for value in dimensions):
            raise _invalid("primitive dimensions_m must be positive") from None
        object.__setattr__(self, "representation", representation)
        object.__setattr__(self, "dimensions_m", dimensions)


@dataclass(frozen=True, slots=True)
class PlanningCompoundPart(_PlanningValue):
    part_id: str
    local_pose: PlanningGeometryLocalPose
    primitive: PlanningPrimitiveGeometry

    def __post_init__(self) -> None:
        object.__setattr__(self, "part_id", _text(self.part_id, "compound part_id", identifier=True))
        if (
            type(self.local_pose) is not PlanningGeometryLocalPose
            or type(self.primitive) is not PlanningPrimitiveGeometry
        ):
            raise _invalid("compound part requires exact local-pose and primitive values") from None


@dataclass(frozen=True, slots=True)
class PlanningCompoundGeometry(_PlanningValue):
    parts: tuple[PlanningCompoundPart, ...]

    def __post_init__(self) -> None:
        parts = _typed_tuple(self.parts, PlanningCompoundPart, "compound parts", allow_empty=False)
        _unique_sorted(parts, "part_id", "compound parts")


@dataclass(frozen=True, slots=True)
class PlanningEntityDescriptor(_PlanningValue):
    entity_id: str
    path: str
    kind: PlanningEntityKind
    enabled: bool
    root_frame_id: str
    link_ids: tuple[str, ...]
    frame_ids: tuple[str, ...]
    geometry_ids: tuple[str, ...]
    joint_ids: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        object.__setattr__(self, "entity_id", _text(self.entity_id, "entity_id", identifier=True))
        object.__setattr__(self, "path", _logical_path(self.path, "entity path"))
        object.__setattr__(self, "kind", _enum(self.kind, PlanningEntityKind, "entity kind"))
        if type(self.enabled) is not bool:
            raise _invalid("entity enabled must be an exact boolean") from None
        object.__setattr__(self, "root_frame_id", _text(self.root_frame_id, "entity root_frame_id", identifier=True))
        for name in ("link_ids", "frame_ids", "geometry_ids"):
            object.__setattr__(self, name, _identifier_tuple(getattr(self, name), f"entity {name}"))
        object.__setattr__(self, "joint_ids", _identifier_tuple(self.joint_ids, "entity joint_ids", ordered=True))
        if self.root_frame_id not in self.frame_ids:
            raise _invalid("entity root frame must be declared in frame_ids") from None


@dataclass(frozen=True, slots=True)
class PlanningLinkDescriptor(_PlanningValue):
    link_id: str
    entity_id: str
    authored_name: str
    frame_id: str
    parent_link_id: str | None = None
    geometry_ids: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        for name in ("link_id", "entity_id", "frame_id"):
            object.__setattr__(self, name, _text(getattr(self, name), name, identifier=True))
        object.__setattr__(self, "authored_name", _text(self.authored_name, "link authored_name"))
        if self.parent_link_id is not None:
            parent = _text(self.parent_link_id, "parent_link_id", identifier=True)
            if parent == self.link_id:
                raise _invalid("a link cannot parent itself") from None
            object.__setattr__(self, "parent_link_id", parent)
        object.__setattr__(self, "geometry_ids", _identifier_tuple(self.geometry_ids, "link geometry_ids"))


@dataclass(frozen=True, slots=True)
class PlanningJointDescriptor(_PlanningValue):
    joint_id: str
    entity_id: str
    authored_name: str
    parent_link_id: str
    child_link_id: str
    joint_type: PlanningJointType
    axis_frame_id: str
    axis_xyz: tuple[float, float, float]
    position_unit: str
    lower: float | None = None
    upper: float | None = None
    max_velocity: float | None = None
    max_effort: float | None = None

    def __post_init__(self) -> None:
        for name in ("joint_id", "entity_id", "parent_link_id", "child_link_id", "axis_frame_id"):
            object.__setattr__(self, name, _text(getattr(self, name), name, identifier=True))
        if self.parent_link_id == self.child_link_id:
            raise _invalid("joint parent and child links must differ") from None
        object.__setattr__(self, "authored_name", _text(self.authored_name, "joint authored_name"))
        joint_type = _enum(self.joint_type, PlanningJointType, "joint type")
        object.__setattr__(self, "joint_type", joint_type)
        axis = _vector(self.axis_xyz, 3, "joint axis_xyz")
        norm = math.sqrt(sum(component * component for component in axis))
        if abs(norm - 1.0) > _QUATERNION_TOLERANCE:
            raise _invalid("joint axis_xyz must be a unit vector") from None
        object.__setattr__(self, "axis_xyz", axis)
        unit = _text(self.position_unit, "joint position_unit")
        expected_unit = "m" if joint_type is PlanningJointType.PRISMATIC else "rad"
        if unit != expected_unit:
            raise _invalid("joint position_unit does not match joint type") from None
        object.__setattr__(self, "position_unit", unit)
        if (self.lower is None) != (self.upper is None):
            raise _invalid("joint limits must both be present or absent") from None
        if self.lower is not None:
            lower = _finite(self.lower, "joint lower")
            upper = _finite(self.upper, "joint upper")
            if lower > upper:
                raise _invalid("joint lower limit exceeds upper limit") from None
            object.__setattr__(self, "lower", lower)
            object.__setattr__(self, "upper", upper)
        for name in ("max_velocity", "max_effort"):
            value = getattr(self, name)
            if value is not None:
                object.__setattr__(self, name, _finite(value, f"joint {name}", positive=True))


@dataclass(frozen=True, slots=True)
class PlanningFrameDescriptor(_PlanningValue):
    frame_id: str
    kind: PlanningFrameKind
    parent_frame_id: str | None
    owner_entity_id: str | None
    owner_link_id: str | None
    role: PlanningFrameRole | None = None
    semantic_key: str | None = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "frame_id", _text(self.frame_id, "frame_id", identifier=True))
        kind = _enum(self.kind, PlanningFrameKind, "frame kind")
        object.__setattr__(self, "kind", kind)
        for name in ("parent_frame_id", "owner_entity_id", "owner_link_id"):
            value = getattr(self, name)
            if value is not None:
                object.__setattr__(self, name, _text(value, name, identifier=True))
        if kind is PlanningFrameKind.WORLD:
            if any(value is not None for value in (self.parent_frame_id, self.owner_entity_id, self.owner_link_id)):
                raise _invalid("world frame cannot have parent or owner") from None
        elif self.parent_frame_id is None or self.owner_entity_id is None:
            raise _invalid("non-world frames require a parent and entity owner") from None
        if self.parent_frame_id == self.frame_id:
            raise _invalid("a frame cannot parent itself") from None
        if kind is PlanningFrameKind.NAMED:
            if self.role is None or self.semantic_key is None:
                raise _invalid("named frames require role and semantic_key") from None
            object.__setattr__(self, "role", _enum(self.role, PlanningFrameRole, "frame role"))
            object.__setattr__(
                self,
                "semantic_key",
                _text(self.semantic_key, "frame semantic_key", identifier=True),
            )
        elif self.role is not None or self.semantic_key is not None:
            raise _invalid("physical frames cannot declare role or semantic_key") from None


@dataclass(frozen=True, slots=True)
class PlanningGeometryDescriptor(_PlanningValue):
    """One geometry with exactly one accepted representation in schema v2.

    Additional resolutions or planner-specific approximations are separate
    ``PLANNING_PROXY`` descriptors with distinct geometry IDs.  They are never
    silent alternatives for this descriptor.  Resource-backed geometry also
    carries its exact decoding layout; that layout participates in the catalog
    content digest independently of the resource byte digest.
    """

    geometry_id: str
    owner_entity_id: str
    owner_link_id: str | None
    parent_frame_id: str
    purpose: PlanningGeometryPurpose
    representation: PlanningGeometryRepresentation
    parent_frame_T_geometry: PlanningGeometryLocalPose
    scale: tuple[float, float, float]
    motion_class: PlanningGeometryMotionClass
    collision_group: int
    collision_mask: int
    provenance_sha256: str
    inline: PlanningHalfspaceGeometry | PlanningPrimitiveGeometry | PlanningCompoundGeometry | None = None
    resource_id: str | None = None
    sha256: str | None = None
    content_profile: PlanningGeometryContentProfile | None = None
    resource_layout: PlanningGeometryResourceLayout | None = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "geometry_id", _text(self.geometry_id, "geometry_id", identifier=True))
        object.__setattr__(
            self, "owner_entity_id", _text(self.owner_entity_id, "geometry owner_entity_id", identifier=True)
        )
        if self.owner_link_id is not None:
            object.__setattr__(
                self, "owner_link_id", _text(self.owner_link_id, "geometry owner_link_id", identifier=True)
            )
        object.__setattr__(
            self, "parent_frame_id", _text(self.parent_frame_id, "geometry parent_frame_id", identifier=True)
        )
        purpose = _enum(self.purpose, PlanningGeometryPurpose, "geometry purpose")
        representation = _enum(self.representation, PlanningGeometryRepresentation, "geometry representation")
        object.__setattr__(self, "purpose", purpose)
        object.__setattr__(self, "representation", representation)
        if type(self.parent_frame_T_geometry) is not PlanningGeometryLocalPose:
            raise _invalid("geometry parent_frame_T_geometry must be an exact local pose") from None
        scale = _vector(self.scale, 3, "geometry scale")
        if any(value <= 0.0 for value in scale):
            raise _invalid("geometry scale must be positive") from None
        object.__setattr__(self, "scale", scale)
        object.__setattr__(
            self,
            "motion_class",
            _enum(self.motion_class, PlanningGeometryMotionClass, "geometry motion_class"),
        )
        object.__setattr__(
            self, "collision_group", _integer(self.collision_group, "collision_group", maximum=2**32 - 1)
        )
        object.__setattr__(self, "collision_mask", _integer(self.collision_mask, "collision_mask", maximum=2**32 - 1))
        object.__setattr__(self, "provenance_sha256", _sha256(self.provenance_sha256, "geometry provenance_sha256"))
        inline_representations = {
            PlanningGeometryRepresentation.HALFSPACE,
            PlanningGeometryRepresentation.BOX,
            PlanningGeometryRepresentation.SPHERE,
            PlanningGeometryRepresentation.CAPSULE,
            PlanningGeometryRepresentation.CYLINDER,
            PlanningGeometryRepresentation.COMPOUND,
        }
        has_resource = any(
            value is not None for value in (self.resource_id, self.sha256, self.content_profile, self.resource_layout)
        )
        if representation in inline_representations:
            if has_resource or self.inline is None:
                raise _invalid("inline geometry cannot contain resource identity") from None
            if representation is PlanningGeometryRepresentation.HALFSPACE:
                if type(self.inline) is not PlanningHalfspaceGeometry or scale != (1.0, 1.0, 1.0):
                    raise _invalid("half-space geometry requires exact inline data and unit scale") from None
            elif representation is PlanningGeometryRepresentation.COMPOUND:
                if type(self.inline) is not PlanningCompoundGeometry:
                    raise _invalid("compound geometry requires exact compound data") from None
            elif type(self.inline) is not PlanningPrimitiveGeometry or self.inline.representation is not representation:
                raise _invalid("primitive geometry does not match its representation") from None
        else:
            if self.inline is not None or not all(
                value is not None
                for value in (self.resource_id, self.sha256, self.content_profile, self.resource_layout)
            ):
                raise _invalid("resource geometry requires a complete digest-pinned identity and layout") from None
            object.__setattr__(self, "resource_id", _text(self.resource_id, "geometry resource_id", identifier=True))
            object.__setattr__(self, "sha256", _sha256(self.sha256, "geometry sha256"))
            profile = _enum(
                self.content_profile,
                PlanningGeometryContentProfile,
                "geometry content_profile",
            )
            if profile is not _RESOURCE_PROFILE_BY_REPRESENTATION[representation]:
                raise _invalid("geometry content_profile does not match its representation") from None
            object.__setattr__(self, "content_profile", profile)
            if (
                type(self.resource_layout) is not PlanningGeometryResourceLayout
                or self.resource_layout.representation is not representation
                or self.resource_layout.content_profile is not profile
            ):
                raise _invalid("geometry resource layout does not match its representation and profile") from None

    @property
    def resolution_key(self) -> tuple[str, PlanningGeometryRepresentation, str] | None:
        if self.sha256 is None:
            return None
        return self.geometry_id, self.representation, self.sha256


def _portable_payload(value: object) -> object:
    if value is None or type(value) in (str, bool, int, float):
        return value
    if _actual_base(value, (StrEnum,)) is StrEnum:
        return str.__str__(value)
    if type(value) is tuple:
        return [_portable_payload(tuple.__getitem__(value, index)) for index in range(tuple.__len__(value))]
    if type(value) in {
        PlanningGeometryLocalPose,
        PlanningHalfspaceGeometry,
        PlanningPrimitiveGeometry,
        PlanningCompoundPart,
        PlanningCompoundGeometry,
        PlanningEntityDescriptor,
        PlanningLinkDescriptor,
        PlanningJointDescriptor,
        PlanningFrameDescriptor,
        PlanningGeometryDescriptor,
        PlanningGeometryResourceLayout,
    }:
        return {field.name: _portable_payload(getattr(value, field.name)) for field in fields(cast(Any, value))}
    raise _invalid("catalog contains a non-portable value") from None


def _catalog_content(
    *,
    schema_version: str,
    provider_id: str,
    world_id: str,
    generation: int,
    environment_index: int,
    catalog_revision: int,
    geometry_revision: int,
    entities: tuple[PlanningEntityDescriptor, ...],
    links: tuple[PlanningLinkDescriptor, ...],
    joints: tuple[PlanningJointDescriptor, ...],
    frames_: tuple[PlanningFrameDescriptor, ...],
    geometries: tuple[PlanningGeometryDescriptor, ...],
) -> dict[str, object]:
    return {
        "schema_version": schema_version,
        "provider_id": provider_id,
        "world_id": world_id,
        "generation": generation,
        "environment_index": environment_index,
        "catalog_revision": catalog_revision,
        "geometry_revision": geometry_revision,
        "entities": _portable_payload(entities),
        "links": _portable_payload(links),
        "joints": _portable_payload(joints),
        "frames": _portable_payload(frames_),
        "geometries": _portable_payload(geometries),
    }


def _content_digest(payload: dict[str, object]) -> str:
    encoder = json.JSONEncoder(ensure_ascii=False, sort_keys=True, separators=(",", ":"))
    digest = hashlib.sha256()
    encoded_size = 0
    for chunk in encoder.iterencode(payload):
        encoded = chunk.encode("utf-8")
        encoded_size += len(encoded)
        if encoded_size > _MAX_CANONICAL_CATALOG_BYTES:
            raise _invalid("catalog canonical content exceeds its byte budget") from None
        digest.update(encoded)
    return digest.hexdigest()


@dataclass(frozen=True, slots=True)
class PlanningSceneCatalog(_PlanningValue):
    provider_id: str
    world_id: str
    generation: int
    environment_index: int
    catalog_revision: int
    geometry_revision: int
    content_sha256: str
    entities: tuple[PlanningEntityDescriptor, ...]
    links: tuple[PlanningLinkDescriptor, ...]
    joints: tuple[PlanningJointDescriptor, ...]
    frames: tuple[PlanningFrameDescriptor, ...]
    geometries: tuple[PlanningGeometryDescriptor, ...]
    schema_version: str = PLANNING_SCENE_SCHEMA_VERSION

    def __post_init__(self) -> None:
        schema_version = _text(self.schema_version, "planning-scene catalog schema_version")
        if schema_version != PLANNING_SCENE_SCHEMA_VERSION:
            raise _invalid("planning-scene catalog schema version is unsupported") from None
        object.__setattr__(self, "schema_version", schema_version)
        object.__setattr__(self, "provider_id", _text(self.provider_id, "catalog provider_id", identifier=True))
        object.__setattr__(self, "world_id", _text(self.world_id, "catalog world_id", identifier=True))
        object.__setattr__(self, "generation", _integer(self.generation, "catalog generation", minimum=1))
        object.__setattr__(self, "environment_index", _integer(self.environment_index, "catalog environment_index"))
        object.__setattr__(self, "catalog_revision", _integer(self.catalog_revision, "catalog revision", minimum=1))
        object.__setattr__(self, "geometry_revision", _integer(self.geometry_revision, "geometry revision", minimum=1))
        entities = _typed_tuple(self.entities, PlanningEntityDescriptor, "catalog entities")
        links = _typed_tuple(self.links, PlanningLinkDescriptor, "catalog links")
        joints = _typed_tuple(self.joints, PlanningJointDescriptor, "catalog joints")
        frames_ = _typed_tuple(self.frames, PlanningFrameDescriptor, "catalog frames", allow_empty=False)
        geometries = _typed_tuple(self.geometries, PlanningGeometryDescriptor, "catalog geometries")
        for values, attribute, label in (
            (entities, "entity_id", "catalog entities"),
            (links, "link_id", "catalog links"),
            (joints, "joint_id", "catalog joints"),
            (frames_, "frame_id", "catalog frames"),
            (geometries, "geometry_id", "catalog geometries"),
        ):
            _unique_sorted(values, attribute, label)
        if sum(map(len, (entities, links, joints, frames_, geometries))) > _MAX_ITEMS:
            raise _invalid("catalog exceeds the aggregate node budget") from None
        relationship_count = sum(
            len(identities)
            for entity in entities
            for identities in (entity.link_ids, entity.frame_ids, entity.geometry_ids, entity.joint_ids)
        ) + sum(len(link.geometry_ids) for link in links)
        relationship_count += sum(
            len(geometry.inline.parts) for geometry in geometries if type(geometry.inline) is PlanningCompoundGeometry
        )
        if relationship_count > _MAX_RELATIONSHIP_REFERENCES:
            raise _invalid("catalog exceeds the aggregate relationship budget") from None

        entity_by_id = {item.entity_id: item for item in entities}
        link_by_id = {item.link_id: item for item in links}
        frame_by_id = {item.frame_id: item for item in frames_}
        if len({entity.path for entity in entities}) != len(entities):
            raise _invalid("catalog entity paths must be unique") from None
        world_frames = tuple(frame.frame_id for frame in frames_ if frame.kind is PlanningFrameKind.WORLD)
        if len(world_frames) != 1:
            raise _invalid("catalog must contain exactly one world frame") from None
        world_frame_id = world_frames[0]
        system_by_id = entity_by_id.get(PLANNING_SYSTEM_ENTITY_ID)
        system_by_path = next((entity for entity in entities if entity.path == PLANNING_SYSTEM_ENTITY_PATH), None)
        if system_by_id is not system_by_path:
            raise _invalid("reserved simulator-effective identity and path must correspond exactly") from None
        if system_by_id is not None and (
            system_by_id.kind is not PlanningEntityKind.OTHER
            or not system_by_id.enabled
            or system_by_id.link_ids
            or system_by_id.joint_ids
            or system_by_id.frame_ids != (system_by_id.root_frame_id,)
            or not system_by_id.geometry_ids
        ):
            raise _invalid("reserved simulator-effective entity shape is invalid") from None

        relationship_owners: dict[str, dict[str, str]] = {
            "link": {},
            "frame": {},
            "geometry": {},
            "joint": {},
        }
        for entity in entities:
            for label, identities in (
                ("link", entity.link_ids),
                ("frame", entity.frame_ids),
                ("geometry", entity.geometry_ids),
                ("joint", entity.joint_ids),
            ):
                for identity in identities:
                    if identity in relationship_owners[label]:
                        raise _invalid(f"catalog {label} identity has multiple entity owners") from None
                    relationship_owners[label][identity] = entity.entity_id
            if entity.root_frame_id not in frame_by_id:
                raise _invalid("entity root frame is absent from catalog frames") from None
            root_frame = frame_by_id[entity.root_frame_id]
            if (
                root_frame.kind is not PlanningFrameKind.ENTITY
                or root_frame.parent_frame_id != world_frame_id
                or root_frame.owner_entity_id != entity.entity_id
                or root_frame.owner_link_id is not None
            ):
                raise _invalid("entity root frame ownership does not close") from None

        for link in links:
            link_entity = entity_by_id.get(link.entity_id)
            primary_frame = frame_by_id.get(link.frame_id)
            if (
                link_entity is None
                or link.link_id not in link_entity.link_ids
                or link.frame_id not in link_entity.frame_ids
                or primary_frame is None
                or primary_frame.kind is not PlanningFrameKind.LINK
                or primary_frame.owner_entity_id != link.entity_id
                or primary_frame.owner_link_id != link.link_id
            ):
                raise _invalid("catalog link ownership does not close") from None
            if link.parent_link_id is None:
                if primary_frame.parent_frame_id != link_entity.root_frame_id:
                    raise _invalid("catalog root link frame must parent to its entity root") from None
            else:
                parent = link_by_id.get(link.parent_link_id)
                if parent is None or parent.entity_id != link.entity_id:
                    raise _invalid("catalog link parent does not close within its entity") from None
            if any(geometry_id not in link_entity.geometry_ids for geometry_id in link.geometry_ids):
                raise _invalid("catalog link geometry ownership does not close") from None
        _reject_cycles(
            {link.link_id: link.parent_link_id for link in links if link.parent_link_id is not None},
            "catalog link graph",
        )

        child_joint_owner: dict[str, str] = {}
        for joint in joints:
            joint_entity = entity_by_id.get(joint.entity_id)
            parent = link_by_id.get(joint.parent_link_id)
            child = link_by_id.get(joint.child_link_id)
            if (
                joint_entity is None
                or joint.joint_id not in joint_entity.joint_ids
                or parent is None
                or child is None
                or parent.entity_id != joint.entity_id
                or child.entity_id != joint.entity_id
                or child.parent_link_id != parent.link_id
                or joint.axis_frame_id not in joint_entity.frame_ids
            ):
                raise _invalid("catalog physical joint references do not close") from None
            axis_frame = frame_by_id.get(joint.axis_frame_id)
            if (
                axis_frame is None
                or axis_frame.kind not in {PlanningFrameKind.LINK, PlanningFrameKind.JOINT}
                or axis_frame.owner_entity_id != joint.entity_id
            ):
                raise _invalid("catalog physical joint axis frame is invalid") from None
            child_frame = frame_by_id[child.frame_id]
            if axis_frame.kind is PlanningFrameKind.JOINT:
                if (
                    axis_frame.owner_link_id != child.link_id
                    or axis_frame.parent_frame_id != parent.frame_id
                    or child_frame.parent_frame_id != axis_frame.frame_id
                ):
                    raise _invalid(
                        "catalog dedicated joint topology must be parent-link -> joint -> child-link"
                    ) from None
            elif (
                axis_frame.frame_id not in {parent.frame_id, child.frame_id}
                or axis_frame.owner_link_id not in {parent.link_id, child.link_id}
                or child_frame.parent_frame_id != parent.frame_id
            ):
                raise _invalid(
                    "catalog endpoint joint link parent topology must connect child-link directly to parent-link"
                ) from None
            if joint.child_link_id in child_joint_owner:
                raise _invalid("catalog physical joints must have unique child links") from None
            child_joint_owner[joint.child_link_id] = joint.joint_id
        non_root_links = frozenset(link.link_id for link in links if link.parent_link_id is not None)
        if frozenset(child_joint_owner) != non_root_links:
            raise _invalid("catalog link parent edges and physical joints must correspond exactly") from None

        for frame in frames_:
            if frame.parent_frame_id is not None and frame.parent_frame_id not in frame_by_id:
                raise _invalid("catalog frame parent is unknown") from None
            parent_frame = None if frame.parent_frame_id is None else frame_by_id.get(frame.parent_frame_id)
            if frame.owner_entity_id is not None:
                frame_entity = entity_by_id.get(frame.owner_entity_id)
                if frame_entity is None or frame.frame_id not in frame_entity.frame_ids:
                    raise _invalid("catalog frame entity ownership does not close") from None
                if frame.kind is not PlanningFrameKind.ENTITY and (
                    parent_frame is None or parent_frame.owner_entity_id != frame.owner_entity_id
                ):
                    raise _invalid("catalog owned frame ancestry crosses an entity boundary") from None
            if frame.owner_link_id is not None:
                frame_link = link_by_id.get(frame.owner_link_id)
                if frame_link is None or frame_link.entity_id != frame.owner_entity_id:
                    raise _invalid("catalog frame link ownership does not close") from None
                if frame.kind is PlanningFrameKind.LINK and frame_link.frame_id != frame.frame_id:
                    raise _invalid("a link frame must be the owned link's primary frame") from None
            if frame.kind is PlanningFrameKind.LINK and frame.owner_link_id is None:
                raise _invalid("a link frame must identify its exact owning link") from None
            if frame.kind is PlanningFrameKind.ENTITY and (
                frame.owner_entity_id is None or frame.owner_link_id is not None
            ):
                raise _invalid("an entity frame must identify only its exact entity owner") from None
            if frame.kind is PlanningFrameKind.NAMED:
                assert frame_entity is not None and parent_frame is not None
                if frame.owner_link_id is None:
                    if frame.parent_frame_id != frame_entity.root_frame_id:
                        raise _invalid("an entity-owned named frame must parent to its entity root") from None
                else:
                    owner_link = link_by_id[frame.owner_link_id]
                    if frame.parent_frame_id != owner_link.frame_id:
                        raise _invalid("a link-owned named frame must parent to its owner link frame") from None
        _reject_cycles(
            {frame.frame_id: frame.parent_frame_id for frame in frames_ if frame.parent_frame_id is not None},
            "catalog frame graph",
        )
        if frozenset(frame.frame_id for frame in frames_ if frame.kind is PlanningFrameKind.ENTITY) != frozenset(
            entity.root_frame_id for entity in entities
        ):
            raise _invalid("catalog entity frames must correspond exactly to entity roots") from None
        joint_axis_owner: dict[str, str] = {}
        for joint in joints:
            axis_frame = frame_by_id[joint.axis_frame_id]
            if axis_frame.kind is not PlanningFrameKind.JOINT:
                continue
            if joint.axis_frame_id in joint_axis_owner:
                raise _invalid("a physical joint frame cannot describe multiple joints") from None
            joint_axis_owner[joint.axis_frame_id] = joint.joint_id
        if frozenset(joint_axis_owner) != frozenset(
            frame.frame_id for frame in frames_ if frame.kind is PlanningFrameKind.JOINT
        ):
            raise _invalid("catalog joint frames must correspond exactly to physical joints") from None
        named_keys: set[tuple[str, str]] = set()
        for frame in frames_:
            if frame.kind is not PlanningFrameKind.NAMED:
                continue
            assert frame.owner_entity_id is not None and frame.semantic_key is not None
            key = frame.owner_entity_id, frame.semantic_key
            if key in named_keys:
                raise _invalid("named frame semantic keys must be unique within an entity") from None
            named_keys.add(key)

        link_geometry_owner: dict[str, str] = {}
        for link in links:
            for geometry_id in link.geometry_ids:
                if geometry_id in link_geometry_owner:
                    raise _invalid("catalog geometry has multiple link owners") from None
                link_geometry_owner[geometry_id] = link.link_id
        for geometry in geometries:
            geometry_entity = entity_by_id.get(geometry.owner_entity_id)
            geometry_frame = frame_by_id.get(geometry.parent_frame_id)
            if (
                geometry_entity is None
                or geometry.geometry_id not in geometry_entity.geometry_ids
                or geometry_frame is None
            ):
                raise _invalid("catalog geometry ownership does not close") from None
            declared_link_owner = link_geometry_owner.get(geometry.geometry_id)
            if declared_link_owner != geometry.owner_link_id:
                raise _invalid("catalog geometry link declarations must correspond exactly") from None
            if geometry.owner_link_id is None:
                if (
                    geometry.parent_frame_id != geometry_entity.root_frame_id
                    or geometry_frame.owner_entity_id != geometry.owner_entity_id
                    or geometry_frame.owner_link_id is not None
                ):
                    raise _invalid("entity-owned geometry must use its exact entity root frame") from None
            else:
                geometry_link = link_by_id.get(geometry.owner_link_id)
                if (
                    geometry_link is None
                    or geometry_link.entity_id != geometry.owner_entity_id
                    or geometry.geometry_id not in geometry_link.geometry_ids
                    or declared_link_owner != geometry_link.link_id
                    or geometry.parent_frame_id != geometry_link.frame_id
                ):
                    raise _invalid("catalog geometry link ownership does not close") from None
            if geometry.owner_entity_id == PLANNING_SYSTEM_ENTITY_ID and (
                geometry.purpose is not PlanningGeometryPurpose.COLLISION
                or geometry.motion_class is not PlanningGeometryMotionClass.STATIC
                or geometry.owner_link_id is not None
            ):
                raise _invalid("simulator-effective system geometry must be static collision geometry") from None

        descriptor_identities = {
            "link": frozenset(link.link_id for link in links),
            "frame": frozenset(frame.frame_id for frame in frames_ if frame.owner_entity_id is not None),
            "geometry": frozenset(geometry.geometry_id for geometry in geometries),
            "joint": frozenset(joint.joint_id for joint in joints),
        }
        if any(
            frozenset(relationship_owners[label]) != descriptor_identities[label] for label in descriptor_identities
        ):
            raise _invalid("catalog entity relationship declarations must be complete") from None
        for entity in entities:
            if entity.kind is PlanningEntityKind.ARTICULATION and not entity.joint_ids:
                raise _invalid("articulated entities require physical joints") from None
            if entity.kind not in {PlanningEntityKind.ARTICULATION, PlanningEntityKind.ROBOT} and entity.joint_ids:
                raise _invalid("non-articulated entities cannot declare joints") from None

        expected = _content_digest(
            _catalog_content(
                schema_version=self.schema_version,
                provider_id=self.provider_id,
                world_id=self.world_id,
                generation=self.generation,
                environment_index=self.environment_index,
                catalog_revision=self.catalog_revision,
                geometry_revision=self.geometry_revision,
                entities=entities,
                links=links,
                joints=joints,
                frames_=frames_,
                geometries=geometries,
            )
        )
        if type(self.content_sha256) is str and not self.content_sha256:
            object.__setattr__(self, "content_sha256", expected)
        elif _sha256(self.content_sha256, "catalog content_sha256") != expected:
            raise _invalid("catalog content_sha256 does not match its canonical content") from None

    @classmethod
    @_planning_method_boundary
    def build(
        cls,
        provider_id: str,
        world_id: str,
        generation: int,
        environment_index: int,
        catalog_revision: int,
        geometry_revision: int,
        entities: tuple[PlanningEntityDescriptor, ...],
        links: tuple[PlanningLinkDescriptor, ...],
        joints: tuple[PlanningJointDescriptor, ...],
        frames: tuple[PlanningFrameDescriptor, ...],
        geometries: tuple[PlanningGeometryDescriptor, ...],
    ) -> PlanningSceneCatalog:
        return cls(
            provider_id,
            world_id,
            generation,
            environment_index,
            catalog_revision,
            geometry_revision,
            "",
            entities,
            links,
            joints,
            frames,
            geometries,
        )

    @property
    def world_frame_id(self) -> str:
        return next(frame.frame_id for frame in self.frames if frame.kind is PlanningFrameKind.WORLD)


@dataclass(frozen=True, slots=True)
class PlanningEntityState(_PlanningValue):
    entity_id: str
    pose: PlanningPose
    twist: PlanningTwist

    def __post_init__(self) -> None:
        object.__setattr__(self, "entity_id", _text(self.entity_id, "entity state entity_id", identifier=True))
        if type(self.pose) is not PlanningPose or type(self.twist) is not PlanningTwist:
            raise _invalid("entity state requires exact pose and twist values") from None
        if self.pose.frame_id != self.twist.frame_id:
            raise _invalid("entity state pose and twist frames must match") from None


@dataclass(frozen=True, slots=True)
class PlanningLinkState(_PlanningValue):
    link_id: str
    pose: PlanningPose
    twist: PlanningTwist

    def __post_init__(self) -> None:
        object.__setattr__(self, "link_id", _text(self.link_id, "link state link_id", identifier=True))
        if type(self.pose) is not PlanningPose or type(self.twist) is not PlanningTwist:
            raise _invalid("link state requires exact pose and twist values") from None
        if self.pose.frame_id != self.twist.frame_id:
            raise _invalid("link state pose and twist frames must match") from None


@dataclass(frozen=True, slots=True)
class PlanningFrameState(_PlanningValue):
    frame_id: str
    world_pose: PlanningPose

    def __post_init__(self) -> None:
        object.__setattr__(self, "frame_id", _text(self.frame_id, "frame state frame_id", identifier=True))
        if type(self.world_pose) is not PlanningPose:
            raise _invalid("frame state world_pose must be an exact PlanningPose") from None


@dataclass(frozen=True, slots=True)
class PlanningArticulationState(_PlanningValue):
    entity_id: str
    joint_ids: tuple[str, ...]
    positions: tuple[float, ...]
    velocities: tuple[float, ...]
    position_units: tuple[str, ...]

    def __post_init__(self) -> None:
        object.__setattr__(self, "entity_id", _text(self.entity_id, "articulation state entity_id", identifier=True))
        joint_ids = _identifier_tuple(self.joint_ids, "articulation state joint_ids", allow_empty=False, ordered=True)
        count = len(joint_ids)
        if (
            type(self.positions) is not tuple
            or type(self.velocities) is not tuple
            or type(self.position_units) is not tuple
        ):
            raise _invalid("articulation state vectors must be immutable tuples") from None
        if len(self.positions) != count or len(self.velocities) != count or len(self.position_units) != count:
            raise _invalid("articulation state vectors must match joint_ids") from None
        positions = tuple(_finite(item, "articulation joint position") for item in self.positions)
        velocities = tuple(_finite(item, "articulation joint velocity") for item in self.velocities)
        units = tuple(_text(item, "articulation joint unit") for item in self.position_units)
        if any(unit not in {"m", "rad"} for unit in units):
            raise _invalid("articulation joint units must be metres or radians") from None
        object.__setattr__(self, "joint_ids", joint_ids)
        object.__setattr__(self, "positions", positions)
        object.__setattr__(self, "velocities", velocities)
        object.__setattr__(self, "position_units", units)


@dataclass(frozen=True, slots=True)
class PlanningGeometryTransform(_PlanningValue):
    geometry_id: str
    world_pose: PlanningPose

    def __post_init__(self) -> None:
        object.__setattr__(
            self, "geometry_id", _text(self.geometry_id, "geometry transform geometry_id", identifier=True)
        )
        if type(self.world_pose) is not PlanningPose:
            raise _invalid("geometry transform world_pose must be an exact PlanningPose") from None


@dataclass(frozen=True, slots=True)
class PlanningAttachment(_PlanningValue):
    attachment_id: str
    parent_entity_id: str
    child_entity_id: str
    parent_frame_id: str
    child_frame_id: str
    parent_T_child: PlanningPose
    geometry_ids: tuple[str, ...]
    parent_link_id: str | None = None
    child_link_id: str | None = None

    def __post_init__(self) -> None:
        for name in ("attachment_id", "parent_entity_id", "child_entity_id", "parent_frame_id", "child_frame_id"):
            object.__setattr__(self, name, _text(getattr(self, name), name, identifier=True))
        for name in ("parent_link_id", "child_link_id"):
            value = getattr(self, name)
            if value is not None:
                object.__setattr__(self, name, _text(value, name, identifier=True))
        if self.parent_entity_id == self.child_entity_id:
            raise _invalid("attachments must connect two different entities") from None
        if type(self.parent_T_child) is not PlanningPose or self.parent_T_child.frame_id != self.parent_frame_id:
            raise _invalid("attachment relative pose must be expressed in parent_frame_id") from None
        geometry_ids = _identifier_tuple(self.geometry_ids, "attachment geometry_ids", allow_empty=False)
        object.__setattr__(self, "geometry_ids", geometry_ids)
        if self.parent_frame_id == self.child_frame_id and not _is_identity_pose(self.parent_T_child):
            raise _invalid("self-frame attachment transform must be canonical identity") from None


def _is_identity_pose(pose: PlanningPose) -> bool:
    return pose.position_m == (0.0, 0.0, 0.0) and pose.orientation_xyzw == (0.0, 0.0, 0.0, 1.0)


def _quaternion_conjugate(value: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    return (-value[0], -value[1], -value[2], value[3])


def _quaternion_multiply(
    left: tuple[float, float, float, float], right: tuple[float, float, float, float]
) -> tuple[float, float, float, float]:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def _rotate(
    vector: tuple[float, float, float], quaternion: tuple[float, float, float, float]
) -> tuple[float, float, float]:
    vector_quaternion = (vector[0], vector[1], vector[2], 0.0)
    result = _quaternion_multiply(
        _quaternion_multiply(quaternion, vector_quaternion), _quaternion_conjugate(quaternion)
    )
    return result[0], result[1], result[2]


def _relative_pose(parent: PlanningPose, child: PlanningPose, parent_frame_id: str) -> PlanningPose:
    inverse = _quaternion_conjugate(parent.orientation_xyzw)
    displacement = tuple(child.position_m[index] - parent.position_m[index] for index in range(3))
    position = _rotate(displacement, inverse)  # type: ignore[arg-type]
    orientation = _quaternion_multiply(inverse, child.orientation_xyzw)
    return PlanningPose(parent_frame_id, position, orientation)


def _compose_pose(parent: PlanningPose, local: PlanningGeometryLocalPose) -> PlanningPose:
    offset = _rotate(local.position_m, parent.orientation_xyzw)
    position = (
        parent.position_m[0] + offset[0],
        parent.position_m[1] + offset[1],
        parent.position_m[2] + offset[2],
    )
    orientation = _quaternion_multiply(parent.orientation_xyzw, local.orientation_xyzw)
    return PlanningPose(parent.frame_id, position, orientation)


def _poses_close(left: PlanningPose, right: PlanningPose) -> bool:
    return all(
        abs(a - b) <= _TRANSFORM_TOLERANCE for a, b in zip(left.position_m, right.position_m, strict=True)
    ) and all(
        abs(a - b) <= _TRANSFORM_TOLERANCE for a, b in zip(left.orientation_xyzw, right.orientation_xyzw, strict=True)
    )


@dataclass(frozen=True, slots=True)
class PlanningSceneState(_PlanningValue):
    provider_id: str
    world_id: str
    generation: int
    environment_index: int
    tick: Tick
    sequence: int
    world_revision: int
    catalog_revision: int
    geometry_revision: int
    catalog_content_sha256: str
    transform_revision: int
    attachment_revision: int
    world_frame_id: str
    entities: tuple[PlanningEntityState, ...]
    links: tuple[PlanningLinkState, ...]
    frames: tuple[PlanningFrameState, ...]
    articulations: tuple[PlanningArticulationState, ...]
    geometry_transforms: tuple[PlanningGeometryTransform, ...]
    attachments: tuple[PlanningAttachment, ...] = ()

    def __post_init__(self) -> None:
        object.__setattr__(self, "provider_id", _text(self.provider_id, "state provider_id", identifier=True))
        object.__setattr__(self, "world_id", _text(self.world_id, "state world_id", identifier=True))
        object.__setattr__(self, "generation", _integer(self.generation, "state generation", minimum=1))
        object.__setattr__(self, "environment_index", _integer(self.environment_index, "state environment_index"))
        object.__setattr__(self, "tick", _tick(self.tick, "state tick"))
        object.__setattr__(self, "sequence", _integer(self.sequence, "state sequence", minimum=1))
        for name in (
            "world_revision",
            "catalog_revision",
            "geometry_revision",
            "transform_revision",
            "attachment_revision",
        ):
            object.__setattr__(self, name, _integer(getattr(self, name), f"state {name}", minimum=1))
        object.__setattr__(
            self,
            "catalog_content_sha256",
            _sha256(self.catalog_content_sha256, "state catalog_content_sha256"),
        )
        world_frame_id = _text(self.world_frame_id, "state world_frame_id", identifier=True)
        object.__setattr__(self, "world_frame_id", world_frame_id)
        entities = _typed_tuple(self.entities, PlanningEntityState, "state entities")
        links = _typed_tuple(self.links, PlanningLinkState, "state links")
        frames_ = _typed_tuple(self.frames, PlanningFrameState, "state frames", allow_empty=False)
        articulations = _typed_tuple(self.articulations, PlanningArticulationState, "state articulations")
        geometry_transforms = _typed_tuple(
            self.geometry_transforms, PlanningGeometryTransform, "state geometry_transforms"
        )
        attachments = _typed_tuple(self.attachments, PlanningAttachment, "state attachments")
        for values, attribute, label in (
            (entities, "entity_id", "state entities"),
            (links, "link_id", "state links"),
            (frames_, "frame_id", "state frames"),
            (articulations, "entity_id", "state articulations"),
            (geometry_transforms, "geometry_id", "state geometry_transforms"),
            (attachments, "attachment_id", "state attachments"),
        ):
            _unique_sorted(values, attribute, label)
        if sum(map(len, (entities, links, frames_, articulations, geometry_transforms, attachments))) > _MAX_ITEMS:
            raise _invalid("state exceeds the aggregate node budget") from None
        relationship_count = 4 * sum(len(item.joint_ids) for item in articulations) + sum(
            len(item.geometry_ids) for item in attachments
        )
        if relationship_count > _MAX_RELATIONSHIP_REFERENCES:
            raise _invalid("state exceeds the aggregate relationship budget") from None
        all_world_poses = (
            tuple(item.pose for item in entities)
            + tuple(item.pose for item in links)
            + tuple(item.world_pose for item in frames_)
            + tuple(item.world_pose for item in geometry_transforms)
        )
        if any(pose.frame_id != world_frame_id for pose in all_world_poses):
            raise _invalid("all committed world poses must use world_frame_id") from None
        frame_by_id = {frame.frame_id: frame.world_pose for frame in frames_}
        world_frame_pose = frame_by_id.get(world_frame_id)
        if world_frame_pose is None or not _is_identity_pose(world_frame_pose):
            raise _invalid("world frame state must be present with canonical identity") from None
        entity_ids = frozenset(item.entity_id for item in entities)
        link_ids = frozenset(item.link_id for item in links)
        geometry_ids = frozenset(item.geometry_id for item in geometry_transforms)
        for attachment in attachments:
            if (
                attachment.parent_entity_id not in entity_ids
                or attachment.child_entity_id not in entity_ids
                or attachment.parent_frame_id not in frame_by_id
                or attachment.child_frame_id not in frame_by_id
                or any(geometry_id not in geometry_ids for geometry_id in attachment.geometry_ids)
                or attachment.parent_link_id is not None
                and attachment.parent_link_id not in link_ids
                or attachment.child_link_id is not None
                and attachment.child_link_id not in link_ids
            ):
                raise _invalid("attachment references do not close within the committed state") from None
            expected = _relative_pose(
                frame_by_id[attachment.parent_frame_id],
                frame_by_id[attachment.child_frame_id],
                attachment.parent_frame_id,
            )
            if not _poses_close(expected, attachment.parent_T_child):
                raise _invalid("attachment relative pose contradicts committed frame transforms") from None

    @_planning_method_boundary
    def validate_against(self, catalog: PlanningSceneCatalog) -> None:
        if type(catalog) is not PlanningSceneCatalog:
            raise _invalid("state validation requires an exact PlanningSceneCatalog") from None
        if self.generation != catalog.generation:
            raise PlanningSceneStaleGenerationError(
                "state and catalog generations do not match",
                operation="planning_scene.state.validate",
            ) from None
        if (
            self.provider_id != catalog.provider_id
            or self.world_id != catalog.world_id
            or self.environment_index != catalog.environment_index
            or self.catalog_revision != catalog.catalog_revision
            or self.geometry_revision != catalog.geometry_revision
            or self.catalog_content_sha256 != catalog.content_sha256
            or self.world_frame_id != catalog.world_frame_id
        ):
            raise _invalid("state and catalog envelopes do not match") from None
        if frozenset(item.entity_id for item in self.entities) != frozenset(
            item.entity_id for item in catalog.entities
        ):
            raise _invalid("state entity set does not close against catalog") from None
        if frozenset(item.link_id for item in self.links) != frozenset(item.link_id for item in catalog.links):
            raise _invalid("state link set does not close against catalog") from None
        if frozenset(item.frame_id for item in self.frames) != frozenset(item.frame_id for item in catalog.frames):
            raise _invalid("state frame set does not close against catalog") from None
        if frozenset(item.geometry_id for item in self.geometry_transforms) != frozenset(
            item.geometry_id for item in catalog.geometries
        ):
            raise _invalid("state geometry set does not close against catalog") from None
        articulated = {entity.entity_id: entity for entity in catalog.entities if entity.joint_ids}
        states = {state.entity_id: state for state in self.articulations}
        if frozenset(articulated) != frozenset(states):
            raise _invalid("state articulation set does not close against catalog") from None
        joint_by_id = {joint.joint_id: joint for joint in catalog.joints}
        for entity_id, entity in articulated.items():
            state = states[entity_id]
            expected_units = tuple(joint_by_id[joint_id].position_unit for joint_id in entity.joint_ids)
            if state.joint_ids != entity.joint_ids or state.position_units != expected_units:
                raise _invalid("articulation state order/units do not match physical catalog joints") from None
        entity_by_id = {entity.entity_id: entity for entity in catalog.entities}
        link_by_id = {link.link_id: link for link in catalog.links}
        frame_by_id = {frame.frame_id: frame for frame in catalog.frames}
        geometry_by_id = {geometry.geometry_id: geometry for geometry in catalog.geometries}
        entity_state_by_id = {item.entity_id: item for item in self.entities}
        link_state_by_id = {item.link_id: item for item in self.links}
        frame_state_by_id = {item.frame_id: item for item in self.frames}
        geometry_transform_by_id = {item.geometry_id: item for item in self.geometry_transforms}
        for entity in catalog.entities:
            if not _poses_close(
                entity_state_by_id[entity.entity_id].pose,
                frame_state_by_id[entity.root_frame_id].world_pose,
            ):
                raise _invalid("committed entity pose contradicts its catalog root frame") from None
        for link in catalog.links:
            if not _poses_close(
                link_state_by_id[link.link_id].pose,
                frame_state_by_id[link.frame_id].world_pose,
            ):
                raise _invalid("committed link pose contradicts its catalog physical frame") from None
        for geometry in catalog.geometries:
            expected = _compose_pose(
                frame_state_by_id[geometry.parent_frame_id].world_pose,
                geometry.parent_frame_T_geometry,
            )
            if not _poses_close(expected, geometry_transform_by_id[geometry.geometry_id].world_pose):
                raise _invalid("committed geometry world pose contradicts catalog transform composition") from None
        system_entity = entity_by_id.get(PLANNING_SYSTEM_ENTITY_ID)
        if system_entity is not None:
            system_state = entity_state_by_id[PLANNING_SYSTEM_ENTITY_ID]
            system_frame_state = frame_state_by_id[system_entity.root_frame_id]
            if (
                not _is_identity_pose(system_state.pose)
                or system_state.twist.linear_m_s != (0.0, 0.0, 0.0)
                or system_state.twist.angular_rad_s != (0.0, 0.0, 0.0)
                or not _is_identity_pose(system_frame_state.world_pose)
            ):
                raise _invalid("simulator-effective system entity state must be static canonical zero") from None
        for attachment in self.attachments:
            if PLANNING_SYSTEM_ENTITY_ID in {attachment.parent_entity_id, attachment.child_entity_id}:
                raise _invalid("simulator-effective system entity cannot participate in attachments") from None
            if attachment.parent_entity_id not in entity_by_id or attachment.child_entity_id not in entity_by_id:
                raise _invalid("attachment entity is absent from catalog") from None
            parent_frame = frame_by_id.get(attachment.parent_frame_id)
            child_frame = frame_by_id.get(attachment.child_frame_id)
            if parent_frame is None or parent_frame.owner_entity_id != attachment.parent_entity_id:
                raise _invalid("attachment parent frame ownership is invalid") from None
            if child_frame is None or child_frame.owner_entity_id != attachment.child_entity_id:
                raise _invalid("attachment child frame ownership is invalid") from None
            if parent_frame.owner_link_id != attachment.parent_link_id:
                raise _invalid("attachment parent link ownership is invalid") from None
            if child_frame.owner_link_id != attachment.child_link_id:
                raise _invalid("attachment child link ownership is invalid") from None
            if attachment.parent_link_id is not None:
                parent_link = link_by_id.get(attachment.parent_link_id)
                if parent_link is None or parent_link.entity_id != attachment.parent_entity_id:
                    raise _invalid("attachment parent link ownership is invalid") from None
            if attachment.child_link_id is not None:
                child_link = link_by_id.get(attachment.child_link_id)
                if child_link is None or child_link.entity_id != attachment.child_entity_id:
                    raise _invalid("attachment child link ownership is invalid") from None
            for geometry_id in attachment.geometry_ids:
                attached_geometry = geometry_by_id.get(geometry_id)
                if attached_geometry is None or attached_geometry.owner_entity_id != attachment.child_entity_id:
                    raise _invalid("attachment geometry owner must be the child entity") from None
                if attachment.child_link_id is not None and attached_geometry.owner_link_id != attachment.child_link_id:
                    raise _invalid("attachment geometry link owner must be the child link") from None


@dataclass(frozen=True, slots=True)
class PlanningSceneDelta(_PlanningValue):
    provider_id: str
    world_id: str
    generation: int
    environment_index: int
    tick: Tick
    base_sequence: int
    sequence: int
    previous_world_revision: int
    world_revision: int
    previous_catalog_revision: int
    catalog_revision: int
    previous_catalog_content_sha256: str | None
    catalog_content_sha256: str | None
    previous_geometry_revision: int
    geometry_revision: int
    previous_transform_revision: int
    transform_revision: int
    previous_attachment_revision: int
    attachment_revision: int
    kind: PlanningSceneDeltaKind
    catalog: PlanningSceneCatalog | None = None
    state: PlanningSceneState | None = None
    attachments: tuple[PlanningAttachment, ...] = ()
    resync_required: bool = False

    def __post_init__(self) -> None:
        object.__setattr__(self, "provider_id", _text(self.provider_id, "delta provider_id", identifier=True))
        object.__setattr__(self, "world_id", _text(self.world_id, "delta world_id", identifier=True))
        object.__setattr__(self, "generation", _integer(self.generation, "delta generation", minimum=1))
        object.__setattr__(self, "environment_index", _integer(self.environment_index, "delta environment_index"))
        object.__setattr__(self, "tick", _tick(self.tick, "delta tick"))
        object.__setattr__(self, "base_sequence", _integer(self.base_sequence, "delta base_sequence", minimum=1))
        object.__setattr__(self, "sequence", _integer(self.sequence, "delta sequence", minimum=1))
        kind = _enum(self.kind, PlanningSceneDeltaKind, "delta kind")
        object.__setattr__(self, "kind", kind)
        if kind is not PlanningSceneDeltaKind.RESYNC and self.sequence <= self.base_sequence:
            raise _invalid("non-resync delta sequence must advance beyond base_sequence") from None
        revision_pairs: dict[str, tuple[int, int]] = {}
        for name in ("world", "catalog", "geometry", "transform", "attachment"):
            previous = _integer(
                getattr(self, f"previous_{name}_revision"), f"delta previous {name} revision", minimum=1
            )
            current = _integer(getattr(self, f"{name}_revision"), f"delta {name} revision", minimum=1)
            if current < previous:
                raise _invalid("delta revisions cannot move backwards") from None
            object.__setattr__(self, f"previous_{name}_revision", previous)
            object.__setattr__(self, f"{name}_revision", current)
            revision_pairs[name] = previous, current
        if kind is PlanningSceneDeltaKind.RESYNC:
            if self.previous_catalog_content_sha256 is not None or self.catalog_content_sha256 is not None:
                raise _invalid("resync delta cannot claim catalog content digests") from None
            if revision_pairs["catalog"][1] != revision_pairs["catalog"][0]:
                raise _invalid("resync delta cannot imply a new catalog revision") from None
            if revision_pairs["geometry"][1] != revision_pairs["geometry"][0]:
                raise _invalid("resync delta cannot imply a new geometry revision") from None
        else:
            object.__setattr__(
                self,
                "previous_catalog_content_sha256",
                _sha256(self.previous_catalog_content_sha256, "delta previous catalog_content_sha256"),
            )
            object.__setattr__(
                self,
                "catalog_content_sha256",
                _sha256(self.catalog_content_sha256, "delta catalog_content_sha256"),
            )
        attachments = _typed_tuple(self.attachments, PlanningAttachment, "delta attachments")
        _unique_sorted(attachments, "attachment_id", "delta attachments")
        if sum(len(item.geometry_ids) for item in attachments) > _MAX_RELATIONSHIP_REFERENCES:
            raise _invalid("delta exceeds the aggregate relationship budget") from None
        if type(self.resync_required) is not bool:
            raise _invalid("delta resync_required must be an exact boolean") from None

        if kind is PlanningSceneDeltaKind.RESYNC:
            if not self.resync_required or self.catalog is not None or self.state is not None or attachments:
                raise _invalid("resync delta must be terminal and payload-free") from None
            return
        if self.resync_required:
            raise _invalid("only resync deltas may require resync") from None
        if revision_pairs["world"][1] <= revision_pairs["world"][0]:
            raise _invalid("a non-empty delta must advance world_revision") from None
        if kind is PlanningSceneDeltaKind.STRUCTURAL:
            if (
                type(self.catalog) is not PlanningSceneCatalog
                or type(self.state) is not PlanningSceneState
                or attachments
            ):
                raise _invalid("structural delta requires exactly a catalog and coherent state") from None
            if revision_pairs["catalog"][1] <= revision_pairs["catalog"][0]:
                raise _invalid("structural delta must advance catalog_revision") from None
            self.state.validate_against(self.catalog)
        elif kind is PlanningSceneDeltaKind.STATE:
            if self.catalog is not None or type(self.state) is not PlanningSceneState or attachments:
                raise _invalid("state delta requires exactly one coherent state payload") from None
            if revision_pairs["catalog"][1] != revision_pairs["catalog"][0]:
                raise _invalid("state delta cannot change catalog_revision") from None
            if self.catalog_content_sha256 != self.previous_catalog_content_sha256:
                raise _invalid("state delta cannot change catalog_content_sha256") from None
            if revision_pairs["geometry"][1] != revision_pairs["geometry"][0]:
                raise _invalid("state delta cannot change geometry_revision") from None
            if revision_pairs["attachment"][1] != revision_pairs["attachment"][0]:
                raise _invalid("state delta cannot change attachment_revision") from None
            if revision_pairs["transform"][1] <= revision_pairs["transform"][0]:
                raise _invalid("non-empty state delta must advance transform_revision") from None
        else:
            if self.catalog is not None or self.state is not None:
                raise _invalid("attachment delta cannot carry catalog or state payload") from None
            if revision_pairs["attachment"][1] <= revision_pairs["attachment"][0]:
                raise _invalid("attachment delta must advance attachment_revision") from None
            if revision_pairs["catalog"][1] != revision_pairs["catalog"][0]:
                raise _invalid("attachment delta cannot change catalog_revision") from None
            if self.catalog_content_sha256 != self.previous_catalog_content_sha256:
                raise _invalid("attachment delta cannot change catalog_content_sha256") from None
            if revision_pairs["geometry"][1] != revision_pairs["geometry"][0]:
                raise _invalid("attachment delta cannot change geometry_revision") from None
            if revision_pairs["transform"][1] != revision_pairs["transform"][0]:
                raise _invalid("attachment delta cannot change transform_revision") from None

        for snapshot in (self.catalog, self.state):
            if snapshot is None:
                continue
            if (
                snapshot.provider_id != self.provider_id
                or snapshot.world_id != self.world_id
                or snapshot.generation != self.generation
                or snapshot.environment_index != self.environment_index
            ):
                raise _invalid("delta payload envelope does not match delta") from None
        if self.catalog is not None and (
            self.catalog.catalog_revision != self.catalog_revision
            or self.catalog.geometry_revision != self.geometry_revision
            or self.catalog.content_sha256 != self.catalog_content_sha256
        ):
            raise _invalid("delta catalog identity does not match its payload") from None
        if self.state is not None and (
            self.state.sequence != self.sequence
            or self.state.tick != self.tick
            or self.state.world_revision != self.world_revision
            or self.state.catalog_revision != self.catalog_revision
            or self.state.catalog_content_sha256 != self.catalog_content_sha256
            or self.state.geometry_revision != self.geometry_revision
            or self.state.transform_revision != self.transform_revision
            or self.state.attachment_revision != self.attachment_revision
        ):
            raise _invalid("delta state revisions do not match its payload") from None

    @_planning_method_boundary
    def apply(
        self,
        catalog: PlanningSceneCatalog,
        state: PlanningSceneState,
    ) -> tuple[PlanningSceneCatalog, PlanningSceneState] | None:
        """Validate continuity/closure and return the coherent resulting pair."""

        if type(catalog) is not PlanningSceneCatalog or type(state) is not PlanningSceneState:
            raise _invalid("delta apply requires exact catalog and state values") from None
        if self.kind is PlanningSceneDeltaKind.RESYNC:
            return None
        if self.generation != state.generation:
            raise PlanningSceneStaleGenerationError(
                "delta and prior state generations do not match",
                operation="planning_scene.delta.apply",
            ) from None
        if (
            self.provider_id != state.provider_id
            or self.world_id != state.world_id
            or self.environment_index != state.environment_index
        ):
            raise _invalid("delta envelope does not match prior state") from None
        state.validate_against(catalog)
        if (
            self.base_sequence != state.sequence
            or self.previous_world_revision != state.world_revision
            or self.previous_catalog_revision != state.catalog_revision
            or self.previous_catalog_content_sha256 != catalog.content_sha256
            or self.previous_geometry_revision != state.geometry_revision
            or self.previous_transform_revision != state.transform_revision
            or self.previous_attachment_revision != state.attachment_revision
        ):
            raise PlanningSceneDeltaContinuityError(
                "delta base continuity does not match prior state",
                operation="planning_scene.delta.apply",
            ) from None
        next_catalog = self.catalog if self.catalog is not None else catalog
        if (
            self.kind is PlanningSceneDeltaKind.STRUCTURAL
            and next_catalog.geometries != catalog.geometries
            and next_catalog.geometry_revision <= catalog.geometry_revision
        ):
            raise _invalid("structural geometry changes must advance geometry_revision") from None
        if self.kind in {PlanningSceneDeltaKind.STRUCTURAL, PlanningSceneDeltaKind.STATE}:
            assert self.state is not None
            next_state = self.state
        else:
            next_state = replace(
                state,
                tick=self.tick,
                sequence=self.sequence,
                world_revision=self.world_revision,
                catalog_revision=self.catalog_revision,
                geometry_revision=self.geometry_revision,
                catalog_content_sha256=cast(str, self.catalog_content_sha256),
                transform_revision=self.transform_revision,
                attachment_revision=self.attachment_revision,
                attachments=self.attachments,
            )
        next_state.validate_against(next_catalog)
        return next_catalog, next_state


@dataclass(frozen=True, slots=True)
class PlanningGeometryResourceDescriptor(_PlanningValue):
    """Validated immutable resource schema.

    Catalog and geometry revisions, the exact catalog digest, and ``resource_layout``
    bind a returned lease to the catalog that authorized it.  Consumers may
    call :meth:`validate_against` before decoding or reusing cached bytes.

    Mesh payloads are canonical right-handed Z-up, little-endian contiguous vertices followed by triangle
    indices. Dense grid shapes use local geometry axes: ``(x, y, z)`` for SDF
    and occupancy, ``(x, y)`` for heightfields, with the last axis contiguous.
    ``grid_origin_m`` is the local position of the all-zero sample and spacing
    is positive per grid axis. SDF and height values are metres; occupancy is
    one byte per cell with the frozen profile semantics.
    """

    provider_id: str
    world_id: str
    generation: int
    environment_index: int
    catalog_revision: int
    geometry_revision: int
    catalog_content_sha256: str
    lease_token: str
    resource_id: str
    geometry_id: str
    representation: PlanningGeometryRepresentation
    storage_kind: PlanningGeometryStorageKind
    locator: str
    format: PlanningGeometryContentProfile
    units: str
    axis_convention: PlanningGeometryAxisConvention
    resource_layout: PlanningGeometryResourceLayout
    byte_size: int
    sha256: str

    def __post_init__(self) -> None:
        object.__setattr__(self, "provider_id", _text(self.provider_id, "resource provider_id", identifier=True))
        object.__setattr__(self, "world_id", _text(self.world_id, "resource world_id", identifier=True))
        object.__setattr__(self, "generation", _integer(self.generation, "resource generation", minimum=1))
        object.__setattr__(self, "environment_index", _integer(self.environment_index, "resource environment_index"))
        object.__setattr__(
            self, "catalog_revision", _integer(self.catalog_revision, "resource catalog_revision", minimum=1)
        )
        object.__setattr__(
            self,
            "geometry_revision",
            _integer(self.geometry_revision, "resource geometry_revision", minimum=1),
        )
        object.__setattr__(
            self,
            "catalog_content_sha256",
            _sha256(self.catalog_content_sha256, "resource catalog_content_sha256"),
        )
        object.__setattr__(self, "lease_token", _text(self.lease_token, "resource lease_token", opaque=True))
        object.__setattr__(self, "resource_id", _text(self.resource_id, "resource_id", identifier=True))
        object.__setattr__(self, "geometry_id", _text(self.geometry_id, "resource geometry_id", identifier=True))
        representation = _enum(self.representation, PlanningGeometryRepresentation, "resource representation")
        if representation in {
            PlanningGeometryRepresentation.HALFSPACE,
            PlanningGeometryRepresentation.BOX,
            PlanningGeometryRepresentation.SPHERE,
            PlanningGeometryRepresentation.CAPSULE,
            PlanningGeometryRepresentation.CYLINDER,
            PlanningGeometryRepresentation.COMPOUND,
        }:
            raise _invalid("inline representations cannot produce resource leases") from None
        object.__setattr__(self, "representation", representation)
        object.__setattr__(
            self, "storage_kind", _enum(self.storage_kind, PlanningGeometryStorageKind, "resource storage_kind")
        )
        object.__setattr__(self, "locator", _text(self.locator, "resource locator", opaque=True))
        profile = _enum(self.format, PlanningGeometryContentProfile, "resource format")
        if profile is not _RESOURCE_PROFILE_BY_REPRESENTATION[representation]:
            raise _invalid("resource format does not match its representation") from None
        object.__setattr__(self, "format", profile)
        units = _text(self.units, "resource units")
        if units != "m":
            raise _invalid("resource units must be SI metres") from None
        object.__setattr__(self, "units", units)
        axis_convention = _enum(
            self.axis_convention,
            PlanningGeometryAxisConvention,
            "resource axis_convention",
        )
        if axis_convention is not PlanningGeometryAxisConvention.RIGHT_HANDED_Z_UP:
            raise _invalid("planning resources must use canonical right-handed Z-up axes") from None
        object.__setattr__(self, "axis_convention", axis_convention)
        if (
            type(self.resource_layout) is not PlanningGeometryResourceLayout
            or self.resource_layout.representation is not representation
            or self.resource_layout.content_profile is not profile
        ):
            raise _invalid("resource layout does not match its representation and format") from None
        byte_size = _integer(self.byte_size, "resource byte_size", maximum=_MAX_RESOURCE_BYTES)
        if byte_size != self.resource_layout.decoded_byte_size:
            raise _invalid("resource layout decoded bytes do not equal byte_size") from None
        object.__setattr__(self, "byte_size", byte_size)
        object.__setattr__(self, "sha256", _sha256(self.sha256, "resource sha256"))

    @property
    def vertex_dtype(self) -> PlanningGeometryDType | None:
        return self.resource_layout.vertex_dtype

    @property
    def vertex_shape(self) -> tuple[int, int] | None:
        return self.resource_layout.vertex_shape

    @property
    def index_dtype(self) -> PlanningGeometryDType | None:
        return self.resource_layout.index_dtype

    @property
    def index_shape(self) -> tuple[int, int] | None:
        return self.resource_layout.index_shape

    @property
    def grid_dtype(self) -> PlanningGeometryDType | None:
        return self.resource_layout.grid_dtype

    @property
    def grid_shape(self) -> tuple[int, ...] | None:
        return self.resource_layout.grid_shape

    @property
    def grid_spacing_m(self) -> tuple[float, ...] | None:
        return self.resource_layout.grid_spacing_m

    @property
    def grid_origin_m(self) -> tuple[float, float, float] | None:
        return self.resource_layout.grid_origin_m

    @_planning_method_boundary
    def validate_against(self, catalog: PlanningSceneCatalog) -> None:
        """Prove this lease metadata is authorized by one exact catalog."""

        if type(catalog) is not PlanningSceneCatalog:
            raise _invalid("resource validation requires an exact PlanningSceneCatalog") from None
        if self.generation != catalog.generation:
            raise PlanningSceneStaleGenerationError(
                "resource and catalog generations do not match",
                operation="planning_geometry.validate",
            ) from None
        if (
            self.provider_id != catalog.provider_id
            or self.world_id != catalog.world_id
            or self.environment_index != catalog.environment_index
            or self.catalog_revision != catalog.catalog_revision
            or self.geometry_revision != catalog.geometry_revision
            or self.catalog_content_sha256 != catalog.content_sha256
        ):
            raise _invalid("resource and catalog envelopes do not match") from None
        geometry = next((item for item in catalog.geometries if item.geometry_id == self.geometry_id), None)
        if geometry is None or geometry.resolution_key is None:
            raise _invalid("resource geometry is absent from the catalog") from None
        if (
            self.resource_id != geometry.resource_id
            or self.representation is not geometry.representation
            or self.format is not geometry.content_profile
            or self.resource_layout != geometry.resource_layout
            or self.sha256 != geometry.sha256
        ):
            raise _invalid("resource metadata does not match the catalog geometry") from None

    @property
    def resolution_key(self) -> tuple[str, PlanningGeometryRepresentation, str]:
        return self.geometry_id, self.representation, self.sha256

    @_planning_method_boundary
    def read_span(self, offset: int = 0, length: int | None = None) -> tuple[int, int]:
        if type(offset) is not int or not 0 <= offset <= self.byte_size:
            raise _invalid("resource read offset must be an exact in-range integer", operation="planning_geometry.read")
        remaining = self.byte_size - offset
        if length is None:
            return offset, min(remaining, PLANNING_GEOMETRY_READ_LIMIT_BYTES)
        if type(length) is not int or not 0 <= length <= PLANNING_GEOMETRY_READ_LIMIT_BYTES or length > remaining:
            raise _invalid("resource read length is out of bounds", operation="planning_geometry.read")
        return offset, length


def _shape(value: object, label: str, *, width: int) -> tuple[int, int]:
    if type(value) is not tuple or len(value) != 2:
        raise _invalid(f"{label} must be an immutable rank-2 shape") from None
    rows = _integer(value[0], f"{label}[0]", minimum=1, maximum=2**31 - 1)
    columns = _integer(value[1], f"{label}[1]", minimum=1, maximum=2**31 - 1)
    if columns != width:
        raise _invalid(f"{label} trailing dimension must be {width}") from None
    return rows, columns


def _grid_shape(value: object, label: str, *, rank: int) -> tuple[int, ...]:
    if type(value) is not tuple or len(value) != rank:
        raise _invalid(f"{label} must be an immutable rank-{rank} shape") from None
    return tuple(_integer(value[index], f"{label}[{index}]", minimum=1, maximum=2**31 - 1) for index in range(rank))


@runtime_checkable
class PlanningGeometryLease(Protocol):
    """Live lease revoked when its catalog layout or generation is replaced."""

    @property
    def descriptor(self) -> PlanningGeometryResourceDescriptor: ...

    @property
    def closed(self) -> bool: ...

    def read(self, offset: int = 0, length: int | None = None) -> bytes: ...

    def close(self) -> None: ...


@runtime_checkable
class PlanningSceneWorld(Protocol):
    """Optional synchronous authority-thread endpoint for ``planning.scene@2``."""

    def planning_scene_catalog(self, environment_index: int = 0) -> PlanningSceneCatalog: ...

    def planning_scene_state(self, environment_index: int = 0) -> PlanningSceneState: ...

    def planning_scene_delta(self, base_sequence: int, environment_index: int = 0) -> PlanningSceneDelta: ...

    def resolve_planning_geometry(
        self,
        geometry_id: str,
        representation: PlanningGeometryRepresentation | None = None,
        environment_index: int = 0,
    ) -> PlanningGeometryLease:
        """Resolve the descriptor's sole representation.

        ``representation`` is an optional exact assertion, not a conversion or
        fallback request.  A mismatch raises ``PlanningSceneRepresentationError``.
        The returned descriptor must validate against the current catalog,
        including its exact resource layout and catalog content digest.
        """

        ...


__all__ = [
    "PLANNING_FRAME_DECLARATIONS_SCHEMA_VERSION",
    "PLANNING_GEOMETRY_READ_LIMIT_BYTES",
    "PLANNING_GRID_INDEX_ORDER",
    "PLANNING_HEIGHTFIELD_SAMPLE_CONVENTION",
    "PLANNING_SCENE_CAPABILITY_ID",
    "PLANNING_SCENE_SCHEMA_VERSION",
    "PLANNING_SYSTEM_ENTITY_ID",
    "PLANNING_SYSTEM_ENTITY_PATH",
    "PLANNING_SDF_SIGN_CONVENTION",
    "PLANNING_VOXEL_OCCUPANCY_CONVENTION",
    "PlanningArticulationState",
    "PlanningAttachment",
    "PlanningCompoundGeometry",
    "PlanningCompoundPart",
    "PlanningEntityDescriptor",
    "PlanningEntityKind",
    "PlanningEntityState",
    "PlanningFrameDescriptor",
    "PlanningFrameDeclaration",
    "PlanningFrameDeclarations",
    "PlanningFrameKind",
    "PlanningFrameRole",
    "PlanningFrameSource",
    "PlanningFrameSourceKind",
    "PlanningFrameState",
    "PlanningGeometryAxisConvention",
    "PlanningGeometryContentProfile",
    "PlanningGeometryDType",
    "PlanningGeometryDescriptor",
    "PlanningGeometryLease",
    "PlanningGeometryLocalPose",
    "PlanningHalfspaceGeometry",
    "PlanningGeometryMotionClass",
    "PlanningGeometryPurpose",
    "PlanningGeometryRepresentation",
    "PlanningGeometryResourceDescriptor",
    "PlanningGeometryResourceLayout",
    "PlanningGeometryStorageKind",
    "PlanningGeometryTransform",
    "PlanningJointDescriptor",
    "PlanningJointType",
    "PlanningLinkDescriptor",
    "PlanningLinkState",
    "PlanningPose",
    "PlanningPrimitiveGeometry",
    "PlanningSceneCatalog",
    "PlanningSceneDelta",
    "PlanningSceneDeltaKind",
    "PlanningSceneState",
    "PlanningSceneWorld",
    "PlanningTwist",
    "parse_planning_frame_declarations",
]
