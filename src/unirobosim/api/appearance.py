"""Serializable effective visual state; all colors are scene-linear RGB."""

import hashlib
import math
import struct
from dataclasses import dataclass, fields
from typing import Protocol, runtime_checkable

from .frozen import FrozenMap


def _vector(value, size, name, bounded=False):
    result = tuple(float(v) for v in value)
    if len(result) != size or any(not math.isfinite(v) or (bounded and not 0 <= v <= 1) for v in result):
        raise ValueError(f"{name} must have {size} finite components" + (" in [0,1]" if bounded else ""))
    return result


def _json(value):
    if isinstance(value, FrozenMap):
        return value.to_dict()
    if isinstance(value, _Value):
        return value.to_dict()
    if isinstance(value, tuple):
        return [_json(x) for x in value]
    return value


class _Value:
    def to_dict(self):
        return {f.name: _json(getattr(self, f.name)) for f in fields(self)}

    @classmethod
    def from_dict(cls, value):
        if not isinstance(value, dict):
            raise TypeError("appearance payload must be an object")
        data = dict(value)
        if "source_parameters" in data:
            data["source_parameters"] = FrozenMap(data["source_parameters"])
        if cls is AppearanceMaterial:
            data["textures"] = tuple(AppearanceTexture.from_dict(x) for x in data.get("textures", ()))
        if cls is AppearanceSnapshot:
            for key, kind in (
                ("materials", AppearanceMaterial),
                ("bindings", AppearanceBinding),
                ("lights", AppearanceLight),
            ):
                data[key] = tuple(kind.from_dict(x) for x in data[key])
            data["environment"] = AppearanceEnvironment.from_dict(data["environment"])
            data["renderer"] = AppearanceRenderer.from_dict(data["renderer"])
        return cls(**data)


@dataclass(frozen=True)
class AppearanceTexture(_Value):
    channel: str
    uri: str
    color_space: str
    uv_set: str = "st"
    sha256: str | None = None

    def __post_init__(self):
        if (
            self.channel not in ("base_color", "roughness", "metallic", "opacity", "normal")
            or self.color_space not in ("srgb", "linear")
            or not self.uri
            or not self.uv_set
        ):
            raise ValueError("invalid appearance texture channel, URI or color space")


@dataclass(frozen=True)
class AppearanceMaterial(_Value):
    material_id: str
    base_color_linear_rgba: tuple[float, float, float, float]
    roughness: float
    metallic: float
    color_source: str = "constant"
    textures: tuple[AppearanceTexture, ...] = ()
    source_parameters: FrozenMap = FrozenMap()

    def __post_init__(self):
        object.__setattr__(self, "base_color_linear_rgba", _vector(self.base_color_linear_rgba, 4, "base_color", True))
        object.__setattr__(self, "textures", tuple(self.textures))
        if not self.material_id or self.color_source not in ("constant", "particle_colors"):
            raise ValueError("invalid appearance material id or color source")
        if any(not math.isfinite(v) or not 0 <= v <= 1 for v in (self.roughness, self.metallic)):
            raise ValueError("roughness/metallic must be finite in [0,1]")


@dataclass(frozen=True)
class AppearanceBinding(_Value):
    entity_path: str
    mesh_key: str
    material_id: str
    environment_indices: tuple[int, ...] | None = None
    source_native_path: str | None = None
    uv_coordinates: tuple[tuple[float, float], ...] | None = None
    uv_interpolation: str = "vertex"
    uv_indices: tuple[int, ...] | None = None
    uv_topology_sha256: str | None = None

    def __post_init__(self):
        if self.uv_coordinates is not None:
            object.__setattr__(self, "uv_coordinates", tuple(_vector(x, 2, "UV") for x in self.uv_coordinates))
            if self.uv_interpolation not in ("vertex", "faceVarying"):
                raise ValueError("unsupported UV interpolation")
        if self.uv_indices is not None:
            indices = tuple(self.uv_indices)
            if self.uv_coordinates is None or any(
                type(i) is not int or not 0 <= i < len(self.uv_coordinates) for i in indices
            ):
                raise ValueError("invalid UV indices")
            object.__setattr__(self, "uv_indices", indices)
        if any(
            not isinstance(v, str) or not v for v in (self.entity_path, self.mesh_key, self.material_id)
        ) or not self.entity_path.startswith("/"):
            raise ValueError("invalid appearance binding identity")
        if self.environment_indices is not None:
            indices = tuple(self.environment_indices)
            if not indices or len(set(indices)) != len(indices) or any(type(x) is not int or x < 0 for x in indices):
                raise ValueError("invalid appearance environments")
            object.__setattr__(self, "environment_indices", indices)


@dataclass(frozen=True)
class AppearanceLight(_Value):
    light_id: str
    kind: str
    color_linear_rgb: tuple[float, float, float]
    intensity: float
    intensity_unit: str
    direction_world: tuple[float, float, float] | None = None
    position_world_m: tuple[float, float, float] = (0, 0, 0)
    orientation_xyzw: tuple[float, float, float, float] = (0, 0, 0, 1)
    shadows: bool = True
    angular_diameter_degrees: float = 0
    source_parameters: FrozenMap = FrozenMap()

    def __post_init__(self):
        object.__setattr__(self, "color_linear_rgb", _vector(self.color_linear_rgb, 3, "light color", True))
        object.__setattr__(self, "position_world_m", _vector(self.position_world_m, 3, "position"))
        object.__setattr__(self, "orientation_xyzw", _vector(self.orientation_xyzw, 4, "orientation"))
        if abs(sum(v * v for v in self.orientation_xyzw) - 1) > 1e-6:
            raise ValueError("appearance orientation must be unit XYZW")
        if (
            not self.light_id
            or self.kind not in ("directional", "dome")
            or self.intensity_unit not in ("linear_rgb_irradiance", "linear_rgb_radiance", "lux", "nit")
        ):
            raise ValueError("unsupported light kind/unit")
        if not math.isfinite(self.intensity) or self.intensity < 0 or not 0 <= self.angular_diameter_degrees <= 180:
            raise ValueError("invalid light intensity/angle")
        if self.kind == "directional":
            direction = _vector(self.direction_world, 3, "direction")
            length = math.sqrt(sum(v * v for v in direction))
            if length == 0:
                raise ValueError("light direction must be nonzero")
            object.__setattr__(self, "direction_world", tuple(v / length for v in direction))


@dataclass(frozen=True)
class AppearanceEnvironment(_Value):
    background_linear_rgb: tuple[float, float, float]
    ambient_linear_rgb: tuple[float, float, float]
    ambient_semantics: str = "lambertian_albedo_multiplier"
    source_parameters: FrozenMap = FrozenMap()

    def __post_init__(self):
        for name in ("background_linear_rgb", "ambient_linear_rgb"):
            object.__setattr__(self, name, _vector(getattr(self, name), 3, name, True))


@dataclass(frozen=True)
class AppearanceRenderer(_Value):
    backend: str
    renderer: str
    color_space: str = "linear-srgb"
    output_transfer: str = "gamma22"
    tone_mapping: str = "clamp"
    exposure_multiplier: float = 1
    source_parameters: FrozenMap = FrozenMap()

    def __post_init__(self):
        if (
            not self.backend
            or not self.renderer
            or not math.isfinite(self.exposure_multiplier)
            or self.exposure_multiplier <= 0
        ):
            raise ValueError("invalid appearance renderer")


@dataclass(frozen=True)
class AppearanceSnapshot(_Value):
    materials: tuple[AppearanceMaterial, ...]
    bindings: tuple[AppearanceBinding, ...]
    lights: tuple[AppearanceLight, ...]
    environment: AppearanceEnvironment
    renderer: AppearanceRenderer
    limitations: tuple[str, ...] = ()
    schema: str = "unirobosim.appearance/1"

    def __post_init__(self):
        if self.schema != "unirobosim.appearance/1":
            raise ValueError("unsupported appearance schema")
        for name, kind in (
            ("materials", AppearanceMaterial),
            ("bindings", AppearanceBinding),
            ("lights", AppearanceLight),
        ):
            value = tuple(getattr(self, name))
            if any(not isinstance(x, kind) for x in value):
                raise TypeError(f"invalid {name}")
            object.__setattr__(self, name, value)
        object.__setattr__(self, "limitations", tuple(self.limitations))
        ids = [x.material_id for x in self.materials]
        if len(set(ids)) != len(ids) or any(x.material_id not in ids for x in self.bindings):
            raise ValueError("appearance material identity is duplicate or missing")
        binding_keys = [(b.entity_path, b.mesh_key, b.environment_indices) for b in self.bindings]
        if len(set(binding_keys)) != len(binding_keys):
            raise ValueError("duplicate appearance binding")
        if len({x.light_id for x in self.lights}) != len(self.lights):
            raise ValueError("duplicate light id")


@dataclass(frozen=True)
class AppearanceApplyResult(_Value):
    material_count: int
    binding_count: int
    light_count: int
    limitations: tuple[str, ...] = ()
    conversion_notes: tuple[str, ...] = ()


def appearance_topology_sha256(vertices_m, face_indices, face_counts):
    """Exact canonical float32 local positions and int64 ordered topology digest."""
    xyz = tuple(float(v) for row in vertices_m for v in row)
    indices = tuple(int(v) for v in face_indices)
    counts = tuple(int(v) for v in face_counts)
    if len(xyz) % 3 or sum(counts) != len(indices) or any(not math.isfinite(v) for v in xyz):
        raise ValueError("invalid visual topology")
    data = struct.pack("<3Q", len(xyz) // 3, len(indices), len(counts))
    data += struct.pack("<" + str(len(xyz)) + "f", *xyz)
    data += struct.pack("<" + str(len(indices)) + "q", *indices)
    data += struct.pack("<" + str(len(counts)) + "q", *counts)
    return hashlib.sha256(data).hexdigest()


@runtime_checkable
class AppearanceCaptureWorld(Protocol):
    """Optional native effective appearance capture, on the world's owner thread."""

    def capture_appearance(self) -> AppearanceSnapshot: ...


@runtime_checkable
class AppearanceApplyWorld(Protocol):
    """Optional validated visual restore without stepping physics or changing topology."""

    def apply_appearance(self, snapshot: AppearanceSnapshot) -> AppearanceApplyResult: ...
