"""Portable build manifest values and the private Core-to-Adapter carrier."""

from __future__ import annotations

import hashlib
import math
import os
import posixpath
import re
import stat
import unicodedata
from dataclasses import dataclass
from pathlib import PurePosixPath
from typing import Literal, cast

from ._codec import canonical_json
from .errors import ASSET_DEPENDENCY_INCOMPLETE, ASSET_IDENTITY_CHANGED, ValidationError

BUILD_RESOURCE_MANIFEST_SCHEMA_VERSION = "unirobosim.build-resource-manifest/v1"
_SHA256 = re.compile(r"^[0-9a-f]{64}$")
_IDENTIFIER = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.:/-]*$")
_PURPOSES = frozenset(("simulation", "collision", "planning", "visual"))
_SOURCE_KINDS = frozenset(("local-file", "cached-remote"))
_MAX_RESOURCES = 100_000
_MAX_TEXT = 4096
_MAX_DEPENDENCY_DEPTH = 1024
_WINDOWS_DRIVE = re.compile(r"^[A-Za-z]:")


def _invalid(message: str, *, detail_code: str | None = None) -> ValidationError:
    details = {} if detail_code is None else {"detail_code": detail_code}
    return ValidationError(message, operation="build_input.validate", details=details)


def _text(value: object, label: str, *, identifier: bool = False) -> str:
    if type(value) is not str or not value or len(value) > _MAX_TEXT or "\x00" in value:
        raise _invalid(f"{label} must be bounded non-empty text") from None
    if any(0xD800 <= ord(character) <= 0xDFFF for character in value):
        raise _invalid(f"{label} must contain valid Unicode") from None
    if identifier and _IDENTIFIER.fullmatch(value) is None:
        raise _invalid(f"{label} must be a stable identifier") from None
    return value


def _digest(value: object, label: str) -> str:
    result = _text(value, label)
    if _SHA256.fullmatch(result) is None:
        raise _invalid(f"{label} must be lowercase SHA-256") from None
    return result


def _integer(value: object, label: str, *, positive: bool = False) -> int:
    if type(value) is not int or value < 0 or (positive and value == 0):
        raise _invalid(f"{label} must be a non-negative exact integer") from None
    return value


def _finite(value: object, label: str, *, positive: bool = False) -> float:
    if type(value) not in (int, float):
        raise _invalid(f"{label} must be an exact number") from None
    try:
        result = float(cast(int | float, value))
    except OverflowError:
        raise _invalid(f"{label} must be finite" + (" and positive" if positive else "")) from None
    if not math.isfinite(result) or (positive and result <= 0.0):
        raise _invalid(f"{label} must be finite" + (" and positive" if positive else "")) from None
    return 0.0 if result == 0.0 else result


def _relative_path(value: object, label: str) -> str:
    result = _text(value, label)
    if (
        "\\" in result
        or result.startswith("/")
        or _WINDOWS_DRIVE.match(result) is not None
        or posixpath.normpath(result) != result
        or unicodedata.normalize("NFC", result) != result
    ):
        raise _invalid(f"{label} must be a normalized POSIX relative path") from None
    parts = PurePosixPath(result).parts
    if not parts or any(part in ("", ".", "..") for part in parts):
        raise _invalid(f"{label} must not escape its bundle") from None
    return result


def _portable_path_key(value: str) -> str:
    """Return the cross-platform collision key for an already validated path."""

    return unicodedata.normalize("NFC", value).casefold()


def _identifier_tuple(value: object, label: str, *, allowed: frozenset[str] | None = None) -> tuple[str, ...]:
    if type(value) is not tuple:
        raise _invalid(f"{label} must be an immutable tuple") from None
    result = tuple(_text(item, f"{label} item", identifier=allowed is None) for item in value)
    if len(result) != len(set(result)):
        raise _invalid(f"{label} must not contain duplicates") from None
    if allowed is not None and any(item not in allowed for item in result):
        raise _invalid(f"{label} contains an unsupported value") from None
    return result


@dataclass(frozen=True, slots=True)
class BuildResourceEntry:
    entity_id: str
    component_id: str
    resource_id: str
    role: str
    media_type: str
    requested_uri: str
    resolved_uri: str
    canonical_source_identity: str
    byte_size: int
    sha256: str
    selected_simulation_input: bool
    purposes: tuple[Literal["simulation", "collision", "planning", "visual"], ...]
    relative_bundle_path: str
    dependencies: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        for field_name in ("entity_id", "component_id", "resource_id", "role"):
            object.__setattr__(self, field_name, _text(getattr(self, field_name), field_name, identifier=True))
        for field_name in ("media_type", "requested_uri", "resolved_uri", "canonical_source_identity"):
            object.__setattr__(self, field_name, _text(getattr(self, field_name), field_name))
        if "/" not in self.media_type:
            raise _invalid("media_type must be a MIME type") from None
        object.__setattr__(self, "byte_size", _integer(self.byte_size, "byte_size"))
        object.__setattr__(self, "sha256", _digest(self.sha256, "sha256"))
        if type(self.selected_simulation_input) is not bool:
            raise _invalid("selected_simulation_input must be an exact boolean") from None
        purposes = _identifier_tuple(self.purposes, "purposes", allowed=_PURPOSES)
        if not purposes or purposes != tuple(sorted(purposes)):
            raise _invalid("purposes must be a non-empty canonical tuple") from None
        object.__setattr__(self, "purposes", purposes)
        object.__setattr__(
            self, "relative_bundle_path", _relative_path(self.relative_bundle_path, "relative_bundle_path")
        )
        dependencies = _identifier_tuple(self.dependencies, "dependencies")
        if dependencies != tuple(sorted(dependencies)) or self.resource_id in dependencies:
            raise _invalid("dependencies must use canonical order and exclude self") from None
        object.__setattr__(self, "dependencies", dependencies)

    def to_dict(self) -> dict[str, object]:
        return {
            "entity_id": self.entity_id,
            "component_id": self.component_id,
            "resource_id": self.resource_id,
            "role": self.role,
            "media_type": self.media_type,
            "requested_uri": self.requested_uri,
            "resolved_uri": self.resolved_uri,
            "canonical_source_identity": self.canonical_source_identity,
            "byte_size": self.byte_size,
            "sha256": self.sha256,
            "selected_simulation_input": self.selected_simulation_input,
            "purposes": list(self.purposes),
            "relative_bundle_path": self.relative_bundle_path,
            "dependencies": list(self.dependencies),
        }


def _quaternion_matrix(value: tuple[float, float, float, float]) -> tuple[tuple[float, float, float], ...]:
    x, y, z, w = value
    return (
        (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)),
        (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)),
        (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)),
    )


@dataclass(frozen=True, slots=True)
class BuildLinkDynamics:
    link_id: str
    mass_kg: float
    center_of_mass_xyz_m: tuple[float, float, float]
    inertia_tensor_com_link_kg_m2: tuple[float, ...]
    principal_moments_kg_m2: tuple[float, float, float]
    principal_axes_xyzw: tuple[float, float, float, float]
    normalization_recipe_sha256: str

    def __post_init__(self) -> None:
        object.__setattr__(self, "link_id", _text(self.link_id, "link_id", identifier=True))
        object.__setattr__(self, "mass_kg", _finite(self.mass_kg, "mass_kg", positive=True))
        if type(self.center_of_mass_xyz_m) is not tuple or len(self.center_of_mass_xyz_m) != 3:
            raise _invalid("center_of_mass_xyz_m must be an exact 3-tuple") from None
        center = tuple(_finite(value, "center_of_mass_xyz_m item") for value in self.center_of_mass_xyz_m)
        if type(self.inertia_tensor_com_link_kg_m2) is not tuple or len(self.inertia_tensor_com_link_kg_m2) != 9:
            raise _invalid("inertia tensor must be an exact row-major 9-tuple") from None
        tensor = tuple(_finite(value, "inertia tensor item") for value in self.inertia_tensor_com_link_kg_m2)
        if type(self.principal_moments_kg_m2) is not tuple or len(self.principal_moments_kg_m2) != 3:
            raise _invalid("principal moments must be an exact 3-tuple") from None
        moments = tuple(_finite(value, "principal moment", positive=True) for value in self.principal_moments_kg_m2)
        if moments != tuple(sorted(moments)) or any(
            moments[index] > sum(moments) - moments[index] + 1.0e-12 for index in range(3)
        ):
            raise _invalid("principal moments must be ordered and satisfy rigid-body triangle inequalities") from None
        if type(self.principal_axes_xyzw) is not tuple or len(self.principal_axes_xyzw) != 4:
            raise _invalid("principal_axes_xyzw must be an exact 4-tuple") from None
        axes = cast(
            tuple[float, float, float, float],
            tuple(_finite(value, "principal axis quaternion item") for value in self.principal_axes_xyzw),
        )
        if not math.isclose(sum(value * value for value in axes), 1.0, rel_tol=0.0, abs_tol=1.0e-12):
            raise _invalid("principal axis quaternion must be unit length") from None
        x, y, z, w = axes
        if w < 0.0 or (w == 0.0 and next((item for item in (x, y, z) if item != 0.0), 1.0) < 0.0):
            raise _invalid("principal axis quaternion must use the canonical sign") from None
        if any(
            not math.isclose(tensor[row * 3 + column], tensor[column * 3 + row], rel_tol=1.0e-7, abs_tol=1.0e-9)
            for row in range(3)
            for column in range(3)
        ):
            raise _invalid("inertia tensor must be symmetric") from None
        rotation = _quaternion_matrix(axes)
        reconstructed = tuple(
            sum(rotation[row][axis] * moments[axis] * rotation[column][axis] for axis in range(3))
            for row in range(3)
            for column in range(3)
        )
        if any(
            not math.isclose(actual, expected, rel_tol=1.0e-7, abs_tol=1.0e-9)
            for actual, expected in zip(tensor, reconstructed, strict=True)
        ):
            raise _invalid("inertia tensor does not match principal moments and axes") from None
        object.__setattr__(self, "center_of_mass_xyz_m", center)
        object.__setattr__(self, "inertia_tensor_com_link_kg_m2", tensor)
        object.__setattr__(self, "principal_moments_kg_m2", moments)
        object.__setattr__(self, "principal_axes_xyzw", axes)
        object.__setattr__(
            self,
            "normalization_recipe_sha256",
            _digest(self.normalization_recipe_sha256, "normalization_recipe_sha256"),
        )

    def to_dict(self) -> dict[str, object]:
        return {
            "link_id": self.link_id,
            "mass_kg": self.mass_kg,
            "center_of_mass_xyz_m": list(self.center_of_mass_xyz_m),
            "inertia_tensor_com_link_kg_m2": list(self.inertia_tensor_com_link_kg_m2),
            "principal_moments_kg_m2": list(self.principal_moments_kg_m2),
            "principal_axes_xyzw": list(self.principal_axes_xyzw),
            "normalization_recipe_sha256": self.normalization_recipe_sha256,
        }


@dataclass(frozen=True, slots=True)
class BuildResourceManifest:
    entries: tuple[BuildResourceEntry, ...]
    link_dynamics: tuple[BuildLinkDynamics, ...] = ()
    schema_version: str = BUILD_RESOURCE_MANIFEST_SCHEMA_VERSION

    def __post_init__(self) -> None:
        if type(self.schema_version) is not str or self.schema_version != BUILD_RESOURCE_MANIFEST_SCHEMA_VERSION:
            raise _invalid("unsupported build resource manifest schema") from None
        if type(self.entries) is not tuple or not self.entries or len(self.entries) > _MAX_RESOURCES:
            raise _invalid("build resource manifest entries must be a bounded non-empty tuple") from None
        if any(type(entry) is not BuildResourceEntry for entry in self.entries):
            raise _invalid("build resource manifest contains an invalid entry") from None
        resource_ids = tuple(entry.resource_id for entry in self.entries)
        if resource_ids != tuple(sorted(resource_ids)) or len(resource_ids) != len(set(resource_ids)):
            raise _invalid("resource entries must use unique canonical resource order") from None
        bundle_paths = tuple(entry.relative_bundle_path for entry in self.entries)
        portable_paths = tuple(sorted(_portable_path_key(path) for path in bundle_paths))
        if len(portable_paths) != len(set(portable_paths)) or any(
            child.startswith(parent + "/") for parent, child in zip(portable_paths, portable_paths[1:], strict=False)
        ):
            raise _invalid("resource entries must use unique non-overlapping portable bundle paths") from None
        resource_set = frozenset(resource_ids)
        if any(dependency not in resource_set for entry in self.entries for dependency in entry.dependencies):
            raise _invalid("resource dependency graph is incomplete", detail_code=ASSET_DEPENDENCY_INCOMPLETE) from None
        dependency_by_id = {entry.resource_id: entry.dependencies for entry in self.entries}
        visited: set[str] = set()
        for root_id in resource_ids:
            if root_id in visited:
                continue
            active: set[str] = {root_id}
            stack: list[tuple[str, int]] = [(root_id, 0)]
            while stack:
                if len(stack) > _MAX_DEPENDENCY_DEPTH:
                    raise _invalid(
                        "resource dependency graph exceeds its depth budget",
                        detail_code=ASSET_DEPENDENCY_INCOMPLETE,
                    ) from None
                resource_id, dependency_index = stack[-1]
                dependencies = dependency_by_id[resource_id]
                if dependency_index == len(dependencies):
                    stack.pop()
                    active.remove(resource_id)
                    visited.add(resource_id)
                    continue
                dependency = dependencies[dependency_index]
                stack[-1] = (resource_id, dependency_index + 1)
                if dependency in active:
                    raise _invalid(
                        "resource dependency graph contains a cycle",
                        detail_code=ASSET_DEPENDENCY_INCOMPLETE,
                    ) from None
                if dependency not in visited:
                    active.add(dependency)
                    stack.append((dependency, 0))
        if type(self.link_dynamics) is not tuple or any(
            type(item) is not BuildLinkDynamics for item in self.link_dynamics
        ):
            raise _invalid("link_dynamics must contain BuildLinkDynamics values") from None
        link_ids = tuple(item.link_id for item in self.link_dynamics)
        if link_ids != tuple(sorted(link_ids)) or len(link_ids) != len(set(link_ids)):
            raise _invalid("link dynamics must use unique canonical link order") from None

    def to_dict(self) -> dict[str, object]:
        return {
            "schema_version": self.schema_version,
            "entries": [entry.to_dict() for entry in self.entries],
            "link_dynamics": [item.to_dict() for item in self.link_dynamics],
        }

    @property
    def canonical_json(self) -> str:
        return canonical_json(self.to_dict())

    @property
    def sha256(self) -> str:
        return hashlib.sha256(self.canonical_json.encode("utf-8")).hexdigest()


@dataclass(frozen=True, slots=True)
class LocalSourceIdentity:
    device: int
    inode: int
    mode: int
    byte_size: int
    mtime_ns: int
    ctime_ns: int

    def __post_init__(self) -> None:
        for field_name in ("device", "inode", "mode", "byte_size", "mtime_ns", "ctime_ns"):
            object.__setattr__(self, field_name, _integer(getattr(self, field_name), field_name))
        if not stat.S_ISREG(self.mode):
            raise _invalid("local source identity must describe a regular file") from None


@dataclass(frozen=True, slots=True)
class BuildSourceEntry:
    resource_id: str
    source_kind: Literal["local-file", "cached-remote"]
    source_root: str
    relative_source_path: str
    expected_identity: LocalSourceIdentity
    expected_sha256: str

    def __post_init__(self) -> None:
        object.__setattr__(self, "resource_id", _text(self.resource_id, "resource_id", identifier=True))
        source_kind = cast(object, self.source_kind)
        if type(source_kind) is not str or source_kind not in _SOURCE_KINDS:
            raise _invalid("source_kind must be local-file or cached-remote") from None
        root = _text(self.source_root, "source_root")
        if not os.path.isabs(root) or os.path.normpath(root) != root:
            raise _invalid("source_root must be an absolute normalized local directory") from None
        object.__setattr__(self, "source_root", root)
        object.__setattr__(
            self, "relative_source_path", _relative_path(self.relative_source_path, "relative_source_path")
        )
        if type(self.expected_identity) is not LocalSourceIdentity:
            raise _invalid("expected_identity must be a LocalSourceIdentity") from None
        object.__setattr__(self, "expected_sha256", _digest(self.expected_sha256, "expected_sha256"))


@dataclass(frozen=True, slots=True, kw_only=True)
class BuildInput:
    manifest: BuildResourceManifest
    sources: tuple[BuildSourceEntry, ...]

    def __post_init__(self) -> None:
        if type(self.manifest) is not BuildResourceManifest or type(self.sources) is not tuple:
            raise _invalid("BuildInput requires an exact manifest and source tuple") from None
        if any(type(source) is not BuildSourceEntry for source in self.sources):
            raise _invalid("BuildInput contains an invalid source") from None
        source_ids = tuple(source.resource_id for source in self.sources)
        manifest_ids = tuple(entry.resource_id for entry in self.manifest.entries)
        if source_ids != tuple(sorted(source_ids)) or len(source_ids) != len(set(source_ids)):
            raise _invalid("BuildInput sources must use unique canonical resource order") from None
        if source_ids != manifest_ids:
            raise _invalid(
                "BuildInput sources must exactly cover manifest resources", detail_code=ASSET_DEPENDENCY_INCOMPLETE
            ) from None
        entry_by_id = {entry.resource_id: entry for entry in self.manifest.entries}
        for source in self.sources:
            entry = entry_by_id[source.resource_id]
            if source.expected_sha256 != entry.sha256 or source.expected_identity.byte_size != entry.byte_size:
                raise _invalid(
                    "BuildInput source identity does not match its manifest", detail_code=ASSET_IDENTITY_CHANGED
                ) from None
