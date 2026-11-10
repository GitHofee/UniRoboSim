"""Optional bounded, per-physics-step contact evidence; disabled by default."""

from __future__ import annotations

import math
import re
from dataclasses import asdict, dataclass
from typing import Protocol, runtime_checkable

from .capabilities import CapabilityId
from .kinematics import KinematicTarget

CONTACT_DIAGNOSTICS_CAPABILITY = CapabilityId("contact.diagnostics@1")


def _integer(value: object, name: str, low: int, high: int) -> None:
    if type(value) is not int or not low <= value <= high:
        raise ValueError(f"{name} must be an integer in [{low}, {high}]")


def _vector(value: object, size: int) -> None:
    if type(value) is not tuple or len(value) != size:
        raise TypeError(f"expected an exact {size}-element tuple")
    if any(type(x) not in (int, float) or not math.isfinite(x) for x in value):
        raise ValueError("contact vectors must contain finite numbers")


@dataclass(frozen=True, slots=True)
class StaticContactTarget:
    """One admitted static collision geometry, selected by its catalog ID."""

    target_id: str
    geometry_id: str

    def __post_init__(self) -> None:
        if (type(self.target_id) is not str or not self.target_id
                or len(self.target_id) > 1024 or "\x00" in self.target_id):
            raise ValueError("target_id must be bounded non-empty text")
        if type(self.geometry_id) is not str or not re.fullmatch(r"geometry\.[a-zA-Z0-9_-]{1,128}", self.geometry_id):
            raise ValueError("geometry_id must be a canonical geometry ID")


@dataclass(frozen=True, slots=True)
class ContactDiagnosticsRequest:
    subject: KinematicTarget
    partners: tuple[KinematicTarget | StaticContactTarget, ...]
    capacity_substeps: int = 4096
    max_contact_points: int = 1024

    def __post_init__(self) -> None:
        if type(self.subject) is not KinematicTarget:
            raise TypeError("subject must be a KinematicTarget")
        if type(self.partners) is not tuple or not 1 <= len(self.partners) <= 8:
            raise ValueError("partners must contain between one and eight targets")
        if any(type(p) not in (KinematicTarget, StaticContactTarget) for p in self.partners):
            raise TypeError("partners must be rigid-link or static-geometry targets")
        targets = (self.subject, *self.partners)
        if len({t.target_id for t in targets}) != len(targets):
            raise ValueError("contact target IDs must be unique")
        selections = {("static", t.geometry_id) if isinstance(t, StaticContactTarget)
                      else (t.entity_path, t.link_name) for t in targets}
        if len(selections) != len(targets):
            raise ValueError("contact bodies must be unique")
        _integer(self.capacity_substeps, "capacity_substeps", 1, 16384)
        _integer(self.max_contact_points, "max_contact_points", 16, 4096)


@dataclass(frozen=True, slots=True)
class ContactForcePoint:
    """Force on the subject, at its own application point, in world SI units."""

    position_m: tuple[float, float, float]
    force_n: tuple[float, float, float]

    def __post_init__(self) -> None:
        _vector(self.position_m, 3)
        _vector(self.force_n, 3)


@dataclass(frozen=True, slots=True)
class ContactPairEvidence:
    partner_id: str
    partner_pose_xyzw: tuple[float, ...]
    normal: tuple[ContactForcePoint, ...]
    friction: tuple[ContactForcePoint, ...]
    normal_force_n: tuple[float, float, float]
    friction_force_n: tuple[float, float, float]
    torque_about_subject_com_nm: tuple[float, float, float]
    partner_pose_start_xyzw: tuple[float, ...]

    def __post_init__(self) -> None:
        if type(self.partner_id) is not str or not self.partner_id or len(self.partner_id) > 1024:
            raise ValueError("partner_id must be a bounded non-empty string")
        _vector(self.partner_pose_xyzw, 7)
        _vector(self.partner_pose_start_xyzw, 7)
        for points in (self.normal, self.friction):
            if type(points) is not tuple or len(points) > 4096:
                raise ValueError("contact points must be a bounded tuple")
            if any(type(p) is not ContactForcePoint for p in points):
                raise TypeError("contact point type is invalid")
        for v in (self.normal_force_n, self.friction_force_n, self.torque_about_subject_com_nm):
            _vector(v, 3)


@dataclass(frozen=True, slots=True)
class ContactDiagnosticsSample:
    sequence: int
    world_step: int
    substep: int
    substeps: int
    physics_dt_s: float
    subject_pose_xyzw: tuple[float, ...]
    subject_com_m: tuple[float, float, float]
    subject_velocity: tuple[float, ...]
    pairs: tuple[ContactPairEvidence, ...]
    subject_mass_kg: float
    subject_pose_start_xyzw: tuple[float, ...]
    subject_com_start_m: tuple[float, float, float]
    subject_velocity_start: tuple[float, ...]

    def __post_init__(self) -> None:
        _integer(self.sequence, "sequence", 1, 2**63 - 1)
        _integer(self.world_step, "world_step", 1, 2**63 - 1)
        _integer(self.substeps, "substeps", 1, 1024)
        _integer(self.substep, "substep", 1, self.substeps)
        if (
            type(self.physics_dt_s) not in (float, int)
            or not math.isfinite(self.physics_dt_s)
            or self.physics_dt_s <= 0
        ):
            raise ValueError("physics_dt_s must be finite and positive")
        _vector(self.subject_pose_xyzw, 7)
        _vector(self.subject_pose_start_xyzw, 7)
        _vector(self.subject_com_start_m, 3)
        _vector(self.subject_velocity_start, 6)
        _vector(self.subject_com_m, 3)
        _vector(self.subject_velocity, 6)
        if (type(self.subject_mass_kg) not in (int, float)
                or not math.isfinite(self.subject_mass_kg) or self.subject_mass_kg <= 0):
            raise ValueError("subject_mass_kg must be finite and positive")
        if type(self.pairs) is not tuple or not 1 <= len(self.pairs) <= 8:
            raise ValueError("pairs must be a bounded non-empty tuple")
        if any(type(p) is not ContactPairEvidence for p in self.pairs):
            raise TypeError("invalid contact pair evidence")
        if len({p.partner_id for p in self.pairs}) != len(self.pairs):
            raise ValueError("duplicate contact partner")


@dataclass(frozen=True, slots=True)
class ContactDiagnosticsBatch:
    """Non-destructive bounded history. A stale cursor must raise, never skip."""

    world_id: str
    generation: int
    epoch: int
    last_sequence: int
    samples: tuple[ContactDiagnosticsSample, ...]

    def __post_init__(self) -> None:
        if type(self.world_id) is not str or not self.world_id or len(self.world_id) > 1024:
            raise ValueError("world_id must be a bounded non-empty string")
        _integer(self.generation, "generation", 1, 2**63 - 1)
        _integer(self.epoch, "epoch", 1, 2**63 - 1)
        _integer(self.last_sequence, "last_sequence", 0, 2**63 - 1)
        if type(self.samples) is not tuple or len(self.samples) > 16384:
            raise ValueError("samples must be a bounded tuple")
        if any(type(s) is not ContactDiagnosticsSample for s in self.samples):
            raise TypeError("invalid contact sample")
        seq = [s.sequence for s in self.samples]
        if seq and (seq[-1] != self.last_sequence or seq != list(range(seq[0], seq[-1] + 1))):
            raise ValueError("contact sample sequence has a gap or mismatched endpoint")

    def to_dict(self) -> dict[str, object]:
        return {"schema": "contact-diagnostics/1", "frame": "environment-local-world", **asdict(self)}


@runtime_checkable
class ContactDiagnosticsWorld(Protocol):
    def start_contact_diagnostics(self, request: ContactDiagnosticsRequest) -> int: ...

    def contact_diagnostics(self, epoch: int, after_sequence: int = 0) -> ContactDiagnosticsBatch: ...

    def stop_contact_diagnostics(self) -> None: ...


__all__ = [
    "CONTACT_DIAGNOSTICS_CAPABILITY",
    "ContactDiagnosticsRequest",
    "StaticContactTarget",
    "ContactForcePoint",
    "ContactPairEvidence",
    "ContactDiagnosticsSample",
    "ContactDiagnosticsBatch",
    "ContactDiagnosticsWorld",
]
