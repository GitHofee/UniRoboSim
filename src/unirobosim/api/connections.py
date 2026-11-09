"""Passive connectors declared in a world, independent of its controller."""

import math
import re
from dataclasses import asdict, dataclass

from .errors import ValidationError
from .values import EntityPath, Pose


@dataclass(frozen=True)
class ConnectionProperties:
    """Finite passive connector constants, all in SI units."""

    stiffness_n_m: float
    damping_ns_m: float
    angular_stiffness_nm_rad: float
    angular_damping_nms_rad: float
    max_force_n: float
    max_torque_nm: float
    break_distance_m: float
    break_angle_rad: float
    admission_position_m: float
    admission_angle_rad: float

    def __post_init__(self) -> None:
        for name in self.__dataclass_fields__:
            value = getattr(self, name)
            try:
                if type(value) not in (int, float):
                    raise TypeError("expected int or float")
                number = float(value)
            except (TypeError, ValueError, OverflowError) as exc:
                raise ValidationError(
                    f"connection {name} must be finite and positive", operation="connection_spec.validate"
                ) from exc
            if not math.isfinite(number) or number <= 0:
                raise ValidationError(
                    f"connection {name} must be finite and positive", operation="connection_spec.validate"
                )
            object.__setattr__(self, name, number)
        if (
            self.admission_position_m >= self.break_distance_m
            or self.admission_angle_rad >= self.break_angle_rad
            or self.break_angle_rad > math.pi
        ):
            raise ValidationError(
                "connection admission must be inside its break envelope", operation="connection_spec.validate"
            )


@dataclass(frozen=True)
class CompliantConnectionSpec:
    """Arm once on measured seating; exert no force before admission.

    The rest pose is child asset-root in parent asset-root coordinates. All
    parameters are SI. A broken connection stays broken until world reset.
    """

    connection_id: str
    parent_path: EntityPath
    child_path: EntityPath
    rest_pose: Pose
    properties: ConnectionProperties

    def __post_init__(self) -> None:
        if (
            type(self.connection_id) is not str
            or re.fullmatch(r"[A-Za-z][A-Za-z0-9_-]{0,127}", self.connection_id) is None
            or not isinstance(self.parent_path, EntityPath)
            or not isinstance(self.child_path, EntityPath)
            or self.parent_path == self.child_path
            or not isinstance(self.rest_pose, Pose)
            or not isinstance(self.properties, ConnectionProperties)
        ):
            raise ValidationError("invalid passive connector declaration", operation="connection_spec.validate")

    def to_dict(self) -> dict[str, object]:
        return {
            "connection_id": self.connection_id,
            "parent_path": self.parent_path.value,
            "child_path": self.child_path.value,
            "rest_pose": {
                "position": list(self.rest_pose.position),
                "orientation_xyzw": list(self.rest_pose.orientation_xyzw),
            },
            "properties": asdict(self.properties),
        }
