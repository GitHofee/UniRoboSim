"""Structured public errors for UniRoboSim."""

from __future__ import annotations

from collections.abc import Mapping
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from .frozen import FrozenMap


class UniRoboSimError(Exception):
    """Base class for every expected public failure."""

    code = "unirobosim.error"

    def __init__(
        self,
        message: str,
        *,
        operation: str,
        backend_id: str | None = None,
        world_id: str | None = None,
        entity_path: str | None = None,
        details: Mapping[str, Any] | None = None,
        cause: BaseException | None = None,
    ) -> None:
        from .frozen import FrozenMap

        super().__init__(message)
        self.message = message
        self.operation = operation
        self.backend_id = backend_id
        self.world_id = world_id
        self.entity_path = entity_path
        self.details: FrozenMap = FrozenMap(details)
        if cause is not None:
            self.__cause__ = cause

    def to_dict(self) -> dict[str, Any]:
        return {
            "code": self.code,
            "message": self.message,
            "operation": self.operation,
            "backend_id": self.backend_id,
            "world_id": self.world_id,
            "entity_path": self.entity_path,
            "details": self.details.to_dict(),
        }

    def __str__(self) -> str:
        scope = f" during {self.operation}" if self.operation else ""
        return f"[{self.code}]{scope}: {self.message}"


class ValidationError(UniRoboSimError):
    code = "unirobosim.validation"


class UnsupportedCapabilityError(UniRoboSimError):
    code = "unirobosim.capability.unsupported"


class CapabilityNegotiationError(UniRoboSimError):
    code = "unirobosim.capability.negotiation_failed"


class LifecycleError(UniRoboSimError):
    code = "unirobosim.lifecycle.invalid_transition"


class WorldBuildError(UniRoboSimError):
    code = "unirobosim.world.build_failed"


class AssetConversionError(UniRoboSimError):
    code = "unirobosim.asset.conversion_failed"


class EntityNotFoundError(UniRoboSimError):
    code = "unirobosim.entity.not_found"


class StaleHandleError(UniRoboSimError):
    code = "unirobosim.handle.stale"


class CommandError(UniRoboSimError):
    code = "unirobosim.command.invalid"


class ProviderRegistrationError(UniRoboSimError):
    code = "unirobosim.provider.registration"


class ProviderSelectionError(UniRoboSimError):
    code = "unirobosim.provider.selection"
