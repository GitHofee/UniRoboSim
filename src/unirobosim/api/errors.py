"""Structured public errors for UniRoboSim."""

from __future__ import annotations

from collections.abc import Mapping
from typing import TYPE_CHECKING, Any, cast

if TYPE_CHECKING:
    from .frozen import FrozenMap


WORLD_SCHEMA_UNSUPPORTED = "WORLD_SCHEMA_UNSUPPORTED"
ENTITY_SCALE_UNSUPPORTED = "ENTITY_SCALE_UNSUPPORTED"
ARTICULATION_AXIS_UNITS_MISMATCH = "ARTICULATION_AXIS_UNITS_MISMATCH"
ARTICULATION_POSITION_AXIS_UNITS_UNSUPPORTED = "ARTICULATION_POSITION_AXIS_UNITS_UNSUPPORTED"
ASSET_IDENTITY_CHANGED = "ASSET_IDENTITY_CHANGED"
ASSET_DEPENDENCY_INCOMPLETE = "ASSET_DEPENDENCY_INCOMPLETE"
NATIVE_PROVENANCE_MISMATCH = "NATIVE_PROVENANCE_MISMATCH"


def _detached_planning_text(
    value: object,
    fallback: str,
    codepoint_limit: int,
    byte_limit: int,
) -> str:
    """Return an exact bounded string without retaining its source object."""

    actual_type = type(value)
    try:
        mro = type.mro(actual_type)
    except BaseException:
        return fallback
    if type(mro) is not list or list.__len__(mro) > 256:
        return fallback
    if not any(list.__getitem__(mro, index) is str for index in range(list.__len__(mro))):
        return fallback
    try:
        source = cast(str, value)
        source_length = str.__len__(source)
        result = (
            str.__getitem__(source, slice(0, codepoint_limit))
            if source_length > codepoint_limit
            else str.__str__(source)
        )
    except BaseException:
        return fallback
    if (
        type(result) is not str
        or not result
        or "\x00" in result
        or any(0xD800 <= ord(character) <= 0xDFFF for character in result)
    ):
        return fallback
    if len(result) > codepoint_limit:
        result = result[:codepoint_limit]
    encoded = result.encode("utf-8")
    if len(encoded) > byte_limit:
        result = encoded[:byte_limit].decode("utf-8", errors="ignore")
    return result or fallback


def _detached_planning_optional_text(value: object) -> str | None:
    if value is None:
        return None
    return _detached_planning_text(value, "unavailable", 512, 2048)


class UniRoboSimError(Exception):
    """Base class for every expected public failure."""

    code = "unirobosim.error"

    def __getattribute__(self, name: str) -> Any:
        if name == "__traceback__":
            try:
                state = BaseException.__getattribute__(self, "__dict__")
                redact = type(state) is dict and dict.get(state, "_redact_traceback_on_read") is True
            except BaseException:
                redact = False
            if redact:
                BaseException.__setattr__(self, "__traceback__", None)
                return None
        return BaseException.__getattribute__(self, name)

    def _redact_retained_traceback(self) -> None:
        """Hide and release a path-bearing boundary traceback on first inspection."""

        self._redact_traceback_on_read = True

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


class AssetNormalizationError(UniRoboSimError):
    code = "unirobosim.asset.normalization_failed"


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


class PlanningSceneError(UniRoboSimError):
    """Base class for expected failures from ``planning.scene@2``."""

    code = "unirobosim.planning_scene"

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
        """Detach all public error fields and never retain an upstream graph.

        Planning-scene failures cross worker and process boundaries.  The
        original ``details`` and ``cause`` are deliberately not retained.
        """

        super().__init__(
            _detached_planning_text(message, "planning-scene operation failed", 1024, 4096),
            operation=_detached_planning_text(operation, "planning_scene", 256, 1024),
            backend_id=_detached_planning_optional_text(backend_id),
            world_id=_detached_planning_optional_text(world_id),
            entity_path=_detached_planning_optional_text(entity_path),
            details=None,
            cause=None,
        )


class PlanningSceneContractError(PlanningSceneError):
    code = "unirobosim.planning_scene.contract"


class PlanningSceneIncompleteError(PlanningSceneError):
    """A demanded World cannot prove a complete simulator-effective scene."""

    code = "unirobosim.planning_scene.incomplete"


class PlanningSceneNotFoundError(PlanningSceneError):
    code = "unirobosim.planning_scene.not_found"


class PlanningSceneRepresentationError(PlanningSceneError):
    code = "unirobosim.planning_scene.representation_unavailable"


class PlanningSceneHashMismatchError(PlanningSceneError):
    code = "unirobosim.planning_scene.hash_mismatch"


class PlanningSceneStaleGenerationError(PlanningSceneError):
    code = "unirobosim.planning_scene.stale_generation"


class PlanningSceneDeltaContinuityError(PlanningSceneError):
    code = "unirobosim.planning_scene.delta_continuity_lost"


class PlanningGeometryResourceRevokedError(PlanningSceneError):
    code = "unirobosim.planning_scene.resource_revoked"


def _snapshot_and_scrub_planning_error(
    error: PlanningSceneError,
    allowed_types: tuple[type[PlanningSceneError], ...],
) -> tuple[
    type[PlanningSceneError],
    Any,
    Any,
    Any,
    Any,
    Any,
]:
    """Copy fields only from exact trusted error types, then drop source links.

    This helper is intentionally private.  Planning-scene boundaries replace the
    caught exception so their public traceback cannot retain caller-controlled
    arguments.  A subclass is untrusted: properties, ``__getattribute__`` and
    extra slots may execute code or expose an arbitrary object graph, so no field
    is read from it.
    """

    exact_type = type(error)
    if any(exact_type is allowed for allowed in allowed_types):
        try:
            state = BaseException.__getattribute__(error, "__dict__")
        except BaseException:
            state = None
        if type(state) is dict:
            failure = (
                exact_type,
                _detached_planning_text(
                    dict.get(state, "message"),
                    "planning-scene operation failed",
                    1024,
                    4096,
                ),
                _detached_planning_text(dict.get(state, "operation"), "planning_scene", 256, 1024),
                _detached_planning_optional_text(dict.get(state, "backend_id")),
                _detached_planning_optional_text(dict.get(state, "world_id")),
                _detached_planning_optional_text(dict.get(state, "entity_path")),
            )
        else:
            failure = (
                PlanningSceneContractError,
                "planning-scene operation failed",
                "planning_scene",
                None,
                None,
                None,
            )
    else:
        failure = (
            PlanningSceneContractError,
            "planning-scene operation failed",
            "planning_scene",
            None,
            None,
            None,
        )

    # Use BaseException directly so a hostile subclass cannot intercept the
    # cleanup.  Clearing the instance dictionary is safe because the caught
    # error is always replaced and never re-raised.
    try:
        state = BaseException.__getattribute__(error, "__dict__")
        if type(state) is dict:
            dict.clear(state)
    except BaseException:
        pass
    for name, value in (
        ("args", ()),
        ("__traceback__", None),
        ("__cause__", None),
        ("__context__", None),
    ):
        try:
            BaseException.__setattr__(error, name, value)
        except BaseException:
            pass
    return failure
