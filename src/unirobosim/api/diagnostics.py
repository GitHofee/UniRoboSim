"""Capability-gated access to portable provider runtime diagnostics."""

from __future__ import annotations

import re
from collections.abc import Mapping
from typing import Any, NoReturn, cast

from .capabilities import CapabilityId
from .errors import UnsupportedCapabilityError, ValidationError
from .reports import ProviderDescriptor, RuntimeDiagnostics

RUNTIME_DIAGNOSTICS_CAPABILITY = CapabilityId("runtime.diagnostics@1")
_CONNECTION_MODES_PROPERTY = "connection_modes"
_CONNECTION_MODE = re.compile(r"^[a-z][a-z0-9]*(?:[._-][a-z0-9]+)*$")


def _fail(
    message: str,
    *,
    cause: Exception,
    backend_id: str | None = None,
    details: Mapping[str, Any] | None = None,
) -> NoReturn:
    error = ValidationError(
        message,
        operation="runtime.diagnostics.read",
        backend_id=backend_id,
        details=details,
        cause=cause,
    )
    raise error from cause


def _declared_connection_modes(declaration: object, *, backend_id: str) -> tuple[str, ...] | None:
    """Return the optional, validated diagnostics connection-mode constraint.

    ``connection_modes`` is an optional property for source compatibility with
    providers that already declared ``runtime.diagnostics@1`` without mode
    metadata. When supplied, it is a non-empty JSON array of unique canonical
    connection-mode strings and becomes an enforced allow-list.
    """

    try:
        properties = declaration.properties  # type: ignore[attr-defined]
        has_constraint = _CONNECTION_MODES_PROPERTY in properties
        raw_modes = properties.get(_CONNECTION_MODES_PROPERTY)
    except Exception as exc:
        _fail(
            "runtime diagnostics capability properties could not be read",
            cause=exc,
            backend_id=backend_id,
        )
    if not has_constraint:
        return None
    if not isinstance(raw_modes, tuple):
        _fail(
            "runtime diagnostics connection_modes must be a JSON array",
            cause=TypeError("connection_modes is not an array"),
            backend_id=backend_id,
            details={"property": _CONNECTION_MODES_PROPERTY},
        )
    if not raw_modes:
        _fail(
            "runtime diagnostics connection_modes must not be empty",
            cause=ValueError("connection_modes is empty"),
            backend_id=backend_id,
            details={"property": _CONNECTION_MODES_PROPERTY},
        )
    if any(not isinstance(mode, str) or not _CONNECTION_MODE.fullmatch(mode) for mode in raw_modes):
        _fail(
            "runtime diagnostics connection_modes contains an invalid mode",
            cause=ValueError("connection_modes contains a non-canonical mode"),
            backend_id=backend_id,
            details={"property": _CONNECTION_MODES_PROPERTY},
        )
    modes = cast(tuple[str, ...], raw_modes)
    if len(modes) != len(set(modes)):
        _fail(
            "runtime diagnostics connection_modes must be unique",
            cause=ValueError("connection_modes contains duplicates"),
            backend_id=backend_id,
            details={"property": _CONNECTION_MODES_PROPERTY},
        )
    return modes


def read_runtime_diagnostics(provider: object) -> RuntimeDiagnostics:
    """Read a validated portable snapshot without exposing adapter internals.

    Every provider-controlled access is fail-closed behind a structured
    :class:`ValidationError`. If the capability declaration contains the optional
    ``connection_modes`` property, the returned mode must be one of that property's
    non-empty array of canonical mode names.
    """

    operation = "runtime.diagnostics.read"
    candidate = cast(Any, provider)
    try:
        descriptor = candidate.descriptor
    except Exception as exc:
        _fail(
            "runtime diagnostics require a provider descriptor",
            cause=exc,
        )
    if not isinstance(descriptor, ProviderDescriptor):
        _fail(
            "runtime diagnostics provider descriptor is invalid",
            cause=TypeError("descriptor is not a ProviderDescriptor"),
        )
    try:
        declaration = descriptor.capabilities.get(RUNTIME_DIAGNOSTICS_CAPABILITY)
    except Exception as exc:
        _fail(
            "runtime diagnostics capability declaration could not be read",
            cause=exc,
            backend_id=descriptor.provider_id,
        )
    if declaration is None:
        raise UnsupportedCapabilityError(
            "provider does not declare portable runtime diagnostics",
            operation=operation,
            backend_id=descriptor.provider_id,
            details={"capability": RUNTIME_DIAGNOSTICS_CAPABILITY.value},
        )
    connection_modes = _declared_connection_modes(declaration, backend_id=descriptor.provider_id)
    try:
        endpoint = candidate.runtime_diagnostics
    except Exception as exc:
        _fail(
            "provider declares runtime diagnostics without implementing its endpoint",
            cause=exc,
            backend_id=descriptor.provider_id,
        )
    if not callable(endpoint):
        _fail(
            "provider runtime diagnostics endpoint is not callable",
            cause=TypeError("runtime_diagnostics endpoint is not callable"),
            backend_id=descriptor.provider_id,
        )
    try:
        snapshot = endpoint()
    except Exception as exc:
        _fail(
            "provider runtime diagnostics endpoint failed",
            cause=exc,
            backend_id=descriptor.provider_id,
        )
    if not isinstance(snapshot, RuntimeDiagnostics):
        _fail(
            "runtime diagnostics endpoint returned an invalid report",
            cause=TypeError("runtime_diagnostics did not return RuntimeDiagnostics"),
            backend_id=descriptor.provider_id,
            details={"report_type": type(snapshot).__name__},
        )
    if snapshot.provider_id != descriptor.provider_id:
        _fail(
            "runtime diagnostics provider identity does not match its descriptor",
            cause=ValueError("reported provider identity differs from descriptor"),
            backend_id=descriptor.provider_id,
            details={"reported_provider_id": snapshot.provider_id},
        )
    if connection_modes is not None and snapshot.connection_mode not in connection_modes:
        _fail(
            "runtime diagnostics connection mode is not declared by the provider",
            cause=ValueError("reported connection mode is outside the declared allow-list"),
            backend_id=descriptor.provider_id,
            details={
                "reported_connection_mode": snapshot.connection_mode,
                "declared_connection_modes": connection_modes,
            },
        )
    return snapshot


__all__ = ["RUNTIME_DIAGNOSTICS_CAPABILITY", "read_runtime_diagnostics"]
