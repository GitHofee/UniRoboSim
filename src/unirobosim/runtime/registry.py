"""Explicit provider registration and capability-based selection."""

from __future__ import annotations

from collections.abc import Callable, Iterable
from threading import RLock

from unirobosim.api.capabilities import CapabilityRequirement
from unirobosim.api.errors import ProviderRegistrationError, ProviderSelectionError
from unirobosim.api.protocols import Provider
from unirobosim.api.reports import ProviderDescriptor

ProviderFactory = Callable[[], Provider]


class ProviderRegistry:
    """A process-local registry with no implicit imports or global singleton."""

    def __init__(self) -> None:
        self._entries: dict[str, tuple[ProviderDescriptor, ProviderFactory]] = {}
        self._lock = RLock()

    def register(self, descriptor: ProviderDescriptor, factory: ProviderFactory) -> None:
        if not isinstance(descriptor, ProviderDescriptor) or not callable(factory):
            raise ProviderRegistrationError(
                "registration requires a ProviderDescriptor and callable factory",
                operation="provider_registry.register",
            )
        with self._lock:
            if descriptor.provider_id in self._entries:
                raise ProviderRegistrationError(
                    "provider ID is already registered",
                    operation="provider_registry.register",
                    backend_id=descriptor.provider_id,
                )
            self._entries[descriptor.provider_id] = (descriptor, factory)

    def unregister(self, provider_id: str) -> None:
        with self._lock:
            if provider_id not in self._entries:
                raise ProviderRegistrationError(
                    "provider ID is not registered",
                    operation="provider_registry.unregister",
                    backend_id=provider_id,
                )
            del self._entries[provider_id]

    def descriptors(self) -> tuple[ProviderDescriptor, ...]:
        with self._lock:
            return tuple(self._entries[key][0] for key in sorted(self._entries))

    def create(self, provider_id: str) -> Provider:
        with self._lock:
            entry = self._entries.get(provider_id)
        if entry is None:
            raise ProviderSelectionError(
                "provider ID is not registered",
                operation="provider_registry.create",
                backend_id=provider_id,
                details={"registered": [item.provider_id for item in self.descriptors()]},
            )
        descriptor, factory = entry
        try:
            provider = factory()
        except Exception as exc:
            raise ProviderSelectionError(
                "provider factory failed",
                operation="provider_registry.create",
                backend_id=provider_id,
                cause=exc,
            ) from exc
        if not isinstance(provider, Provider):
            raise ProviderRegistrationError(
                "provider factory returned an object that does not satisfy the Provider protocol",
                operation="provider_registry.create",
                backend_id=provider_id,
            )
        if provider.descriptor != descriptor:
            raise ProviderRegistrationError(
                "provider factory descriptor differs from its registration descriptor",
                operation="provider_registry.create",
                backend_id=provider_id,
            )
        return provider

    def select(
        self,
        requirements: Iterable[CapabilityRequirement] = (),
        *,
        provider_id: str | None = None,
    ) -> Provider:
        requested = tuple(requirements)
        candidates = (
            (provider_id,) if provider_id is not None else tuple(item.provider_id for item in self.descriptors())
        )
        attempts: list[dict[str, object]] = []
        for candidate_id in candidates:
            try:
                provider = self.create(candidate_id)
                probe = provider.probe()
                negotiation = provider.descriptor.capabilities.negotiate(requested)
                attempts.append(
                    {
                        "provider_id": candidate_id,
                        "available": probe.available,
                        "reason": probe.reason,
                        "negotiation": negotiation.to_dict(),
                    }
                )
                if probe.available and negotiation.accepted:
                    return provider
            except ProviderSelectionError as exc:
                attempts.append({"provider_id": candidate_id, "error": exc.to_dict()})
        raise ProviderSelectionError(
            "no available provider satisfies the requested capabilities",
            operation="provider_registry.select",
            backend_id=provider_id,
            details={"attempts": attempts},
        )
