from __future__ import annotations

from dataclasses import FrozenInstanceError, replace
from typing import Any

import pytest

from unirobosim import (
    RUNTIME_DIAGNOSTICS_CAPABILITY,
    CapabilityDeclaration,
    CapabilitySet,
    FrozenMap,
    ProviderDescriptor,
    RuntimeDiagnostics,
    RuntimeDiagnosticsProvider,
    UnsupportedCapabilityError,
    ValidationError,
    read_runtime_diagnostics,
)
from unirobosim.testing import FAKE_DESCRIPTOR, FakeProvider


def _descriptor_with_diagnostics(
    properties: FrozenMap | None = None,
) -> ProviderDescriptor:
    return replace(
        FAKE_DESCRIPTOR,
        capabilities=CapabilitySet(
            (
                *FAKE_DESCRIPTOR.capabilities,
                CapabilityDeclaration(
                    RUNTIME_DIAGNOSTICS_CAPABILITY,
                    properties
                    if properties is not None
                    else FrozenMap(
                        {
                            "scope": "provider",
                            "connection_modes": ["reference"],
                            "fields": ["connection_mode", "live_worlds", "native_clients"],
                        }
                    ),
                ),
            )
        ),
    )


class DiagnosticProvider(FakeProvider):
    @property
    def descriptor(self) -> ProviderDescriptor:
        return _descriptor_with_diagnostics()

    def runtime_diagnostics(self) -> RuntimeDiagnostics:
        return RuntimeDiagnostics(self.descriptor.provider_id, "reference", 1, 2)


def test_runtime_diagnostics_are_immutable_portable_and_protocol_checked() -> None:
    snapshot = RuntimeDiagnostics("reference.fake", "in-process", 1, 2)
    assert snapshot.to_dict() == {
        "provider_id": "reference.fake",
        "connection_mode": "in-process",
        "live_worlds": 1,
        "native_clients": 2,
    }
    with pytest.raises(FrozenInstanceError, match="cannot assign to field"):
        snapshot.live_worlds = 0  # type: ignore[misc]
    provider = DiagnosticProvider()
    assert isinstance(provider, RuntimeDiagnosticsProvider)
    assert read_runtime_diagnostics(provider).to_dict()["native_clients"] == 2


@pytest.mark.parametrize(
    ("args", "message"),
    (
        (("INVALID", "direct", 0, 0), "provider ID"),
        (("reference.fake", "DIRECT", 0, 0), "connection mode"),
        (("reference.fake", "direct", -1, 0), "non-negative"),
        (("reference.fake", "direct", 0, True), "non-negative"),
    ),
)
def test_runtime_diagnostics_reject_invalid_values(args: tuple[object, ...], message: str) -> None:
    with pytest.raises(ValidationError, match=message):
        RuntimeDiagnostics(*args)  # type: ignore[arg-type]


def test_runtime_diagnostics_are_capability_gated_before_endpoint_access() -> None:
    class GatedProvider(FakeProvider):
        endpoint_reads = 0

        @property
        def runtime_diagnostics(self):
            self.endpoint_reads += 1
            raise AssertionError("the endpoint must not be read before capability gating")

    provider = GatedProvider()
    with pytest.raises(UnsupportedCapabilityError) as error:
        read_runtime_diagnostics(provider)
    assert error.value.details["capability"] == RUNTIME_DIAGNOSTICS_CAPABILITY.value
    assert provider.endpoint_reads == 0


def test_runtime_diagnostics_reject_declared_but_nonconforming_provider() -> None:
    class MissingEndpoint(FakeProvider):
        @property
        def descriptor(self) -> ProviderDescriptor:
            return _descriptor_with_diagnostics()

    with pytest.raises(ValidationError, match="without implementing") as missing_error:
        read_runtime_diagnostics(MissingEndpoint())
    assert isinstance(missing_error.value.__cause__, AttributeError)
    assert missing_error.value.operation == "runtime.diagnostics.read"
    assert missing_error.value.backend_id == FAKE_DESCRIPTOR.provider_id

    class WrongReport(MissingEndpoint):
        def runtime_diagnostics(self):
            return {"live_worlds": 0}

    with pytest.raises(ValidationError, match="invalid report") as report_error:
        read_runtime_diagnostics(WrongReport())
    assert isinstance(report_error.value.__cause__, TypeError)
    assert report_error.value.details["report_type"] == "dict"

    class WrongIdentity(MissingEndpoint):
        def runtime_diagnostics(self):
            return RuntimeDiagnostics("reference.different", "reference", 0, 0)

    with pytest.raises(ValidationError, match="identity") as identity_error:
        read_runtime_diagnostics(WrongIdentity())
    assert isinstance(identity_error.value.__cause__, ValueError)
    assert identity_error.value.details["reported_provider_id"] == "reference.different"


def test_runtime_diagnostics_reject_objects_without_provider_descriptors() -> None:
    with pytest.raises(ValidationError, match="provider descriptor") as error:
        read_runtime_diagnostics(object())
    assert isinstance(error.value.__cause__, AttributeError)
    assert error.value.operation == "runtime.diagnostics.read"


def test_runtime_diagnostics_wrap_descriptor_getter_failure_without_reading_endpoint() -> None:
    sentinel = RuntimeError("hostile descriptor getter")

    class HostileProvider:
        endpoint_reads = 0

        @property
        def descriptor(self):
            raise sentinel

        @property
        def runtime_diagnostics(self):
            self.endpoint_reads += 1
            raise AssertionError("endpoint must not be inspected")

    provider = HostileProvider()
    with pytest.raises(ValidationError, match="provider descriptor") as error:
        read_runtime_diagnostics(provider)
    assert error.value.__cause__ is sentinel
    assert provider.endpoint_reads == 0


def test_runtime_diagnostics_wrap_invalid_descriptor_type_with_a_cause() -> None:
    class InvalidDescriptorProvider:
        descriptor = {"provider_id": "reference.fake"}

    with pytest.raises(ValidationError, match="descriptor is invalid") as error:
        read_runtime_diagnostics(InvalidDescriptorProvider())
    assert isinstance(error.value.__cause__, TypeError)
    assert error.value.backend_id is None


def test_runtime_diagnostics_wrap_capability_lookup_failure() -> None:
    sentinel = RuntimeError("hostile capability index")

    class HostileCapabilitySet(CapabilitySet):
        def get(self, capability):
            raise sentinel

    descriptor = replace(FAKE_DESCRIPTOR, capabilities=HostileCapabilitySet())

    class HostileCapabilitiesProvider:
        @property
        def descriptor(self) -> ProviderDescriptor:
            return descriptor

    with pytest.raises(ValidationError, match="declaration could not be read") as error:
        read_runtime_diagnostics(HostileCapabilitiesProvider())
    assert error.value.__cause__ is sentinel
    assert error.value.backend_id == FAKE_DESCRIPTOR.provider_id


def test_runtime_diagnostics_wrap_capability_property_failure() -> None:
    sentinel = RuntimeError("hostile capability properties")

    class HostileDeclaration(CapabilityDeclaration):
        _armed = False

        def __getattribute__(self, name: str):
            if name == "properties" and object.__getattribute__(self, "_armed"):
                raise sentinel
            return super().__getattribute__(name)

    declaration = HostileDeclaration(RUNTIME_DIAGNOSTICS_CAPABILITY)
    object.__setattr__(declaration, "_armed", True)
    descriptor = replace(
        FAKE_DESCRIPTOR,
        capabilities=CapabilitySet((*FAKE_DESCRIPTOR.capabilities, declaration)),
    )

    class HostilePropertiesProvider:
        @property
        def descriptor(self) -> ProviderDescriptor:
            return descriptor

        def runtime_diagnostics(self) -> RuntimeDiagnostics:
            raise AssertionError("endpoint must not run after invalid capability metadata")

    with pytest.raises(ValidationError, match="properties could not be read") as error:
        read_runtime_diagnostics(HostilePropertiesProvider())
    assert error.value.__cause__ is sentinel
    assert error.value.backend_id == FAKE_DESCRIPTOR.provider_id


def test_runtime_diagnostics_wrap_endpoint_getter_failure_and_read_it_once() -> None:
    sentinel = RuntimeError("hostile endpoint getter")

    class HostileEndpointProvider(FakeProvider):
        endpoint_reads = 0

        @property
        def descriptor(self) -> ProviderDescriptor:
            return _descriptor_with_diagnostics()

        @property
        def runtime_diagnostics(self):
            self.endpoint_reads += 1
            raise sentinel

    provider = HostileEndpointProvider()
    with pytest.raises(ValidationError, match="without implementing") as error:
        read_runtime_diagnostics(provider)
    assert error.value.__cause__ is sentinel
    assert provider.endpoint_reads == 1


def test_runtime_diagnostics_reject_non_callable_endpoint_with_a_cause() -> None:
    class NonCallableEndpointProvider(FakeProvider):
        @property
        def descriptor(self) -> ProviderDescriptor:
            return _descriptor_with_diagnostics()

        runtime_diagnostics = RuntimeDiagnostics("reference.fake", "reference", 0, 0)

    with pytest.raises(ValidationError, match="not callable") as error:
        read_runtime_diagnostics(NonCallableEndpointProvider())
    assert isinstance(error.value.__cause__, TypeError)
    assert error.value.backend_id == FAKE_DESCRIPTOR.provider_id


@pytest.mark.parametrize(
    "sentinel",
    (
        RuntimeError("native diagnostics query failed"),
        ValidationError("nested typed failure", operation="adapter.runtime_diagnostics"),
    ),
)
def test_runtime_diagnostics_wrap_endpoint_call_failures(sentinel: Exception) -> None:
    class FailingEndpointProvider(FakeProvider):
        calls = 0

        @property
        def descriptor(self) -> ProviderDescriptor:
            return _descriptor_with_diagnostics()

        def runtime_diagnostics(self) -> RuntimeDiagnostics:
            self.calls += 1
            raise sentinel

    provider = FailingEndpointProvider()
    with pytest.raises(ValidationError, match="endpoint failed") as error:
        read_runtime_diagnostics(provider)
    assert error.value.__cause__ is sentinel
    assert error.value.operation == "runtime.diagnostics.read"
    assert error.value.backend_id == FAKE_DESCRIPTOR.provider_id
    assert provider.calls == 1


def test_runtime_diagnostics_reads_endpoint_exactly_once() -> None:
    class CountingEndpointProvider(FakeProvider):
        endpoint_reads = 0
        calls = 0

        @property
        def descriptor(self) -> ProviderDescriptor:
            return _descriptor_with_diagnostics()

        @property
        def runtime_diagnostics(self):
            self.endpoint_reads += 1

            def read() -> RuntimeDiagnostics:
                self.calls += 1
                return RuntimeDiagnostics("reference.fake", "reference", 0, 0)

            return read

    provider = CountingEndpointProvider()
    assert read_runtime_diagnostics(provider).connection_mode == "reference"
    assert provider.endpoint_reads == 1
    assert provider.calls == 1


def test_runtime_diagnostics_accepts_legacy_declaration_without_mode_constraint() -> None:
    class UnconstrainedProvider(FakeProvider):
        @property
        def descriptor(self) -> ProviderDescriptor:
            return _descriptor_with_diagnostics(FrozenMap({"scope": "provider"}))

        def runtime_diagnostics(self) -> RuntimeDiagnostics:
            return RuntimeDiagnostics("reference.fake", "legacy-direct", 0, 0)

    assert read_runtime_diagnostics(UnconstrainedProvider()).connection_mode == "legacy-direct"


def test_runtime_diagnostics_rejects_mode_outside_declared_allow_list() -> None:
    class MismatchedModeProvider(FakeProvider):
        @property
        def descriptor(self) -> ProviderDescriptor:
            return _descriptor_with_diagnostics(FrozenMap({"connection_modes": ["direct", "gui"]}))

        def runtime_diagnostics(self) -> RuntimeDiagnostics:
            return RuntimeDiagnostics("reference.fake", "remote", 0, 0)

    with pytest.raises(ValidationError, match="not declared") as error:
        read_runtime_diagnostics(MismatchedModeProvider())
    assert isinstance(error.value.__cause__, ValueError)
    assert error.value.details["reported_connection_mode"] == "remote"
    assert error.value.details["declared_connection_modes"] == ("direct", "gui")


@pytest.mark.parametrize(
    ("connection_modes", "message", "cause_type"),
    (
        ("direct", "JSON array", TypeError),
        ([], "must not be empty", ValueError),
        (["DIRECT"], "invalid mode", ValueError),
        (["direct", 1], "invalid mode", ValueError),
        (["direct", "direct"], "must be unique", ValueError),
    ),
)
def test_runtime_diagnostics_rejects_invalid_connection_mode_declarations(
    connection_modes: Any,
    message: str,
    cause_type: type[Exception],
) -> None:
    class InvalidModeDeclarationProvider(FakeProvider):
        @property
        def descriptor(self) -> ProviderDescriptor:
            return _descriptor_with_diagnostics(FrozenMap({"connection_modes": connection_modes}))

        def runtime_diagnostics(self) -> RuntimeDiagnostics:
            raise AssertionError("invalid declaration must fail before endpoint execution")

    with pytest.raises(ValidationError, match=message) as error:
        read_runtime_diagnostics(InvalidModeDeclarationProvider())
    assert isinstance(error.value.__cause__, cause_type)
    assert error.value.details["property"] == "connection_modes"
