from __future__ import annotations

from unirobosim import (
    CapabilityId,
    CapabilityRequirement,
    ProviderRegistry,
    ProviderSelectionError,
)
from unirobosim.testing import FAKE_DESCRIPTOR, FakeProvider


def main() -> None:
    factory_calls = 0

    def create_fake() -> FakeProvider:
        nonlocal factory_calls
        factory_calls += 1
        return FakeProvider()

    registry = ProviderRegistry()
    registry.register(FAKE_DESCRIPTOR, create_fake)

    assert factory_calls == 0
    assert registry.descriptors() == (FAKE_DESCRIPTOR,)

    requirement = CapabilityRequirement(CapabilityId("state.articulation@1"))
    provider = registry.select((requirement,))
    assert provider.descriptor.provider_id == "reference.fake"
    assert factory_calls == 1
    print(f"selected={provider.descriptor.provider_id} factory_calls={factory_calls}")

    unsupported = CapabilityRequirement(CapabilityId("sensor.lidar@1"))
    try:
        registry.select((unsupported,))
    except ProviderSelectionError as error:
        attempts = error.details["attempts"]
        assert attempts
        assert not attempts[0]["negotiation"]["accepted"]
        print(f"rejected=sensor.lidar@1 attempts={len(attempts)}")
    else:
        raise AssertionError("an unsupported required capability must reject selection")


if __name__ == "__main__":
    main()
