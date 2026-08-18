from __future__ import annotations

import unittest

from unirobosim import CapabilityId, CapabilityRequirement, ProviderRegistrationError, ProviderSelectionError
from unirobosim.runtime import ProviderRegistry
from unirobosim.testing import FAKE_DESCRIPTOR, FakeProvider


class ProviderRegistryTests(unittest.TestCase):
    def test_registration_is_lazy_and_descriptors_are_sorted(self) -> None:
        calls = []

        def factory():
            calls.append(True)
            return FakeProvider()

        registry = ProviderRegistry()
        registry.register(FAKE_DESCRIPTOR, factory)
        self.assertEqual(calls, [])
        self.assertEqual(registry.descriptors(), (FAKE_DESCRIPTOR,))
        self.assertIsInstance(registry.create("reference.fake"), FakeProvider)
        self.assertEqual(calls, [True])

    def test_duplicate_and_missing_registration_fail_explicitly(self) -> None:
        registry = ProviderRegistry()
        registry.register(FAKE_DESCRIPTOR, FakeProvider)
        with self.assertRaises(ProviderRegistrationError):
            registry.register(FAKE_DESCRIPTOR, FakeProvider)
        registry.unregister("reference.fake")
        with self.assertRaises(ProviderRegistrationError):
            registry.unregister("reference.fake")
        with self.assertRaises(ProviderSelectionError):
            registry.create("reference.fake")

    def test_selection_uses_availability_and_capabilities(self) -> None:
        registry = ProviderRegistry()
        registry.register(FAKE_DESCRIPTOR, FakeProvider)
        provider = registry.select((CapabilityRequirement(CapabilityId("state.articulation@1")),))
        self.assertEqual(provider.descriptor.provider_id, "reference.fake")
        with self.assertRaises(ProviderSelectionError) as caught:
            registry.select((CapabilityRequirement(CapabilityId("sensor.camera@1")),))
        self.assertTrue(caught.exception.details["attempts"])

    def test_unavailable_provider_is_not_selected(self) -> None:
        registry = ProviderRegistry()
        registry.register(FAKE_DESCRIPTOR, lambda: FakeProvider(available=False))
        with self.assertRaises(ProviderSelectionError):
            registry.select(provider_id="reference.fake")


if __name__ == "__main__":
    unittest.main()
