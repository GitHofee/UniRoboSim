from __future__ import annotations

import unittest

from tests.helpers import make_world_spec
from unirobosim import (
    CapabilityId,
    CapabilityNegotiationError,
    CapabilityRequirement,
    EntityNotFoundError,
    EntityPath,
    LifecycleError,
    Provider,
    ProviderSelectionError,
    Session,
    SessionState,
    World,
    WorldBuildError,
    WorldState,
)
from unirobosim.testing import FakeProvider


class FakeLifecycleTests(unittest.TestCase):
    def test_provider_protocol_and_probe(self) -> None:
        provider = FakeProvider()
        self.assertIsInstance(provider, Provider)
        report = provider.probe()
        self.assertTrue(report.available)
        self.assertEqual(report.descriptor.provider_id, "reference.fake")

    def test_unavailable_provider_rejects_open(self) -> None:
        provider = FakeProvider(available=False)
        self.assertFalse(provider.probe().available)
        with self.assertRaises(ProviderSelectionError):
            provider.open()

    def test_build_failure_is_transactional_and_retryable(self) -> None:
        session = FakeProvider(build_failures=1).open()
        spec = make_world_spec()
        with self.assertRaises(WorldBuildError):
            session.build(spec)
        self.assertEqual(session.state, SessionState.OPEN)
        world = session.build(spec)
        self.assertEqual(world.generation, 1)
        self.assertEqual(session.state, SessionState.READY)
        session.close()

    def test_negotiation_failure_does_not_poison_session(self) -> None:
        session = FakeProvider().open()
        unsupported = make_world_spec(requirements=(CapabilityRequirement(CapabilityId("sensor.camera@1")),))
        with self.assertRaises(CapabilityNegotiationError) as caught:
            session.build(unsupported)
        self.assertFalse(caught.exception.details["negotiation"]["accepted"])
        self.assertEqual(session.state, SessionState.OPEN)
        world = session.build(make_world_spec())
        self.assertEqual(world.generation, 1)
        session.close()

    def test_only_one_live_world_per_session(self) -> None:
        session = FakeProvider().open()
        world = session.build(make_world_spec())
        with self.assertRaises(LifecycleError):
            session.build(make_world_spec(world_id="second"))
        world.close()
        self.assertEqual(session.state, SessionState.OPEN)
        rebuilt = session.build(make_world_spec(world_id="second"))
        self.assertEqual(rebuilt.generation, 2)
        session.close()

    def test_close_is_idempotent_and_cascades(self) -> None:
        session = FakeProvider().open()
        world = session.build(make_world_spec())
        self.assertIsInstance(session, Session)
        self.assertIsInstance(world, World)
        session.close()
        session.close()
        world.close()
        self.assertEqual(session.state, SessionState.CLOSED)
        self.assertEqual(world.state, WorldState.CLOSED)
        with self.assertRaises(LifecycleError):
            world.step()
        with self.assertRaises(LifecycleError):
            session.negotiate(())
        with self.assertRaises(LifecycleError):
            session.build(make_world_spec())

    def test_world_context_returns_session_to_open(self) -> None:
        session = FakeProvider().open()
        with session.build(make_world_spec()) as world:
            self.assertEqual(world.state, WorldState.READY)
        self.assertEqual(session.state, SessionState.OPEN)
        session.close()

    def test_resolve_missing_entity_is_structured(self) -> None:
        session = FakeProvider().open()
        world = session.build(make_world_spec())
        with self.assertRaises(EntityNotFoundError) as caught:
            world.resolve(EntityPath("/missing"))
        self.assertEqual(caught.exception.entity_path, "/missing")
        session.close()

    def test_multiple_sessions_are_independent(self) -> None:
        provider = FakeProvider()
        first = provider.open()
        second = provider.open()
        first_world = first.build(make_world_spec(world_id="same"))
        second_world = second.build(make_world_spec(world_id="same"))
        self.assertNotEqual(first.session_id, second.session_id)
        self.assertEqual(first_world.generation, second_world.generation)
        first.close()
        self.assertEqual(second.state, SessionState.READY)
        second_world.step()
        second.close()


if __name__ == "__main__":
    unittest.main()
