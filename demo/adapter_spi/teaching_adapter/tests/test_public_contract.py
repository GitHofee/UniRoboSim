"""Public-contract acceptance tests for the teaching Adapter.

The suite deliberately imports no Core internals and no FakeProvider.  Real Adapter
projects should extend the same semantic checks with native simulator acceptance.
"""

from __future__ import annotations

import pytest
from unirobosim import (
    ArrayValue,
    ArticulationCommand,
    CapabilityId,
    CapabilityNegotiationError,
    CapabilityRequirement,
    CommandError,
    CommandMode,
    EntityKind,
    EntityNotFoundError,
    EntityPath,
    EntitySpec,
    EnvironmentSpec,
    LifecycleError,
    Provider,
    Session,
    SessionState,
    StaleHandleError,
    UnsupportedCapabilityError,
    World,
    WorldBuildError,
    WorldSpec,
    WorldState,
)

from unirobosim_teaching import DESCRIPTOR, TeachingProvider, create_provider


def _spec(*, world_id: str = "teaching-test", environments: int = 2) -> WorldSpec:
    return WorldSpec(
        world_id,
        (
            EntitySpec(
                EntityPath("/arm"),
                EntityKind.ARTICULATION,
                joint_names=("shoulder", "wrist"),
                initial_joint_positions=(0.1, -0.2),
            ),
        ),
        environments=EnvironmentSpec(environments),
    )


def test_provider_shape_and_probe_are_discovery_safe() -> None:
    provider = create_provider()
    assert isinstance(provider, Provider)
    assert provider.descriptor == DESCRIPTOR
    assert provider.open_count == 0
    report = provider.probe()
    assert report.available and report.descriptor == DESCRIPTOR
    assert provider.open_count == 0


def test_session_world_shape_and_lifecycle() -> None:
    provider = TeachingProvider()
    session = provider.open()
    assert isinstance(session, Session)
    assert str(session.state) == SessionState.OPEN.value
    world = session.build(_spec())
    assert isinstance(world, World)
    assert str(session.state) == SessionState.READY.value
    assert str(world.state) == WorldState.READY.value
    world.close()
    assert str(world.state) == WorldState.CLOSED.value
    assert str(session.state) == SessionState.OPEN.value
    session.close()
    session.close()
    assert str(session.state) == SessionState.CLOSED.value


def test_build_failure_is_transactional_and_retryable() -> None:
    session = TeachingProvider(build_failures=1).open()
    with pytest.raises(WorldBuildError, match="injected failure"):
        session.build(_spec())
    assert str(session.state) == SessionState.OPEN.value
    world = session.build(_spec())
    assert world.generation == 1
    session.close()


def test_capability_negotiation_is_honest() -> None:
    session = TeachingProvider().open()
    supported = (CapabilityRequirement(CapabilityId("state.articulation@1")),)
    assert session.negotiate(supported).accepted
    unsupported = (CapabilityRequirement(CapabilityId("sensor.camera.rgb@1")),)
    assert not session.negotiate(unsupported).accepted
    spec = WorldSpec(_spec().world_id, _spec().entities, requirements=unsupported)
    with pytest.raises(CapabilityNegotiationError) as caught:
        session.build(spec)
    assert caught.value.backend_id == DESCRIPTOR.provider_id
    assert str(session.state) == SessionState.OPEN.value
    session.close()


def test_one_live_world_per_session_and_close_cascades() -> None:
    session = TeachingProvider().open()
    world = session.build(_spec())
    with pytest.raises(LifecycleError):
        session.build(_spec(world_id="second"))
    session.close()
    assert str(session.state) == SessionState.CLOSED.value
    assert str(world.state) == WorldState.CLOSED.value
    with pytest.raises(LifecycleError):
        world.step()


def test_resolve_command_step_state_and_partial_reset() -> None:
    session = TeachingProvider().open()
    world = session.build(_spec())
    handle = world.resolve(EntityPath("/arm"))
    with pytest.raises(EntityNotFoundError):
        world.resolve(EntityPath("/missing"))
    world.apply_articulation_command(
        ArticulationCommand(
            handle,
            CommandMode.POSITION,
            ArrayValue.from_rows(((0.7,),)),
            environment_indices=(1,),
            degree_of_freedom_indices=(0,),
            target_units=("rad",),
        )
    )
    world.step()
    assert world.read_articulation(handle).joint_positions.rows() == ((0.1, -0.2), (0.7, -0.2))
    result = world.reset((1,))
    assert result.environment_indices == (1,)
    assert world.read_articulation(handle).joint_positions.rows() == ((0.1, -0.2), (0.1, -0.2))
    session.close()


def test_command_shape_and_unit_errors_are_structured() -> None:
    session = TeachingProvider().open()
    world = session.build(_spec())
    handle = world.resolve(EntityPath("/arm"))
    with pytest.raises(CommandError, match="shape"):
        world.apply_articulation_command(
            ArticulationCommand(
                handle,
                CommandMode.POSITION,
                ArrayValue.from_rows(((0.2, 0.3),)),
                environment_indices=(0, 1),
                degree_of_freedom_indices=(0, 1),
            )
        )
    with pytest.raises(CommandError, match="units"):
        world.apply_articulation_command(
            ArticulationCommand(
                handle,
                CommandMode.POSITION,
                ArrayValue.from_rows(((0.2,),)),
                environment_indices=(0,),
                degree_of_freedom_indices=(0,),
                target_units=("m",),
            )
        )
    session.close()


def test_partial_reset_preserves_pending_work_for_other_environments() -> None:
    session = TeachingProvider().open()
    world = session.build(_spec())
    handle = world.resolve(EntityPath("/arm"))
    world.apply_articulation_command(
        ArticulationCommand(
            handle,
            CommandMode.POSITION,
            ArrayValue.from_rows(((0.3,), (0.8,))),
            environment_indices=(0, 1),
            degree_of_freedom_indices=(0,),
        )
    )
    world.reset((1,))
    world.step()
    assert world.read_articulation(handle).joint_positions.rows() == ((0.3, -0.2), (0.1, -0.2))
    session.close()


def test_rebuild_rejects_stale_handle() -> None:
    session = TeachingProvider().open()
    first = session.build(_spec())
    stale = first.resolve(EntityPath("/arm"))
    first.close()
    second = session.build(_spec(world_id="rebuilt"))
    with pytest.raises(StaleHandleError):
        second.read_articulation(stale)
    session.close()


def test_unsupported_world_endpoint_fails_instead_of_fabricating_state() -> None:
    session = TeachingProvider().open()
    world = session.build(_spec(environments=1))
    articulation_handle = world.resolve(EntityPath("/arm"))
    with pytest.raises(UnsupportedCapabilityError) as caught:
        world.read_sensor(articulation_handle)
    assert caught.value.details["capability"] == "sensor.camera@1"
    assert caught.value.backend_id == DESCRIPTOR.provider_id
    session.close()
