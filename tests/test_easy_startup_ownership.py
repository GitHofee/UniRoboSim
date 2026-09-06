"""Independent public Easy lifecycle ownership gates; no native backend imports.

TemporaryFile is a real owned resource. Faults are explicit test injections;
the finally rescue is never counted as product cleanup. Interrupted builds
raise a BaseException *after* the fake session has allocated its real fake world.
"""

from __future__ import annotations

import tempfile
from dataclasses import dataclass

import pytest

from unirobosim import LifecycleError, Sim, SimState, WorldBuildError
from unirobosim.testing import FakeProvider
from unirobosim.testing.fake_backend import FakeSession


class BuildInterrupted(BaseException):
    """Controlled stand-in for cancellation, not an actual process interrupt."""


class ReleaseFailure(OSError):
    pass


class ReleaseInterrupted(BaseException):
    pass


@dataclass(frozen=True)
class Plan:
    build: str = "success"
    release: type[BaseException] | None = None
    release_failures: int = 0
    partial_release: bool = False


class OwnedSession(FakeSession):
    def __init__(self, descriptor, plan):
        super().__init__(descriptor, build_failures=int(plan.build == "build_error"))
        self.plan = plan
        self.resource = tempfile.TemporaryFile()
        self.close_calls = 0
        self.build_error = None

    def build(self, spec, *, build_input=None):
        try:
            world = super().build(spec, build_input=build_input)
            if self.plan.build == "interrupted":
                raise BuildInterrupted("injected interruption after world allocation")
            return world
        except BaseException as error:
            self.build_error = error
            raise

    def close(self):
        self.close_calls += 1
        if self.plan.partial_release:
            super().close()
        if self.close_calls <= self.plan.release_failures:
            assert self.plan.release is not None
            raise self.plan.release("injected release failure")
        super().close()
        self.resource.close()


class OwnedProvider(FakeProvider):
    def __init__(self, *plans):
        super().__init__()
        self.plans = plans
        self.sessions = []

    def open(self):
        assert len(self.sessions) < len(self.plans), "unexpected provider.open call"
        session = OwnedSession(self.descriptor, self.plans[len(self.sessions)])
        self.sessions.append(session)
        return session

    def rescue(self):
        # Bypass fault injection only in test-finally; not public-API evidence.
        for session in self.sessions:
            FakeSession.close(session)
            session.resource.close()


def make_sim(provider):
    sim = Sim(provider=provider)
    sim.add_box("box")
    return sim


def observe_start_failure(sim, provider):
    with pytest.raises((WorldBuildError, BuildInterrupted, ReleaseFailure, ReleaseInterrupted)) as captured:
        sim.start()
    session = provider.sessions[0]
    error = captured.value
    print({"state": sim.state.value, "top_error": type(error).__name__,
           "context": type(error.__context__).__name__ if error.__context__ else None,
           "close_calls": session.close_calls, "resource_closed": session.resource.closed})
    return error


@pytest.mark.parametrize("build", ["build_error", "interrupted"])
def test_clean_rollback_retains_original_error_and_allows_edit_and_start_retry(build):
    provider = OwnedProvider(Plan(build), Plan())
    sim = make_sim(provider)
    try:
        error = observe_start_failure(sim, provider)
        original = provider.sessions[0]
        assert error is original.build_error
        assert original.resource.closed, "clean rollback must release the opened session"
        assert original.close_calls == 1
        assert sim.state is SimState.CONFIGURING
        sim.add_box("after_failure")
        sim.start()
        assert sim.state is SimState.RUNNING
        assert len(provider.sessions) == 2
        assert sim.build_report.entity_count == 2
        sim.close()
        sim.close()
        assert all(session.resource.closed for session in provider.sessions)
        assert sim.state is SimState.CLOSED
    finally:
        provider.rescue()


@pytest.mark.parametrize("build", ["build_error", "interrupted"])
@pytest.mark.parametrize("release", [ReleaseFailure, ReleaseInterrupted])
def test_failed_rollback_blocks_restart_without_opening_or_overwriting_session(build, release):
    provider = OwnedProvider(Plan(build, release, 1), Plan())
    sim = make_sim(provider)
    try:
        observe_start_failure(sim, provider)
        original = provider.sessions[0]
        assert not original.resource.closed
        assert sim.state not in (SimState.CLOSED, SimState.RUNNING)
        try:
            with pytest.raises(LifecycleError):
                sim.start()
        finally:
            print({"after_restart_state": sim.state.value, "opened_sessions": len(provider.sessions),
                   "old_resource_closed": original.resource.closed,
                   "newest_resource_closed": provider.sessions[-1].resource.closed})
        assert len(provider.sessions) == 1, "retry must not acquire a replacement while cleanup is unresolved"
        with pytest.raises(LifecycleError):
            sim.add_box("must_not_edit_pending_cleanup")
        with pytest.raises(LifecycleError):
            _ = sim.world
        sim.close()
        assert original.resource.closed
        assert sim.state is SimState.CLOSED
    finally:
        provider.rescue()


@pytest.mark.parametrize("build", ["build_error", "interrupted"])
@pytest.mark.parametrize("release", [ReleaseFailure, ReleaseInterrupted])
@pytest.mark.parametrize("partial_release", [False, True])
def test_public_close_retries_same_resource_and_does_not_false_close(build, release, partial_release):
    provider = OwnedProvider(Plan(build, release, 2, partial_release))
    sim = make_sim(provider)
    try:
        error = observe_start_failure(sim, provider)
        session = provider.sessions[0]
        # Both faults remain inspectable; cleanup error is the top-level error.
        assert type(error) is release
        assert error.__context__ is session.build_error
        assert not session.resource.closed
        with pytest.raises(release):
            sim.close()
        assert sim.state is not SimState.CLOSED
        assert not session.resource.closed
        assert session.close_calls == 2
        sim.close()
        sim.close()
        assert session.resource.closed
        assert session.close_calls == 3
        assert len(provider.sessions) == 1
        assert sim.state is SimState.CLOSED
    finally:
        provider.rescue()


@pytest.mark.parametrize("release", [None, ReleaseFailure, ReleaseInterrupted])
def test_successful_build_normal_close_and_transient_close_failure_are_unchanged(release):
    provider = OwnedProvider(Plan(release=release, release_failures=int(release is not None)))
    sim = make_sim(provider)
    try:
        sim.start()
        assert sim.state is SimState.RUNNING
        sim.step()
        if release is not None:
            with pytest.raises(release):
                sim.close()
            assert sim.state is not SimState.CLOSED
            assert not provider.sessions[0].resource.closed
        sim.close()
        sim.close()
        assert provider.sessions[0].resource.closed
        assert provider.sessions[0].close_calls == (1 if release is None else 2)
        assert sim.state is SimState.CLOSED
    finally:
        provider.rescue()


@pytest.mark.parametrize("build", ["build_error", "interrupted"])
def test_context_exit_retries_failed_start_rollback(build):
    provider = OwnedProvider(Plan(build, ReleaseFailure, 1))
    sim = make_sim(provider)
    try:
        with pytest.raises(ReleaseFailure):
            with sim:
                sim.start()
        assert provider.sessions[0].resource.closed
        assert provider.sessions[0].close_calls == 2
        assert sim.state is SimState.CLOSED
    finally:
        provider.rescue()
