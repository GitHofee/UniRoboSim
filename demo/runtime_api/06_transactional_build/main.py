from __future__ import annotations

from unirobosim import (
    EntityKind,
    EntityPath,
    EntitySpec,
    SessionState,
    StaleHandleError,
    WorldBuildError,
    WorldSpec,
)
from unirobosim.testing import FakeProvider


def main() -> None:
    spec = WorldSpec(
        world_id="runtime-transaction",
        entities=(
            EntitySpec(
                EntityPath("/arm"),
                EntityKind.ARTICULATION,
                joint_names=("joint",),
            ),
        ),
    )
    session = FakeProvider(build_failures=1).open()
    try:
        try:
            session.build(spec)
        except WorldBuildError as error:
            assert session.state is SessionState.OPEN
            print(f"first_build=failed operation={error.operation} session={session.state.value}")
        else:
            raise AssertionError("the injected first build must fail")

        first_world = session.build(spec)
        old_handle = first_world.resolve(EntityPath("/arm"))
        assert first_world.generation == 1
        first_world.close()

        second_world = session.build(spec)
        try:
            assert second_world.generation == 2
            try:
                second_world.read_articulation(old_handle)
            except StaleHandleError as error:
                assert error.details["expected_generation"] == 2
                assert error.details["actual_generation"] == 1
                print("retry=passed old_handle=stale generation=1->2")
            else:
                raise AssertionError("a handle from an older World generation must be rejected")
        finally:
            second_world.close()
    finally:
        session.close()


if __name__ == "__main__":
    main()
