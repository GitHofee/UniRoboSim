"""Exercise transactional build and the explicit Session lifecycle."""

from unirobosim_teaching import TeachingProvider

from unirobosim import EntityKind, EntityPath, EntitySpec, SessionState, WorldBuildError, WorldSpec


def world_spec() -> WorldSpec:
    return WorldSpec(
        "session-demo",
        (EntitySpec(EntityPath("/arm"), EntityKind.ARTICULATION, joint_names=("joint",)),),
    )


def main() -> None:
    session = TeachingProvider(build_failures=1).open()
    try:
        try:
            session.build(world_spec())
        except WorldBuildError as error:
            print(f"first_build={error.code}")
        else:
            raise AssertionError("the injected first build must fail")

        assert session.state is SessionState.OPEN
        world = session.build(world_spec())
        assert session.state is SessionState.READY
        print(f"second_build_generation={world.generation}")
        world.close()
        assert session.state is SessionState.OPEN
    finally:
        session.close()
    assert session.state is SessionState.CLOSED


if __name__ == "__main__":
    main()
