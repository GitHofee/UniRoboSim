from __future__ import annotations

from unirobosim import (
    EntityKind,
    EntityPath,
    EntitySpec,
    Provider,
    Session,
    SessionState,
    World,
    WorldSpec,
    WorldState,
)
from unirobosim.testing import FakeProvider


def main() -> None:
    provider = FakeProvider()
    assert isinstance(provider, Provider)

    probe = provider.probe()
    assert probe.available
    print(f"provider={probe.descriptor.provider_id} available={probe.available}")

    session = provider.open()
    assert isinstance(session, Session)
    try:
        spec = WorldSpec(
            world_id="runtime-lifecycle",
            entities=(
                EntitySpec(
                    path=EntityPath("/cabinet"),
                    kind=EntityKind.ARTICULATION,
                    joint_names=("door_hinge",),
                ),
            ),
        )
        world = session.build(spec)
        assert isinstance(world, World)
        try:
            assert session.state is SessionState.READY
            assert world.state is WorldState.READY
            assert world.build_report.world_id == spec.world_id
            print(f"world={world.world_id} generation={world.generation} entities={world.build_report.entity_count}")
        finally:
            world.close()

        assert session.state is SessionState.OPEN
    finally:
        session.close()

    assert session.state is SessionState.CLOSED
    print("lifecycle=closed-cleanly")


if __name__ == "__main__":
    main()
