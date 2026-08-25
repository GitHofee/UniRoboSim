from __future__ import annotations

from unirobosim import (
    PHYSICAL_WORLD_SCHEMA_VERSION,
    ArrayValue,
    ArticulationCommand,
    CommandMode,
    EntityKind,
    EntityPath,
    EntitySpec,
    EnvironmentSpec,
    PhysicsSpec,
    WorldSpec,
)
from unirobosim.testing import FakeProvider


def main() -> None:
    session = FakeProvider().open()
    try:
        spec = WorldSpec(
            world_id="runtime-articulation",
            schema_version=PHYSICAL_WORLD_SCHEMA_VERSION,
            environments=EnvironmentSpec(count=2),
            physics=PhysicsSpec(time_step_seconds=0.01, gravity_m_s2=(0.0, 0.0, 0.0)),
            entities=(
                EntitySpec(
                    path=EntityPath("/cabinet"),
                    kind=EntityKind.ARTICULATION,
                    joint_names=("door_hinge", "drawer_slide"),
                    initial_joint_positions=(0.1, -0.2),
                    joint_position_units=("rad", "m"),
                ),
            ),
        )
        world = session.build(spec)
        try:
            cabinet = world.resolve(EntityPath("/cabinet"))
            initial = world.read_articulation(cabinet)
            assert initial.joint_positions.rows() == ((0.1, -0.2), (0.1, -0.2))

            command = ArticulationCommand(
                handle=cabinet,
                mode=CommandMode.POSITION,
                targets=ArrayValue.from_rows(((0.75,),)),
                environment_indices=(1,),
                degree_of_freedom_indices=(0,),
                target_units=("rad",),
            )
            world.apply_articulation_command(command)
            tick = world.step()
            state = world.read_articulation(cabinet)

            assert tick.step_index == 1
            assert state.joint_positions.shape == (2, 2)
            assert state.joint_positions.rows() == ((0.1, -0.2), (0.75, -0.2))
            print(f"tick={tick.step_index} joint_positions={state.joint_positions.rows()}")
        finally:
            world.close()
    finally:
        session.close()


if __name__ == "__main__":
    main()
