"""Resolve a portable handle, submit a typed command, and read typed state."""

from unirobosim_teaching import TeachingProvider

from unirobosim import (
    ArrayValue,
    ArticulationCommand,
    CommandMode,
    EntityKind,
    EntityPath,
    EntitySpec,
    EnvironmentSpec,
    WorldSpec,
)


def main() -> None:
    spec = WorldSpec(
        "world-control-demo",
        (
            EntitySpec(
                EntityPath("/cabinet"),
                EntityKind.ARTICULATION,
                joint_names=("door_hinge", "drawer_slide"),
                initial_joint_positions=(0.0, 0.1),
            ),
        ),
        environments=EnvironmentSpec(2),
    )
    session = TeachingProvider().open()
    try:
        world = session.build(spec)
        handle = world.resolve(EntityPath("/cabinet"))
        world.apply_articulation_command(
            ArticulationCommand(
                handle,
                CommandMode.POSITION,
                ArrayValue.from_rows(((0.65,),)),
                environment_indices=(1,),
                degree_of_freedom_indices=(0,),
                target_units=("rad",),
            )
        )
        world.step()
        state = world.read_articulation(handle)
        assert state.joint_positions.rows() == ((0.0, 0.1), (0.65, 0.1))
        print(f"joint_positions={state.joint_positions.rows()}")
        print(f"tick={state.tick.step_index}")
        world.close()
    finally:
        session.close()


if __name__ == "__main__":
    main()
