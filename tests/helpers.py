from __future__ import annotations

from unirobosim import EntityKind, EntityPath, EntitySpec, EnvironmentSpec, PhysicsSpec, WorldSpec


def make_world_spec(
    *,
    world_id: str = "test-world",
    environments: int = 2,
    initial: tuple[float, ...] = (0.1, -0.2),
    requirements=None,
) -> WorldSpec:
    kwargs = {}
    if requirements is not None:
        kwargs["requirements"] = tuple(requirements)
    return WorldSpec(
        world_id=world_id,
        environments=EnvironmentSpec(environments),
        physics=PhysicsSpec(time_step_seconds=0.01),
        entities=(
            EntitySpec(path=EntityPath("/ground"), kind=EntityKind.RIGID_BODY),
            EntitySpec(
                path=EntityPath("/robot"),
                kind=EntityKind.ARTICULATION,
                joint_names=tuple(f"joint_{index}" for index in range(len(initial))),
                initial_joint_positions=initial,
            ),
        ),
        **kwargs,
    )
