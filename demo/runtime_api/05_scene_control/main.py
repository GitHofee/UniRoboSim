from __future__ import annotations

from unirobosim import (
    CapabilityId,
    CapabilityRequirement,
    EntityKind,
    EntityPath,
    EntitySpec,
    Pose,
    SceneCommand,
    SceneCommandKind,
    SceneCommandStatus,
    SceneControlWorld,
    WorldSpec,
)
from unirobosim.testing import FakeProvider


def main() -> None:
    requirements = tuple(
        CapabilityRequirement(CapabilityId(capability))
        for capability in (
            "scene.snapshot@1",
            "scene.delta@1",
            "scene.command.pose@1",
        )
    )

    session = FakeProvider().open()
    try:
        negotiation = session.negotiate(requirements)
        if not negotiation.accepted:
            raise RuntimeError(f"scene control is unavailable: {negotiation.to_dict()}")

        spec = WorldSpec(
            world_id="runtime-scene-control",
            entities=(EntitySpec(EntityPath("/box"), EntityKind.RIGID_BODY),),
            requirements=requirements,
        )
        world = session.build(spec)
        try:
            if not isinstance(world, SceneControlWorld):
                raise RuntimeError("provider declared scene control without implementing SceneControlWorld")

            initial = world.scene_snapshot()
            target = Pose(position=(1.0, -2.0, 0.5))
            command = SceneCommand(
                command_id="pose-1",
                client_id="framework-demo",
                lease_id="lease-1",
                expected_generation=world.generation,
                kind=SceneCommandKind.SET_POSE,
                entity_path=EntityPath("/box"),
                environment_index=0,
                target_pose=target,
            )
            applied = world.apply_scene_command(command)
            duplicate = world.apply_scene_command(command)
            delta = world.scene_delta(initial.sequence)
            current = world.scene_snapshot()

            assert applied.status is SceneCommandStatus.APPLIED
            assert duplicate.status is SceneCommandStatus.DUPLICATE
            assert delta.base_sequence == initial.sequence
            assert delta.sequence == current.sequence
            assert current.entities[0].pose == target
            print(
                f"capabilities=accepted initial={initial.sequence} current={current.sequence} "
                f"duplicate={duplicate.status.value}"
            )
        finally:
            world.close()
    finally:
        session.close()


if __name__ == "__main__":
    main()
