"""Publish a portable marker and inspect the scene snapshot."""

from unirobosim import (
    ArrayValue,
    DebugLifetime,
    DebugPrimitive,
    DebugPrimitiveKind,
    EntityPath,
    Sim,
)
from unirobosim.testing import FakeProvider


def main() -> None:
    with Sim(provider=FakeProvider(), world_id="debug-and-scene") as sim:
        sim.add_box("box", size_m=(0.2, 0.3, 0.4))
        sim.start()

        marker = DebugPrimitive(
            primitive_id="goal",
            layer="tutorial",
            kind=DebugPrimitiveKind.POINT_SET,
            geometry_m=ArrayValue.from_nested([[[0.0, 0.0, 1.0]]]),
            environment_indices=(0,),
            color_rgba=(1.0, 0.2, 0.1, 1.0),
            lifetime=DebugLifetime.manual(),
        )
        publish_report = sim.debug.publish(marker)
        snapshot = sim.scene_snapshot()
        box = next(entity for entity in snapshot.entities if entity.path == EntityPath("/box"))
        cleared = sim.debug.clear(layer="tutorial")

        assert publish_report.accepted_count == 1
        assert box.visuals[0].dimensions_m == (0.2, 0.3, 0.4)
        assert cleared == 1
        print(f"scene_sequence={snapshot.sequence}")
        print(f"box_dimensions_m={box.visuals[0].dimensions_m}")
        print(f"debug_primitives_cleared={cleared}")


if __name__ == "__main__":
    main()
