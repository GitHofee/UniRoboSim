from __future__ import annotations

import math

from unirobosim import (
    ArrayValue,
    BoxGeometrySpec,
    CameraModality,
    CameraSpec,
    EntityKind,
    EntityPath,
    EntitySpec,
    EnvironmentSpec,
    PhysicsSpec,
    Pose,
    RigidBodyCommand,
    WorldSpec,
)
from unirobosim.testing import FakeProvider


def main() -> None:
    session = FakeProvider().open()
    try:
        spec = WorldSpec(
            world_id="runtime-sensors",
            environments=EnvironmentSpec(count=2),
            physics=PhysicsSpec(time_step_seconds=0.1, gravity_m_s2=(0.0, 0.0, 0.0)),
            entities=(
                EntitySpec(
                    path=EntityPath("/box"),
                    kind=EntityKind.RIGID_BODY,
                    pose=Pose(position=(0.0, 0.0, 1.0)),
                    box=BoxGeometrySpec(dimensions_m=(0.2, 0.3, 0.4), mass_kg=2.0),
                ),
                EntitySpec(
                    path=EntityPath("/camera"),
                    kind=EntityKind.CAMERA_SENSOR,
                    camera=CameraSpec(
                        width_px=8,
                        height_px=6,
                        modalities=(CameraModality.RGB, CameraModality.DEPTH),
                    ),
                ),
            ),
        )
        world = session.build(spec)
        try:
            box = world.resolve(EntityPath("/box"))
            camera = world.resolve(EntityPath("/camera"))
            world.apply_rigid_body_command(
                RigidBodyCommand(
                    handle=box,
                    forces_n=ArrayValue.from_rows(((2.0, 0.0, 0.0), (2.0, 0.0, 0.0))),
                    torques_n_m=ArrayValue.from_rows(((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))),
                )
            )
            world.step()

            rigid_state = world.read_rigid_body(box)
            assert rigid_state.positions_m.shape == (2, 3)
            assert math.isclose(rigid_state.positions_m.rows()[0][0], 0.01)

            sample = world.read_sensor(camera)
            rgb = sample.channel(CameraModality.RGB)
            depth = sample.channel(CameraModality.DEPTH)
            assert rgb.shape == (2, 6, 8, 3)
            assert rgb.dtype == "uint8" and rgb.is_packed
            assert len(rgb.to_bytes()) == 2 * 6 * 8 * 3
            assert depth.shape == (2, 6, 8) and depth.dtype == "float32"
            print(
                f"box_x={rigid_state.positions_m.rows()[0][0]:.3f} "
                f"rgb={rgb.shape}/{rgb.dtype} depth={depth.shape}/{depth.dtype}"
            )
        finally:
            world.close()
    finally:
        session.close()


if __name__ == "__main__":
    main()
