"""Run the same EasyAPI scene on one explicitly selected native backend."""

import argparse
import math

from unirobosim import Sim


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--backend", required=True, choices=("isaaclab", "mujoco", "pybullet"))
    parser.add_argument("--steps", type=int, default=30)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.steps <= 0:
        raise ValueError("--steps must be positive")

    with Sim(backend=args.backend, world_id="native-backend-switch", time_step_seconds=1.0 / 60.0) as sim:
        box = sim.add_box(
            "red_box",
            size_m=0.1,
            mass_kg=0.2,
            color_rgba=(1.0, 0.0, 0.0, 1.0),
            position_m=(0.0, 0.0, 1.0),
        )
        camera = sim.add_camera("camera", resolution=(1280, 720), outputs=("rgb",))
        sim.require("state.rigid_body@1")
        sim.require("sensor.camera.rgb@1")
        sim.start()

        sim.step(args.steps)
        position = box.state.positions_m.rows()[0]
        rgb = camera.read("rgb")
        assert all(math.isfinite(float(value)) for value in position)
        assert rgb.shape == (1, 720, 1280, 3)
        assert rgb.dtype == "uint8"
        print(f"backend_argument={args.backend}")
        print(f"provider={sim.provider_descriptor.provider_id}")
        print(f"box_position_m={position}")
        print(f"rgb_shape={rgb.shape}")


if __name__ == "__main__":
    main()
