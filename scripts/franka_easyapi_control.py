"""One backend-branch-free EasyAPI Franka control conformance program."""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

from unirobosim import AssetBundle, Sim

JOINT_NAMES = (
    "panda_joint1",
    "panda_joint2",
    "panda_joint3",
    "panda_joint4",
    "panda_joint5",
    "panda_joint6",
    "panda_joint7",
    "robotiq_85_left_knuckle_joint",
    "robotiq_85_right_knuckle_joint",
    "robotiq_85_left_inner_knuckle_joint",
    "robotiq_85_right_inner_knuckle_joint",
    "robotiq_85_left_finger_tip_joint",
    "robotiq_85_right_finger_tip_joint",
)
INITIAL = (0.0, -0.569, 0.0, -2.81, 0.0, 3.037, 0.741, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
FINAL = (0.25, -0.45, 0.12, -2.65, 0.08, 2.9, 0.65, 0.2, -0.2, 0.2, -0.2, -0.2, 0.2)


def _smoothstep(value: float) -> float:
    return value * value * (3.0 - 2.0 * value)


def run(backend: str, manifest: Path, output: Path, *, steps: int = 480) -> dict[str, object]:
    started = time.time()
    bundle = AssetBundle.from_manifest(manifest)
    sim = Sim(backend=backend, world_id="franka-easyapi-control", time_step_seconds=1.0 / 240.0)
    robot = sim.add_articulation(
        "franka",
        joint_names=JOINT_NAMES,
        initial_positions=INITIAL,
        asset=bundle,
    )
    trace: list[list[float]] = []
    targets: list[list[float]] = []
    try:
        sim.start()
        initial = tuple(float(value) for value in robot.state.joint_positions.rows()[0])
        for step in range(steps):
            phase = _smoothstep((step + 1) / steps)
            target = tuple(start + (end - start) * phase for start, end in zip(INITIAL, FINAL, strict=True))
            robot.command(target)
            sim.step()
            state = tuple(float(value) for value in robot.state.joint_positions.rows()[0])
            if not all(math.isfinite(value) for value in state):
                raise RuntimeError(f"non-finite joint state at step {step}")
            targets.append(list(target))
            trace.append(list(state))
        final = tuple(trace[-1])
        result: dict[str, object] = {
            "status": "passed",
            "backend_argument": backend,
            "provider_id": sim.provider_descriptor.provider_id,
            "asset_uri": sim.world_spec.entities[0].asset_uri,
            "asset_metadata": sim.world_spec.entities[0].metadata.to_dict()["unirobosim_asset"],
            "joint_names": list(JOINT_NAMES),
            "steps": steps,
            "time_step_seconds": 1.0 / 240.0,
            "initial_joint_positions": list(initial),
            "final_joint_positions": list(final),
            "final_targets": list(FINAL),
            "final_max_abs_error_rad": max(abs(actual - target) for actual, target in zip(final, FINAL, strict=True)),
            "duration_wall_seconds": time.time() - started,
            "targets": targets,
            "joint_positions": trace,
        }
    finally:
        sim.close()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--backend", required=True, choices=("isaaclab", "mujoco", "pybullet"))
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--steps", type=int, default=480)
    args = parser.parse_args()
    result = run(args.backend, args.manifest.resolve(), args.output.resolve(), steps=args.steps)
    print(
        json.dumps({key: value for key, value in result.items() if key not in {"targets", "joint_positions"}}, indent=2)
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
