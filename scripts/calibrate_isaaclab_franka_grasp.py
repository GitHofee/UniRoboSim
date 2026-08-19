"""Calibrate a shared Franka grasp pose in parallel Isaac Lab environments.

This is a development-only physical probe.  It uses the public EasyAPI scene
control contract to place one otherwise identical red cube in each environment;
the formal acceptance program never moves the cube through scene control.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from franka_easyapi_control import INITIAL, JOINT_NAMES, _smoothstep
from franka_easyapi_pick_place import (
    CUBE_MASS_KG,
    CUBE_SIZE_M,
    GRASP_ARM,
    LIFT_ARM,
    OPEN,
    PREGRASP_ARM,
    _target,
)

from unirobosim import (
    AssetBundle,
    EntityPath,
    Pose,
    SceneCommand,
    SceneCommandKind,
    SceneCommandStatus,
    SceneDragMode,
    Sim,
)


def _scene_command(
    sim: Sim,
    command_id: str,
    kind: SceneCommandKind,
    environment: int,
    *,
    target: Pose | None = None,
) -> SceneCommand:
    return SceneCommand(
        command_id,
        "isaac-grasp-calibration",
        "isaac-grasp-calibration-lease",
        sim.world.generation,
        kind,
        EntityPath("/red_cube"),
        environment,
        target,
        f"calibration-drag-{environment}",
        SceneDragMode.KINEMATIC if kind is SceneCommandKind.DRAG_BEGIN else None,
        (0.5, 0.0, 0.5) if kind is SceneCommandKind.DRAG_BEGIN else None,
    )


def _place_cube(sim: Sim, environment: int, position: tuple[float, float, float]) -> None:
    commands = (
        _scene_command(sim, f"begin-{environment}", SceneCommandKind.DRAG_BEGIN, environment),
        _scene_command(
            sim,
            f"update-{environment}",
            SceneCommandKind.DRAG_UPDATE,
            environment,
            target=Pose(position),
        ),
        _scene_command(sim, f"end-{environment}", SceneCommandKind.DRAG_END, environment),
    )
    for command in commands:
        result = sim.world.apply_scene_command(command)
        if result.status is not SceneCommandStatus.APPLIED:
            raise RuntimeError(f"scene command {command.command_id} failed: {result}")


def run(backend: str, manifest: Path, output: Path, *, xs: tuple[float, ...]) -> dict[str, object]:
    if not xs:
        raise ValueError("at least one x candidate is required")
    close_angles = (0.45, 0.48, 0.50, 0.52, 0.55)
    candidates = tuple(((x, 0.0, CUBE_SIZE_M / 2.0), close_angle) for x in xs for close_angle in close_angles)
    sim = Sim(
        backend=backend,
        world_id="franka-grasp-calibration",
        num_envs=len(candidates),
        time_step_seconds=1.0 / 240.0,
    )
    robot = sim.add_articulation(
        "franka",
        joint_names=JOINT_NAMES,
        initial_positions=INITIAL,
        asset=AssetBundle.from_manifest(manifest),
    )
    cube = sim.add_box(
        "red_cube",
        size_m=(CUBE_SIZE_M,) * 3,
        mass_kg=CUBE_MASS_KG,
        color_rgba=(1.0, 0.0, 0.0, 1.0),
        static_friction=0.5,
        dynamic_friction=0.5,
        restitution=0.0,
        position_m=candidates[0][0],
    )
    max_z = [candidate[0][2] for candidate in candidates]
    try:
        sim.start()
        for environment, (position, _) in enumerate(candidates):
            _place_cube(sim, environment, position)
        current = tuple(tuple(INITIAL) for _ in candidates)
        phases = (
            (120, INITIAL[:7], False),
            (240, PREGRASP_ARM, False),
            (240, GRASP_ARM, False),
            (420, GRASP_ARM, True),
            (120, GRASP_ARM, True),
            (360, LIFT_ARM, True),
        )
        for steps, arm, closed in phases:
            start = current
            target = tuple(
                _target(
                    arm,
                    (
                        (close_angle, -close_angle, close_angle, -close_angle, -close_angle, close_angle)
                        if closed
                        else OPEN
                    ),
                )
                for _, close_angle in candidates
            )
            for step in range(steps):
                phase = _smoothstep((step + 1) / steps)
                command = tuple(
                    tuple(a + (b - a) * phase for a, b in zip(start_row, target_row, strict=True))
                    for start_row, target_row in zip(start, target, strict=True)
                )
                robot.command(command)
                sim.step()
                positions = cube.state.positions_m.rows()
                for environment, position in enumerate(positions):
                    max_z[environment] = max(max_z[environment], float(position[2]))
            current = target
        final_positions = cube.state.positions_m.rows()
        gripper_positions = tuple(row[7:] for row in robot.state.joint_positions.rows())
        contacts = cube.contact(force_threshold_n=1.0e-4)
        rows = [
            {
                "environment": environment,
                "initial_position_m": list(candidate[0]),
                "close_angle_rad": candidate[1],
                "final_position_m": list(final_positions[environment]),
                "max_z_m": max_z[environment],
                "lift_delta_m": max_z[environment] - candidate[0][2],
                "final_gripper_positions": list(gripper_positions[environment]),
                "in_contact_at_end": bool(contacts.in_contact.values[environment]),
            }
            for environment, candidate in enumerate(candidates)
        ]
        rows.sort(key=lambda row: float(row["lift_delta_m"]), reverse=True)
        result: dict[str, object] = {
            "status": "passed",
            "purpose": "development calibration; not a formal acceptance iteration",
            "backend_argument": backend,
            "cube_primitive": "box",
            "cube_color_rgba": [1.0, 0.0, 0.0, 1.0],
            "candidate_count": len(candidates),
            "results_by_lift": rows,
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
    parser.add_argument("--x-values", default="0.47,0.48,0.49,0.50,0.51,0.52")
    args = parser.parse_args()
    xs = tuple(float(value) for value in args.x_values.split(",") if value)
    result = run(args.backend, args.manifest.resolve(), args.output.resolve(), xs=xs)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
