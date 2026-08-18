"""Evaluate three-backend Franka red-cube pick-and-place conformance."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import subprocess
from pathlib import Path
from typing import Any

BACKENDS = ("isaaclab", "mujoco", "pybullet")
ARM_MAX_RAD = 0.055
ARM_RMSE_RAD = 0.02
GRIPPER_MAX_RAD = 0.10
GRIPPER_RMSE_RAD = 0.04
CUBE_MAX_M = 0.045
CUBE_RMSE_M = 0.012
TARGET_MAX_M = 0.03


def _rmse(values: list[float]) -> float:
    return math.sqrt(sum(value * value for value in values) / len(values))


def _video_probe(path: Path) -> dict[str, object]:
    result = subprocess.run(
        (
            "ffprobe",
            "-v",
            "error",
            "-select_streams",
            "v:0",
            "-show_entries",
            "stream=width,height,codec_name,avg_frame_rate,nb_frames",
            "-of",
            "json",
            str(path),
        ),
        check=True,
        capture_output=True,
        text=True,
    )
    stream = json.loads(result.stdout)["streams"][0]
    return {
        "path": str(path.resolve()),
        "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "width": int(stream["width"]),
        "height": int(stream["height"]),
        "codec": stream["codec_name"],
        "average_frame_rate": stream["avg_frame_rate"],
        "frame_count": int(stream["nb_frames"]),
    }


def compare(inputs: dict[str, Path], *, require_video: bool) -> dict[str, object]:
    payloads = {backend: json.loads(path.read_text(encoding="utf-8")) for backend, path in inputs.items()}
    errors: list[str] = []
    videos: dict[str, object] = {}
    for backend in BACKENDS:
        payload = payloads[backend]
        if payload.get("backend_argument") != backend:
            errors.append(f"{backend}: backend_argument does not match")
        if payload.get("cube_primitive") != "box" or payload.get("cube_color_rgba") != [1.0, 0.0, 0.0, 1.0]:
            errors.append(f"{backend}: red EasyAPI box contract is absent")
        if not payload.get("physical_lift_detected") or int(payload.get("contact_sample_count", 0)) <= 0:
            errors.append(f"{backend}: contact/lift evidence is absent")
        if float(payload.get("cube_final_target_error_m", math.inf)) > TARGET_MAX_M:
            errors.append(f"{backend}: final target error exceeds {TARGET_MAX_M} m")
        if require_video:
            video_value = payload.get("video")
            red_range = payload.get("native_red_sample_count_range")
            red_positive_frames = payload.get("native_red_positive_frame_count")
            if (
                not video_value
                or not red_range
                or int(red_range[1]) <= 0
                or (red_positive_frames is not None and int(red_positive_frames) <= 0)
            ):
                errors.append(f"{backend}: native red-pixel video evidence is absent")
                continue
            video_path = Path(video_value)
            if not video_path.is_file():
                errors.append(f"{backend}: video file is missing")
                continue
            probe = _video_probe(video_path)
            if (probe["width"], probe["height"]) != (1920, 1080):
                errors.append(f"{backend}: video is not 1920x1080")
            if probe["sha256"] != payload.get("video_sha256"):
                errors.append(f"{backend}: video SHA-256 does not match telemetry")
            probe["native_red_sample_count_range"] = red_range
            probe["native_red_positive_frame_count"] = red_positive_frames
            videos[backend] = probe

    pairwise: dict[str, object] = {}
    for left_index, left in enumerate(BACKENDS):
        for right in BACKENDS[left_index + 1 :]:
            left_samples = payloads[left]["samples"]
            right_samples = payloads[right]["samples"]
            if len(left_samples) != len(right_samples):
                errors.append(f"{left}-{right}: sample count differs")
                continue
            arm_errors: list[float] = []
            gripper_errors: list[float] = []
            cube_component_errors: list[float] = []
            cube_distances: list[float] = []
            stage_distances: dict[str, list[float]] = {}
            for a, b in zip(left_samples, right_samples, strict=True):
                if (a["step"], a["stage"]) != (b["step"], b["stage"]):
                    errors.append(f"{left}-{right}: sample timeline differs")
                    break
                joint_a = a["joint_positions"]
                joint_b = b["joint_positions"]
                arm_errors.extend(abs(x - y) for x, y in zip(joint_a[:7], joint_b[:7], strict=True))
                gripper_errors.extend(abs(x - y) for x, y in zip(joint_a[7:], joint_b[7:], strict=True))
                differences = [
                    x - y for x, y in zip(a["cube_position_m"], b["cube_position_m"], strict=True)
                ]
                cube_component_errors.extend(differences)
                distance = math.sqrt(sum(value * value for value in differences))
                cube_distances.append(distance)
                stage_distances.setdefault(a["stage"], []).append(distance)
            metrics: dict[str, Any] = {
                "arm_max_abs_rad": max(arm_errors),
                "arm_rmse_rad": _rmse(arm_errors),
                "gripper_max_abs_rad": max(gripper_errors),
                "gripper_rmse_rad": _rmse(gripper_errors),
                "cube_position_max_distance_m": max(cube_distances),
                "cube_position_component_rmse_m": _rmse(cube_component_errors),
                "cube_position_by_stage": {
                    stage: {"max_distance_m": max(values), "mean_distance_m": sum(values) / len(values)}
                    for stage, values in stage_distances.items()
                },
            }
            pairwise[f"{left}-{right}"] = metrics
            limits = (
                ("arm_max_abs_rad", ARM_MAX_RAD),
                ("arm_rmse_rad", ARM_RMSE_RAD),
                ("gripper_max_abs_rad", GRIPPER_MAX_RAD),
                ("gripper_rmse_rad", GRIPPER_RMSE_RAD),
                ("cube_position_max_distance_m", CUBE_MAX_M),
                ("cube_position_component_rmse_m", CUBE_RMSE_M),
            )
            for metric, limit in limits:
                if metrics[metric] > limit:
                    errors.append(f"{left}-{right}: {metric}={metrics[metric]:.9f} exceeds {limit}")

    return {
        "schema": "unirobosim.acceptance/franka-red-cube-pick-place-v1",
        "status": "passed" if not errors else "failed",
        "inputs": {backend: str(path.resolve()) for backend, path in inputs.items()},
        "thresholds": {
            "arm_max_abs_rad": ARM_MAX_RAD,
            "arm_rmse_rad": ARM_RMSE_RAD,
            "gripper_max_abs_rad": GRIPPER_MAX_RAD,
            "gripper_rmse_rad": GRIPPER_RMSE_RAD,
            "cube_position_max_distance_m": CUBE_MAX_M,
            "cube_position_component_rmse_m": CUBE_RMSE_M,
            "final_target_error_m": TARGET_MAX_M,
        },
        "pairwise": pairwise,
        "videos": videos,
        "errors": errors,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    for backend in BACKENDS:
        parser.add_argument(f"--{backend}", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--require-video", action="store_true")
    args = parser.parse_args()
    inputs = {backend: getattr(args, backend).resolve() for backend in BACKENDS}
    result = compare(inputs, require_video=args.require_video)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
