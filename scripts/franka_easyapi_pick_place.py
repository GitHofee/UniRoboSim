"""Physically pick and place a red cube with one backend-branch-free EasyAPI program."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path
from typing import cast

from franka_easyapi_control import INITIAL, JOINT_NAMES, _smoothstep
from franka_easyapi_video import RED, RESOLUTION, _look_at_xyzw, _red_sample_count

from unirobosim import AssetBundle, Sim

OPEN = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
CLOSED = (0.50, -0.50, 0.50, -0.50, -0.50, 0.50)
PREGRASP_ARM = (0.0, 0.133108, 0.0, -2.587089, 0.0, 3.004822, 0.741)
GRASP_ARM = (0.0, 0.364447, 0.0, -2.49188, 0.0, 3.028851, 0.741)
LIFT_ARM = (0.0, -0.149313, 0.0, -2.571791, 0.0, 2.74386, 0.741)
TRANSFER_ARM = (0.328293, -0.34627, 0.326163, -2.701747, 0.052873, 2.569566, 0.741)
PLACE_HOVER_ARM = (0.252562, 0.055486, 0.359978, -2.677783, 0.055968, 2.796788, 0.741)
PLACE_ARM = (0.246196, 0.245104, 0.336757, -2.620698, 0.056396, 2.855853, 0.741)
RETREAT_ARM = (0.215111, -0.126293, 0.422091, -2.653832, 0.052398, 2.589583, 0.741)
CUBE_SIZE_M = 0.04
CUBE_MASS_KG = 0.02
CUBE_INITIAL_M = (0.50, 0.0, CUBE_SIZE_M / 2.0)
CUBE_TARGET_M = (0.34, 0.25, CUBE_SIZE_M / 2.0)


@dataclass(frozen=True)
class Segment:
    name: str
    steps: int
    target: tuple[float, ...]


def _target(arm: tuple[float, ...], grip: tuple[float, ...]) -> tuple[float, ...]:
    return (*arm, *grip)


SEGMENTS = (
    Segment("settle", 120, _target(INITIAL[:7], OPEN)),
    Segment("approach", 960, _target(PREGRASP_ARM, OPEN)),
    Segment("descend", 480, _target(GRASP_ARM, OPEN)),
    Segment("close", 420, _target(GRASP_ARM, CLOSED)),
    Segment("grip_hold", 120, _target(GRASP_ARM, CLOSED)),
    Segment("lift", 720, _target(LIFT_ARM, CLOSED)),
    Segment("transfer", 960, _target(TRANSFER_ARM, CLOSED)),
    Segment("place_hover", 480, _target(PLACE_HOVER_ARM, CLOSED)),
    Segment("place", 240, _target(PLACE_ARM, CLOSED)),
    Segment("open", 840, _target(PLACE_ARM, OPEN)),
    Segment("retreat", 960, _target(RETREAT_ARM, OPEN)),
    Segment("final_settle", 240, _target(RETREAT_ARM, OPEN)),
)


def _distance(left: tuple[float, ...], right: tuple[float, ...]) -> float:
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(left, right, strict=True)))


def _encoder(video: Path, fps: int) -> subprocess.Popen[bytes]:
    video.parent.mkdir(parents=True, exist_ok=True)
    return subprocess.Popen(
        (
            "ffmpeg",
            "-hide_banner",
            "-loglevel",
            "error",
            "-y",
            "-f",
            "rawvideo",
            "-pixel_format",
            "rgb24",
            "-video_size",
            f"{RESOLUTION[0]}x{RESOLUTION[1]}",
            "-framerate",
            str(fps),
            "-i",
            "-",
            "-an",
            "-c:v",
            "libx264",
            "-preset",
            "medium",
            "-crf",
            "18",
            "-pix_fmt",
            "yuv420p",
            str(video),
        ),
        stdin=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )


def run(
    backend: str,
    manifest: Path,
    output: Path,
    *,
    video: Path | None = None,
    state_stride: int = 4,
    video_stride: int = 24,
) -> dict[str, object]:
    if state_stride <= 0 or video_stride <= 0:
        raise ValueError("strides must be positive")
    started = time.time()
    sim = Sim(backend=backend, world_id="franka-easyapi-red-cube-pick-place", time_step_seconds=1.0 / 240.0)
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
        color_rgba=RED,
        static_friction=0.5,
        dynamic_friction=0.5,
        restitution=0.0,
        position_m=CUBE_INITIAL_M,
    )
    camera = None
    encoder = None
    if video is not None:
        eye = (1.45, 1.45, 1.15)
        camera = sim.add_camera(
            "acceptance_camera",
            resolution=RESOLUTION,
            outputs=("rgb",),
            position_m=eye,
            orientation_xyzw=_look_at_xyzw(eye, (0.39, 0.12, 0.24)),
        )
        encoder = _encoder(video, 10)

    samples: list[dict[str, object]] = []
    red_counts: list[int] = []
    global_step = 0
    current = tuple(INITIAL)
    try:
        sim.start()
        cube_spec = next(entity for entity in sim.world_spec.entities if entity.path.value == "/red_cube")
        assert cube_spec.box is not None and cube_spec.box.color_rgba == RED
        for segment in SEGMENTS:
            start = current
            for local_step in range(segment.steps):
                phase = _smoothstep((local_step + 1) / segment.steps)
                target = tuple(
                    initial + (final - initial) * phase for initial, final in zip(start, segment.target, strict=True)
                )
                robot.command(target)
                sim.step()
                global_step += 1
                if global_step % state_stride == 0 or local_step + 1 == segment.steps:
                    joint_positions = tuple(float(value) for value in robot.state.joint_positions.rows()[0])
                    cube_position = tuple(float(value) for value in cube.state.positions_m.rows()[0])
                    if not all(math.isfinite(value) for value in (*joint_positions, *cube_position)):
                        raise RuntimeError(f"non-finite state at step {global_step}")
                    contact = cube.contact(force_threshold_n=1.0e-4)
                    contact_force = tuple(float(value) for value in contact.net_normal_forces_n.rows()[0])
                    samples.append(
                        {
                            "step": global_step,
                            "stage": segment.name,
                            "target": list(target),
                            "joint_positions": list(joint_positions),
                            "cube_position_m": list(cube_position),
                            "cube_contact": bool(contact.in_contact.values[0]),
                            "cube_contact_force_n": math.sqrt(sum(value * value for value in contact_force)),
                        }
                    )
                if camera is not None and encoder is not None and global_step % video_stride == 0:
                    rgb = camera.read("rgb")
                    expected_shape = (1, RESOLUTION[1], RESOLUTION[0], 3)
                    if rgb.shape != expected_shape or rgb.dtype != "uint8":
                        raise RuntimeError(f"unexpected native RGB output: shape={rgb.shape}, dtype={rgb.dtype}")
                    frame = bytes(cast(tuple[int, ...], rgb.values))
                    red_counts.append(_red_sample_count(frame, pixel_stride=16))
                    assert encoder.stdin is not None
                    encoder.stdin.write(frame)
            current = segment.target
    finally:
        sim.close()
        if encoder is not None:
            if encoder.stdin is not None:
                encoder.stdin.close()
            stderr = b"" if encoder.stderr is None else encoder.stderr.read()
            return_code = encoder.wait()
            if return_code != 0:
                raise RuntimeError(f"ffmpeg failed with code {return_code}: {stderr.decode('utf-8', errors='replace')}")

    final_cube = tuple(cast(list[float], samples[-1]["cube_position_m"]))
    max_cube_z = max(cast(list[float], sample["cube_position_m"])[2] for sample in samples)
    final_error = _distance(final_cube, CUBE_TARGET_M)
    lifted = max_cube_z >= CUBE_INITIAL_M[2] + 0.08
    result: dict[str, object] = {
        "status": "passed" if lifted and final_error <= 0.03 and (not red_counts or max(red_counts) > 0) else "failed",
        "backend_argument": backend,
        "provider_id": sim.provider_descriptor.provider_id,
        "asset_uri": next(entity.asset_uri for entity in sim.world_spec.entities if entity.path.value == "/franka"),
        "joint_names": list(JOINT_NAMES),
        "time_step_seconds": 1.0 / 240.0,
        "total_steps": global_step,
        "state_stride": state_stride,
        "cube_primitive": "box",
        "cube_size_m": CUBE_SIZE_M,
        "cube_mass_kg": CUBE_MASS_KG,
        "cube_color_rgba": list(RED),
        "cube_static_friction": 0.5,
        "cube_dynamic_friction": 0.5,
        "cube_restitution": 0.0,
        "cube_initial_m": list(CUBE_INITIAL_M),
        "cube_target_m": list(CUBE_TARGET_M),
        "cube_final_m": list(final_cube),
        "cube_final_target_error_m": final_error,
        "cube_max_z_m": max_cube_z,
        "physical_lift_detected": lifted,
        "contact_sample_count": sum(bool(sample["cube_contact"]) for sample in samples),
        "native_red_sample_count_range": None if not red_counts else [min(red_counts), max(red_counts)],
        "native_red_positive_frame_count": None if not red_counts else sum(count > 0 for count in red_counts),
        "video_resolution": None if video is None else list(RESOLUTION),
        "video_fps": None if video is None else 10,
        "video": None if video is None else str(video.resolve()),
        "video_sha256": None if video is None else hashlib.sha256(video.read_bytes()).hexdigest(),
        "duration_wall_seconds": time.time() - started,
        "samples": samples,
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--backend", required=True, choices=("isaaclab", "mujoco", "pybullet"))
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--video", type=Path)
    parser.add_argument("--state-stride", type=int, default=4)
    parser.add_argument("--video-stride", type=int, default=24)
    args = parser.parse_args()
    result = run(
        args.backend,
        args.manifest.resolve(),
        args.output.resolve(),
        video=None if args.video is None else args.video.resolve(),
        state_stride=args.state_stride,
        video_stride=args.video_stride,
    )
    print(json.dumps({key: value for key, value in result.items() if key != "samples"}, indent=2))
    return 0 if result["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
