"""Record one backend-native Franka camera video from one branch-free EasyAPI scene."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import subprocess
import time
from pathlib import Path
from typing import cast

from franka_easyapi_control import FINAL, INITIAL, JOINT_NAMES, _smoothstep

from unirobosim import AssetBundle, Sim

RED = (1.0, 0.0, 0.0, 1.0)
RESOLUTION = (1920, 1080)


def _normalize(vector: tuple[float, float, float]) -> tuple[float, float, float]:
    length = math.sqrt(sum(value * value for value in vector))
    if length <= 1.0e-12:
        raise ValueError("cannot normalize a zero vector")
    return tuple(value / length for value in vector)  # type: ignore[return-value]


def _cross(left: tuple[float, float, float], right: tuple[float, float, float]) -> tuple[float, float, float]:
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


def _matrix_to_xyzw(matrix: tuple[tuple[float, float, float], ...]) -> tuple[float, float, float, float]:
    m00, m01, m02 = matrix[0]
    m10, m11, m12 = matrix[1]
    m20, m21, m22 = matrix[2]
    trace = m00 + m11 + m22
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (m21 - m12) / scale
        y = (m02 - m20) / scale
        z = (m10 - m01) / scale
    elif m00 > m11 and m00 > m22:
        scale = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        w = (m21 - m12) / scale
        x = 0.25 * scale
        y = (m01 + m10) / scale
        z = (m02 + m20) / scale
    elif m11 > m22:
        scale = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        w = (m02 - m20) / scale
        x = (m01 + m10) / scale
        y = 0.25 * scale
        z = (m12 + m21) / scale
    else:
        scale = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        w = (m10 - m01) / scale
        x = (m02 + m20) / scale
        y = (m12 + m21) / scale
        z = 0.25 * scale
    return _normalize_quaternion((x, y, z, w))


def _normalize_quaternion(values: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    length = math.sqrt(sum(value * value for value in values))
    return tuple(value / length for value in values)  # type: ignore[return-value]


def _look_at_xyzw(
    eye: tuple[float, float, float],
    target: tuple[float, float, float],
) -> tuple[float, float, float, float]:
    """Return an OpenGL camera rotation (-Z forward, +Y up)."""

    forward = _normalize(tuple(target[axis] - eye[axis] for axis in range(3)))  # type: ignore[arg-type]
    right = _normalize(_cross(forward, (0.0, 0.0, 1.0)))
    up = _cross(right, forward)
    back = tuple(-value for value in forward)
    # Rotation columns are the local +X, +Y and +Z axes in world coordinates.
    matrix = (
        (right[0], up[0], back[0]),
        (right[1], up[1], back[1]),
        (right[2], up[2], back[2]),
    )
    return _matrix_to_xyzw(matrix)


def _red_sample_count(frame: bytes, *, pixel_stride: int = 16) -> int:
    return sum(
        1
        for index in range(0, len(frame), 3 * pixel_stride)
        if frame[index] >= 48 and frame[index] >= frame[index + 1] + 20 and frame[index] >= frame[index + 2] + 20
    )


def run(backend: str, manifest: Path, video: Path, report: Path, *, frames: int = 90) -> dict[str, object]:
    if frames <= 0:
        raise ValueError("frames must be positive")
    started = time.time()
    bundle = AssetBundle.from_manifest(manifest)
    eye = (1.7, 1.7, 1.25)
    camera_orientation = _look_at_xyzw(eye, (0.15, 0.0, 0.48))
    sim = Sim(backend=backend, world_id="franka-easyapi-native-video", time_step_seconds=1.0 / 240.0)
    robot = sim.add_articulation(
        "franka",
        joint_names=JOINT_NAMES,
        initial_positions=INITIAL,
        asset=bundle,
    )
    sim.add_box(
        "red_cube",
        size_m=(0.08, 0.08, 0.08),
        mass_kg=0.1,
        color_rgba=RED,
        position_m=(0.55, 0.0, 0.04),
    )
    camera = sim.add_camera(
        "acceptance_camera",
        resolution=RESOLUTION,
        outputs=("rgb",),
        position_m=eye,
        orientation_xyzw=camera_orientation,
    )
    video.parent.mkdir(parents=True, exist_ok=True)
    ffmpeg = subprocess.Popen(
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
            "30",
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
    red_counts: list[int] = []
    frame_min = 255
    frame_max = 0
    try:
        sim.start()
        cube_spec = next(entity for entity in sim.world_spec.entities if entity.path.value == "/red_cube")
        assert cube_spec.box is not None and cube_spec.box.color_rgba == RED
        for frame_index in range(frames):
            phase = _smoothstep((frame_index + 1) / frames)
            target = tuple(start + (end - start) * phase for start, end in zip(INITIAL, FINAL, strict=True))
            robot.command(target)
            sim.step(8)
            rgb = camera.read("rgb")
            expected_shape = (1, RESOLUTION[1], RESOLUTION[0], 3)
            if rgb.shape != expected_shape or rgb.dtype != "uint8":
                raise RuntimeError(f"unexpected native RGB output: shape={rgb.shape}, dtype={rgb.dtype}")
            frame = bytes(cast(tuple[int, ...], rgb.values))
            red_counts.append(_red_sample_count(frame))
            frame_min = min(frame_min, min(frame))
            frame_max = max(frame_max, max(frame))
            assert ffmpeg.stdin is not None
            ffmpeg.stdin.write(frame)
    finally:
        sim.close()
        if ffmpeg.stdin is not None:
            ffmpeg.stdin.close()
        stderr = b"" if ffmpeg.stderr is None else ffmpeg.stderr.read()
        return_code = ffmpeg.wait()
    if return_code != 0:
        raise RuntimeError(f"ffmpeg failed with code {return_code}: {stderr.decode('utf-8', errors='replace')}")
    if min(red_counts) <= 0 or frame_max <= frame_min:
        raise RuntimeError(
            "native frames did not retain visible red geometry/contrast: "
            f"red={red_counts}, range={frame_min}:{frame_max}"
        )
    result: dict[str, object] = {
        "status": "passed",
        "backend_argument": backend,
        "provider_id": sim.provider_descriptor.provider_id,
        "asset_uri": next(entity.asset_uri for entity in sim.world_spec.entities if entity.path.value == "/franka"),
        "frames": frames,
        "fps": 30,
        "resolution": list(RESOLUTION),
        "camera_frame": "OpenGL (-Z forward, +Y up), XYZW quaternion",
        "camera_position_m": list(eye),
        "camera_orientation_xyzw": list(camera_orientation),
        "cube_primitive": "box",
        "cube_color_rgba": list(RED),
        "native_red_sample_count_range": [min(red_counts), max(red_counts)],
        "native_rgb_value_range": [frame_min, frame_max],
        "video": str(video.resolve()),
        "video_sha256": hashlib.sha256(video.read_bytes()).hexdigest(),
        "duration_wall_seconds": time.time() - started,
    }
    report.parent.mkdir(parents=True, exist_ok=True)
    report.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--backend", required=True, choices=("isaaclab", "mujoco", "pybullet"))
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--video", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--frames", type=int, default=90)
    args = parser.parse_args()
    result = run(
        args.backend,
        args.manifest.resolve(),
        args.video.resolve(),
        args.report.resolve(),
        frames=args.frames,
    )
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
