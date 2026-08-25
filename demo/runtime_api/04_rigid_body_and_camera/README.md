# 04 - Rigid body and camera observations

[中文](README.zh-CN.md) | [Runtime API index](../README.md)

## Audience

This case is for framework developers implementing portable object control,
observation transport, or camera recording without exposing backend SDK arrays.

## What this demonstrates

The World contains a procedural rigid box and a synchronous RGB/depth camera in
two environments. A typed wrench is applied, then the program reads portable
rigid state and packed camera channels.

## Prerequisites

- Complete case 03 or understand batch-first Runtime values
- UniRoboSim Core 0.10; no image library or native renderer is required

## Run

```bash
PYTHONPATH=src python demo/runtime_api/04_rigid_body_and_camera/main.py
```

## Expected result

```text
box_x=0.010 rgb=(2, 6, 8, 3)/uint8 depth=(2, 6, 8)/float32
```

## Code walkthrough

1. `BoxGeometrySpec` carries portable dimensions, mass, material, and color intent.
2. Zero gravity isolates the effect of the submitted wrench in this teaching case.
3. `RigidBodyCommand` contains one force and torque row per selected environment.
4. `read_rigid_body()` returns positions, XYZW orientations, and twists as immutable
   `ArrayValue` objects.
5. `read_sensor()` samples all requested channels at one simulation tick.
6. RGB is a packed uint8 array. `to_bytes()` keeps the compact path and avoids
   materializing one Python integer for every pixel component.

## Common failures

- Wrench arrays use `[environment, xyz]`, not `[xyz]`.
- RGB shape is `[environment, height, width, rgb]`; depth has no final channel axis.
- Read a channel with `CameraModality`, not an arbitrary string.
- FakeProvider returns a deterministic test pattern. It is not evidence of native
  image quality, camera calibration, or rendering performance.

## Verification level

This verifies portable wrench/state semantics, camera channel shapes, dtypes, and
packed byte access. Native physics and rendered imagery remain adapter acceptance.

## Next

Continue with [05 - scene control](../05_scene_control/README.md).
