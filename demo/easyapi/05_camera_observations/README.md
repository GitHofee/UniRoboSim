# 05 — Camera observations

[English](README.md) | [简体中文](README.zh-CN.md)

## Audience

For developers consuming camera observations or recording image streams.

## Prerequisites

UniRoboSim Core 0.10. No image library is required.

```bash
PYTHONPATH=src python demo/easyapi/05_camera_observations/main.py
```

## Expected result

The program reports RGB shape `(2, 6, 8, 3)`, depth shape `(2, 6, 8)`, normals shape `(2, 6, 8, 3)`, and 288 packed RGB bytes.

## Code walkthrough

`add_camera()` declares width-first resolution `(8,6)` and the requested modalities. `camera.sample()` returns channels from one tick. `sample.channel(CameraModality.RGB)` uses the typed enum; `camera.read("normals")` shows the string shortcut. RGB is a packed `uint8` value, so `to_bytes()` avoids expanding every pixel into Python integers. Shape order is environment, height, width, then channel.

## Common errors

- Resolution is `(width, height)`, while output shape is `[environment, height, width, channel]`.
- Call `to_bytes()` only for `uint8` values.
- FakeProvider images are deterministic test patterns, not rendered scene pixels.

## Verification level

Dependency-free camera modality, shape, dtype, and packed-storage contract test.

## Next

Continue to [06 — Capabilities and provider](../06_capabilities_and_provider/README.md).
