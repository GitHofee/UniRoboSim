# 08 — Debug and scene snapshot

[English](README.md) | [简体中文](README.zh-CN.md)

## Audience

For developers building inspection tools, visual debugging, or browser scene controls.

## Prerequisites

Understand entities and batched coordinates from earlier cases.

```bash
PYTHONPATH=src python demo/easyapi/08_debug_and_scene_snapshot/main.py
```

## Expected result

One debug point is accepted, the snapshot reports box dimensions `(0.2, 0.3, 0.4)`, and one primitive is cleared.

## Code walkthrough

`ArrayValue.from_nested([[[x,y,z]]])` creates geometry with shape `[environment, point, xyz]`. `DebugPrimitive` supplies a stable ID, layer, kind, color, environment selection, and manual lifetime. `sim.debug.publish()` sends it through the portable debug endpoint. `scene_snapshot()` reads backend-neutral entities and visuals; no native object escapes. `sim.debug.clear(layer="tutorial")` removes the marker by a stable filter.

## Common errors

- Geometry batch size must match `environment_indices` exactly.
- Quaternions use XYZW order and debug positions use metres.
- FakeProvider stores debug data in memory; it does not display a rendered overlay.

## Verification level

Dependency-free portable debug and scene-snapshot contract test.

## Next

Continue to [09 — Deformable and fluid](../09_deformable_and_fluid/README.md).
