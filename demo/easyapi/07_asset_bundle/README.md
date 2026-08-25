# 07 — Asset bundle

[English](README.md) | [简体中文](README.zh-CN.md)

## Audience

For developers whose logical robot or object has different native files for different simulator providers.

## Prerequisites

UniRoboSim Core 0.10. The case includes its own tiny URDF and manifest.

```bash
PYTHONPATH=src python demo/easyapi/07_asset_bundle/main.py
```

## Expected result

The program prints logical name `demo_robot`, selector `fake`, the absolute URDF path, and its verified SHA-256 digest.

## Files and code walkthrough

`assets/demo_robot.urdf` is a small source asset. `demo_robot.asset.json` assigns that file to provider selector `fake` and pins its content digest. `Path(__file__)` makes the example independent of the current directory. `AssetBundle.from_manifest()` validates the schema and resolves relative paths. Passing the bundle to `add_articulation()` delays provider-specific selection until `start()`. The resolved URI and provenance are recorded under `entity.metadata["unirobosim_asset"]`.

## Common errors

- Editing the URDF without updating its SHA-256 causes an intentional identity failure.
- Real deployments normally include selectors such as `isaaclab`, `mujoco`, and `pybullet`.
- FakeProvider verifies selection and provenance but does not parse or simulate this URDF.

## Verification level

Dependency-free manifest parsing, provider selection, digest, and provenance contract test; not native asset loading.

## Next

Continue to [08 — Debug and scene snapshot](../08_debug_and_scene_snapshot/README.md).
