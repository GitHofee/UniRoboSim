# EasyAPI examples

[English](README.md) | [简体中文](README.zh-CN.md)

These examples are for application developers who want to create and control a simulation without implementing a framework or simulator adapter. Start with case 01 and follow the numbered directories in order.

Cases 01–09 use `FakeProvider`, UniRoboSim's deterministic contract-test backend. They need no simulator SDK, but their motion and camera values are not physics- or rendering-fidelity claims. Case 10 keeps the source code unchanged while selecting an installed native backend explicitly.

| Case | What it teaches | Runs without a simulator SDK |
| --- | --- | --- |
| [01 First simulation](01_first_simulation/README.md) | Build, start, read, and close a world | Yes |
| [02 Rigid-body state](02_rigid_body_state/README.md) | Apply a wrench and read typed state | Yes |
| [03 Articulation control](03_articulation_control/README.md) | Control a named joint on a non-robot mechanism | Yes |
| [04 Multiple environments](04_multiple_environments/README.md) | Target selected environments and reset one environment | Yes |
| [05 Camera observations](05_camera_observations/README.md) | Read RGB, depth, and normals efficiently | Yes |
| [06 Capabilities and provider](06_capabilities_and_provider/README.md) | Declare requirements and inspect the selected provider | Yes |
| [07 Asset bundle](07_asset_bundle/README.md) | Resolve one logical asset with a pinned digest | Yes |
| [08 Debug and scene snapshot](08_debug_and_scene_snapshot/README.md) | Publish a marker and inspect portable scene state | Yes |
| [09 Deformable and fluid](09_deformable_and_fluid/README.md) | Command deformable nodes and fluid particles | Yes |
| [10 Switch native backend](10_switch_native_backend/README.md) | Select Isaac Lab, MuJoCo, or PyBullet with one argument | Isaac Lab 0.10.1 passed; MuJoCo/PyBullet pending 0.10 adapters |

Run an SDK-free example from the repository root:

```bash
PYTHONPATH=src python demo/easyapi/01_first_simulation/main.py
```

All code imports public Core APIs. `FakeProvider` is intentionally imported from `unirobosim.testing`; it is never an implicit production backend.
