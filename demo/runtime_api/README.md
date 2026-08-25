# Runtime API demos

[中文](README.zh-CN.md)

These examples are for developers building frameworks, services, schedulers, or
other infrastructure on top of UniRoboSim. They start with the explicit
Provider -> Session -> World lifecycle and progress to commands, observations,
optional scene control, transactional recovery, and verified asset inputs.

All examples use `unirobosim.testing.FakeProvider`. It is a deterministic
contract-test backend, not a native physics simulator or renderer. Replace its
factory with a compatible installed adapter after the contract flow is clear.

## Run from a source checkout

Use Python 3.11 or 3.12 from the repository root:

```bash
PYTHONPATH=src python demo/runtime_api/01_provider_session_world/main.py
```

If UniRoboSim 0.10 is installed in the environment, `PYTHONPATH=src` can be
omitted.

## Learning path

| Case | Focus |
|---|---|
| [01](01_provider_session_world/README.md) | Explicit Provider, Session, and World ownership |
| [02](02_capability_selection/README.md) | Lazy registry and capability-based selection |
| [03](03_articulation_command/README.md) | Typed batched articulation commands and state |
| [04](04_rigid_body_and_camera/README.md) | Rigid-body wrench and packed camera observations |
| [05](05_scene_control/README.md) | Capability-gated snapshot, delta, and idempotent mutation |
| [06](06_transactional_build/README.md) | Retryable build and stale-handle rejection |
| [07](07_verified_build_input/README.md) | Digest-pinned local asset handoff |

Each program contains executable assertions. A zero exit status means the
portable contract demonstrated by that case was satisfied by FakeProvider.
Native dynamics, rendering quality, SDK lifecycle, and GPU behavior require the
independent acceptance suite of the selected adapter.
