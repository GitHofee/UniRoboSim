# 01 — First simulation

[English](README.md) | [简体中文](README.zh-CN.md)

## Audience

Start here if you have basic Python knowledge and have not used UniRoboSim before.

## Prerequisites

- Python 3.11 or 3.12
- UniRoboSim Core 0.10
- No native simulator SDK

Run from the repository root:

```bash
PYTHONPATH=src python demo/easyapi/01_first_simulation/main.py
```

## Expected result

The program prints provider `reference.fake`, the box position `((0.0, 0.0, 0.5),)`, and `simulation_closed=true`.

## Code walkthrough

`Sim(provider=FakeProvider(), world_id=...)` creates a configurable simulation and explicitly selects the contract-test provider. `add_box()` declares one portable rigid-body primitive before the world starts. `start()` compiles the declarations and returns a `BuildReport`. `box.state` reads typed state from the running world. The `with` block always closes the owned session; the final assertion proves the lifecycle reached `SimState.CLOSED`.

## Common errors

- Import `FakeProvider` from `unirobosim.testing`, not from `unirobosim`.
- Add entities before `start()`; a running world cannot be edited through EasyAPI.
- FakeProvider validates contracts but is not a real physics engine.

## Verification level

Dependency-free executable contract smoke test. It does not verify native physics or rendering.

## Next

Continue to [02 — Rigid-body state](../02_rigid_body_state/README.md).
