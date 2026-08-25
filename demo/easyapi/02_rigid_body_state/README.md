# 02 — Rigid-body state

[English](README.md) | [简体中文](README.zh-CN.md)

## Audience

For beginners who need to apply a physical command and read portable state.

## Prerequisites

Complete case 01. No native simulator SDK is required.

```bash
PYTHONPATH=src python demo/easyapi/02_rigid_body_state/main.py
```

## Expected result

After one 0.1-second test step, the program prints `position_x_m=0.010000` and `velocity_x_m_s=0.100000`.

## Code walkthrough

The `Sim` constructor sets a 0.1-second step and zero gravity so the result is easy to inspect. `add_box(..., mass_kg=2.0)` records physical intent in the portable world. `apply_wrench(force_n=(2,0,0))` sends a force command. `step()` advances the world and returns a typed `Tick`. `box.state` contains batched position, XYZW orientation, linear velocity, and angular velocity arrays. `math.isclose()` is used because simulation values are floating point.

## Common errors

- `apply_wrench()` takes SI units: newtons and newton-metres.
- State arrays are batch-first; `rows()[0]` selects environment zero.
- The FakeProvider point-mass update is deterministic test behavior, not a fidelity benchmark.

## Verification level

Dependency-free command/state contract test.

## Next

Continue to [03 — Articulation control](../03_articulation_control/README.md).
