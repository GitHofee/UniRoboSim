# 03 — Articulation control

[English](README.md) | [简体中文](README.zh-CN.md)

## Audience

For application developers controlling robots or ordinary articulated objects such as cabinets, laptops, and appliances.

## Prerequisites

UniRoboSim Core 0.10. No native simulator SDK is required.

```bash
PYTHONPATH=src python demo/easyapi/03_articulation_control/main.py
```

## Expected result

The door hinge reaches `0.6`; the uncommanded drawer remains at `-0.2`.

## Code walkthrough

`add_articulation()` declares two named degrees of freedom and their initial positions. The entity is a cabinet, demonstrating that an articulation is not necessarily a robot. `cabinet.command((0.6,), joints=("door_hinge",), mode="position")` addresses one joint by its portable name. `step()` applies the queued target. `cabinet.state.joint_positions` returns a batch-first typed array in the same declared joint order.

## Common errors

- The number of target values must equal the number of selected joints.
- Joint names must exactly match `joint_names`; unknown names fail before native dispatch.
- Position, velocity, and effort have different units and semantics.

## Verification level

Dependency-free named-joint command and state contract test.

## Next

Continue to [04 — Multiple environments](../04_multiple_environments/README.md).
