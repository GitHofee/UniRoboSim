# 03 - Typed articulation command

[中文](README.zh-CN.md) | [Runtime API index](../README.md)

## Audience

Use this case when implementing a controller bridge, model-serving loop, rule-based
executor, or recorder that exchanges batched joint commands and states.

## What this demonstrates

The example creates a non-robot articulation with one rotational hinge and one
linear slide in two environments. It resolves a portable handle, commands only
one selected joint in one selected environment, steps once, and reads typed state.

## Prerequisites

- Understand Provider, Session, and World ownership from case 01
- UniRoboSim Core 0.10; no native simulator is required

## Run

```bash
PYTHONPATH=src python demo/runtime_api/03_articulation_command/main.py
```

## Expected result

```text
tick=1 joint_positions=((0.1, -0.2), (0.75, -0.2))
```

## Code walkthrough

1. `EnvironmentSpec(count=2)` establishes the leading batch dimension.
2. The physical v0alpha5 World schema allows explicit rotational and prismatic
   axis units; `joint_position_units=("rad", "m")` closes those semantics.
3. `world.resolve(EntityPath("/cabinet"))` returns a generation-scoped handle.
4. The target has shape `(1, 1)` because one environment and one degree of freedom
   are selected.
5. `target_units=("rad",)` closes the selected axis contract.
6. Commands are submitted before `step()`; the state read after the step carries
   the same batch-first environment order.

## Common failures

- Runtime API never broadcasts command arrays. The shape must exactly match the
  selected environments and joints.
- Target rows follow `environment_indices`; columns follow
  `degree_of_freedom_indices`.
- Rotational position uses `rad`; prismatic position uses `m`.
- A handle is valid only for the Session, World ID, and generation that created it.

## Verification level

FakeProvider applies deterministic reference integration. The test verifies command
shape, selection, units, handles, and state layout, not native actuator dynamics.

## Next

Continue with [04 - rigid body and camera](../04_rigid_body_and_camera/README.md).
