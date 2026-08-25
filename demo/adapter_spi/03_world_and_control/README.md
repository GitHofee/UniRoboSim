# 03 — World and control

English | [简体中文](README.zh-CN.md)

**Audience:** Adapter developers implementing entity lookup, typed commands, stepping,
and typed state reads.
**Verification:** executable public-contract semantic check; no simulator is launched.

## Run

Complete the [one-time setup](../README.md#one-time-setup), then run from the
repository root:

```bash
python demo/adapter_spi/03_world_and_control/main.py
```

Expected output:

```text
joint_positions=((0.0, 0.1), (0.65, 0.1))
tick=1
```

## What the code does

1. [`main.py`](main.py) creates an immutable `WorldSpec` containing one cabinet
   articulation and two environments. The cabinet demonstrates that an articulation
   is not necessarily a robot.
2. `provider.open()` and `session.build(spec)` establish the explicit Provider →
   Session → World lifecycle.
3. `world.resolve(EntityPath("/cabinet"))` converts a stable logical path into a
   generation-bound `EntityHandle` owned by this World.
4. `ArticulationCommand` selects environment 1, degree of freedom 0, position mode,
   one `[environment, joint]` target, and the matching `rad` unit.
5. `apply_articulation_command()` queues the command. `step()` commits it and advances
   the tick. `read_articulation()` returns typed batch-first state.
6. The assertion proves that environment 0 and the unselected drawer joint remain
   unchanged. The `finally` block closes the Session even if a check fails.

## Implementation principles

- Resolve paths once, then validate every Handle against provider, Session, World,
  generation, path, kind, and token before native use.
- Validate command mode, selection, exact target shape, and units before calling a
  native SDK.
- Define a clear command timing rule. This Adapter queues commands and applies them
  on the next `step()`.
- Return immutable portable state; never return a vendor joint array or native object.

## Common failures

- Target shape must exactly match selected environments and joints. Here it is
  `(1, 1)`, not `(2, 2)`.
- `target_units=("rad",)` must match the selected rotational axis.
- A Handle from a closed or rebuilt World is stale and must be rejected.
- Closing only the application-side object is insufficient; the Adapter must release
  every native resource it owns.

Previous: [02 — Session lifecycle](../02_session_lifecycle/README.md).
Next: [04 — Capability honesty](../04_capability_honesty/README.md).
