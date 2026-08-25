# 05 - Capability-gated scene control

[中文](README.zh-CN.md) | [Runtime API index](../README.md)

## Audience

Use this case when building a browser console, interactive debugger, scene editor,
or another client that needs portable snapshots and controlled scene mutation.

## What this demonstrates

Scene control is an optional Runtime extension. The program first negotiates all
required capabilities, then verifies `SceneControlWorld`, reads a snapshot, sends
an idempotent pose command, and consumes the resulting delta.

## Prerequisites

- Understand capability negotiation from case 02
- UniRoboSim Core 0.10; no browser or native renderer is required

## Run

```bash
PYTHONPATH=src python demo/runtime_api/05_scene_control/main.py
```

## Expected result

```text
capabilities=accepted initial=0 current=1 duplicate=duplicate
```

## Code walkthrough

1. Three hard requirements cover snapshots, deltas, and pose mutation.
2. `session.negotiate()` runs before the optional protocol is accessed. A framework
   must stop here if the report is not accepted.
3. The same requirements are embedded in `WorldSpec`, so build performs the final
   authoritative negotiation.
4. Only after the gate passes does the code check `SceneControlWorld` and call its
   methods.
5. `SceneCommand` includes a stable `command_id`, client/lease identity, current
   World generation, entity path, environment, and target pose.
6. Repeating the same command returns `DUPLICATE` instead of applying it twice.
7. `scene_delta(initial.sequence)` lets a client update an existing view without
   reconstructing its protocol from backend-native data.

## Common failures

- Never call an optional extension only because the Python object has that method;
  capability negotiation is the semantic gate.
- Use the current World generation. A stale generation is rejected.
- Every mutation needs a unique, stable `command_id`; retries reuse the same ID.
- Scene pose commands are interaction/debug operations, not controller trajectories.

## Verification level

This verifies capability gating, scene value contracts, sequence progression, and
command idempotency. FakeProvider does not render the scene or implement a browser.

## Next

Continue with [06 - transactional build](../06_transactional_build/README.md).
