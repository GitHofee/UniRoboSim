# 01 - Provider, Session, and World

[中文](README.zh-CN.md) | [Runtime API index](../README.md)

## Audience

Start here if you are building a framework on UniRoboSim and have not used the
strict Runtime API before.

## What this demonstrates

Runtime ownership has three explicit levels: a `Provider` describes and opens a
backend, a `Session` owns one backend connection, and a `World` owns one built
simulation. The program checks all three public structural protocols and closes
resources in reverse order.

## Prerequisites

- Python 3.11 or 3.12
- UniRoboSim Core 0.10
- No native simulator; `FakeProvider` is included as a contract-test utility

## Run

From the repository root:

```bash
PYTHONPATH=src python demo/runtime_api/01_provider_session_world/main.py
```

## Expected result

```text
provider=reference.fake available=True
world=runtime-lifecycle generation=1 entities=1
lifecycle=closed-cleanly
```

## Code walkthrough

1. `FakeProvider()` supplies a deterministic implementation of the public
   contracts. `isinstance(provider, Provider)` checks its structural API.
2. `probe()` reports availability without opening a Session or building a World.
3. `EntitySpec` describes an articulated cabinet. An articulation is not assumed
   to be a robot; here it has one door hinge.
4. `WorldSpec` is immutable input compiled by an upper-layer framework.
5. `session.build(spec)` moves the Session from `OPEN` to `READY` and returns the
   World that owns simulation state.
6. Nested `try/finally` blocks close the World before its Session. This works for
   every compliant adapter and does not rely on FakeProvider conveniences.

## Common failures

- An articulation without `joint_names` is invalid.
- A `WorldSpec` must contain at least one entity.
- Building a second live World in the same Session raises `LifecycleError`.
- Do not keep native SDK handles above Runtime API; retain `EntityPath` and the
  portable handle returned by `World.resolve()` instead.

## Verification level

This is a dependency-free Core contract smoke test. It verifies lifecycle and
portable types, not native physics, rendering, or SDK cleanup.

## Next

Continue with [02 - capability selection](../02_capability_selection/README.md).
