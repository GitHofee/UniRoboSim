# 04 — Multiple environments

[English](README.md) | [简体中文](README.zh-CN.md)

## Audience

For developers who need batched state while retaining explicit control over each environment.

## Prerequisites

Understand the rigid and articulation commands from cases 02 and 03.

```bash
PYTHONPATH=src python demo/easyapi/04_multiple_environments/main.py
```

## Expected result

Only environments 0 and 2 move. Resetting environment 2 restores only its joint value; environments 0 and 1 remain unchanged.

## Code walkthrough

`num_envs=3` requests three environments in one world. The two-row wrench and joint target matrices correspond exactly to `environments=(0,2)`. State still returns three rows, one per environment. `reset((2,))` performs a partial reset and returns a `ResetResult` identifying the affected environment. The assertions check both command routing and isolation.

## Common errors

- For a selected environment list, the first target dimension must have the same length.
- Environment indices must be unique and within `[0, num_envs)`.
- Native visible-window profiles may support fewer environments than headless profiles; capability negotiation must decide this.

## Verification level

Dependency-free batched command, state, and partial-reset contract test.

## Next

Continue to [05 — Camera observations](../05_camera_observations/README.md).
