# 09 — Deformable and fluid

[English](README.md) | [简体中文](README.zh-CN.md)

## Audience

For developers who need portable declarations and point-level control for cloth, soft bodies, or particle fluids.

## Prerequisites

Understand batched commands from case 04. No native simulator SDK is required for this contract example.

```bash
PYTHONPATH=src python demo/easyapi/09_deformable_and_fluid/main.py
```

## Expected result

Cloth node zero moves to `(0.1, 0.2, 1.2)` and fluid particle zero advances to x `0.1` in the deterministic test model.

## Code walkthrough

`add_deformable()` declares rest positions, triangle topology, and a kinematic node. `add_particle_fluid()` declares fixed particle positions and radius. These methods automatically add their required point-control capabilities. `cloth.command(..., nodes=(0,), mode="position")` targets one node. `water.command(..., mode="velocity")` broadcasts one xyz velocity to all particles. Typed state remains `[environment, point, xyz]`.

## Common errors

- Surface topology needs valid triangle indices; volume topology uses tetrahedra.
- Point targets must have one accepted shape: xyz, point-by-xyz, or environment-by-point-by-xyz.
- FakeProvider has no elasticity, incompressibility, collision, viscosity, or surface-tension solver. Use this case only for API validation.

## Verification level

Dependency-free soft-matter declaration, command, state, and capability contract test; not physical-fidelity acceptance.

## Next

Continue to [10 — Switch native backend](../10_switch_native_backend/README.md).
