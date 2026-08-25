# 10 — Switch a native backend

[English](README.md) | [简体中文](README.zh-CN.md)

## Audience

For application developers ready to move the same EasyAPI source from contract tests to a real simulator.

## Prerequisites

- UniRoboSim Core 0.10
- A matching Core 0.10 adapter installed for the chosen backend
- That adapter's native simulator SDK and runtime prerequisites
- Python 3.12 for Isaac Lab; do not try to install its Adapter into a Python 3.11
  FastSim environment

The public Isaac Lab Adapter `0.10.1` supports Core `>=0.10,<0.11`. The public
MuJoCo and PyBullet `0.9.1` lines still require Core `<0.10`, so those two choices
remain pending a compatible Adapter release.

Run the verified visible Isaac Lab path with:

```bash
UNIROBOSIM_ISAACLAB_LAUNCH_PROFILE=visible \
  python demo/easyapi/10_switch_native_backend/main.py --backend isaaclab
```

After compatible Adapter releases are installed, replace only the argument with
`mujoco` or `pybullet`; the scene and observation code stays unchanged.

## Expected result

The program prints the explicit backend argument, selected provider ID, finite box position, and RGB shape `(1, 720, 1280, 3)`.

## Code walkthrough

`argparse` makes backend choice explicit at the application boundary. `Sim(backend=args.backend)` asks installed `unirobosim.backends` entry points for the named provider. The box, camera, capability requirements, stepping, state read, and image read contain no backend branch. `require()` ensures a physics-only profile cannot silently accept a camera world. The assertions check portable results, not backend-specific objects.

## Common errors

- `ProviderSelectionError` usually means the adapter is absent, unavailable, incompatible, or lacks a required capability.
- Do not set `Sim(headless=False)` in Core 0.10; select visible/headless behavior through the adapter's documented launch profile.
- Native image and physics quality must be accepted separately from this portable contract smoke test.

## Verification level

Real visible Isaac Lab 3.0 / Isaac Sim 6.0.1 execution passed with Adapter 0.10.1:
30 physics steps, provider `nvidia.isaaclab`, finite rigid-body state, and native RGB
shape `(1, 720, 1280, 3)`. MuJoCo and PyBullet remain version-blocked, not passed.

## Next

Use [RuntimeAPI](../../runtime_api/README.md) when building a framework that owns Provider, Session, and World directly.
