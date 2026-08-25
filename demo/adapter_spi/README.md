# Adapter SPI learning path

English | [简体中文](README.zh-CN.md)

This path is for simulator maintainers who need to connect a new backend to
UniRoboSim. It uses one deliberately small articulation-only distribution in
[`teaching_adapter/`](teaching_adapter/) so every lifecycle and data-boundary decision
can be inspected without a vendor SDK.

The teaching Adapter is not a physics engine. It is an executable reference for the
public Provider → Session → World shape, capability honesty, handle ownership,
transactional build behavior, Python entry-point discovery, and packaging gates.

## One-time setup

Install Core from this repository and the teaching Adapter into the same Python
3.11 or 3.12 environment:

```bash
python -m pip install -e .
python -m pip install -e ./demo/adapter_spi/teaching_adapter
```

Case 06 also needs `pytest`. Case 07 needs `pip`, `venv`, setuptools, and wheel in
the current environment; it creates and deletes its own clean temporary venv.

## Cases

| Case | Focus | Verification |
|---|---|---|
| [01 — Descriptor and probe](01_descriptor_and_probe/README.md) | Declare only implemented capabilities and keep discovery cheap | Executable Core-only check |
| [02 — Session lifecycle](02_session_lifecycle/README.md) | Make build transactional and lifecycle states explicit | Executable injected-failure check |
| [03 — World and control](03_world_and_control/README.md) | Map paths to handles and close a typed command/state loop | Executable semantic check |
| [04 — Capability honesty](04_capability_honesty/README.md) | Reject unsupported requirements and endpoints | Executable negative check |
| [05 — Entry-point discovery](05_entry_point_discovery/README.md) | Make `Sim(backend="teaching")` find an installed Adapter | Installed-distribution check |
| [06 — Public contract suite](06_public_contract_suite/README.md) | Test lifecycle, commands, reset, stale handles, and errors | 10 public-only pytest checks |
| [07 — Clean wheel gate](07_clean_wheel_gate/README.md) | Build two wheels and verify discovery in an isolated venv | Clean-install release check |

## Where native SDK code belongs

Replace the teaching World's in-memory joint table with native SDK objects inside
the Adapter package. Application code, FastSim, and Core must continue to see only
portable UniRoboSim values. Package import, factory creation, and `probe()` must not
launch the simulator. Allocate native resources during `Session.build()`, commit the
World only after all validation and allocation succeed, and release every owned
resource through idempotent `close()`.

Core 0.10 does not publish a conformance helper module. Case 06 is therefore named a
public-contract acceptance suite: structural `isinstance()` checks are included, but
semantic tests provide the actual evidence.
