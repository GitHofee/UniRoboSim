# 06 — Public contract suite

English | [简体中文](README.zh-CN.md)

**Audience:** Adapter maintainers preparing a semantically testable backend before
adding native simulator acceptance.
**Verification:** nine executable pytest checks using public Core contracts only.

## Run

Complete the [one-time setup](../README.md#one-time-setup), install `pytest`, then run
from the repository root:

```bash
python -m pip install pytest
python demo/adapter_spi/06_public_contract_suite/main.py
```

Expected stable result:

```text
10 passed
public_contract_suite=passed
```

Pytest may add timing or platform text around these markers.

## What the code does

1. [`main.py`](main.py) resolves the teaching Adapter's `tests/` directory relative
   to its own file, so it can be launched from the repository root reliably.
2. It invokes `python -m pytest -q` with the same interpreter that launched the demo.
3. A failing pytest return code is propagated as the process exit status; the success
   marker is printed only after every check passes.
4. [`test_public_contract.py`](../teaching_adapter/tests/test_public_contract.py)
   imports public `unirobosim` symbols and the teaching distribution. It deliberately
   imports neither Core internals nor `FakeProvider`.
5. The nine semantic checks cover discovery-safe probe, lifecycle, transactional
   retry, capability negotiation, one-live-World ownership, command/state/reset,
   shape and unit errors, stale Handles, and unsupported endpoints.

## Implementation principles

- Core 0.10 does **not** publish a conformance helper module. This is an Adapter-owned
  public-contract acceptance suite, not a call to a nonexistent Core helper.
- Structural `isinstance()` checks are useful but insufficient; semantic behavior,
  errors, state transitions, and cleanup need executable assertions.
- A real Adapter should retain these portable tests and add native simulator tests
  for allocation, actual control, observations, rendering, and repeated cleanup.
- Negative tests are first-class evidence because fail-closed behavior is part of
  backend portability.

## Common failures

- Running with an interpreter that does not have both Core and the teaching
  distribution installed causes import or discovery failures.
- Calling bare `pytest` can use a different environment; use `python -m pytest`.
- Passing only structural Protocol checks does not prove command timing, units,
  batching, stale-handle rejection, or cleanup semantics.
- Do not describe this suite as native physics acceptance; the teaching Adapter is an
  in-memory contract implementation.

Previous: [05 — Entry-point discovery](../05_entry_point_discovery/README.md).
Next: [07 — Clean wheel gate](../07_clean_wheel_gate/README.md).
