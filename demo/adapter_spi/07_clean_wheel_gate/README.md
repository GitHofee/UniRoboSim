# 07 — Clean wheel gate

English | [简体中文](README.zh-CN.md)

**Audience:** Adapter maintainers validating release artifacts rather than an editable
source checkout.
**Verification:** isolated Core-wheel plus Adapter-wheel installation and discovery
smoke test.

## Run

Use Python 3.11 or 3.12 on Linux. The current environment must already contain
`pip`, `venv`, setuptools, and wheel. From the repository root:

```bash
python demo/adapter_spi/07_clean_wheel_gate/main.py
```

Expected final output markers:

```text
clean_wheel_discovery=passed
clean_wheel_gate=passed
```

Wheel build and pip installation logs appear before these markers.

## What the code does

1. [`main.py`](main.py) locates the Core repository, teaching Adapter source, and
   [`discovery_smoke.py`](discovery_smoke.py) without depending on the current working
   directory.
2. `TemporaryDirectory` creates an isolated disposable root. The script builds one
   Core wheel and one Adapter wheel with `pip wheel --no-deps --no-build-isolation`.
3. It creates a new temporary venv, asserts that exactly two wheels were built, and
   installs only those local wheels with `pip install --no-index`.
4. Every subprocess receives a copied environment with `PYTHONPATH` explicitly
   removed. Source checkout paths therefore cannot make a broken wheel appear valid.
5. The clean interpreter runs `discovery_smoke.py`, which verifies distribution
   versions, installed entry-point metadata, `Sim(backend="teaching")`, and one
   articulation command/state loop.
6. Leaving `TemporaryDirectory` deletes the temporary wheels and venv. The final gate
   marker is printed only if all subprocesses succeeded.

## Implementation principles

- Release acceptance must test built artifacts in isolation; editable installs can
  hide missing packages, data, metadata, or entry points.
- Clear source-path injection such as `PYTHONPATH` at the clean-install boundary.
- Disable dependency downloads for the isolated install so the gate proves exactly
  which local artifacts are sufficient.
- Check both metadata discovery and executable semantics after installation.
- Real Adapter release automation should add its pinned native SDK installation and
  native smoke test to an equivalent clean environment.

## Common failures

- `--no-build-isolation` requires setuptools and wheel in the launching environment.
- A missing package in wheel configuration may work in editable mode but fail inside
  the temporary venv.
- Missing `[project.entry-points."unirobosim.backends"]` metadata makes the installed
  Adapter undiscoverable.
- The script intentionally clears `PYTHONPATH`; relying on repository imports is a
  packaging defect, not a reason to restore it.
- The temporary environment is deleted automatically, so copy external logs before
  the process exits if a CI system needs to retain them.

Previous: [06 — Public contract suite](../06_public_contract_suite/README.md).
Return to the [Adapter SPI learning path](../README.md).
