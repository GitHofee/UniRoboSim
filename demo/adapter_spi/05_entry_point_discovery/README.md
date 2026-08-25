# 05 — Entry-point discovery

English | [简体中文](README.zh-CN.md)

**Audience:** Adapter maintainers packaging an implementation for normal application
discovery.
**Verification:** installed-distribution discovery and EasyAPI selection check.

## Run

This case requires the teaching Adapter to be installed as a Python distribution;
adding its source directory to `PYTHONPATH` is not equivalent because entry-point
metadata comes from the installed distribution. From the repository root:

```bash
python -m pip install -e .
python -m pip install -e ./demo/adapter_spi/teaching_adapter
python demo/adapter_spi/05_entry_point_discovery/main.py
```

Expected output:

```text
selected_provider=example.teaching
joint_positions=((0.5,),)
```

## What the code does

1. [`teaching_adapter/pyproject.toml`](../teaching_adapter/pyproject.toml) registers
   entry-point name `teaching` in group `unirobosim.backends`, pointing to the public
   zero-argument `create_provider` factory.
2. [`main.py`](main.py) reads installed entry-point metadata and asserts that the
   `teaching` distribution entry exists.
3. `Sim(backend="teaching")` asks EasyAPI discovery to load that named factory, probe
   the returned Provider, and negotiate the scene requirements.
4. The rest is ordinary backend-neutral EasyAPI: declare an articulation, start,
   command, step, and read state.
5. The assertions prove both installed discovery and the selected descriptor identity.

## Implementation principles

- Entry-point names are concise user selectors; `ProviderDescriptor.provider_id` is
  the stable globally meaningful backend identity.
- The registered factory must take no arguments and must not start a simulator.
- Put vendor-specific configuration behind explicit provider construction or a
  documented launch profile, not hidden global discovery behavior.
- Keep package name, import package, entry-point name, provider ID, and documentation
  deliberate and consistent, even when they are not identical strings.

## Common failures

- Running without installing the teaching distribution triggers the explicit
  `install demo/adapter_spi/teaching_adapter first` assertion.
- Installing Core and the Adapter into different Python environments makes metadata
  invisible to the executing interpreter.
- A factory that imports or launches the native SDK during discovery makes backend
  selection slow and unsafe.

Previous: [04 — Capability honesty](../04_capability_honesty/README.md).
Next: [06 — Public contract suite](../06_public_contract_suite/README.md).
