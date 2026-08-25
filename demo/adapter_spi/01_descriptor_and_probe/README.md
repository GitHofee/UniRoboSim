# 01 — Descriptor and probe

English | [简体中文](README.zh-CN.md)

**Audience:** a backend developer defining the first public surface of an Adapter.
**Verification:** executable public-contract check; no simulator is launched.

## Run

Complete the [one-time setup](../README.md#one-time-setup), then run from the
repository root:

```bash
python demo/adapter_spi/01_descriptor_and_probe/main.py
```

Expected output:

```text
provider=example.teaching
capabilities=4
probe_side_effect_free=true
```

## What the code does

1. [`main.py`](main.py) constructs `TeachingProvider`; this must be cheap enough for
   discovery and must not open a native client.
2. `isinstance(provider, Provider)` checks the public structural shape. It does not
   prove semantic correctness.
3. `probe()` reports availability and the exact immutable descriptor without changing
   `open_count`.
4. The assertions ensure the Adapter declares a supported World schema and keeps
   probe side-effect free.

Read [`adapter.py`](../teaching_adapter/src/unirobosim_teaching/adapter.py) next to the
program. `CAPABILITIES` contains only the profile, multi-environment, articulation
state, and articulation-position contracts that this teaching implementation really
supports.

## Common failures

- An invalid provider ID is rejected while building `ProviderDescriptor`; use a
  stable lowercase dotted identifier.
- Do not import a native SDK at module import time merely to compute availability.
- Do not advertise camera, rigid-body, or soft-matter capabilities because a vendor
  SDK could theoretically provide them. Declare only the behavior this Adapter has
  implemented and tested.

Next: [02 — Session lifecycle](../02_session_lifecycle/README.md).
