# 04 — Capability honesty

English | [简体中文](README.zh-CN.md)

**Audience:** Adapter developers defining capability negotiation and unsupported
endpoint behavior.
**Verification:** executable negative public-contract check.

## Run

Complete the [one-time setup](../README.md#one-time-setup), then run from the
repository root:

```bash
python demo/adapter_spi/04_capability_honesty/main.py
```

Expected output:

```text
build_rejected=unirobosim.capability.negotiation_failed
endpoint_rejected=unirobosim.capability.unsupported
```

## What the code does

1. [`main.py`](main.py) declares one programmatic articulation and a required
   `sensor.camera.rgb@1` capability.
2. `session.build()` negotiates the complete immutable World before allocating a
   World. The teaching Adapter has no camera implementation, so it raises structured
   `CapabilityNegotiationError` with backend identity intact.
3. The same still-open Session builds a supported articulation-only World, proving
   the rejected build was transactional.
4. The program deliberately calls `world.read_sensor(handle)`. The base World shape
   includes that method, but this Adapter raises `UnsupportedCapabilityError` with
   detail `sensor.camera@1` rather than inventing an empty image.
5. The World and Session are closed after both negative paths are verified.

## Implementation principles

- A descriptor is a tested promise, not a list of everything the vendor SDK might
  theoretically support.
- Reject unsupported required capabilities before native allocation.
- Keep every base World endpoint present, but fail unsupported operations with a
  structured public error carrying operation, backend, World, and capability context.
- Never approximate a missing camera, contact, soft-body, or fluid result silently.

## Common failures

- Advertising a capability before its lifecycle, validation, state, and cleanup paths
  pass tests creates false portability.
- Returning `None`, zero arrays, or placeholder pixels hides unsupported behavior.
- Throwing a vendor exception leaks backend details and breaks portable error handling.
- A failed negotiation must leave the Session `OPEN`, not partially `READY`.

Previous: [03 — World and control](../03_world_and_control/README.md).
Next: [05 — Entry-point discovery](../05_entry_point_discovery/README.md).
