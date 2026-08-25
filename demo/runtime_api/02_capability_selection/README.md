# 02 - Capability-based provider selection

[中文](README.zh-CN.md) | [Runtime API index](../README.md)

## Audience

This case is for framework authors who need to choose a backend without importing
simulator-specific packages in their framework code.

## What this demonstrates

`ProviderRegistry` stores descriptors and factories in one process. Registration
is lazy: listing descriptors does not instantiate a Provider. Selection checks
both `probe()` availability and versioned capability requirements, and a failed
selection returns structured attempts.

## Prerequisites

- Complete case 01
- UniRoboSim Core 0.10; no native simulator is required

## Run

```bash
PYTHONPATH=src python demo/runtime_api/02_capability_selection/main.py
```

## Expected result

```text
selected=reference.fake factory_calls=1
rejected=sensor.lidar@1 attempts=1
```

## Code walkthrough

1. `create_fake()` increments a counter so the example can prove lazy creation.
2. `register()` binds `FAKE_DESCRIPTOR` to that factory. The descriptor can now be
   inspected without importing or launching a native SDK.
3. `CapabilityRequirement(CapabilityId("state.articulation@1"))` expresses a hard
   requirement with an explicit major contract version.
4. `select()` creates, probes, negotiates, and returns the first accepted Provider.
5. The lidar requirement is intentionally unsupported. The caught
   `ProviderSelectionError.details["attempts"]` records why each candidate failed.

## Common failures

- `state.articulation` is invalid; capability IDs include a major version such as
  `state.articulation@1`.
- A registered descriptor must exactly equal the descriptor returned by its factory.
- Duplicate provider IDs are rejected.
- `ProviderRegistry` is explicit and process-local. Installed Adapter discovery is
  performed by EasyAPI through the `unirobosim.backends` entry-point group.

## Verification level

This verifies deterministic registry, availability, and capability-negotiation
semantics against FakeProvider. It does not prove an installed native Adapter is
available.

## Next

Continue with [03 - typed articulation command](../03_articulation_command/README.md).
