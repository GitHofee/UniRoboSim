# 06 — Capabilities and provider

[English](README.md) | [简体中文](README.zh-CN.md)

## Audience

For developers who want an application to fail early when a backend cannot provide required behavior.

## Prerequisites

Understand the build lifecycle from case 01.

```bash
PYTHONPATH=src python demo/easyapi/06_capabilities_and_provider/main.py
```

## Expected result

The selected provider is `reference.fake`. The required state capability and optional demo capability are both printed with their required flags.

## Code walkthrough

`require()` declares behavior without which the application must not run. `optional()` records a preference that may remain unmatched. The reason strings explain intent in diagnostics and locked world descriptions. `start()` negotiates these requirements with the selected provider. `provider_descriptor` then exposes stable provider identity and capabilities; `world_spec.requirements` preserves the compiled declarations.

## Common errors

- Capability IDs include an explicit contract version such as `@1`.
- Declaring the same capability twice is rejected.
- Core automatically includes `profile.core-robotics@1`; do not assume only your two declarations exist.

## Verification level

Dependency-free capability declaration and provider-inspection contract test.

## Next

Continue to [07 — Asset bundle](../07_asset_bundle/README.md).
