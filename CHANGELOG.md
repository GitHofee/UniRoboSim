# Changelog

## 0.7.0 - 2026-08-19

- Publish the accepted EasyAPI, RuntimeAPI, scene-control, debug and asset-preparation
  feature set as the coordinated 0.7 release.
- Preserve `unirobosim.world/v0alpha4` as the tested wire contract; no schema
  migration is hidden in the package-version change.
- Align the Fake provider identity with the released Core package.

## 0.7.0a0 - 2026-08-19

- Separate provider format compatibility from semantic physics readiness through
  `asset.normalization@1`.
- Add strict normalization request, inspection, result, discovery, selection, error and provenance
  contracts without importing simulator or OpenUSD SDKs into core.
- Let EasyAPI prepare rigid bodies and generic articulations with one `asset_options` mapping;
  retain `conversion_options` as the rigid-body compatibility alias.
- Preserve ready native assets and `prebuilt_only` behavior without invoking a normalizer.

## 0.4.0a0 - 2026-08-18

- Add the `v0alpha4` camera entity and batch-first RGB `uint8` / depth `float32` sample contract.
- Add backend-neutral point-set and line-list debug primitives with stable IDs, layers, environment
  selection, primitive budgets, physics-step lifetimes, and structured publish reports.
- Add failure-isolated `DebugBus` fan-out plus native-world, canonical JSONL trace, and in-memory test sinks.
- Extend the Fake Reference Backend with deterministic camera samples and native debug lifecycle behavior.
- Preserve explicit `v0alpha1`, `v0alpha2`, and `v0alpha3` WorldSpec compatibility.

## 0.3.0a0 - 2026-08-18

- Add the `v0alpha3` rigid-body root-link pose/twist state contract.
- Add strict persistent world-frame force/torque commands with selected-environment semantics.
- Add aggregated normal-contact force and binary-contact state without overstating friction,
  manifold, impulse, or contact-point availability.
- Preserve explicit `v0alpha1` and `v0alpha2` WorldSpec compatibility.

## 0.2.0a0 - 2026-08-18

- Add the `v0alpha2` surface-deformable, volume-deformable, and fixed particle-fluid contracts.
- Add batch-first deformable/fluid state and strict point position, velocity, and force commands.
- Extend the Fake Reference Backend with deterministic independent point-mass dynamics, partial
  reset, kinematic-node rules, capability limitations, and randomized conformance coverage.
- Preserve explicit `v0alpha1` compatibility for rigid/articulation-only worlds.

## 0.1.0a0 - 2026-08-18

- Add the `v0alpha1` backend-neutral foundation contract.
- Add portable immutable values, capability negotiation, world specifications, provider protocols,
  and a provider registry.
- Add the deterministic Fake Reference Backend and zero-dependency conformance suite.
