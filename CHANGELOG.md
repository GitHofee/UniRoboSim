# Changelog

## 0.10.7 - 2026-09-16

- Publish immutable point-closure descriptors with SI anchors, constraint hashes and exact coupled tree-joint paths.
- Use planning catalog v3 for nonempty closures and preserve empty v2 hashes and serialized bytes.
- Validate topology and descriptor integrity across worker transport.

## 0.10.5 - 2026-09-02

- Add the backend-neutral `checkpoint@1` capability and opaque physical-world
  checkpoint contracts.
- Restore rigid, articulation, deformable, fluid, control-target, and attachment
  state without rewinding the live simulation clock.
- Keep checkpoint support optional so worlds that never request it gain no per-tick
  work.

## 0.10.4 - 2026-09-01

- Define entity pose as the imported asset/model root frame; USD adapters expose
  the spawned entity Prim rather than an inferred articulation root link.
- Allow entity-level pose commands for articulations while preserving every
  physical body's relative transform and the current joint state.
- Keep physical root and child-link state available through explicit link state.

## 0.10.3 - 2026-09-01

- Add immutable triangle-mesh debug resources and batched mesh instances with
  non-uniform scale and solid, wireframe, or solid-with-edges presentation.
- Keep mesh topology on a cold, content-addressed path so repeated frame updates
  carry transforms and appearance only.
- Preserve render-only semantics: debug meshes never become collision or planning
  geometry.

## 0.10.2 - 2026-09-01

- Allow positive XYZ scale on `composite_scene` entities and automatically require
  the additive `entity.scale.composite_scene@1` Provider capability for non-unit
  values.
- Expose the same value through
  `Sim.add_composite_scene(..., scale_xyz=(x, y, z))` in EasyAPI.
- Preserve fail-closed backend negotiation: Providers that do not implement
  physically consistent composite scaling reject the World before native allocation.
- Keep rigid/static-scene XYZ scale and standalone articulation uniform-scale
  semantics unchanged.

## 0.10.0 - 2026-08-24

- Add `unirobosim.world/v0alpha6`, the unit-scale `composite_scene` container, and
  capability-gated build-time embedded rigid-body/articulation bindings without
  changing v0alpha4 or v0alpha5 payloads.
- Keep composite resource closure digest-pinned through the existing
  `BuildResourceManifest`/`BuildInput` contract and require both
  `scene.composite@1` and `entity.embedded-binding@1` before native authoring.
- Extend EasyAPI and the Fake Reference Backend with compose-once, embedded
  resolve/state/control/reset/close conformance. Native adapter acceptance remains a
  separate release gate.

## 0.9.2 - 2026-08-24

- Add the physical-v0alpha5 `static_scene` entity with one required asset, positive
  scale, at-most-one-per-World validation, and automatic `scene.static@1` plus
  non-unit `entity.scale.static_scene@1` capability demand.
- Add typed camera mounts whose authored pose is parent-local, with closed World
  validation for parent identity and kind, and expose RGB, depth, and float32 XYZ
  normals consistently through Core, EasyAPI, and the Fake Reference Backend.
- Preserve every existing unmounted v0alpha4/v0alpha5 serialized payload while
  mapping static scenes to planning kind `other`; native adapter acceptance remains
  a separate verification gate.

## 0.9.1 - 2026-08-23

- Add compact immutable uint8 array storage for high-resolution RGB frames while
  preserving the existing `ArrayValue`, `SensorChannel`, and EasyAPI signatures.
- Keep legacy `ArrayValue.values` tuple access available as an explicit lazy
  compatibility path and add `to_bytes()` for allocation-free recording and IPC.
- Exercise the packed RGB contract in the Fake Reference Backend without changing
  the serialized World schema or the `sensor.camera.rgb@1` capability semantics.

## 0.9.0 - 2026-08-23

- Add the explicit `unirobosim.world/v0alpha5` physical schema while preserving
  v0alpha4 construction and the complete Core 0.8 Planning API.
- Add ordered articulation axis units, rigid/articulation scale capability demand,
  self-closing articulation state, selected-command units and command-time metre
  capability rejection before side effects.
- Add canonical `BuildResourceManifest` values and the keyword-only private
  `BuildInput` source carrier.
- Add typed Provider World-schema declarations and deterministic wheel/sdist mode
  normalization across build umasks.
- Native Isaac Lab, MuJoCo and PyBullet 0.9 adapter acceptance remains unverified in
  this Core-only release.

## 0.8.0 - 2026-08-23

- Publish the accepted backend-neutral `planning.scene@2` catalog, state, delta,
  frame, articulation-topology, attachment, and geometry-resource contracts.
- Export `PlanningGeometryResourceLayout`, `PlanningGeometryResourceDescriptor`,
  `PlanningSceneCatalog`, and the related resource/world protocols from both
  `unirobosim.api` and the top-level package.
- Keep planning geometry representation exact, catalog-pinned, hash-verified, and
  free of native backend handles or implicit representation fallback.
- Preserve `unirobosim.world/v0alpha4`; this Core-only release does not change a
  FastSim dependency pin or claim native GPU, GUI, or adapter acceptance.

## 0.7.1 - 2026-08-21

- Add the optional, capability-gated `runtime.diagnostics@1` provider endpoint for
  portable provider-owned world and native-client counts.
- Fail closed with structured causal errors when descriptor or diagnostics endpoint
  access violates the public contract.
- Define the optional `connection_modes` capability property as an enforced,
  non-empty allow-list while retaining declarations that omit the property.
- Preserve `unirobosim.world/v0alpha4`; this patch adds no serialized world-schema
  migration.

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
