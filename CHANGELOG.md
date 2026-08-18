# Changelog

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
