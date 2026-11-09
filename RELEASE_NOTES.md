# UniRoboSim 0.10.13 (candidate)

Adds typed passive compliant connections between rigid scene objects, with required
backend capability negotiation, finite SI property validation and deterministic
world serialization. Empty declarations preserve existing output. Includes the
previously released render visibility API. Rejects overflowing numeric constants
with ValidationError and normalizes equal integer/float inputs identically.

Software validation is recorded in the integration report. Native connector and
full phone-task acceptance are separate gates; this candidate is not yet published.

# UniRoboSim 0.10.12

Adds the optional typed render.visibility.apply@1 contract for explicit visibility changes during strict render replay. Immutable requests identify an entity and exact relative prim, with expected invisible to inherited transition; receipts carry authored/effective before and after values. Existing defaults and physical replay identity contracts remain unchanged. CPU contract/import/release checks: 21 passed. Backend GPU behavior is separately gated.

# UniRoboSim 0.10.11

Adds WorldSpec elastic material, permanent deformable mounts and explicit contact/rest offsets.
Mode-specific capability gates prevent solver/boundary substitution; separate offset gates prevent silent ignoring.
Existing default serialization remains unchanged. New typed fields require Core >=0.10.11,<0.11 on both host and worker.

Mounted node initial velocities are entity-local relative velocities composed with the restored parent twist. Backing nodes reject nonzero relative initial velocities. Material IDs and inline elastic material are exclusive. Overflow/nonfinite/bool scalars return ValidationError.

See README for exact capability IDs, velocity frames and native damping limitations. EasyAPI does not expose these fields yet. CPU contract tests do not establish native solver reaction or GPU reset correctness; adapter conformance is required. No Mission/cuRobo or SDK code changes are included.
