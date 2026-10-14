# UniRoboSim 0.10.11

Adds WorldSpec elastic material, permanent deformable mounts and explicit contact/rest offsets.
Mode-specific capability gates prevent solver/boundary substitution; separate offset gates prevent silent ignoring.
Existing default serialization remains unchanged. New typed fields require Core >=0.10.11,<0.11 on both host and worker.

Mounted node initial velocities are entity-local relative velocities composed with the restored parent twist. Backing nodes reject nonzero relative initial velocities. Material IDs and inline elastic material are exclusive. Overflow/nonfinite/bool scalars return ValidationError.

See README for exact capability IDs, velocity frames and native damping limitations. EasyAPI does not expose these fields yet. CPU contract tests do not establish native solver reaction or GPU reset correctness; adapter conformance is required. No Mission/cuRobo or SDK code changes are included.
