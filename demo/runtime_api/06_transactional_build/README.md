# 06 - Transactional build and stale handles

[中文](README.zh-CN.md) | [Runtime API index](../README.md)

## Audience

This case is for framework authors implementing retries, worker recovery, or long-
running services where a failed build must not poison the simulator connection.

## What this demonstrates

FakeProvider injects one build failure. The Session remains open and can retry.
After the first successful World is closed, rebuilding advances its generation,
and the new World rejects the old entity handle.

## Prerequisites

- Understand explicit lifecycle from case 01
- UniRoboSim Core 0.10; no native simulator is required

## Run

```bash
PYTHONPATH=src python demo/runtime_api/06_transactional_build/main.py
```

## Expected result

```text
first_build=failed operation=session.build session=open
retry=passed old_handle=stale generation=1->2
```

## Code walkthrough

1. `FakeProvider(build_failures=1)` is a deterministic fault injector.
2. The first `session.build()` raises `WorldBuildError` before committing a World.
3. `SessionState.OPEN` proves the failure is transactional and retryable.
4. The next build succeeds with generation 1 and creates a handle for `/arm`.
5. Closing that World returns the Session to `OPEN`; rebuilding produces generation 2.
6. Reading with the generation-1 handle raises `StaleHandleError`, preventing a
   framework from silently controlling a replacement entity.

## Common failures

- Do not discard a healthy Session automatically for every `WorldBuildError`;
  inspect the structured error and the documented adapter policy.
- A Session cannot build while its previous World is live.
- Reset does not create a new World generation. Close plus rebuild does.
- Resolve new handles after every rebuild; never patch generation fields yourself.

## Verification level

This is a deterministic Core transaction and handle-identity test. Native adapters
must separately inject failures around their own allocations and prove cleanup.

## Next

Continue with [07 - verified build input](../07_verified_build_input/README.md).
