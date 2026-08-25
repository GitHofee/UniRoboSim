# 02 — Session lifecycle and transactional build

English | [简体中文](README.zh-CN.md)

**Audience:** Adapter developers implementing native allocation and cleanup.
**Verification:** executable failure-injection and lifecycle check.

## Run

```bash
python demo/adapter_spi/02_session_lifecycle/main.py
```

Expected output:

```text
first_build=unirobosim.world.build_failed
second_build_generation=1
```

## What the code does

1. `TeachingProvider(build_failures=1)` injects a failure before native-resource
   commit. This represents a native scene-build error.
2. The first `session.build()` raises `WorldBuildError`; the Session must remain
   `OPEN`, with no half-built World attached.
3. The same immutable `WorldSpec` is retried. The second build commits generation 1
   and moves the Session to `READY`.
4. Closing the World returns the Session to `OPEN`; the `finally` block closes the
   Session even when a check fails.

The important implementation order in
[`TeachingSession.build()`](../teaching_adapter/src/unirobosim_teaching/adapter.py) is:
validate → allocate candidate resources → construct candidate World → commit Session
state. Never mutate the live Session before an operation that can still fail.

## Common failures

- A Session may own only one live World. A second build while `READY` must fail.
- Build failure must not force the application to throw away the Session.
- `close()` must be idempotent and must cascade to resources still owned by the
  Session.

Next: [03 — World and control](../03_world_and_control/README.md).
