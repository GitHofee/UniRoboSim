# 07 - Digest-pinned BuildInput

[中文](README.zh-CN.md) | [Runtime API index](../README.md)

## Audience

This advanced case is for framework and asset-pipeline developers handing local or
cached resources to an Adapter without allowing an asset to change between
validation and native build.

## What this demonstrates

The program creates a temporary asset, records its file identity and SHA-256,
constructs a canonical `BuildResourceManifest`, binds it to a physical `WorldSpec`,
and passes the matching `BuildInput` into `Session.build()`.

## Prerequisites

- Complete case 06
- Understand SHA-256 and local file identity
- UniRoboSim Core 0.10; no native simulator is required

## Run

```bash
PYTHONPATH=src python demo/runtime_api/07_verified_build_input/main.py
```

## Expected result

The digest prefixes are deterministic for the authored content and contract, and
the temporary directory is always removed:

```text
manifest=<12 hex characters> world=<12 hex characters> entities=1
temporary_asset=removed
```

## Code walkthrough

1. `TemporaryDirectory` supplies an isolated source root and removes it after use;
   no machine-specific absolute path is stored in the demo repository.
2. SHA-256 identifies the exact bytes expected by the manifest.
3. `BuildResourceEntry` records logical ownership, media type, role, purposes,
   canonical bundle path, byte size, and digest.
4. `LocalSourceIdentity` captures device, inode, mode, size, and nanosecond times
   from one regular file.
5. `BuildSourceEntry` maps the manifest resource to a source-root-relative file.
6. `BuildInput` requires exact one-to-one resource coverage.
7. A v0alpha5 asset-backed `WorldSpec` carries `manifest.sha256`; build accepts only
   the matching `BuildInput` and rechecks the source before native consumption.
8. The build fingerprint binds the resulting World to the immutable World digest.

## Common failures

- Asset-backed v0alpha5/v0alpha6 Worlds require
  `build_resource_manifest_sha256`.
- Every manifest resource needs exactly one canonical source entry.
- Relative paths are normalized POSIX paths and cannot contain `..` or backslashes.
- Changing, replacing, or relinking the file after identity capture causes build to
  fail instead of consuming unverified bytes.
- Do not print or persist `source_root` if logs must remain machine-independent.

## Verification level

This verifies the Core asset identity and build handoff against FakeProvider's
strict file recheck. It does not prove that a native adapter can parse the payload.

## Where to go next

You have completed the Runtime API path. Continue with the
[Adapter SPI demos](../../adapter_spi/README.md) to learn how a simulator implements
these contracts.
