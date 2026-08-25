from __future__ import annotations

import hashlib
import tempfile
from pathlib import Path

from unirobosim import (
    PHYSICAL_WORLD_SCHEMA_VERSION,
    BuildInput,
    BuildResourceEntry,
    BuildResourceManifest,
    BuildSourceEntry,
    EntityKind,
    EntityPath,
    EntitySpec,
    LocalSourceIdentity,
    WorldSpec,
)
from unirobosim.testing import FakeProvider


def local_identity(path: Path) -> LocalSourceIdentity:
    stat = path.stat()
    return LocalSourceIdentity(
        device=stat.st_dev,
        inode=stat.st_ino,
        mode=stat.st_mode,
        byte_size=stat.st_size,
        mtime_ns=stat.st_mtime_ns,
        ctime_ns=stat.st_ctime_ns,
    )


def main() -> None:
    with tempfile.TemporaryDirectory(prefix="unirobosim-build-input-") as temporary_directory:
        root = Path(temporary_directory)
        asset = root / "arm.bin"
        payload = b"portable teaching asset\n"
        asset.write_bytes(payload)
        digest = hashlib.sha256(payload).hexdigest()

        resource = BuildResourceEntry(
            entity_id="entity.arm",
            component_id="arm.source",
            resource_id="resource.arm",
            role="simulation",
            media_type="application/octet-stream",
            requested_uri="asset://teaching-arm",
            resolved_uri="cache://teaching-arm",
            canonical_source_identity=f"sha256:{digest}",
            byte_size=len(payload),
            sha256=digest,
            selected_simulation_input=True,
            purposes=("collision", "planning", "simulation", "visual"),
            relative_bundle_path="assets/arm.bin",
        )
        manifest = BuildResourceManifest(entries=(resource,))
        build_input = BuildInput(
            manifest=manifest,
            sources=(
                BuildSourceEntry(
                    resource_id=resource.resource_id,
                    source_kind="local-file",
                    source_root=str(root),
                    relative_source_path=asset.name,
                    expected_identity=local_identity(asset),
                    expected_sha256=digest,
                ),
            ),
        )
        spec = WorldSpec(
            world_id="runtime-verified-asset",
            schema_version=PHYSICAL_WORLD_SCHEMA_VERSION,
            build_resource_manifest_sha256=manifest.sha256,
            entities=(
                EntitySpec(
                    path=EntityPath("/arm"),
                    kind=EntityKind.ARTICULATION,
                    joint_names=("joint",),
                    asset_uri="asset://teaching-arm",
                ),
            ),
        )

        session = FakeProvider().open()
        try:
            world = session.build(spec, build_input=build_input)
            try:
                assert world.build_report.fingerprint.world_digest == spec.digest
                assert spec.build_resource_manifest_sha256 == manifest.sha256
                print(
                    f"manifest={manifest.sha256[:12]} world={spec.digest[:12]} "
                    f"entities={world.build_report.entity_count}"
                )
            finally:
                world.close()
        finally:
            session.close()

    assert not root.exists()
    print("temporary_asset=removed")


if __name__ == "__main__":
    main()
