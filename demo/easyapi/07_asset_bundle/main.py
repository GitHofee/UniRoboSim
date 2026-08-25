"""Resolve one logical asset from a digest-pinned manifest."""

from pathlib import Path

from unirobosim import AssetBundle, Sim
from unirobosim.testing import FakeProvider


def main() -> None:
    case_directory = Path(__file__).resolve().parent
    bundle = AssetBundle.from_manifest(case_directory / "demo_robot.asset.json")

    with Sim(provider=FakeProvider(), world_id="asset-bundle") as sim:
        sim.add_articulation("demo_robot", joint_names=("base_joint",), asset=bundle)
        sim.start()

        entity = sim.world_spec.entities[0]
        resolved = entity.metadata["unirobosim_asset"]
        assert entity.asset_uri == str((case_directory / "assets" / "demo_robot.urdf").resolve())
        assert resolved["selector"] == "fake"
        assert resolved["media_type"] == "model/vnd.urdf+xml"
        assert resolved["sha256"] == bundle.variants["fake"]["sha256"]
        print(f"logical_name={resolved['logical_name']}")
        print(f"selector={resolved['selector']}")
        print(f"asset_uri={entity.asset_uri}")
        print(f"sha256={resolved['sha256']}")


if __name__ == "__main__":
    main()
