"""Read typed camera channels and preserve packed RGB storage."""

from unirobosim import CameraModality, Sim
from unirobosim.testing import FakeProvider


def main() -> None:
    with Sim(provider=FakeProvider(), world_id="camera-observations", num_envs=2) as sim:
        camera = sim.add_camera(
            "camera",
            resolution=(8, 6),
            outputs=("rgb", "depth", "normals"),
        )
        sim.start()
        sim.step()

        sample = camera.sample()
        rgb = sample.channel(CameraModality.RGB)
        depth = sample.channel(CameraModality.DEPTH)
        normals = camera.read("normals")
        rgb_bytes = rgb.to_bytes()

        assert rgb.shape == (2, 6, 8, 3)
        assert depth.shape == (2, 6, 8)
        assert normals.shape == (2, 6, 8, 3)
        assert rgb.dtype == "uint8" and rgb.is_packed
        assert len(rgb_bytes) == 2 * 6 * 8 * 3
        print(f"rgb_shape={rgb.shape} packed={rgb.is_packed} bytes={len(rgb_bytes)}")
        print(f"depth_shape={depth.shape} normals_shape={normals.shape}")


if __name__ == "__main__":
    main()
