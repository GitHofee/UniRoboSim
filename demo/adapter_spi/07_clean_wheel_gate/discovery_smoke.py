"""Runs inside the clean venv created by main.py."""

from importlib.metadata import entry_points, version

from unirobosim import Sim


def main() -> None:
    assert version("unirobosim") == "0.10.0"
    assert version("unirobosim-teaching") == "0.1.0"
    assert any(entry.name == "teaching" for entry in entry_points(group="unirobosim.backends"))
    with Sim(backend="teaching", world_id="clean-wheel-smoke") as sim:
        arm = sim.add_articulation("arm", joint_names=("joint",))
        sim.start()
        arm.command((0.25,))
        sim.step()
        assert arm.state.joint_positions.rows() == ((0.25,),)
    print("clean_wheel_discovery=passed")


if __name__ == "__main__":
    main()
