"""Discover the installed Adapter and select it through EasyAPI."""

from importlib.metadata import entry_points

from unirobosim import Sim


def main() -> None:
    installed = {entry.name for entry in entry_points(group="unirobosim.backends")}
    assert "teaching" in installed, "install demo/adapter_spi/teaching_adapter first"

    with Sim(backend="teaching", world_id="entry-point-demo") as sim:
        arm = sim.add_articulation("arm", joint_names=("joint",))
        sim.start()
        arm.command((0.5,))
        sim.step()
        assert arm.state.joint_positions.rows() == ((0.5,),)
        assert sim.provider_descriptor.provider_id == "example.teaching"
        print(f"selected_provider={sim.provider_descriptor.provider_id}")
        print(f"joint_positions={arm.state.joint_positions.rows()}")


if __name__ == "__main__":
    main()
