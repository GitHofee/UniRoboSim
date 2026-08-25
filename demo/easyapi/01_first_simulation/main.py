"""Create, start, inspect, and close one EasyAPI simulation."""

from unirobosim import Sim, SimState
from unirobosim.testing import FakeProvider


def main() -> None:
    sim = Sim(provider=FakeProvider(), world_id="first-simulation")

    with sim:
        box = sim.add_box("box", position_m=(0.0, 0.0, 0.5))
        report = sim.start()
        positions = box.state.positions_m.rows()

        assert sim.state is SimState.RUNNING
        assert report.entity_count == 1
        assert positions == ((0.0, 0.0, 0.5),)
        print(f"provider={sim.provider_descriptor.provider_id}")
        print(f"box_positions_m={positions}")

    assert sim.state is SimState.CLOSED
    print("simulation_closed=true")


if __name__ == "__main__":
    main()
