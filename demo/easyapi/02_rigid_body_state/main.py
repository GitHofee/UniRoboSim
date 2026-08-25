"""Apply a wrench and read portable rigid-body state."""

import math

from unirobosim import Sim
from unirobosim.testing import FakeProvider


def main() -> None:
    with Sim(
        provider=FakeProvider(),
        world_id="rigid-body-state",
        time_step_seconds=0.1,
        gravity_m_s2=(0.0, 0.0, 0.0),
    ) as sim:
        box = sim.add_box("box", mass_kg=2.0, position_m=(0.0, 0.0, 0.0))
        sim.start()

        box.apply_wrench(force_n=(2.0, 0.0, 0.0))
        tick = sim.step()
        state = box.state

        position_x = float(state.positions_m.rows()[0][0])
        velocity_x = float(state.linear_velocities_m_s.rows()[0][0])
        assert tick.step_index == 1
        assert math.isclose(position_x, 0.01)
        assert math.isclose(velocity_x, 0.1)
        assert state.orientations_xyzw.shape == (1, 4)
        print(f"tick={tick.step_index}")
        print(f"position_x_m={position_x:.6f}")
        print(f"velocity_x_m_s={velocity_x:.6f}")


if __name__ == "__main__":
    main()
