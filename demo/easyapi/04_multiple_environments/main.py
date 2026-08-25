"""Send commands to selected environments and reset one environment."""

import math

from unirobosim import Sim
from unirobosim.testing import FakeProvider


def main() -> None:
    with Sim(
        provider=FakeProvider(),
        world_id="multiple-environments",
        num_envs=3,
        time_step_seconds=0.1,
        gravity_m_s2=(0.0, 0.0, 0.0),
    ) as sim:
        box = sim.add_box("box", position_m=(0.0, 0.0, 0.0))
        arm = sim.add_articulation("arm", joint_names=("joint",), initial_positions=(0.0,))
        sim.start()

        box.apply_wrench(((1.0, 0.0, 0.0), (3.0, 0.0, 0.0)), environments=(0, 2))
        arm.command(((0.2,), (0.8,)), environments=(0, 2))
        sim.step()

        position_x = tuple(float(row[0]) for row in box.state.positions_m.rows())
        before_reset = arm.state.joint_positions.rows()
        reset = sim.reset((2,))
        after_reset = arm.state.joint_positions.rows()

        assert all(
            math.isclose(actual, expected) for actual, expected in zip(position_x, (0.01, 0.0, 0.03), strict=True)
        )
        assert before_reset == ((0.2,), (0.0,), (0.8,))
        assert reset.environment_indices == (2,)
        assert after_reset == ((0.2,), (0.0,), (0.0,))
        print(f"position_x_m={position_x}")
        print(f"joints_before_reset={before_reset}")
        print(f"joints_after_reset={after_reset}")


if __name__ == "__main__":
    main()
