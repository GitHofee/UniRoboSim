"""Control a named joint on a non-robot articulated object."""

from unirobosim import Sim
from unirobosim.testing import FakeProvider


def main() -> None:
    with Sim(provider=FakeProvider(), world_id="articulation-control") as sim:
        cabinet = sim.add_articulation(
            "cabinet",
            joint_names=("door_hinge", "drawer_slide"),
            initial_positions=(0.1, -0.2),
        )
        sim.start()

        cabinet.command((0.6,), joints=("door_hinge",), mode="position")
        sim.step()
        positions = cabinet.state.joint_positions.rows()

        assert cabinet.joint_names == ("door_hinge", "drawer_slide")
        assert positions == ((0.6, -0.2),)
        print(f"joint_names={cabinet.joint_names}")
        print(f"joint_positions={positions}")


if __name__ == "__main__":
    main()
