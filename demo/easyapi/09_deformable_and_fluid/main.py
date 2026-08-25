"""Command deformable nodes and particle-fluid particles."""

import math

from unirobosim import Sim
from unirobosim.testing import FakeProvider


def main() -> None:
    with Sim(
        provider=FakeProvider(),
        world_id="deformable-and-fluid",
        time_step_seconds=0.1,
        gravity_m_s2=(0.0, 0.0, 0.0),
    ) as sim:
        cloth = sim.add_deformable(
            "cloth",
            rest_positions_m=((0.0, 0.0, 1.0), (1.0, 0.0, 1.0), (0.0, 1.0, 1.0)),
            surface_triangles=((0, 1, 2),),
            kinematic_nodes=(0,),
        )
        water = sim.add_particle_fluid(
            "water",
            positions_m=((0.0, 0.0, 1.0), (0.02, 0.0, 1.0)),
            particle_radius_m=0.01,
        )
        sim.start()

        cloth.command((0.1, 0.2, 1.2), nodes=(0,), mode="position")
        water.command((1.0, 0.0, 0.0), mode="velocity")
        sim.step()

        cloth_node = cloth.state.node_positions_m.nested()[0][0]
        water_particle = water.state.particle_positions_m.nested()[0][0]
        requirement_ids = {item.capability.value for item in sim.world_spec.requirements}
        assert cloth_node == (0.1, 0.2, 1.2)
        assert math.isclose(float(water_particle[0]), 0.1)
        assert "control.deformable.points@1" in requirement_ids
        assert "control.fluid.particles@1" in requirement_ids
        print(f"cloth_node_0_m={cloth_node}")
        print(f"water_particle_0_m={water_particle}")


if __name__ == "__main__":
    main()
