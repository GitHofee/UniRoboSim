from __future__ import annotations

import random
import unittest

from tests.helpers import make_world_spec
from unirobosim import ArrayValue, ArticulationCommand, CommandMode, EntityPath
from unirobosim.testing import FakeProvider


class RandomizedStateMachineTests(unittest.TestCase):
    def _run(self, seed: int):
        randomizer = random.Random(seed)
        environments = 4
        degrees = 3
        time_step = 0.01
        initial = (0.1, -0.2, 0.3)
        session = FakeProvider().open()
        world = session.build(make_world_spec(environments=environments, initial=initial))
        handle = world.resolve(EntityPath("/robot"))

        positions = [list(initial) for _ in range(environments)]
        velocities = [[0.0] * degrees for _ in range(environments)]
        modes = [[CommandMode.POSITION] * degrees for _ in range(environments)]
        targets = [list(initial) for _ in range(environments)]

        try:
            for action_index in range(300):
                action = randomizer.choice(("command", "step", "reset"))
                if action == "command":
                    environment_selection = tuple(
                        randomizer.sample(range(environments), randomizer.randint(1, environments))
                    )
                    degree_selection = tuple(randomizer.sample(range(degrees), randomizer.randint(1, degrees)))
                    mode = randomizer.choice(tuple(CommandMode))
                    rows = tuple(
                        tuple(randomizer.uniform(-2.0, 2.0) for _ in degree_selection) for _ in environment_selection
                    )
                    world.apply_articulation_command(
                        ArticulationCommand(
                            handle,
                            mode,
                            ArrayValue.from_rows(rows),
                            environment_selection,
                            degree_selection,
                        )
                    )
                    for row_index, environment in enumerate(environment_selection):
                        for column_index, degree in enumerate(degree_selection):
                            modes[environment][degree] = mode
                            targets[environment][degree] = rows[row_index][column_index]
                elif action == "reset":
                    selection = tuple(randomizer.sample(range(environments), randomizer.randint(1, environments)))
                    world.reset(selection)
                    for environment in selection:
                        positions[environment] = list(initial)
                        velocities[environment] = [0.0] * degrees
                        modes[environment] = [CommandMode.POSITION] * degrees
                        targets[environment] = list(initial)
                else:
                    count = randomizer.randint(1, 3)
                    world.step(count)
                    for _ in range(count):
                        for environment in range(environments):
                            for degree in range(degrees):
                                mode = modes[environment][degree]
                                target = targets[environment][degree]
                                if mode is CommandMode.POSITION:
                                    previous = positions[environment][degree]
                                    positions[environment][degree] = target
                                    velocities[environment][degree] = (target - previous) / time_step
                                elif mode is CommandMode.VELOCITY:
                                    velocities[environment][degree] = target
                                    positions[environment][degree] += target * time_step
                                else:
                                    velocities[environment][degree] += target * time_step
                                    positions[environment][degree] += velocities[environment][degree] * time_step

                state = world.read_articulation(handle)
                actual_positions = state.joint_positions.rows()
                actual_velocities = state.joint_velocities.rows()
                for environment in range(environments):
                    for degree in range(degrees):
                        self.assertAlmostEqual(
                            actual_positions[environment][degree],
                            positions[environment][degree],
                            places=12,
                            msg=f"position mismatch at action {action_index}",
                        )
                        self.assertAlmostEqual(
                            actual_velocities[environment][degree],
                            velocities[environment][degree],
                            places=12,
                            msg=f"velocity mismatch at action {action_index}",
                        )
            return state, world.tick
        finally:
            session.close()

    def test_fixed_seed_matches_independent_reference_model(self) -> None:
        first_state, first_tick = self._run(20260818)
        second_state, second_tick = self._run(20260818)
        self.assertEqual(first_state, second_state)
        self.assertEqual(first_tick, second_tick)

    def test_multiple_seeds_match_reference_model(self) -> None:
        for seed in range(8):
            with self.subTest(seed=seed):
                self._run(seed)


if __name__ == "__main__":
    unittest.main()
