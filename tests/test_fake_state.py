from __future__ import annotations

import unittest

from tests.helpers import make_world_spec
from unirobosim import (
    ArrayValue,
    ArticulationCommand,
    CommandError,
    CommandMode,
    EntityPath,
    StaleHandleError,
    ValidationError,
)
from unirobosim.testing import FakeProvider


class FakeStateTests(unittest.TestCase):
    def setUp(self) -> None:
        self.session = FakeProvider().open()
        self.world = self.session.build(make_world_spec(environments=3, initial=(0.1, -0.2)))
        self.robot = self.world.resolve(EntityPath("/robot"))

    def tearDown(self) -> None:
        self.session.close()

    def _command(self, mode, rows, environments=None, degrees=None):
        self.world.apply_articulation_command(
            ArticulationCommand(
                handle=self.robot,
                mode=mode,
                targets=ArrayValue.from_rows(rows),
                environment_indices=environments,
                degree_of_freedom_indices=degrees,
            )
        )

    def test_initial_state_is_batch_first(self) -> None:
        state = self.world.read_articulation(self.robot)
        self.assertEqual(state.joint_positions.shape, (3, 2))
        self.assertEqual(state.joint_positions.rows(), ((0.1, -0.2),) * 3)
        self.assertEqual(state.joint_velocities.rows(), ((0.0, 0.0),) * 3)

    def test_position_command_preserves_selection_order(self) -> None:
        self._command(CommandMode.POSITION, ((2.0,), (1.0,)), environments=(2, 0), degrees=(1,))
        tick = self.world.step()
        state = self.world.read_articulation(self.robot)
        self.assertEqual(tick.step_index, 1)
        self.assertEqual(state.joint_positions.rows(), ((0.1, 1.0), (0.1, -0.2), (0.1, 2.0)))
        self.assertAlmostEqual(state.joint_velocities.rows()[0][1], 120.0)
        self.assertAlmostEqual(state.joint_velocities.rows()[2][1], 220.0)
        self.world.step()
        self.assertEqual(self.world.read_articulation(self.robot).joint_velocities.rows()[0][1], 0.0)

    def test_velocity_command_integrates_over_multiple_steps(self) -> None:
        self._command(CommandMode.VELOCITY, ((1.5, -2.0),) * 3)
        self.world.step(4)
        rows = self.world.read_articulation(self.robot).joint_positions.rows()
        for row in rows:
            self.assertAlmostEqual(row[0], 0.16)
            self.assertAlmostEqual(row[1], -0.28)
        self.assertAlmostEqual(self.world.tick.sim_time_seconds, 0.04)

    def test_effort_uses_documented_unit_mass_test_rule(self) -> None:
        self._command(CommandMode.EFFORT, ((10.0,),), environments=(1,), degrees=(0,))
        self.world.step(2)
        state = self.world.read_articulation(self.robot)
        self.assertAlmostEqual(state.joint_velocities.rows()[1][0], 0.2)
        self.assertAlmostEqual(state.joint_positions.rows()[1][0], 0.103)

    def test_partial_reset_restores_only_selected_environments(self) -> None:
        self._command(CommandMode.POSITION, ((1.0, 2.0),) * 3)
        self.world.step()
        result = self.world.reset((1,))
        self.assertEqual(result.environment_indices, (1,))
        self.assertEqual(result.tick.step_index, 1)
        rows = self.world.read_articulation(self.robot).joint_positions.rows()
        self.assertEqual(rows, ((1.0, 2.0), (0.1, -0.2), (1.0, 2.0)))
        self.world.step()
        rows = self.world.read_articulation(self.robot).joint_positions.rows()
        self.assertEqual(rows, ((1.0, 2.0), (0.1, -0.2), (1.0, 2.0)))

    def test_invalid_selection_and_shape_never_broadcast(self) -> None:
        invalid_commands = (
            ArticulationCommand(self.robot, CommandMode.POSITION, ArrayValue.from_rows(((1.0, 2.0),))),
            ArticulationCommand(
                self.robot,
                CommandMode.POSITION,
                ArrayValue.from_rows(((1.0,),)),
                environment_indices=(3,),
                degree_of_freedom_indices=(0,),
            ),
        )
        for command in invalid_commands:
            with self.subTest(command=command), self.assertRaises((CommandError, ValidationError)):
                self.world.apply_articulation_command(command)
        with self.assertRaises(ValidationError):
            self.world.reset((0, 0))
        with self.assertRaises(ValidationError):
            self.world.step(0)

    def test_rigid_body_cannot_be_read_as_articulation(self) -> None:
        ground = self.world.resolve(EntityPath("/ground"))
        with self.assertRaises(CommandError):
            self.world.read_articulation(ground)

    def test_old_handle_is_stale_after_rebuild(self) -> None:
        old_handle = self.robot
        self.world.close()
        self.world = self.session.build(make_world_spec(environments=3, initial=(0.1, -0.2)))
        with self.assertRaises(StaleHandleError):
            self.world.read_articulation(old_handle)

    def test_handle_from_another_session_is_stale(self) -> None:
        other_session = FakeProvider().open()
        other_world = other_session.build(make_world_spec(environments=3))
        try:
            with self.assertRaises(StaleHandleError):
                other_world.read_articulation(self.robot)
        finally:
            other_session.close()


if __name__ == "__main__":
    unittest.main()
