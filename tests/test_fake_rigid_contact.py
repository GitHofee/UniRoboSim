from __future__ import annotations

import math
import unittest

from unirobosim import (
    ArrayValue,
    CommandError,
    ContactState,
    EntityKind,
    EntityPath,
    EntitySpec,
    EnvironmentSpec,
    PhysicsSpec,
    Pose,
    RigidBodyCommand,
    RigidBodyState,
    StaleHandleError,
    Tick,
    ValidationError,
    WorldSpec,
)
from unirobosim.testing import FakeProvider


class FakeRigidContactTests(unittest.TestCase):
    def setUp(self) -> None:
        self.session = FakeProvider().open()
        self.spec = WorldSpec(
            "rigid-contract",
            (
                EntitySpec(EntityPath("/box"), EntityKind.RIGID_BODY, pose=Pose(position=(0.0, 0.0, 1.0))),
                EntitySpec(EntityPath("/robot"), EntityKind.ARTICULATION, joint_names=("joint",)),
            ),
            environments=EnvironmentSpec(3),
            physics=PhysicsSpec(time_step_seconds=0.01, gravity_m_s2=(0.0, 0.0, 0.0)),
        )
        self.world = self.session.build(self.spec)
        self.box = self.world.resolve(EntityPath("/box"))

    def tearDown(self) -> None:
        self.session.close()

    def test_initial_state_is_owned_batch_first_root_link_state(self) -> None:
        state = self.world.read_rigid_body(self.box)
        self.assertIsInstance(state, RigidBodyState)
        self.assertEqual(state.positions_m.shape, (3, 3))
        self.assertEqual(state.orientations_xyzw.shape, (3, 4))
        self.assertEqual(state.positions_m.rows(), ((0.0, 0.0, 1.0),) * 3)
        self.assertEqual(state.orientations_xyzw.rows(), ((0.0, 0.0, 0.0, 1.0),) * 3)
        self.assertEqual(state.linear_velocities_m_s.rows(), ((0.0, 0.0, 0.0),) * 3)

    def test_persistent_wrench_preserves_selection_order_and_partial_reset(self) -> None:
        self.world.apply_rigid_body_command(
            RigidBodyCommand(
                self.box,
                ArrayValue.from_rows(((2.0, 0.0, 0.0), (1.0, 0.0, 0.0))),
                ArrayValue.from_rows(((0.0, 0.0, 2.0), (0.0, 0.0, 3.0))),
                environment_indices=(2, 0),
            )
        )
        self.world.step(2)
        state = self.world.read_rigid_body(self.box)
        self.assertAlmostEqual(state.linear_velocities_m_s.rows()[2][0], 0.04)
        self.assertAlmostEqual(state.linear_velocities_m_s.rows()[0][0], 0.02)
        self.assertEqual(state.linear_velocities_m_s.rows()[1], (0.0, 0.0, 0.0))
        self.assertAlmostEqual(state.positions_m.rows()[2][0], 0.0006)
        self.assertAlmostEqual(state.positions_m.rows()[0][0], 0.0003)
        self.assertAlmostEqual(state.angular_velocities_rad_s.rows()[0][2], 0.06)
        self.assertNotEqual(state.orientations_xyzw.rows()[0], (0.0, 0.0, 0.0, 1.0))

        self.world.reset((2,))
        self.world.step()
        reset_state = self.world.read_rigid_body(self.box)
        self.assertEqual(reset_state.positions_m.rows()[2], (0.0, 0.0, 1.0))
        self.assertEqual(reset_state.linear_velocities_m_s.rows()[2], (0.0, 0.0, 0.0))
        self.assertAlmostEqual(reset_state.linear_velocities_m_s.rows()[0][0], 0.03)

    def test_contact_is_explicitly_zero_for_collision_free_fake(self) -> None:
        contact = self.world.read_contact(self.box, force_threshold_n=0.0)
        self.assertIsInstance(contact, ContactState)
        self.assertEqual(contact.net_normal_forces_n.shape, (3, 3))
        self.assertEqual(contact.net_normal_forces_n.rows(), ((0.0, 0.0, 0.0),) * 3)
        self.assertEqual(contact.in_contact.shape, (3,))
        self.assertEqual(contact.in_contact.values, (False, False, False))

    def test_invalid_types_shapes_thresholds_and_kinds_fail_before_mutation(self) -> None:
        with self.assertRaises(CommandError):
            self.world.apply_rigid_body_command(object())  # type: ignore[arg-type]
        with self.assertRaises(CommandError):
            self.world.apply_rigid_body_command(
                RigidBodyCommand(
                    self.box,
                    ArrayValue.from_rows(((1.0, 0.0, 0.0),)),
                    ArrayValue.from_rows(((0.0, 0.0, 0.0),)),
                )
            )
        with self.assertRaises(ValidationError):
            RigidBodyCommand(
                self.box,
                ArrayValue.from_rows(((1.0, 0.0, 0.0),)),
                ArrayValue.from_rows(((0.0, 0.0),)),
            )
        with self.assertRaises(ValidationError):
            self.world.read_contact(self.box, -1.0)
        with self.assertRaises(ValidationError):
            self.world.read_contact(self.box, math.nan)
        robot = self.world.resolve(EntityPath("/robot"))
        with self.assertRaises(CommandError):
            self.world.read_rigid_body(robot)
        with self.assertRaises(CommandError):
            self.world.read_contact(robot)

    def test_state_value_validation_and_stale_handles(self) -> None:
        tick = Tick(0, 0.0)
        with self.assertRaises(ValidationError):
            RigidBodyState(
                ArrayValue.from_rows(((0.0, 0.0, 0.0),)),
                ArrayValue.from_rows(((0.0, 0.0, 0.0, 2.0),)),
                ArrayValue.from_rows(((0.0, 0.0, 0.0),)),
                ArrayValue.from_rows(((0.0, 0.0, 0.0),)),
                tick,
            )
        with self.assertRaises(ValidationError):
            ContactState(
                ArrayValue.from_rows(((0.0, 0.0, 0.0),)),
                ArrayValue((2,), (False, False), dtype="bool"),
                tick,
            )
        old_handle = self.box
        self.world.close()
        self.world = self.session.build(self.spec)
        with self.assertRaises(StaleHandleError):
            self.world.read_rigid_body(old_handle)


if __name__ == "__main__":
    unittest.main()
