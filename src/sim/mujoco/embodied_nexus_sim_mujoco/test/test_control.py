"""Unit tests for MuJoCo adapter command semantics."""

import unittest

from embodied_nexus_sim_mujoco.control import make_trajectory, wheel_speeds


class ControlTests(unittest.TestCase):
    """Exercise controller mappings and invalid commands."""

    def test_wheel_speeds(self):
        """Map straight motion and turning to expected wheel directions.

        Returns:
            None.
        """
        left, right = wheel_speeds(0.16459, 0.0)
        self.assertAlmostEqual(left, 1.0)
        self.assertAlmostEqual(right, 1.0)
        left, right = wheel_speeds(0.0, 1.0)
        self.assertLess(left, 0.0)
        self.assertGreater(right, 0.0)

    def test_trajectory_uses_measured_start_and_reorders(self):
        """Interpolate from measurements and honor joint names.

        Returns:
            None.
        """
        trajectory = make_trajectory(["b", "a"], [4.0, 2.0], ["a", "b"], [0.0, 2.0], 3.0, 2.0)
        self.assertEqual(trajectory.sample(4.0), (1.0, 3.0))
        self.assertEqual(trajectory.sample(5.0), (2.0, 4.0))
        with self.assertRaises(ValueError):
            make_trajectory(["a", "a"], [1.0, 2.0], ["a", "b"], [0.0, 0.0], 0.0, 1.0)
