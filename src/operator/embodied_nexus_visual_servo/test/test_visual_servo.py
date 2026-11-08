"""Tests for independent visual servo calculations."""

import math
import unittest

import numpy as np

from embodied_nexus_visual_servo.control import JointVelocityTracker, angular_velocity, cartesian_velocity, map_servo_velocities
from embodied_nexus_visual_servo.perception import (
    estimate_clipped_geometry, estimate_geometry, masked_depth_points, select_target, validate_frame_pair)


class VisualServoTest(unittest.TestCase):
    """Verify target geometry, time gating, and velocity handling."""

    def test_target_selection_rejects_unknown(self):
        """Select configured targets and reject unknown or unsupported shapes.

        Returns:
            None.
        """
        targets = {"green cylinder": {"shape": "cylinder"}}
        self.assertEqual(select_target(targets, "green cylinder")["shape"], "cylinder")
        with self.assertRaises(ValueError):
            select_target(targets, "unknown")

    def test_freshness_and_clock_regression(self):
        """Require matched stamps, fresh data and a non-regressing receive clock.

        Returns:
            None.
        """
        validate_frame_pair(2.0, 2.03, 10.0, 10.1, 10.2, 0.08, 0.75)
        for values in ((2, 2.2, 10, 10, 10.2), (2, 2, 10, 10, 11), (2, 2, 10, 10, 9)):
            with self.assertRaises(ValueError):
                validate_frame_pair(*values, 0.08, 0.75)

    def test_cylinder_geometry_from_ground(self):
        """Estimate a horizontal cylinder's height, center and principal axis.

        Returns:
            None.
        """
        x, y = np.meshgrid(np.linspace(0.83, 0.93, 40), np.linspace(-0.015, 0.015, 20))
        points = np.column_stack((x.ravel(), y.ravel(), np.full(x.size, -0.196)))
        estimate = estimate_geometry(points, -0.236, "cylinder")
        self.assertAlmostEqual(estimate["center"][0], 0.88, places=3)
        self.assertAlmostEqual(estimate["center"][2], -0.216, places=3)
        self.assertAlmostEqual(estimate["size"][1], 0.04, places=3)
        self.assertAlmostEqual(abs(math.cos(estimate["yaw"])), 1.0, places=3)

    def test_depth_projection_honors_color_extrinsic(self):
        """Use color-depth extrinsics to select depth points within the mask.

        Returns:
            None.
        """
        depth = np.ones((7, 7), dtype=float)
        mask = np.zeros((7, 7), dtype=bool)
        mask[2:5, 2:5] = True
        transform = np.eye(4)
        transform[0, 3] = 1.0
        points = masked_depth_points(depth, mask, [1, 1, 0, 0], [1, 1, 0, 0],
                                     transform, minimum_points=5)
        self.assertEqual(len(points), 9)
        self.assertEqual(set(points[:, 0]), {1.0, 2.0, 3.0})

    def test_clipped_target_requires_matching_visible_endpoint(self):
        """Track a clipped cylinder only when a previously observed end matches.

        Returns:
            None.
        """
        x, y = np.meshgrid(np.linspace(0.83, 0.89, 40), np.linspace(-0.015, 0.015, 20))
        points = np.column_stack((x.ravel(), y.ravel(), np.full(x.size, 0.04)))
        previous = {"center": [0.88, 0, 0.02], "size": [0.10, 0.04, 0.04], "yaw": 0.0}
        estimate = estimate_clipped_geometry(points, 0.0, previous)
        self.assertAlmostEqual(estimate["center"][0], 0.88, places=2)
        points[:, 0] += 0.1
        with self.assertRaises(ValueError):
            estimate_clipped_geometry(points, 0.0, previous)

    def test_velocity_limits_and_arrival(self):
        """Bound approach speed and produce zero velocity after convergence.

        Returns:
            None.
        """
        speed, reached = cartesian_velocity([1, 0, 0], [0, 0, 0], 1, 0.03, 0.005)
        self.assertFalse(reached)
        self.assertAlmostEqual(speed[0], 0.03)
        self.assertEqual(cartesian_velocity([0, 0, 0], [0, 0, 0], 1, 0.03, 0.005),
                         ([0.0, 0.0, 0.0], True))
        self.assertEqual(angular_velocity([0, 0, 0, -1], [0, 0, 0, 1], 1, 0.1), [0, 0, 0])
        self.assertLessEqual(np.linalg.norm(angular_velocity([0, 0, 1, 0], [0, 0, 0, 1], 1, 0.1)), 0.1)

    def test_joint_mapping_rejects_bad_commands(self):
        """Reorder velocities and reject missing joints, NaNs and excess speed.

        Returns:
            None.
        """
        self.assertEqual(map_servo_velocities(["b", "a"], [0.1, -0.1], ["a", "b"], 0.2),
                         [-0.1, 0.1])
        for names, values in ((["a"], [0.1]), (["a", "b"], [float("nan"), 0]),
                              (["a", "b"], [0.3, 0])):
            with self.assertRaises(ValueError):
                map_servo_velocities(names, values, ["a", "b"], 0.2)

    def test_joint_tracker_corrects_drift_and_limits_output(self):
        """Correct measured drift while respecting the device speed limit.

        Returns:
            None.
        """
        tracker = JointVelocityTracker([0.0, 0.0], gain=4.0, limit=0.2)
        self.assertLess(tracker.step([0.03, 0.0], [0.0, 0.0], 0.1)[0], 0.0)
        self.assertLessEqual(max(abs(x) for x in tracker.step([0.10, 0.0], [0.0, 0.0], 0.1)), 0.2)
        with self.assertRaises(RuntimeError):
            tracker.step([0.2, 0.0], [0.0, 0.0], 0.1)


if __name__ == "__main__":
    unittest.main()
