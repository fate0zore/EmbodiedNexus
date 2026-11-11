"""Tests for dynamic target tracking command calculations."""

import unittest
import math

import numpy as np

from embodied_nexus_dynamic_grasp.control import target_grasp_orientation, tracking_velocity
from embodied_nexus_dynamic_grasp.tracking import _axis


class TrackingVelocityTest(unittest.TestCase):
    """Exercise feedforward, feedback and velocity limits."""

    def test_grasp_orientation_tracks_tilted_object(self):
        """Align the tool with the full three-dimensional target long axis.

        Returns:
            None.
        """
        tilt = math.pi / 4
        target = [0, math.sin(tilt / 2), 0, math.cos(tilt / 2)]
        tool = target_grasp_orientation(target, "box", [1, 0, 0, 0])
        x, y, z, w = tool
        tool_y = np.array([2*(x*y-z*w), 1-2*(x*x+z*z), 2*(y*z+x*w)])
        long_axis = np.array([math.cos(tilt), 0, -math.sin(tilt)])
        self.assertGreater(abs(float(np.dot(tool_y, long_axis))), 0.999)
        self.assertLess(1 - 2 * (x*x + y*y), 0.0)
        tool = target_grasp_orientation(target, "cylinder", [1, 0, 0, 0])
        x, y, z, w = tool
        tool_y = np.array([2*(x*y-z*w), 1-2*(x*x+z*z), 2*(y*z+x*w)])
        self.assertGreater(abs(float(np.dot(tool_y, _axis(target)))), 0.999)

    def test_upright_cylinder_has_defined_approach(self):
        """Keep a continuous tool orientation when the cylinder is vertical.

        Returns:
            None.
        """
        result = target_grasp_orientation([0, 0, 0, 1], "cylinder", [1, 0, 0, 0])
        self.assertAlmostEqual(float(np.linalg.norm(result)), 1.0)

    def test_combines_feedforward_and_position_error(self):
        """Add target feedforward to the offset pose feedback.

        Returns:
            None.
        """
        velocity, error = tracking_velocity([0.1, 0, 0], [0.01, 0, 0], [0, 0, 0.1],
                                           [0, 0, 0.12], 0.1, 1.0)
        np.testing.assert_allclose(error, [0.1, 0, 0.02])
        np.testing.assert_allclose(velocity, [0.02, 0, 0.002])

    def test_speed_is_bounded_as_a_vector(self):
        """Enforce a total speed bound without distorting the command direction.

        Returns:
            None.
        """
        velocity, _ = tracking_velocity([1, 1, 0], [0, 0, 0], [0, 0, 0],
                                        [0, 0, 0], 1.0, 0.01)
        self.assertAlmostEqual(np.linalg.norm(velocity), 0.01)
        self.assertAlmostEqual(velocity[0], velocity[1])

    def test_rejects_nonfinite_measurement(self):
        """Reject invalid measurements before producing a motion command.

        Returns:
            None.
        """
        with self.assertRaises(ValueError):
            tracking_velocity([float("nan"), 0, 0], [0, 0, 0], [0, 0, 0],
                              [0, 0, 0], 1.0, 0.01)
