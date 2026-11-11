"""Tests for the independent dynamic target motion estimator."""

import math
import unittest

import numpy as np

from embodied_nexus_dynamic_grasp.tracking import MotionTracker, equivalent_orientation


class MotionTrackerTest(unittest.TestCase):
    """Exercise motion, stationarity, observation gaps and geometric symmetry."""

    def test_long_motion_does_not_treat_slow_drift_as_rest(self):
        """Require sustained low drift before classifying a long move as stopped.

        Returns:
            None.
        """
        tracker = MotionTracker('cylinder', window_sec=5.0, max_gap_sec=0.3,
                                linear_limit=0.001, position_limit=0.002, velocity_alpha=1.0)
        for index in range(101):
            stamp = index * 0.1
            result = tracker.update(stamp, [stamp * 0.002, 0, 0], [0, 0, 0, 1])
            self.assertFalse(result.stationary)
        for index in range(1, 52):
            result = tracker.update(10.0 + index * 0.1, [.02, 0, 0], [0, 0, 0, 1])
        self.assertTrue(result.stationary)

    def test_stationary_requires_continuous_window_and_motion_clears_it(self):
        """Require a complete observation window, and clear it on movement.

        Returns:
            None.
        """
        tracker = MotionTracker("box", max_gap_sec=0.3, velocity_alpha=1.0)
        for index in range(10):
            result = tracker.update(index * 0.1, [0, 0, 0], [0, 0, 0, 1])
            self.assertFalse(result.stationary)
        result = tracker.update(1.0, [0, 0, 0], [0, 0, 0, 1])
        self.assertTrue(result.stationary)
        result = tracker.update(1.1, [0.01, 0, 0], [0, 0, 0, 1])
        self.assertFalse(result.stationary)
        self.assertAlmostEqual(result.linear_velocity[0], 0.1)

    def test_gap_and_clock_reversal_clear_evidence(self):
        """Never carry stationary evidence across dropped frames or clock resets.

        Returns:
            None.
        """
        tracker = MotionTracker("box", max_gap_sec=0.25)
        for index in range(11):
            result = tracker.update(index * 0.1, [0, 0, 0], [0, 0, 0, 1])
        self.assertTrue(result.stationary)
        self.assertFalse(tracker.update(1.5, [0, 0, 0], [0, 0, 0, 1]).stationary)
        with self.assertRaises(ValueError):
            tracker.update(1.4, [0, 0, 0], [0, 0, 0, 1])
        self.assertFalse(tracker.update(1.6, [0, 0, 0], [0, 0, 0, 1]).stationary)

    def test_box_half_turn_is_equivalent(self):
        """Avoid false rotation when a box detector reverses its axes.

        Returns:
            None.
        """
        result = equivalent_orientation([0, 0, 1, 0], [0, 0, 0, 1], "box")
        self.assertAlmostEqual(abs(result[3]), 1.0)
        tracker = MotionTracker("box", velocity_alpha=1.0)
        tracker.update(0.0, [0, 0, 0], [0, 0, 0, 1])
        result = tracker.update(0.1, [0, 0, 0], [0, 0, 1, 0])
        self.assertAlmostEqual(np.linalg.norm(result.angular_velocity), 0.0)

    def test_cylinder_spin_is_unobservable_but_axis_motion_is_not(self):
        """Ignore axial spin while detecting changes to the cylinder axis.

        Returns:
            None.
        """
        tracker = MotionTracker("cylinder", velocity_alpha=1.0)
        tracker.update(0.0, [0, 0, 0], [0, 0, 0, 1])
        result = tracker.update(0.1, [0, 0, 0], [0, 0, math.sin(0.2), math.cos(0.2)])
        self.assertAlmostEqual(np.linalg.norm(result.angular_velocity), 0.0)
        result = tracker.update(0.2, [0, 0, 0], [math.sin(0.1), 0, 0, math.cos(0.1)])
        self.assertGreater(np.linalg.norm(result.angular_velocity), 1.0)

    def test_invalid_observation_discards_history(self):
        """Discard all previous evidence after invalid visual data.

        Returns:
            None.
        """
        tracker = MotionTracker("box")
        tracker.update(0.0, [0, 0, 0], [0, 0, 0, 1])
        with self.assertRaises(ValueError):
            tracker.update(0.1, [float("nan"), 0, 0], [0, 0, 0, 1])
        self.assertFalse(tracker.update(0.2, [0, 0, 0], [0, 0, 0, 1]).stationary)


if __name__ == "__main__":
    unittest.main()
