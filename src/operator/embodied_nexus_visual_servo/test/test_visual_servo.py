"""Tests for independent visual servo calculations."""

import math
import unittest
from types import SimpleNamespace

import numpy as np

from embodied_nexus_visual_servo.control import (JointVelocityTracker, angular_velocity,
    cartesian_velocity, continuous_grasp_orientation, map_servo_velocities, orientation_error)
from embodied_nexus_visual_servo.perception import (
    GeometryTracker, YOLOEDetector, estimate_clipped_geometry, estimate_geometry,
    incomplete_box_extent, masked_depth_points, select_target, validate_frame_pair)


class VisualServoTest(unittest.TestCase):
    """Verify target geometry, time gating, and velocity handling."""

    def test_progressive_box_occlusion_uses_reliable_extent(self):
        """Reject gradual mask shrinkage that passes a previous-frame check.

        Returns:
            None.
        """
        reference = 0.075
        lengths = [0.074, 0.071, 0.068, 0.063, 0.060]
        self.assertFalse(incomplete_box_extent({'size': [lengths[2]]}, reference))
        self.assertTrue(incomplete_box_extent({'size': [lengths[3]]}, reference))
        self.assertTrue(all(new >= 0.7 * old for old, new in zip(lengths, lengths[1:])))

    def test_grasp_orientation_stays_continuous_across_axis_boundary(self):
        """Keep the jaw orientation continuous when an axis crosses zero or pi.

        Returns:
            None.
        """
        initial = [math.cos(math.pi / 4), math.sin(math.pi / 4), 0.0, 0.0]
        after_zero = continuous_grasp_orientation(-0.001, initial)
        after_pi = continuous_grasp_orientation(math.pi + 0.001, after_zero)
        self.assertLess(orientation_error(initial, after_zero), 0.002)
        self.assertLess(orientation_error(after_zero, after_pi), 0.003)

    def test_tracker_revises_initial_center_and_wraps_yaw(self):
        """Update a stable initial estimate without reacting to a single outlier.

        Returns:
            None.
        """
        tracker = GeometryTracker()
        def sample(x, yaw):
            """Build an observation at x and yaw.

            Args:
                x: Center X in meters.
                yaw: Axis yaw in radians.

            Returns:
                A geometry mapping.
            """
            return {'center': [x, 0, 0], 'size': [0.1, 0.04, 0.04], 'yaw': yaw}
        for _ in range(4):
            self.assertIsNone(tracker.update(sample(0.88, 0.01)))
        self.assertAlmostEqual(tracker.update(sample(0.88, math.pi + 0.01))['yaw'], 0.01)
        self.assertAlmostEqual(tracker.update(sample(0.98, 0.01))['center'][0], 0.88)
        for _ in range(4):
            value = tracker.update(sample(0.89, math.pi + 0.01))
        self.assertAlmostEqual(value['center'][0], 0.89)
        self.assertAlmostEqual(value['yaw'], 0.01)
        partial = tracker.update(sample(0.91, 0.4), clipped=True)
        self.assertAlmostEqual(partial['center'][0], 0.893)
        self.assertAlmostEqual(partial['yaw'], value['yaw'])
        self.assertEqual(partial['size'], value['size'])

    def test_orientation_error_is_quaternion_sign_invariant(self):
        """Measure actual angle instead of angular speed at the speed cap.

        Returns:
            None.
        """
        self.assertAlmostEqual(orientation_error([0, 0, 0, 1], [0, 0, 0, -1]), 0.0)
        angle = math.radians(4)
        self.assertAlmostEqual(orientation_error([0, 0, 0, 1], [0, 0, math.sin(angle/2), math.cos(angle/2)]), angle)

    def test_rotated_detection_returns_mask_in_original_coordinates(self):
        """Retry on a fresh frame and restore a rotated detection to RGB coordinates.

        Returns:
            None.
        """
        class Tensor:
            """Expose the tensor operations used by the detector."""

            def __init__(self, value):
                """Store an array.

                Args:
                    value: Array data.

                Returns:
                    None.
                """
                self.value = np.asarray(value)

            def detach(self):
                """Return the detached wrapper.

                Returns:
                    This wrapper.
                """
                return self

            def cpu(self):
                """Return the CPU wrapper.

                Returns:
                    This wrapper.
                """
                return self

            def numpy(self):
                """Return the wrapped array.

                Returns:
                    Array data.
                """
                return self.value

        class Model:
            """Detect the red test patch only on the second invocation."""

            def __init__(self):
                """Initialize the invocation counter.

                Returns:
                    None.
                """
                self.calls = 0

            def predict(self, image, **kwargs):
                """Return a synthetic segmentation derived from the rotated image.

                Args:
                    image: Input BGR image.
                    **kwargs: Ignored prediction options.

                Returns:
                    A one-element prediction list.
                """
                self.calls += 1
                if self.calls == 1:
                    return [SimpleNamespace(boxes=None, masks=None)]
                boxes = type('Boxes', (), {'__len__': lambda self: 1})()
                boxes.conf = Tensor([0.9])
                return [SimpleNamespace(boxes=boxes, masks=SimpleNamespace(data=[Tensor(image[:, :, 2] > 128)]))]

        detector = YOLOEDetector.__new__(YOLOEDetector)
        detector.model, detector.confidence = Model(), 0.1
        detector.preferred_angle, detector.retry_index = 0, 0
        rgb = np.zeros((80, 120, 3), dtype=np.uint8)
        rgb[25:50, 35:85, 0] = 255
        with self.assertRaises(RuntimeError):
            detector.segment(rgb)
        mask, _ = detector.segment(rgb)
        truth = rgb[:, :, 0] > 0
        self.assertGreater(np.sum(mask & truth) / np.sum(mask | truth), 0.94)
        detector.preferred_angle, detector.retry_index = 0, 4
        mask, _ = detector.segment(rgb)
        self.assertEqual(detector.preferred_angle, 360)
        np.testing.assert_array_equal(mask, truth)

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
