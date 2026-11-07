"""Unit tests for RGB-D box geometry and grasp orientation helpers."""

import math
import unittest
import numpy as np

from embodied_nexus_operator.geometry import (estimate_box_pose, grasp_closing_yaw, measure_target_geometry,
                                              rotate_vector, top_down_quaternion, validate_target_mask)


class GraspClosingYawTests(unittest.TestCase):
    """Check gripper jaw orientation for rectangular and square boxes."""

    def test_rectangular_box_closes_across_short_side(self):
        """Rotate the PCA long-edge angle by 90 degrees for jaw closure."""
        result = grasp_closing_yaw(0.25, [0.08, 0.04, 0.03])
        self.assertAlmostEqual(result, 0.25 - math.pi / 2.0)

    def test_square_box_uses_configured_symmetric_direction(self):
        """Use the configured fallback when the horizontal sides are equal."""
        result = grasp_closing_yaw(0.25, [0.04, 0.04, 0.03], -1.48)
        self.assertAlmostEqual(result, -1.48)

    def test_closing_yaw_is_normalized_for_symmetric_jaws(self):
        """Normalize the unoriented jaw axis modulo pi."""
        result = grasp_closing_yaw(2.8, [0.08, 0.04, 0.03])
        self.assertGreaterEqual(result, -math.pi / 2.0)
        self.assertLess(result, math.pi / 2.0)

    def test_invalid_box_size_is_rejected(self):
        """Reject incomplete dimensions before creating an invalid grasp pose."""
        with self.assertRaises(ValueError):
            grasp_closing_yaw(0.25, [0.08, 0.04])

    def test_cylinder_height_and_tool_offset_lower_grasp(self):
        """Place a 50 mm target center and the tool offset at their expected heights.

        Returns:
            None.
        """
        points = np.array([[x, y, 0.05] for x in np.linspace(-0.05, 0.05, 20)
                           for y in np.linspace(-0.02, 0.02, 10)])
        estimate = estimate_box_pose(points, [0.10, 0.05, 0.05], 0.0)
        self.assertAlmostEqual(estimate["center"][2], 0.025)
        offset = rotate_vector([0, 0, 0.009], top_down_quaternion(0.0))
        self.assertAlmostEqual(estimate["center"][2] - offset[2], 0.034)


class PointCloudMeasurementTests(unittest.TestCase):
    """Exercise dynamic object dimensions and incomplete observations."""

    def test_multiple_horizontal_cylinders(self):
        """Measure two visible cylinder surfaces without preset dimensions.

        Returns:
            None.
        """
        for diameter in (0.04, 0.05):
            x, theta = np.meshgrid(np.linspace(-0.05, 0.05, 80), np.linspace(0.05, math.pi - 0.05, 60))
            radius = diameter / 2
            points = np.column_stack((x.ravel(), (radius * np.cos(theta)).ravel(),
                                      (radius + radius * np.sin(theta)).ravel()))
            result = measure_target_geometry(points, 0.0, shape="cylinder")
            self.assertAlmostEqual(result["size"][0], 0.10, delta=0.008)
            self.assertAlmostEqual(result["size"][1], diameter, delta=0.002)
            self.assertAlmostEqual(result["center"][2], radius, delta=0.001)

    def test_general_object_and_outliers(self):
        """Measure a rectangular visible top while rejecting distant outliers.

        Returns:
            None.
        """
        x, y = np.meshgrid(np.linspace(-0.05, 0.05, 50), np.linspace(-0.02, 0.02, 40))
        points = np.column_stack((x.ravel(), y.ravel(), np.full(x.size, 0.03)))
        points = np.vstack((points, [[2, 2, 0.03], [float('nan'), 0, 0.03]]))
        result = measure_target_geometry(points, 0.0)
        self.assertAlmostEqual(result["size"][0], 0.10, delta=0.01)
        self.assertAlmostEqual(result["size"][1], 0.04, delta=0.005)
        self.assertAlmostEqual(result["size"][2], 0.03, delta=0.001)

    def test_rejects_missing_and_truncated_data(self):
        """Reject small point sets and masks clipped by the image boundary.

        Returns:
            None.
        """
        with self.assertRaisesRegex(ValueError, "too few"):
            measure_target_geometry(np.zeros((20, 3)), 0.0)
        mask = np.zeros((30, 30), dtype=bool)
        mask[2:20, 0:10] = True
        with self.assertRaisesRegex(ValueError, "boundary"):
            validate_target_mask(mask, 80)

    def test_irregular_visible_surface(self):
        """Measure a nonrectangular tabletop silhouette as a bounding box.

        Returns:
            None.
        """
        x, y = np.meshgrid(np.linspace(-0.05, 0.05, 60), np.linspace(-0.03, 0.03, 60))
        keep = (x / 0.05) ** 2 + (y / 0.03) ** 2 < 1
        points = np.column_stack((x[keep], y[keep], np.full(keep.sum(), 0.025)))
        result = measure_target_geometry(points, 0.0)
        self.assertGreater(result["size"][0], result["size"][1])
        self.assertAlmostEqual(result["size"][2], 0.025)


if __name__ == "__main__":
    unittest.main()
