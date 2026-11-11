"""Tests for observable 3-D target geometry."""

import math
import unittest

import numpy as np

from embodied_nexus_dynamic_grasp.geometry import estimate_geometry_3d, matrix_quaternion
from embodied_nexus_dynamic_grasp.tracking import _axis


class GeometryTest(unittest.TestCase):
    """Verify orientation and center estimates in three dimensions."""

    def test_supported_box_top_face_uses_support_plane(self):
        """Infer supported box thickness only with an explicit support assumption.

        Returns:
            None.
        """
        rng = np.random.default_rng(22)
        points = np.column_stack((rng.uniform(.46, .54, 3000),
                                  rng.uniform(-.02, .02, 3000),
                                  np.full(3000, -.206)))
        with self.assertRaises(ValueError):
            estimate_geometry_3d(points, -.236, 'box')
        result = estimate_geometry_3d(points, -.236, 'box', supported_box=True)
        self.assertAlmostEqual(result['center'][2], -.221, places=4)
        self.assertAlmostEqual(result['size'][2], .03, places=4)

    def test_supported_box_rejects_unobservable_plane(self):
        """Reject a planar side face when support does not determine thickness.

        Returns:
            None.
        """
        rng = np.random.default_rng(23)
        points = np.column_stack((rng.uniform(.46, .54, 3000), np.zeros(3000),
                                  rng.uniform(-.23, -.19, 3000)))
        with self.assertRaises(ValueError):
            estimate_geometry_3d(points, -.236, 'box', supported_box=True)

    def test_matrix_to_quaternion(self):
        """Convert a quarter-turn rotation without reversing its direction.

        Returns:
            None.
        """
        matrix = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
        quaternion = matrix_quaternion(matrix)
        self.assertAlmostEqual(abs(quaternion[2]), math.sqrt(0.5))
        self.assertAlmostEqual(quaternion[2] * quaternion[3], 0.5)

    def test_tilted_cylinder_axis_and_center(self):
        """Recover the tilted long axis and center of a complete synthetic cloud.

        Returns:
            None.
        """
        rng = np.random.default_rng(11)
        axis = np.array([0.8, 0., 0.6])
        side = np.array([-0.6, 0., 0.8])
        angles = rng.uniform(0, 2 * math.pi, 5000)
        axial = rng.uniform(-0.05, 0.05, len(angles))
        points = (np.array([0.5, 0.1, 0.3]) + axial[:, None] * axis +
                  0.02 * np.cos(angles)[:, None] * side +
                  0.02 * np.sin(angles)[:, None] * np.array([0., 1., 0.]))
        result = estimate_geometry_3d(points, -0.236, "cylinder")
        self.assertGreater(abs(float(np.dot(_axis(result["orientation"]), axis))), 0.99)
        np.testing.assert_allclose(result["center"], [0.5, 0.1, 0.3], atol=0.003)

    def test_rejects_orientation_without_long_axis(self):
        """Reject an isotropic point cloud whose long axis is not observable.

        Returns:
            None.
        """
        rng = np.random.default_rng(7)
        points = rng.normal(size=(3000, 3)) * 0.01 + [0.5, 0.0, 0.3]
        with self.assertRaises(ValueError):
            estimate_geometry_3d(points, -0.236, "cylinder")
