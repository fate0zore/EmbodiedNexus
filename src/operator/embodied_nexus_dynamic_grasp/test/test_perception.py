"""Tests for image timestamps and calibrated depth projection."""

import unittest

import numpy as np

from embodied_nexus_dynamic_grasp.perception import masked_depth_points, validate_frame_pair


class PerceptionTest(unittest.TestCase):
    """Reject unsafe pairs and check the fixed-frame projection inputs."""

    def test_pair_rejects_stale_and_mismatched_frames(self):
        """Require a synchronized, freshly received color and depth pair.

        Returns:
            None.
        """
        validate_frame_pair(1.0, 1.02, 4.9, 4.9, 5.0, 0.08, 0.75)
        with self.assertRaises(ValueError):
            validate_frame_pair(1.0, 1.1, 4.9, 4.9, 5.0, 0.08, 0.75)
        with self.assertRaises(ValueError):
            validate_frame_pair(1.0, 1.02, 4.0, 4.9, 5.0, 0.08, 0.75)

    def test_masked_points_follow_camera_intrinsics(self):
        """Project valid mask pixels through the stated camera intrinsics.

        Returns:
            None.
        """
        depth = np.ones((8, 8), dtype=float)
        mask = np.zeros((8, 8), dtype=bool)
        mask[2:6, 2:6] = True
        points = masked_depth_points(depth, mask, [2, 2, 4, 4], [2, 2, 4, 4],
                                     np.eye(4), minimum_points=8)
        self.assertGreaterEqual(len(points), 8)
        np.testing.assert_allclose(points[:, 2], 1.0)
        self.assertTrue(np.all(points[:, 0] >= -1.0))
        self.assertTrue(np.all(points[:, 0] <= 0.5))
