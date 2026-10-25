from __future__ import annotations

import importlib.util
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest.mock import Mock

if importlib.util.find_spec("xacro") is None:
    raise unittest.SkipTest("Gazebo profile tests require ROS 2 Humble")

import rclpy
import numpy as np
from embodied_nexus_sim_gazebo.camera_info import SimCameraInfo
from embodied_nexus_sim_gazebo.camera_geometry import camera_geometry, CAMERA_FROM_OPTICAL, pose_xyz_rpy

source = Path(__file__).resolve().parents[1] / "launch/sim.launch.py"
spec = importlib.util.spec_from_file_location("gazebo_launch", source)
gazebo_launch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gazebo_launch)


class PerformanceProfileTests(unittest.TestCase):
    def test_profiles_set_sensor_resolution_and_rate_without_changing_scan_coverage(self):
        for profile, width, height, rate in (("balanced", 320, 240, 5), ("full", 640, 480, 10)):
            with self.subTest(profile=profile):
                robot = ET.fromstring(gazebo_launch._inject_fortress_plugins('<robot name="test"/>', "/controllers.yaml", profile))
                camera = robot.find(".//sensor[@type='camera']")
                depth = robot.find(".//sensor[@type='depth_camera']")
                self.assertEqual(camera.findtext("camera/image/width"), str(width))
                self.assertEqual(camera.findtext("camera/image/height"), str(height))
                lidars = robot.findall(".//sensor[@type='gpu_lidar']")
                self.assertEqual(len(lidars), 2)
                for sensor in [camera, depth, *lidars]:
                    self.assertEqual(sensor.findtext("update_rate"), str(rate))
                self.assertTrue(all(sensor.findtext("lidar/scan/vertical/samples") == "16" for sensor in lidars))

    def test_camera_intrinsics_match_balanced_resolution(self):
        rclpy.init()
        node = SimCameraInfo("color", "/test", 320, 240, 5)
        try:
            node.publisher = Mock()
            node.publish_info()
            info = node.publisher.publish.call_args.args[0]
            self.assertEqual((info.width, info.height), (320, 240))
            self.assertAlmostEqual(info.k[0], 461.146637 / 2)
            self.assertAlmostEqual(info.k[4], 461.36908 / 2)
            self.assertAlmostEqual(info.k[2], 323.604004 / 2)
            self.assertAlmostEqual(info.k[5], 246.426437 / 2)
        finally:
            node.destroy_node()
            rclpy.shutdown()

    def test_optical_mount_uses_active_config_and_factory_rgbd_baseline(self):
        geometry = camera_geometry(640, 480)
        color = geometry["color"]["pose"] @ CAMERA_FROM_OPTICAL
        depth = geometry["depth"]["pose"] @ CAMERA_FROM_OPTICAL
        np.testing.assert_allclose(color[:3, 3], [0.0201, 0, 0.05], atol=1e-9)
        np.testing.assert_allclose(color[:3, 2], [0, 0, 1], atol=1e-9)
        depth_to_color = np.linalg.inv(color) @ depth
        np.testing.assert_allclose(depth_to_color[:3, 3], [-0.0136681299, -0.000136599287, -0.0022318964], atol=1e-9)
        xyz, rpy = pose_xyz_rpy(geometry["color"]["pose"])
        self.assertAlmostEqual(rpy[1], -np.pi / 2)
        self.assertEqual(geometry["calibration"]["streams"]["color"]["distortion"], [0.0] * 5)

    def test_gazebo_projection_and_camera_info_use_the_same_distinct_intrinsics(self):
        robot = ET.fromstring(gazebo_launch._inject_fortress_plugins('<robot name="test"/>', "/controllers.yaml"))
        color = robot.find(".//sensor[@type='camera']/camera/lens/intrinsics")
        depth = robot.find(".//sensor[@type='depth_camera']/camera/lens/intrinsics")
        self.assertAlmostEqual(float(color.findtext("fx")), 461.146637 / 2)
        self.assertAlmostEqual(float(depth.findtext("fx")), 372.082153 / 2)
