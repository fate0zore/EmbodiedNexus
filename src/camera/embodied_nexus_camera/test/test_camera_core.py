import json
import socket
import struct
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from embodied_nexus_camera.calibration import CameraModel, RgbdCalibration, _model_from_entry, load_calibration
from embodied_nexus_camera.config import DEFAULT_CONFIG, apply_runtime_patch, load_config, validate_config
from embodied_nexus_camera.geometry import depth_to_xyzrgb, pointcloud_xyz, sample_xyz_rgb
from embodied_nexus_camera.web_server import CameraHub, _read_ws_message, _ws_frame
from embodied_nexus_camera.runtime import CameraRuntime


class CameraConfigTest(unittest.TestCase):
    def test_default_configuration_is_valid(self):
        validate_config(DEFAULT_CONFIG)

    def test_simulator_rgbd_topics_use_calibrated_frame_names(self):
        profile = Path(__file__).resolve().parents[1] / "config" / "gemini335_sim.yaml"
        from embodied_nexus_camera.config import load_config
        config = load_config(profile)
        self.assertEqual(config["source"]["backend"], "ros2_topics")
        self.assertEqual(config["capture"]["color_frame_id"], "camera_color_optical_frame")
        self.assertEqual(config["capture"]["depth_frame_id"], "camera_depth_optical_frame")

    def test_ros_topic_source_cannot_loop_to_own_output(self):
        config = json.loads(json.dumps(DEFAULT_CONFIG))
        config["source"]["backend"] = "ros2_topics"
        config["source"]["color_topic"] = "/embodied_nexus/cameras/camera_front/color/image_raw"
        with self.assertRaisesRegex(ValueError, "must differ"):
            validate_config(config)

    def test_runtime_update_rejects_capture_changes(self):
        with self.assertRaisesRegex(ValueError, "requires restart"):
            apply_runtime_patch(DEFAULT_CONFIG, {"capture": {"color_fps": 10}})

    def test_runtime_update_is_validated_and_does_not_mutate_base(self):
        changed = apply_runtime_patch(DEFAULT_CONFIG, {"streams": {"jpeg_quality": 70}})
        self.assertEqual(changed["streams"]["jpeg_quality"], 70)
        self.assertEqual(DEFAULT_CONFIG["streams"]["jpeg_quality"], 85)

    def test_pointcloud_roi_is_runtime_configurable_and_validated(self):
        changed = apply_runtime_patch(DEFAULT_CONFIG, {"pointcloud": {"roi": [10, 20, 100, 80]}})
        self.assertEqual(changed["pointcloud"]["roi"], [10, 20, 100, 80])
        with self.assertRaisesRegex(ValueError, "pointcloud.roi"):
            apply_runtime_patch(DEFAULT_CONFIG, {"pointcloud": {"roi": [-1, 0, 10, 10]}})

    def test_v4l2_controls_are_numeric_and_capture_scoped(self):
        config = json.loads(json.dumps(DEFAULT_CONFIG))
        config["capture"]["color_controls"] = {"exposure_time_absolute": 156, "gain": 16}
        validate_config(config)
        with self.assertRaisesRegex(ValueError, "finite numeric"):
            config["capture"]["depth_controls"] = {"gain": True}
            validate_config(config)

    def test_calibration_sample_delete_updates_manifest(self):
        with tempfile.TemporaryDirectory() as temp:
            config = json.loads(json.dumps(DEFAULT_CONFIG))
            config["calibration_session"]["root"] = temp
            session = Path(temp) / "session-1"
            session.mkdir()
            (session / "session.json").write_text("{}", encoding="utf-8")
            (session / "frame_0001.png").write_bytes(b"sample")
            (session / "capture_manifest.json").write_text(json.dumps([{"file": "frame_0001.png"}]), encoding="utf-8")
            runtime = CameraRuntime(config, lambda *_: None)
            result = runtime.delete_calibration_sample("session-1", "frame_0001.png")
            self.assertEqual(result["remaining_samples"], 0)
            self.assertFalse((session / "frame_0001.png").exists())
            self.assertEqual(json.loads((session / "capture_manifest.json").read_text()), [])
            with self.assertRaisesRegex(ValueError, "basename"):
                runtime.delete_calibration_sample("session-1", "../outside.png")


class CalibrationAndGeometryTest(unittest.TestCase):
    def test_external_xyz_cloud_layout_and_rgb_projection(self):
        cloud = SimpleNamespace(
            fields=[SimpleNamespace(name=name, count=1, datatype=7, offset=index * 4) for index, name in enumerate(("x", "y", "z"))],
            is_bigendian=False, width=2, height=1, point_step=12, row_step=24,
            data=struct.pack("<ffffff", 0.0, 0.0, 1.0, 1.0, 0.0, 1.0),
        )
        xyz = pointcloud_xyz(cloud)
        np.testing.assert_allclose(xyz, [[0, 0, 1], [1, 0, 1]])
        model = CameraModel(4, 4, 1, 1, 2, 2, [0, 0, 0, 0], calibrated=True)
        color = np.zeros((4, 4, 3), np.uint8)
        color[2, 2] = [10, 20, 30]  # BGR
        rgb = sample_xyz_rgb(xyz, color, model)
        np.testing.assert_array_equal(rgb, [[30, 20, 10], [0, 0, 0]])

    def test_yahboom_factory_profile_loads_from_package_config(self):
        config_path = Path(__file__).resolve().parents[1] / "config" / "gemini335.yaml"
        config = load_config(config_path)
        calibration = load_calibration(
            factory_file=config["calibration"]["factory_file"],
            color_size=(640, 480), depth_size=(640, 480),
        )
        self.assertTrue(calibration.usable_for_color_cloud)
        self.assertEqual(calibration.color.distortion_model, "rational_polynomial")
        self.assertEqual(len(calibration.color.distortion), 8)
        self.assertEqual(len(calibration.depth.distortion), 8)

    def test_resolution_scaling_requires_explicit_opt_in(self):
        config_path = Path(__file__).resolve().parents[1] / "config" / "gemini335.yaml"
        config = load_config(config_path)
        args = {
            "factory_file": config["calibration"]["factory_file"],
            "color_size": (1280, 960),
            "depth_size": (1280, 960),
        }
        unscaled = load_calibration(**args)
        self.assertIsNone(unscaled.color)
        self.assertIsNone(unscaled.depth)
        self.assertIn("allow_resolution_scaling", unscaled.issue)
        scaled = load_calibration(**args, allow_resolution_scaling=True)
        reference = load_calibration(factory_file=args["factory_file"])
        self.assertEqual(scaled.color.width, 1280)
        self.assertEqual(scaled.color.height, 960)
        self.assertAlmostEqual(scaled.color.fx, 2 * reference.color.fx)

    def test_orbbec_native_factory_distortion_order_is_preserved(self):
        model = _model_from_entry({
            "image_size": [640, 480],
            "intrinsic": {"fx": 500, "fy": 501, "cx": 320, "cy": 240},
            "distortion": [1, 2, 3, 4, 5, 6, 7, 8],
            "distortion_model": "modified_brown_conrady",
        })
        self.assertEqual(model.distortion_model, "rational_polynomial")
        self.assertEqual(model.cv_distortion.tolist(), [1, 2, 7, 8, 3, 4, 5, 6])

    def test_rgbd_projection_returns_meters_and_rgb_order(self):
        model = CameraModel(4, 4, 2, 2, 1.5, 1.5, [0, 0, 0, 0, 0], calibrated=True)
        calibration = RgbdCalibration(model, model, 0.001, np.eye(3), np.zeros(3), "test", "test")
        depth = np.full((4, 4), 1000, np.uint16)
        color = np.zeros((4, 4, 3), np.uint8)
        color[:, :] = [10, 20, 30]  # BGR input
        points = depth_to_xyzrgb(depth, color, calibration, stride=1, max_points=100)
        self.assertEqual(points.shape, (16, 6))
        np.testing.assert_allclose(points[:, 2], 1.0)
        np.testing.assert_allclose(points[:, 3:], np.tile([30, 20, 10], (16, 1)))

    def test_depth_roi_uses_original_image_coordinates_and_checks_bounds(self):
        model = CameraModel(4, 4, 2, 2, 1.5, 1.5, [0, 0, 0, 0, 0], calibrated=True)
        calibration = RgbdCalibration(model, model, 0.001, np.eye(3), np.zeros(3), "test", "test")
        depth = np.full((4, 4), 1000, np.uint16)
        color = np.zeros((4, 4, 3), np.uint8)
        points = depth_to_xyzrgb(depth, color, calibration, stride=1, roi=[1, 1, 2, 2])
        self.assertEqual(points.shape, (4, 6))
        self.assertAlmostEqual(points[:, 0].min(), -0.25)
        self.assertAlmostEqual(points[:, 0].max(), 0.25)
        with self.assertRaisesRegex(ValueError, "outside the depth frame"):
            depth_to_xyzrgb(depth, color, calibration, roi=[3, 3, 2, 2])

    def test_point_cloud_requires_calibration_and_pair_timing(self):
        model = CameraModel(2, 2, 1, 1, 0.5, 0.5, [0, 0, 0, 0], calibrated=False)
        calibration = RgbdCalibration(model, model, 0.001, np.eye(3), np.zeros(3))
        with self.assertRaisesRegex(ValueError, "calibration"):
            depth_to_xyzrgb(np.ones((2, 2), np.uint16), np.zeros((2, 2, 3), np.uint8), calibration)
        model.calibrated = True
        calibration.depth_to_color_rotation = np.eye(3)
        with self.assertRaisesRegex(ValueError, "time delta"):
            depth_to_xyzrgb(
                np.ones((2, 2), np.uint16), np.zeros((2, 2, 3), np.uint8), calibration,
                color_stamp=1.0, depth_stamp=1.1,
            )


class WebTransportTest(unittest.TestCase):
    def test_latest_frame_cache_and_encoded_frame(self):
        hub = CameraHub("camera_front")
        image = np.zeros((8, 12, 3), np.uint8)
        frame = hub.publish("color", image, stamp_sec=123.0, encoding="bgr8", width=12, height=8)
        payload = hub.encode(frame, "jpeg", 80)
        self.assertTrue(payload.startswith(b"\xff\xd8"))
        self.assertEqual(hub.latest("color").sequence, 1)
        self.assertTrue(hub.status(DEFAULT_CONFIG)["streams"]["color"]["available"])

    def test_pointcloud_error_is_not_cleared_by_new_color_frames(self):
        hub = CameraHub("camera_front")
        hub.error("no synchronized pair", "points")
        hub.publish("color", np.zeros((2, 2, 3), np.uint8), stamp_sec=1.0)
        self.assertIn("no synchronized pair", hub.status(DEFAULT_CONFIG)["last_error"])
        hub.publish("points", b"cloud", stamp_sec=1.0)
        self.assertEqual(hub.status(DEFAULT_CONFIG)["last_error"], "")
        hub.record_drops(3)
        self.assertEqual(hub.status(DEFAULT_CONFIG)["dropped_frames"], 3)

    def test_masked_client_websocket_frame(self):
        class FakeSocket:
            def __init__(self, data): self.data = bytearray(data)
            def recv(self, size):
                chunk = self.data[:size]
                del self.data[:size]
                return bytes(chunk)
        message = b"hello"
        mask = b"abcd"
        masked = bytes(value ^ mask[i % 4] for i, value in enumerate(message))
        packet = bytes([0x81, 0x80 | len(message)]) + mask + masked
        opcode, decoded = _read_ws_message(FakeSocket(packet))
        self.assertEqual(opcode, 1)
        self.assertEqual(decoded, message)
        self.assertEqual(_ws_frame(2, b"x")[0], 0x82)


if __name__ == "__main__":
    unittest.main()
