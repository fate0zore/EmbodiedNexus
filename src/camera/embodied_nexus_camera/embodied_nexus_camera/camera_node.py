from __future__ import annotations

import argparse
import json
import os
import select
import subprocess
import threading
import time
from array import array as byte_array
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from embodied_nexus_camera_interfaces.action import SolveRgbCalibration
from embodied_nexus_camera_interfaces.srv import ApplyCameraConfiguration, GetCameraConfiguration, ValidateCameraConfiguration

from .calibration import CameraModel, camera_info_dict, load_calibration
from .config import load_config, validate_config
from .geometry import depth_to_xyzrgb, pointcloud_xyz, sample_xyz_rgb
from .runtime import CameraRuntime
from .web_server import CameraWebServer


def _share_config() -> str:
    try:
        from ament_index_python.packages import get_package_share_directory

        return str(Path(get_package_share_directory("embodied_nexus_camera")) / "config" / "gemini335.yaml")
    except Exception:
        return str(Path(__file__).resolve().parents[1] / "config" / "gemini335.yaml")


def _stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _ros_stamp(stamp_sec: float):
    from builtin_interfaces.msg import Time

    msg = Time()
    msg.sec = int(stamp_sec)
    msg.nanosec = int((stamp_sec - int(stamp_sec)) * 1e9)
    return msg


def _read_pipe_exact(stream, size: int, stop: threading.Event, timeout_sec: float) -> bytearray | None:
    deadline = time.monotonic() + timeout_sec
    raw = bytearray()
    while len(raw) < size and not stop.is_set():
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError(f"depth stream timed out after {timeout_sec:g}s")
        ready, _, _ = select.select([stream], [], [], min(0.2, remaining))
        if not ready:
            continue
        chunk = os.read(stream.fileno(), size - len(raw))
        if not chunk:
            return raw
        raw.extend(chunk)
    return None if stop.is_set() else raw


def _image_message(array: np.ndarray, encoding: str, frame_id: str, stamp_sec: float) -> Image:
    msg = Image()
    msg.header.stamp = _ros_stamp(stamp_sec)
    msg.header.frame_id = frame_id
    msg.height, msg.width = array.shape[:2]
    msg.encoding = encoding
    msg.is_bigendian = 0
    channels = 1 if array.ndim == 2 else array.shape[2]
    msg.step = int(msg.width * array.dtype.itemsize * channels)
    msg.data = _ros_byte_array(np.ascontiguousarray(array))
    return msg


def _ros_byte_array(payload: np.ndarray | bytes) -> byte_array:
    result = byte_array("B")
    if isinstance(payload, np.ndarray):
        result.frombytes(memoryview(np.ascontiguousarray(payload)).cast("B"))
    else:
        result.frombytes(payload)
    return result


def _array_from_image(msg: Image) -> tuple[np.ndarray, str]:
    enc = msg.encoding.lower()
    if enc == "bgr8":
        return np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3).copy(), "bgr8"
    if enc == "rgb8":
        arr = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR), "bgr8"
    if enc == "mono8":
        arr = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width)
        return cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR), "bgr8"
    if enc == "16uc1":
        return np.frombuffer(msg.data, np.uint16).reshape(msg.height, msg.width).copy(), "16UC1"
    if enc == "32fc1":
        arr = np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.width)
        return np.clip(arr * 1000.0, 0, 65535).astype(np.uint16), "16UC1"
    raise ValueError(f"unsupported ROS Image encoding: {msg.encoding}")


def _quaternion_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    norm = x*x + y*y + z*z + w*w
    if norm < 1e-12:
        return np.eye(3)
    scale = 2.0 / norm
    return np.asarray([
        [1-scale*(y*y+z*z), scale*(x*y-z*w), scale*(x*z+y*w)],
        [scale*(x*y+z*w), 1-scale*(x*x+z*z), scale*(y*z-x*w)],
        [scale*(x*z-y*w), scale*(y*z+x*w), 1-scale*(x*x+y*y)],
    ], dtype=np.float64)


class CameraNode(Node):
    def __init__(self, config_path: str | None = None, overrides: dict[str, Any] | None = None) -> None:
        super().__init__("embodied_nexus_camera")
        self.declare_parameter("config_file", config_path or _share_config())
        self.declare_parameter("interface_mode", "")
        self.declare_parameter("web.bind_host", "")
        self.declare_parameter("web.port", 0)
        path = str(self.get_parameter("config_file").value)
        self.config = load_config(path)
        override_mode = str(self.get_parameter("interface_mode").value)
        if override_mode:
            self.config["interface_mode"] = override_mode
        override_host = str(self.get_parameter("web.bind_host").value)
        if override_host:
            self.config["web"]["bind_host"] = override_host
        override_port = int(self.get_parameter("web.port").value)
        if override_port:
            self.config["web"]["port"] = override_port
        if overrides:
            for key, value in overrides.items():
                group, leaf = key.split(".", 1)
                self.config[group][leaf] = value
        validate_config(self.config)
        self.runtime = CameraRuntime(self.config, self._log_line)
        self.hub = self.runtime.hub
        self._stop = threading.Event()
        self._capture_threads: list[threading.Thread] = []
        self._cap_lock = threading.Lock()
        self.latest_color: tuple[np.ndarray, float] | None = None
        self.latest_depth: tuple[np.ndarray, float] | None = None
        self.latest_color_calibration_version = ""
        self.latest_depth_calibration_version = ""
        self.latest_color_frame_id = str(self.config["capture"]["color_frame_id"])
        self.latest_depth_frame_id = str(self.config["capture"]["depth_frame_id"])
        self.color_period = 0.0
        self.depth_period = 0.0
        self._last_ros_publish = {"color": 0.0, "depth": 0.0, "points": 0.0}
        self._last_cloud_attempt = 0.0
        self._last_cloud_depth_stamp: float | None = None
        self._next_pointcloud_publish = 0.0
        self._cloud_event = threading.Event()
        self._cloud_lock = threading.Lock()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        camera_id = self.config["camera_id"]
        self.output_ns = f"/embodied_nexus/cameras/{camera_id}"
        self.ros_enabled = self.config["interface_mode"] in {"ros2", "both"}
        if self.ros_enabled:
            self.color_pub = self.create_publisher(Image, f"{self.output_ns}/color/image_raw", qos_profile_sensor_data)
            self.depth_pub = self.create_publisher(Image, f"{self.output_ns}/depth/image_raw", qos_profile_sensor_data)
            self.color_info_pub = self.create_publisher(CameraInfo, f"{self.output_ns}/color/camera_info", qos_profile_sensor_data)
            self.depth_info_pub = self.create_publisher(CameraInfo, f"{self.output_ns}/depth/camera_info", qos_profile_sensor_data)
            self.points_pub = self.create_publisher(PointCloud2, f"{self.output_ns}/points", qos_profile_sensor_data)
            self.status_pub = self.create_publisher(String, f"{self.output_ns}/status", 10)
            self.calibration_status_pub = self.create_publisher(String, f"{self.output_ns}/calibration/status", 10)
            self.create_service(Trigger, f"{self.output_ns}/calibration/start", self._srv_calibration_start)
            self.create_service(Trigger, f"{self.output_ns}/calibration/capture", self._srv_calibration_capture)
            self.create_service(Trigger, f"{self.output_ns}/calibration/solve", self._srv_calibration_solve)
            self.create_subscription(String, f"{self.output_ns}/calibration/apply", self._apply_calibration_msg, 10)
            self.create_service(GetCameraConfiguration, f"{self.output_ns}/config/get", self._srv_get_config)
            self.create_service(ValidateCameraConfiguration, f"{self.output_ns}/config/validate", self._srv_validate_config)
            self.create_service(ApplyCameraConfiguration, f"{self.output_ns}/config/apply", self._srv_apply_config)
        else:
            self.color_pub = self.depth_pub = self.color_info_pub = self.depth_info_pub = None
            self.points_pub = self.status_pub = self.calibration_status_pub = None
        self.calibration_action = None
        if self.ros_enabled:
            self.calibration_action = ActionServer(
                self, SolveRgbCalibration, f"{self.output_ns}/calibration/solve_action",
                execute_callback=self._execute_calibration_action,
                goal_callback=self._calibration_goal_request,
                cancel_callback=self._calibration_cancel_request,
            )
        self._calibration_session_id = ""
        self._install_parameters()
        self.web_server = None
        if self.config["interface_mode"] in {"web", "both"}:
            web = self.config["web"]
            token = os.environ.get(str(web["auth_token_env"]), "")
            self.web_server = CameraWebServer(self.runtime, web["bind_host"], web["port"], token)
            self.web_server.start()
            self.get_logger().info(f"camera Web API listening on {web['bind_host']}:{web['port']}")
        if self.config["pointcloud"]["source"] == "depth":
            self._spawn(self._cloud_worker, "camera-pointcloud-worker")
        self._start_backend()
        self.create_timer(1.0, self._publish_status)

    def _log_line(self, level: str, message: str) -> None:
        logger = self.get_logger()
        getattr(logger, level if level in {"info", "warning", "error", "debug"} else "info")(message)

    def _srv_get_config(self, request, response):
        try:
            response.success = True
            response.config_json = json.dumps(self.runtime.public_config(), ensure_ascii=False)
            response.error = ""
        except Exception as exc:
            response.success = False
            response.config_json = "{}"
            response.error = str(exc)
        return response

    def _config_patch_response(self, patch_json: str, apply: bool):
        try:
            patch = json.loads(patch_json)
            if not isinstance(patch, dict):
                raise ValueError("configuration patch must be a JSON object")
            config = self.runtime.apply_config(patch) if apply else self.runtime.validate_config_patch(patch)
            return True, json.dumps(config, ensure_ascii=False), ""
        except (ValueError, KeyError, TypeError) as exc:
            return False, "{}", str(exc)

    def _srv_validate_config(self, request, response):
        response.success, response.config_json, response.error = self._config_patch_response(request.patch_json, False)
        return response

    def _srv_apply_config(self, request, response):
        response.success, response.config_json, response.error = self._config_patch_response(request.patch_json, True)
        return response

    def _install_parameters(self) -> None:
        hot_values = {
            "streams.color_output_fps": float(self.config["streams"]["color_output_fps"]),
            "streams.depth_output_fps": float(self.config["streams"]["depth_output_fps"]),
            "streams.jpeg_quality": int(self.config["streams"]["jpeg_quality"]),
            "pointcloud.min_depth_m": float(self.config["pointcloud"]["min_depth_m"]),
            "pointcloud.max_depth_m": float(self.config["pointcloud"]["max_depth_m"]),
            "pointcloud.stride": int(self.config["pointcloud"]["stride"]),
            "pointcloud.max_points": int(self.config["pointcloud"]["max_points"]),
            "pointcloud.output_fps": float(self.config["pointcloud"]["output_fps"]),
        }
        for name, value in hot_values.items():
            self.declare_parameter(name, value)
        self.add_on_set_parameters_callback(self._on_parameter_update)

    @staticmethod
    def _apply_v4l2_controls(device: str, controls: dict[str, Any]) -> None:
        if not controls:
            return
        command = ["v4l2-ctl", "-d", device]
        command.extend(f"--set-ctrl={name}={value}" for name, value in controls.items())
        result = subprocess.run(command, capture_output=True, text=True, check=False)
        if result.returncode:
            detail = result.stderr.strip() or result.stdout.strip() or f"exit code {result.returncode}"
            raise RuntimeError(f"failed to apply V4L2 controls on {device}: {detail}")

    def _on_parameter_update(self, parameters):
        from rcl_interfaces.msg import SetParametersResult

        patch: dict[str, dict[str, Any]] = {}
        for parameter in parameters:
            if "." not in parameter.name:
                continue
            group, leaf = parameter.name.split(".", 1)
            patch.setdefault(group, {})[leaf] = parameter.value
        try:
            if patch:
                self.runtime.apply_config(patch)
        except (ValueError, KeyError, TypeError) as exc:
            return SetParametersResult(successful=False, reason=str(exc))
        return SetParametersResult(successful=True)

    def _start_backend(self) -> None:
        backend = self.config["source"]["backend"]
        if backend == "gemini_v4l2":
            self._spawn(self._color_v4l2_loop, "camera-color-capture")
            self._spawn(self._depth_v4l2_loop, "camera-depth-capture")
        elif backend == "ros2_topics":
            src = self.config["source"]
            self.create_subscription(Image, src["color_topic"], self._ros_color, qos_profile_sensor_data)
            self.create_subscription(Image, src["depth_topic"], self._ros_depth, qos_profile_sensor_data)
            self.create_subscription(CameraInfo, src["color_info_topic"], lambda msg: self._publish_info_copy(msg, "color"), qos_profile_sensor_data)
            self.create_subscription(CameraInfo, src["depth_info_topic"], lambda msg: self._publish_info_copy(msg, "depth"), qos_profile_sensor_data)
        elif backend == "mock":
            self._spawn(self._mock_loop, "camera-mock-source")
        if self.config["pointcloud"]["source"] == "ros2_topic":
            self.create_subscription(PointCloud2, self.config["pointcloud"]["input_topic"], self._ros_cloud, qos_profile_sensor_data)

    def _spawn(self, target, name: str) -> None:
        thread = threading.Thread(target=target, name=name, daemon=True)
        self._capture_threads.append(thread)
        thread.start()

    def _color_v4l2_loop(self) -> None:
        cfg = self.config["capture"]
        source = self.config["source"]
        backoff = 0.2
        while not self._stop.is_set():
            cap = None
            try:
                self._apply_v4l2_controls(source["color_device"], cfg["color_controls"])
                cap = cv2.VideoCapture(source["color_device"], cv2.CAP_V4L2)
                if not cap.isOpened():
                    raise RuntimeError(f"cannot open RGB device {source['color_device']}")
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*str(cfg["color_fourcc"])[:4]))
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(cfg["color_width"]))
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(cfg["color_height"]))
                cap.set(cv2.CAP_PROP_FPS, float(cfg["color_fps"]))
                actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                actual_fps = float(cap.get(cv2.CAP_PROP_FPS))
                self.get_logger().info(f"RGB opened {source['color_device']} at {actual_w}x{actual_h}, negotiated {actual_fps:g} FPS")
                while not self._stop.is_set():
                    ok, frame = cap.read()
                    if not ok or frame is None:
                        raise RuntimeError("RGB capture returned no frame")
                    stamp = time.time()
                    self._accept_color(frame, stamp, cfg["color_frame_id"], "host_receive")
                    backoff = 0.2
                    # V4L2 read blocks at the negotiated device rate; do not add a second frame delay.
            except Exception as exc:
                if not self._stop.is_set():
                    self.hub.error(str(exc), "color")
                    self.get_logger().warning(f"RGB capture reconnecting: {exc}")
                    self._stop.wait(backoff)
                    backoff = min(backoff * 2.0, float(cfg["reconnect_backoff_sec"]))
            finally:
                if cap is not None:
                    cap.release()

    def _depth_v4l2_loop(self) -> None:
        cfg = self.config["capture"]
        device = self.config["source"]["depth_device"]
        width, height = int(cfg["depth_width"]), int(cfg["depth_height"])
        frame_pixels = width * height
        frame_bytes = frame_pixels * 2
        backoff = 0.2
        command = ["v4l2-ctl", "-d", device,
                   f"--set-fmt-video=width={width},height={height},pixelformat={str(cfg['depth_fourcc'])[:4]}",
                   f"--set-parm={max(float(cfg['depth_fps']), 1.0):g}", "--stream-mmap=4",
                   "--stream-count=10000000", "--stream-to=/dev/stdout"]
        while not self._stop.is_set():
            process = None
            try:
                self._apply_v4l2_controls(device, cfg["depth_controls"])
                process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=0)
                while not self._stop.is_set():
                    raw = _read_pipe_exact(process.stdout, frame_bytes, self._stop, float(cfg["capture_timeout_sec"]))
                    if raw is None:
                        break
                    if len(raw) != frame_bytes:
                        raise RuntimeError(f"depth stream ended after {len(raw)} of {frame_bytes} frame bytes")
                    depth = np.frombuffer(raw, dtype=np.uint16).reshape(height, width)
                    self._accept_depth(depth, time.time(), cfg["depth_frame_id"], "host_receive")
                    backoff = 0.2
            except Exception as exc:
                if not self._stop.is_set():
                    self.hub.error(str(exc), "depth")
                    self.get_logger().warning(f"depth capture reconnecting: {exc}")
                    self._stop.wait(backoff)
                    backoff = min(backoff * 2.0, float(cfg["reconnect_backoff_sec"]))
            finally:
                if process is not None:
                    if process.poll() is None:
                        process.terminate()
                        try:
                            process.wait(timeout=1.0)
                        except subprocess.TimeoutExpired:
                            process.kill()
                    if process.stdout:
                        process.stdout.close()

    def _mock_loop(self) -> None:
        cfg = self.config["capture"]
        width, height = int(cfg["color_width"]), int(cfg["color_height"])
        yy, xx = np.mgrid[:height, :width]
        depth = np.full((int(cfg["depth_height"]), int(cfg["depth_width"])), 1200, dtype=np.uint16)
        while not self._stop.is_set():
            stamp = time.time()
            frame = np.zeros((height, width, 3), dtype=np.uint8)
            frame[..., 0] = xx % 256
            frame[..., 1] = yy % 256
            frame[..., 2] = int(stamp * 25) % 256
            self._accept_color(frame, stamp, cfg["color_frame_id"], "mock_clock")
            self._accept_depth(depth, stamp, cfg["depth_frame_id"], "mock_clock")
            self._stop.wait(1.0 / max(float(cfg["color_fps"]), 1.0))

    def _accept_color(self, bgr: np.ndarray, stamp: float, frame_id: str, stamp_source: str) -> None:
        now = time.monotonic()
        self.runtime.update_color(bgr)
        self.latest_color = (bgr, stamp)
        self.latest_color_frame_id = frame_id
        self.latest_color_calibration_version = self.runtime.calibration.version
        self.hub.publish("color", bgr, stamp_sec=stamp, frame_id=frame_id, encoding="bgr8", width=bgr.shape[1], height=bgr.shape[0], extra={"timestamp_source": stamp_source})
        self._cloud_event.set()
        if self.config["streams"]["publish_color"] and self.ros_enabled and now - self._last_ros_publish["color"] >= 1.0 / max(float(self.runtime.config["streams"]["color_output_fps"]), 0.1):
            self.color_pub.publish(_image_message(bgr, "bgr8", frame_id, stamp))
            model = self.runtime.calibration.color
            if model:
                info = self._camera_info(model, frame_id, stamp)
                self.color_info_pub.publish(info)
                self.hub.publish("color_info", {"K": list(info.k), "D": list(info.d), "model": info.distortion_model, "calibrated": model.calibrated}, stamp_sec=stamp, frame_id=frame_id, encoding="json")
            self._last_ros_publish["color"] = now

    def _accept_depth(self, depth: np.ndarray, stamp: float, frame_id: str, stamp_source: str) -> None:
        now = time.monotonic()
        self.latest_depth = (depth, stamp)
        self.latest_depth_frame_id = frame_id
        self.latest_depth_calibration_version = self.runtime.calibration.version
        self.hub.publish("depth", depth, stamp_sec=stamp, frame_id=frame_id, encoding="16UC1", width=depth.shape[1], height=depth.shape[0], extra={"timestamp_source": stamp_source, "depth_scale_m": self.runtime.calibration.depth_scale_m})
        if self.config["streams"]["publish_depth"] and self.ros_enabled and now - self._last_ros_publish["depth"] >= 1.0 / max(float(self.runtime.config["streams"]["depth_output_fps"]), 0.1):
            self.depth_pub.publish(_image_message(depth, "16UC1", frame_id, stamp))
            model = self.runtime.calibration.depth
            if model:
                info = self._camera_info(model, frame_id, stamp)
                self.depth_info_pub.publish(info)
                self.hub.publish("depth_info", {"K": list(info.k), "D": list(info.d), "model": info.distortion_model, "calibrated": model.calibrated}, stamp_sec=stamp, frame_id=frame_id, encoding="json")
            self._last_ros_publish["depth"] = now
        if self.config["streams"]["publish_depth_color"]:
            valid = depth > 0
            if np.any(valid):
                low, high = np.percentile(depth[valid], [2, 98])
                high = max(float(high), float(low) + 1.0)
                normalized = np.clip((depth.astype(np.float32) - low) * 255.0 / (high - low), 0, 255).astype(np.uint8)
            else:
                normalized = np.zeros_like(depth, dtype=np.uint8)
            colorized = cv2.applyColorMap(normalized, cv2.COLORMAP_TURBO)
            self.hub.publish("depth_color", colorized, stamp_sec=stamp, frame_id=frame_id, encoding="bgr8", width=depth.shape[1], height=depth.shape[0], extra={"timestamp_source": stamp_source, "preview_only": True})
        self._cloud_event.set()

    def _publish_depth_cloud(self) -> None:
        self._cloud_event.set()

    def _cloud_worker(self) -> None:
        while not self._stop.is_set():
            self._cloud_event.wait(timeout=0.5)
            self._cloud_event.clear()
            if not self._stop.is_set():
                self._generate_depth_cloud()

    def _generate_depth_cloud(self) -> None:
        if not self._cloud_lock.acquire(blocking=False):
            return
        try:
            self._generate_depth_cloud_locked()
        finally:
            self._cloud_lock.release()

    def _generate_depth_cloud_locked(self) -> None:
        if not self.config["streams"]["publish_points"] or self.config["pointcloud"]["source"] != "depth":
            return
        now = time.monotonic()
        if now - self._last_cloud_attempt < 0.05:
            return
        self._last_cloud_attempt = now
        color_record, depth_record = self.latest_color, self.latest_depth
        if color_record is None or depth_record is None:
            return
        if depth_record[1] == self._last_cloud_depth_stamp:
            return
        if self.latest_color_calibration_version != self.runtime.calibration.version or self.latest_depth_calibration_version != self.runtime.calibration.version:
            return
        pair_delta_ms = abs(color_record[1] - depth_record[1]) * 1000.0
        pair_limit_ms = float(self.runtime.config["pointcloud"]["max_pair_delta_ms"])
        if pair_delta_ms > pair_limit_ms:
            self.hub.error(f"pointcloud unavailable: RGB/depth time delta {pair_delta_ms:.1f}ms exceeds limit {pair_limit_ms:g}ms", "points")
            return
        interval = 1.0 / max(float(self.runtime.config["pointcloud"]["output_fps"]), 0.1)
        if now < self._next_pointcloud_publish:
            self._last_cloud_depth_stamp = depth_record[1]
            return
        try:
            points = depth_to_xyzrgb(
                depth_record[0], color_record[0], self.runtime.calibration,
                stride=int(self.runtime.config["pointcloud"]["stride"]),
                min_depth_m=float(self.runtime.config["pointcloud"]["min_depth_m"]),
                max_depth_m=float(self.runtime.config["pointcloud"]["max_depth_m"]),
                max_pair_delta_ms=float(self.runtime.config["pointcloud"]["max_pair_delta_ms"]),
                color_stamp=color_record[1], depth_stamp=depth_record[1],
                max_points=int(self.runtime.config["pointcloud"]["max_points"]),
                roi=self.runtime.config["pointcloud"].get("roi", []),
            )
        except ValueError as exc:
            self.hub.error(f"pointcloud unavailable: {exc}", "points")
            if "time delta" not in str(exc):
                self._last_cloud_depth_stamp = depth_record[1]
            return
        self._last_cloud_depth_stamp = depth_record[1]
        target_frame = str(self.runtime.config["pointcloud"].get("target_frame", "")).strip()
        source_frame = self.latest_depth_frame_id
        if target_frame and target_frame != source_frame:
            try:
                transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
                t, q = transform.transform.translation, transform.transform.rotation
                xyz = points[:, :3] @ _quaternion_matrix(q.x, q.y, q.z, q.w).T + np.asarray([t.x, t.y, t.z])
                points[:, :3] = xyz
                source_frame = target_frame
            except Exception as exc:
                self.hub.error(f"pointcloud target transform unavailable: {exc}", "points")
                return
        from std_msgs.msg import Header
        point_stamp = _ros_stamp(depth_record[1])
        ros_header = Header(stamp=point_stamp, frame_id=source_frame)
        rgb = (points[:, 3].astype(np.uint32) << 16) | (points[:, 4].astype(np.uint32) << 8) | points[:, 5].astype(np.uint32)
        packed = np.empty((len(points), 4), dtype=np.float32)
        packed[:, :3] = points[:, :3]
        packed[:, 3] = rgb.view(np.float32)
        msg = PointCloud2()
        msg.header = ros_header
        msg.height = 1
        msg.width = len(points)
        msg.fields = [PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                      PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                      PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
                      PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1)]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = msg.point_step * msg.width
        msg.data = _ros_byte_array(packed)
        msg.is_dense = bool(np.isfinite(packed).all())
        if self.ros_enabled:
            self.points_pub.publish(msg)
        fields = [{"name": f.name, "offset": f.offset, "datatype": f.datatype, "count": f.count} for f in msg.fields]
        self.hub.publish("points", bytes(msg.data), stamp_sec=depth_record[1], frame_id=source_frame, encoding="PointCloud2", width=msg.width, height=msg.height, extra={"point_count": len(points), "fields": fields, "point_step": msg.point_step, "row_step": msg.row_step, "is_bigendian": msg.is_bigendian, "is_dense": msg.is_dense, "unit": "m", "timestamp_source": "host_receive"})
        self._last_ros_publish["points"] = now
        self._next_pointcloud_publish = time.monotonic() + interval

    def _ros_color(self, msg: Image) -> None:
        try:
            image, enc = _array_from_image(msg)
            self._accept_color(image, _stamp_seconds(msg.header.stamp) or time.time(), msg.header.frame_id, "ros2_header")
        except Exception as exc:
            self.hub.error(f"ROS 2 color input: {exc}", "color")

    def _ros_depth(self, msg: Image) -> None:
        try:
            image, _ = _array_from_image(msg)
            self._accept_depth(image, _stamp_seconds(msg.header.stamp) or time.time(), msg.header.frame_id, "ros2_header")
        except Exception as exc:
            self.hub.error(f"ROS 2 depth input: {exc}", "depth")

    def _publish_info_copy(self, msg: CameraInfo, channel: str) -> None:
        model = CameraModel(
            width=int(msg.width), height=int(msg.height), fx=float(msg.k[0]), fy=float(msg.k[4]),
            cx=float(msg.k[2]), cy=float(msg.k[5]), distortion=list(msg.d),
            distortion_model=str(msg.distortion_model), calibrated=bool(msg.k[0] and msg.k[4]),
        )
        if channel == "color":
            self.runtime.calibration.color = model
        else:
            self.runtime.calibration.depth = model
        publisher = self.color_info_pub if channel == "color" else self.depth_info_pub
        if self.ros_enabled:
            publisher.publish(msg)
        self.hub.publish(channel + "_info", {"K": list(msg.k), "D": list(msg.d), "model": msg.distortion_model, "calibrated": bool(msg.k[0])}, stamp_sec=_stamp_seconds(msg.header.stamp) or time.time(), frame_id=msg.header.frame_id, encoding="json")

    def _ros_cloud(self, msg: PointCloud2) -> None:
        if not self.config["streams"]["publish_points"]:
            return
        output = msg
        colorization_error = ""
        names = {field.name.lower() for field in msg.fields}
        if not names.intersection({"rgb", "rgba"}) and {"x", "y", "z"}.issubset(names):
            try:
                output = self._colorize_ros_cloud(msg)
            except (ValueError, RuntimeError) as exc:
                colorization_error = str(exc)
                colorization_error = str(exc)
        if self.ros_enabled:
            self.points_pub.publish(output)
        fields = [{"name": f.name, "offset": f.offset, "datatype": f.datatype, "count": f.count} for f in output.fields]
        extra = {"point_count": output.width * output.height, "fields": fields, "point_step": output.point_step, "row_step": output.row_step, "is_bigendian": output.is_bigendian, "is_dense": output.is_dense, "timestamp_source": "ros2_header"}
        if colorization_error:
            extra["colorization_error"] = colorization_error
        self.hub.publish("points", bytes(output.data), stamp_sec=_stamp_seconds(output.header.stamp), frame_id=output.header.frame_id, encoding="PointCloud2", width=output.width, height=output.height, extra=extra)
        if colorization_error:
            self.hub.error(f"external XYZ cloud was forwarded without RGB: {colorization_error}", "points")

    def _colorize_ros_cloud(self, msg: PointCloud2) -> PointCloud2:
        color_record = self.latest_color
        model = self.runtime.calibration.color
        cloud_stamp = _stamp_seconds(msg.header.stamp)
        if color_record is None or model is None or not model.calibrated:
            raise ValueError("RGB frame and valid RGB intrinsics are required")
        if cloud_stamp <= 0:
            raise ValueError("PointCloud2 has no usable timestamp")
        delta_ms = abs(color_record[1] - cloud_stamp) * 1000.0
        max_delta = float(self.runtime.config["pointcloud"]["max_pair_delta_ms"])
        if delta_ms > max_delta:
            raise ValueError(f"RGB/PointCloud2 time delta {delta_ms:.1f}ms exceeds limit {max_delta:g}ms")
        xyz = pointcloud_xyz(msg)
        color_frame = self.latest_color_frame_id
        if not msg.header.frame_id:
            raise ValueError("PointCloud2 frame_id is empty")
        if msg.header.frame_id == color_frame:
            xyz_color = xyz
        else:
            try:
                transform = self.tf_buffer.lookup_transform(
                    color_frame, msg.header.frame_id,
                    rclpy.time.Time.from_msg(msg.header.stamp),
                )
            except Exception as exc:
                raise ValueError(f"TF {msg.header.frame_id} -> {color_frame} unavailable") from exc
            t, q = transform.transform.translation, transform.transform.rotation
            xyz_color = xyz @ _quaternion_matrix(q.x, q.y, q.z, q.w).T + np.asarray([t.x, t.y, t.z])
        rgb = sample_xyz_rgb(xyz_color, color_record[0], model).reshape(msg.height, msg.width, 3)
        words = (rgb[..., 0].astype(np.uint32) << 16) | (rgb[..., 1].astype(np.uint32) << 8) | rgb[..., 2].astype(np.uint32)
        packed_rgb_bytes = words.astype(">u4" if msg.is_bigendian else "<u4").view(np.uint8).reshape(msg.height, msg.width, 4)
        source = np.ndarray(
            (msg.height, msg.width, msg.point_step), dtype=np.uint8, buffer=msg.data,
            strides=(msg.row_step, msg.point_step, 1),
        )
        point_step = int(msg.point_step) + 4
        combined = np.zeros((msg.height, msg.width, point_step), dtype=np.uint8)
        combined[:, :, :msg.point_step] = source
        combined[:, :, msg.point_step:] = packed_rgb_bytes
        output = PointCloud2()
        output.header = msg.header
        output.height, output.width = msg.height, msg.width
        output.fields = list(msg.fields) + [PointField(name="rgb", offset=msg.point_step, datatype=PointField.FLOAT32, count=1)]
        output.is_bigendian = msg.is_bigendian
        output.point_step = point_step
        output.row_step = point_step * output.width
        output.data = _ros_byte_array(combined)
        output.is_dense = msg.is_dense
        return output

    def _camera_info(self, model: CameraModel, frame_id: str, stamp: float) -> CameraInfo:
        data = camera_info_dict(model, frame_id, stamp)
        msg = CameraInfo()
        msg.header.frame_id = frame_id
        msg.header.stamp = _ros_stamp(stamp)
        msg.width, msg.height = data["width"], data["height"]
        msg.distortion_model = data["distortion_model"]
        msg.d, msg.k, msg.r, msg.p = data["d"], data["k"], data["r"], data["p"]
        return msg

    def _publish_status(self) -> None:
        if not self.ros_enabled:
            return
        status = self.hub.status(self.runtime.config, self.runtime.calibration.version)
        msg = String(); msg.data = json.dumps(status, separators=(",", ":"))
        self.status_pub.publish(msg)

    def _srv_calibration_start(self, request, response):
        try:
            session = self.runtime.start_calibration({})
            self._calibration_session_id = session["session_id"]
            response.success = True; response.message = json.dumps(session)
        except Exception as exc:
            response.success = False; response.message = str(exc)
        return response

    def _srv_calibration_capture(self, request, response):
        try:
            result = self.runtime.capture_calibration(self._calibration_session_id)
            response.success = True; response.message = json.dumps(result)
        except Exception as exc:
            response.success = False; response.message = str(exc)
        return response

    def _srv_calibration_solve(self, request, response):
        try:
            result = self.runtime.solve_calibration(self._calibration_session_id)
            response.success = True; response.message = json.dumps(result)
        except Exception as exc:
            response.success = False; response.message = str(exc)
        return response

    def _calibration_goal_request(self, request):
        if not request.session_id and not self._calibration_session_id:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _calibration_cancel_request(self, goal_handle):
        return CancelResponse.ACCEPT

    def _execute_calibration_action(self, goal_handle):
        result = SolveRgbCalibration.Result()
        session_id = goal_handle.request.session_id or self._calibration_session_id
        try:
            self.runtime.solve_calibration(session_id)
            while True:
                state = self.runtime.calibration_solve_status(session_id)
                feedback = SolveRgbCalibration.Feedback()
                feedback.state = str(state.get("state", "solving"))
                feedback.processed_frames = int(state.get("processed_frames", 0))
                feedback.total_frames = int(state.get("total_frames", 0))
                feedback.progress = float(state.get("progress", 0.0))
                goal_handle.publish_feedback(feedback)
                if goal_handle.is_cancel_requested:
                    self.runtime.cancel_calibration_solve(session_id)
                if session_id not in self.runtime._solving:
                    break
                time.sleep(0.05)
            state = self.runtime.calibration_solve_status(session_id)
            if state.get("state") == "cancelled" or goal_handle.is_cancel_requested:
                result.success = False
                result.message = state.get("error", "calibration solve cancelled")
                goal_handle.canceled()
                return result
            candidate = self.runtime.result_for_session(session_id)
            if candidate is None:
                result.success = False
                result.message = state.get("error", "calibration solve failed")
                goal_handle.abort()
                return result
            result.success = True
            result.calibration_id = candidate["calibration_id"]
            result.rms_px = float(candidate["rms_px"])
            result.used_frames = int(candidate["used_frames"])
            result.accepted = bool(candidate["accepted"])
            result.message = "candidate is accepted" if result.accepted else "candidate saved but below configured quality thresholds"
            goal_handle.succeed()
            return result
        except Exception as exc:
            result.success = False
            result.message = str(exc)
            goal_handle.abort()
            return result

    def _apply_calibration_msg(self, msg: String) -> None:
        try:
            result = self.runtime.apply_calibration(msg.data.strip())
            out = {"ok": True, **result}
        except Exception as exc:
            out = {"ok": False, "error": str(exc)}
        state = String(); state.data = json.dumps(out)
        self.calibration_status_pub.publish(state)

    def destroy_node(self) -> bool:
        self.stop_capture()
        if self.web_server:
            self.web_server.close()
        return super().destroy_node()

    def stop_capture(self) -> None:
        self._stop.set()
        self._cloud_event.set()
        deadline = time.monotonic() + 5.0
        for thread in self._capture_threads:
            while thread.is_alive() and time.monotonic() < deadline:
                try:
                    thread.join(timeout=min(0.25, max(deadline - time.monotonic(), 0.0)))
                except KeyboardInterrupt:
                    continue


def main(args=None) -> None:
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--config", default=None)
    known, ros_args = parser.parse_known_args(args)
    rclpy.init(args=ros_args)
    node = CameraNode(known.config)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        node.get_logger().error(str(exc))
    finally:
        node.stop_capture()
        try:
            executor.shutdown(timeout_sec=2.0)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
