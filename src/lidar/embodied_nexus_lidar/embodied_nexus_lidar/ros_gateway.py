from __future__ import annotations

import json
import os
import threading
from pathlib import Path
from typing import Any

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import String

from embodied_nexus_lidar_interfaces.srv import (
    AcquireLidarControl, ApplyLidarConfiguration, GetLidarConfiguration,
    GetLidarInfo, GetLidarState, ReleaseLidarControl, RenewLidarControl,
    SetLidarCapture, SetLidarMotor, SetLidarMotorSpeed,
    ValidateLidarConfiguration,
)

from .config import load_config, validate_config
from .geometry import CloudFrame, ScanFrame, cloud_from_ros
from .runtime import StreamHub
from .web_server import LidarWebServer


class LidarRosGateway(Node):
    """HTTP/WebSocket facade; device control and UDP remain in the lidar node."""

    def __init__(self, config_path: str | None = None, web_port: int = 0) -> None:
        super().__init__("embodied_nexus_lidar_web")
        self.declare_parameter("config_file", config_path or _default_config_path())
        self.declare_parameter("web_port", int(web_port))
        self.declare_parameter("interface_mode", "")
        self.declare_parameter("web_bind_host", "")
        self.declare_parameter("web_auth_token_env", "")
        self.config = load_config(str(self.get_parameter("config_file").value))
        mode = str(self.get_parameter("interface_mode").value).strip()
        if mode:
            self.config["interface_mode"] = mode
        validate_config(self.config)
        override_port = int(self.get_parameter("web_port").value)
        if override_port > 0:
            self.config["web"]["port"] = override_port
        bind_host = str(self.get_parameter("web_bind_host").value).strip()
        if bind_host:
            self.config["web"]["bind_host"] = bind_host
        token_env = str(self.get_parameter("web_auth_token_env").value).strip()
        if token_env:
            self.config["web"]["auth_token_env"] = token_env
        self.device_id = self.config["device_id"]
        self.prefix = f"/embodied_nexus/lidars/{self.device_id}"
        stream_base = self.prefix if self.config["interface_mode"] in {"ros2", "both"} else f"{self.prefix}/_internal"
        points_topic = _topic(self.config["source"]["output_topic"], stream_base)
        scan_topic = _topic(self.config["source"]["scan_output_topic"], stream_base)
        self._state: dict[str, Any] = {"device_id": self.device_id, "backend": self.config["backend"], "connected": False}
        self._state_lock = threading.RLock()
        self._web_clients = 0
        self._clients_lock = threading.Lock()
        self.hub = StreamHub()
        self.create_subscription(PointCloud2, points_topic, self._on_cloud, qos_profile_sensor_data)
        self.create_subscription(LaserScan, scan_topic, self._on_scan, qos_profile_sensor_data)
        self.create_subscription(String, f"{self.prefix}/_internal/state_json", self._on_state, 10)
        self._service_types = {
            "info": (GetLidarInfo, f"{self.prefix}/info"),
            "state": (GetLidarState, f"{self.prefix}/get_state"),
            "config": (GetLidarConfiguration, f"{self.prefix}/config/get"),
            "validate": (ValidateLidarConfiguration, f"{self.prefix}/config/validate"),
            "apply": (ApplyLidarConfiguration, f"{self.prefix}/config/apply"),
            "capture": (SetLidarCapture, f"{self.prefix}/capture"),
            "acquire": (AcquireLidarControl, f"{self.prefix}/control/acquire"),
            "renew": (RenewLidarControl, f"{self.prefix}/control/renew"),
            "release": (ReleaseLidarControl, f"{self.prefix}/control/release"),
            "motor": (SetLidarMotor, f"{self.prefix}/motor"),
            "speed": (SetLidarMotorSpeed, f"{self.prefix}/motor/speed"),
        }
        self._service_clients = {
            key: self.create_client(kind, name)
            for key, (kind, name) in self._service_types.items()
        }
        web = self.config["web"]
        token = os.environ.get(str(web["auth_token_env"]), "")
        self.web = LidarWebServer(self, str(web["bind_host"]), int(self.config["web"]["port"]), token)
        self.web.start()
        self.get_logger().info(f"lidar Web API listening on {web['bind_host']}:{self.config['web']['port']}")

    def _call(self, name: str, request: Any, timeout: float = 5.0):
        client = self._service_clients[name]
        if not client.wait_for_service(timeout_sec=timeout):
            raise TimeoutError(f"ROS service {self._service_types[name][1]} is unavailable")
        done = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _future: done.set())
        if not done.wait(timeout):
            raise TimeoutError(f"ROS service {self._service_types[name][1]} timed out")
        result = future.result()
        if result is None:
            raise RuntimeError(f"ROS service {self._service_types[name][1]} returned no response")
        return result

    @staticmethod
    def _require(response, success: bool, error: str = ""):
        if not success:
            if "already held" in error.lower():
                raise RuntimeError(error)
            if "lease" in error.lower() or "held" in error.lower():
                raise PermissionError(error)
            raise RuntimeError(error or "lidar request failed")
        return response

    def info(self) -> dict[str, Any]:
        response = self._call("info", GetLidarInfo.Request())
        self._require(response, response.success, response.error)
        return json.loads(response.descriptor_json)

    def status(self) -> dict[str, Any]:
        response = self._call("state", GetLidarState.Request())
        self._require(response, response.success, response.error)
        state = json.loads(response.state_json)
        state["dropped_frames"] = self.hub.dropped()
        return state

    def get_config(self) -> dict[str, Any]:
        response = self._call("config", GetLidarConfiguration.Request())
        self._require(response, response.success, response.error)
        return json.loads(response.config_json)

    def validate_config(self, patch: dict[str, Any]) -> dict[str, Any]:
        request = ValidateLidarConfiguration.Request()
        request.patch_json = json.dumps(patch)
        response = self._call("validate", request)
        self._require(response, response.valid, response.error)
        return json.loads(response.config_json)

    def apply_config(self, patch: dict[str, Any]) -> dict[str, Any]:
        request = ApplyLidarConfiguration.Request()
        request.patch_json = json.dumps(patch)
        response = self._call("apply", request)
        self._require(response, response.success, response.error)
        self.config = json.loads(response.config_json)
        return self.config

    def set_capture(self, capturing: bool) -> dict[str, Any]:
        request = SetLidarCapture.Request()
        request.capturing = bool(capturing)
        response = self._call("capture", request)
        self._require(response, response.success, response.error)
        return {"capturing": response.capturing}

    def acquire(self, owner: str, ttl_sec: float | None = None) -> dict[str, Any]:
        request = AcquireLidarControl.Request()
        request.owner = owner
        request.ttl_sec = float(ttl_sec or 0.0)
        response = self._call("acquire", request)
        self._require(response, response.success, response.error)
        return {"lease_id": response.lease_id, "expires_in_sec": response.expires_in_sec}

    def renew(self, lease_id: str) -> dict[str, Any]:
        request = RenewLidarControl.Request()
        request.lease_id = lease_id
        response = self._call("renew", request)
        self._require(response, response.success, response.error)
        return {"expires_in_sec": response.expires_in_sec}

    def release(self, lease_id: str) -> dict[str, Any]:
        request = ReleaseLidarControl.Request()
        request.lease_id = lease_id
        response = self._call("release", request)
        self._require(response, response.success, response.error)
        return {"released": True, "stop_sent": response.stop_sent, "error": response.error}

    def set_motor(self, lease_id: str, enabled: bool) -> dict[str, Any]:
        request = SetLidarMotor.Request()
        request.lease_id, request.enabled = lease_id, bool(enabled)
        response = self._call("motor", request)
        self._require(response, response.success, response.error)
        return {"command_sent": response.command_sent,
                "hardware_state_confirmed": response.hardware_state_confirmed,
                "motor_state": response.motor_state}

    def set_speed(self, lease_id: str, speed_hz: int) -> dict[str, Any]:
        request = SetLidarMotorSpeed.Request()
        request.lease_id, request.speed_hz = lease_id, int(speed_hz)
        response = self._call("speed", request)
        self._require(response, response.success, response.error)
        return {"command_sent": response.command_sent,
                "hardware_state_confirmed": response.hardware_state_confirmed,
                "requested_speed_hz": response.requested_speed_hz,
                "actual_speed_known": response.actual_speed_known,
                "actual_speed_hz": response.actual_speed_hz if response.actual_speed_known else None}

    def acquire_client(self) -> bool:
        with self._clients_lock:
            if self._web_clients >= int(self.config["streams"]["max_clients"]):
                return False
            self._web_clients += 1
            return True

    def release_client(self) -> None:
        with self._clients_lock:
            self._web_clients = max(self._web_clients - 1, 0)

    def _on_cloud(self, message: PointCloud2) -> None:
        try:
            self.hub.publish("points", cloud_from_ros(message))
        except Exception as exc:
            self.get_logger().warning(f"Web point-cloud cache rejected input: {exc}")

    def _on_scan(self, message: LaserScan) -> None:
        stamp = float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1e-9
        self.hub.publish("scan", ScanFrame(
            stamp, message.header.frame_id, message.angle_min, message.angle_max,
            message.angle_increment, message.time_increment, message.scan_time,
            message.range_min, message.range_max, list(message.ranges), list(message.intensities),
        ))

    def _on_state(self, message: String) -> None:
        try:
            state = json.loads(message.data)
        except (TypeError, ValueError):
            return
        with self._state_lock:
            self._state = state

    def close(self) -> None:
        self.web.close()
        super().destroy_node()


def _default_config_path() -> str:
    try:
        from ament_index_python.packages import get_package_share_directory
        return str(Path(get_package_share_directory("embodied_nexus_lidar")) / "config" / "c16.yaml")
    except Exception:
        return str(Path(__file__).resolve().parents[1] / "config" / "c16.yaml")


def _topic(path: str, namespace: str) -> str:
    return path if path.startswith("/") else f"{namespace}/{path.strip('/')}"


def main(argv=None) -> int:
    rclpy.init(args=argv, signal_handler_options=SignalHandlerOptions.NO)
    node = LidarRosGateway()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown(timeout_sec=2.0)
        node.close()
        if rclpy.ok():
            rclpy.shutdown()
    return 0
