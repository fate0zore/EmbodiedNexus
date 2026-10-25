from __future__ import annotations

import os
import re
import signal
import socket
import subprocess
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Callable


SUPPORTED_TYPES = {
    "std_msgs/msg/String", "tf2_msgs/msg/TFMessage", "sensor_msgs/msg/PointCloud2",
    "sensor_msgs/msg/LaserScan", "sensor_msgs/msg/Image", "sensor_msgs/msg/JointState",
}


def _explicit_qos_available() -> bool:
    """The rosbridge subscribe qos field was added after Humble's 2.0.8 release."""
    try:
        from ament_index_python.packages import get_package_share_directory
        package_xml = Path(get_package_share_directory("rosbridge_library")) / "package.xml"
        version = ET.parse(package_xml).getroot().findtext("version", "0")
        numbers = tuple(int(part) for part in re.findall(r"\d+", version)[:3])
        return numbers >= (2, 1, 0)
    except Exception:
        return False


class RosbridgeRuntime:
    """Own the optional, loopback-only rosbridge process used by the viewer."""

    def __init__(self, settings: dict[str, Any], topic_reader: Callable[[], list[tuple[str, str]]], *,
                 popen=subprocess.Popen, wait_sec: float = 5.0):
        self.enabled = bool(settings["enabled"])
        self.port = int(settings["port"])
        self.resource_packages = tuple(settings["resource_packages"])
        self.explicit_qos = _explicit_qos_available()
        self._topic_reader = topic_reader
        self._process = None
        self._error = ""
        if not self.enabled:
            self._error = "可视化已在平台配置中关闭"
            return
        try:
            self._process = popen([
                "ros2", "launch", "rosbridge_server", "rosbridge_websocket_launch.xml",
                f"address:=127.0.0.1", f"port:={self.port}",
                "max_message_size:=33554432", "incoming_queue_size:=20", "write_queue_size:=2",
                # Empty ROS launch parameter values are inferred as an empty
                # array on Humble, which leaves these parameters uninitialized.
                # Keep the rosapi write/service globs explicitly non-matching;
                # Core's WebSocket proxy independently enforces read-only access.
                "topics_pub_glob:=__embodied_nexus_core_read_only__",
                "services_glob:=__embodied_nexus_core_read_only__",
            ], stdin=subprocess.DEVNULL, stdout=None, stderr=None, start_new_session=True)
            deadline = time.monotonic() + wait_sec
            while time.monotonic() < deadline:
                if self._process.poll() is not None:
                    self._error = "rosbridge 启动失败，请安装 ros-humble-rosbridge-server >= 2.0.8"
                    self._process = None
                    break
                try:
                    with socket.create_connection(("127.0.0.1", self.port), timeout=0.1):
                        break
                except OSError:
                    time.sleep(0.05)
            else:
                self._error = "rosbridge 未能在启动时限内监听"
                self.close()
        except (OSError, ValueError) as exc:
            self._error = f"无法启动 rosbridge：{exc}"
            self._process = None

    def ready(self) -> bool:
        if not self.enabled or self._process is None or self._process.poll() is not None:
            return False
        try:
            with socket.create_connection(("127.0.0.1", self.port), timeout=0.15):
                return True
        except OSError:
            return False

    def topic_types(self) -> dict[str, str]:
        try:
            result = {}
            for name, types in self._topic_reader():
                for msg_type in types:
                    if msg_type not in SUPPORTED_TYPES:
                        continue
                    if msg_type == "std_msgs/msg/String" and not name.rstrip("/").endswith("robot_description"):
                        continue
                    result[name] = msg_type
            return result
        except Exception:
            return {}

    def info(self) -> dict[str, Any]:
        available = self.ready()
        error = ""
        if not available:
            error = self._error
            if not error:
                error = "rosbridge 进程已退出" if self._process is not None and self._process.poll() is not None else "rosbridge 未监听"
        return {
            "enabled": self.enabled,
            "available": available,
            "port": self.port,
            "resource_packages": list(self.resource_packages),
            "explicit_qos": self.explicit_qos,
            "error": error,
            "topics": [{"name": name, "type": msg_type}
                       for name, msg_type in sorted(self.topic_types().items())],
            "presets": [
                {"id": "devices", "label": "现有设备", "fixed_frame": "base_link",
                 "robot_description": "/robot_description"},
                {"id": "gazebo_arachne", "label": "Gazebo Arachne", "fixed_frame": "odom",
                 "robot_description": "/sim/arachne/robot_description"},
            ],
        }

    def close(self) -> None:
        process, self._process = self._process, None
        if process is None or process.poll() is not None:
            return
        if os.name == "nt":
            process.terminate()
        else:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                return
        try:
            process.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            if os.name == "nt":
                process.kill()
            else:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    return
            process.wait(timeout=1.0)
