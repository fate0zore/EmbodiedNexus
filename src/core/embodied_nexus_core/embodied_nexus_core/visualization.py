from __future__ import annotations

import inspect
import os
import re
import signal
import socket
import subprocess
import time
from pathlib import Path
from typing import Any, Callable


SUPPORTED_TYPES = {
    "std_msgs/msg/String", "tf2_msgs/msg/TFMessage", "sensor_msgs/msg/PointCloud2",
    "sensor_msgs/msg/LaserScan", "sensor_msgs/msg/Image", "sensor_msgs/msg/JointState",
}


def _explicit_qos_available() -> bool:
    """Detect the installed rosbridge capability instead of trusting its version tag.

    Args:
        无参数。

    Returns:
        布尔值，表示检测已安装 rosbridge 是否支持所需 QoS 能力是否成功或成立。
    """
    try:
        from rosbridge_library.capabilities.subscribe import Subscription
        return "qos" in inspect.signature(Subscription.subscribe).parameters
    except Exception:
        return False


class RosbridgeRuntime:
    """Own the optional, loopback-only rosbridge process used by the viewer."""

    def __init__(self, settings: dict[str, Any], topic_reader: Callable[[], list[tuple[str, str]]], *,
                 popen=subprocess.Popen, wait_sec: float = 30.0):
        """初始化对象并保存构造参数或依赖。

        Args:
            settings: settings，作为初始化对象并保存构造参数或依赖所需的输入。 类型为 dict[str, Any]。
            topic_reader: 话题reader，作为初始化对象并保存构造参数或依赖所需的输入。 类型为 Callable[[], list[tuple[str, str]]]。
            popen: popen，作为初始化对象并保存构造参数或依赖所需的输入。
            wait_sec: 等待sec，作为初始化对象并保存构造参数或依赖所需的输入。 类型为 float。

        Returns:
            无返回值。
        """
        self.enabled = bool(settings["enabled"])
        self.port = int(settings["port"])
        self.resource_packages = tuple(settings["resource_packages"])
        self.explicit_qos = _explicit_qos_available()
        self._topic_reader = topic_reader
        self._process = None
        self._model_process = None
        self._error = ""
        self._model_error = ""
        model_settings = settings.get("robot_model", {})
        self.robot_model_enabled = bool(model_settings.get("enabled", False))
        self._configured_joint_states_topic = str(
            model_settings.get("joint_states_topic", "/embodied_nexus/arms/arm_main/joint_states")
        )
        if not self.enabled:
            self._error = "可视化已在平台配置中关闭"
            return
        if self.robot_model_enabled:
            try:
                self._model_process = popen([
                    "ros2", "launch", "embodied_nexus_sim_description",
                    "real_robot_state_publisher.launch.py",
                    f"joint_states_topic:={self._configured_joint_states_topic}",
                ], stdin=subprocess.DEVNULL, stdout=None, stderr=None, start_new_session=True)
                # Let an immediately failing launch report its exit while
                # keeping Core startup independent from model availability.
                time.sleep(0.1)
                if self._model_process.poll() is not None:
                    self._model_error = "robot_state_publisher 启动失败，请检查描述包和 ROS 依赖"
                    self._model_process = None
            except (OSError, ValueError, KeyError) as exc:
                self._model_error = f"无法启动 robot_state_publisher：{exc}"
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
        """就绪当前设备或操作。

        Args:
            无参数。

        Returns:
            布尔值，表示就绪当前设备或操作是否成功或成立。
        """
        if not self.enabled or self._process is None or self._process.poll() is not None:
            return False
        try:
            with socket.create_connection(("127.0.0.1", self.port), timeout=0.15):
                return True
        except OSError:
            return False

    def topic_types(self) -> dict[str, str]:
        """话题类型。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, str]。
        """
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
        """获取设备能力与静态信息。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any]。
        """
        available = self.ready()
        error = ""
        if not available:
            error = self._error
            if not error:
                error = "rosbridge 进程已退出" if self._process is not None and self._process.poll() is not None else "rosbridge 未监听"
        model_process_alive = self._model_process is not None and self._model_process.poll() is None
        model_topic_available = any(
            name.rstrip("/") == "/robot_description" and "std_msgs/msg/String" in types
            for name, types in self._safe_topics()
        )
        model_error = self._model_error
        if self.robot_model_enabled and not model_error and not model_process_alive:
            model_error = "robot_state_publisher 进程已退出"
        elif self.robot_model_enabled and not model_error and not model_topic_available:
            model_error = "等待 /robot_description 话题"
        return {
            "enabled": self.enabled,
            "available": available,
            "port": self.port,
            "resource_packages": list(self.resource_packages),
            "explicit_qos": self.explicit_qos,
            "error": error,
            "robot_model": {
                "enabled": self.robot_model_enabled,
                "available": model_process_alive and model_topic_available,
                "description_topic": "/robot_description",
                "joint_states_topic": self._configured_joint_states_topic,
                "error": model_error if self.robot_model_enabled else "",
            },
            "topics": [{"name": name, "type": msg_type}
                       for name, msg_type in sorted(self.topic_types().items())],
            "presets": [
                {"id": "devices", "label": "现有设备", "fixed_frame": "base_link",
                 "robot_description": "/robot_description"},
                {"id": "gazebo_arachne", "label": "Gazebo Arachne", "fixed_frame": "base_link",
                 "robot_description": "/sim/arachne/robot_description"},
            ],
        }

    def close(self) -> None:
        """关闭连接并释放相关资源。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        self._close_process("_model_process")
        self._close_process("_process")

    def _safe_topics(self) -> list[tuple[str, list[str]]]:
        """过滤并规范化允许转发的 ROS 话题。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。 返回类型为 list[tuple[str, list[str]]]。
        """
        try:
            return self._topic_reader()
        except Exception:
            return []

    def _close_process(self, attribute: str) -> None:
        """关闭处理。

        Args:
            attribute: attribute，作为关闭处理所需的输入。 类型为 str。

        Returns:
            无返回值。
        """
        process = getattr(self, attribute)
        setattr(self, attribute, None)
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
