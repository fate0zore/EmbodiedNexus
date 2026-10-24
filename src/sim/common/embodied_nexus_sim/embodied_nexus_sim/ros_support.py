from __future__ import annotations

import math
import re
import threading
import time
from typing import Any

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter


def configured_topic(options: dict[str, Any], key: str, default: str) -> str:
    topic = str(options.get(key, default)).strip()
    if not topic.startswith("/") or "//" in topic:
        raise ValueError(f"backend_options.{key} must be an absolute ROS topic")
    return topic


def validate_options(options: dict[str, Any], allowed: set[str]) -> None:
    unknown = sorted(set(options) - allowed)
    if unknown:
        raise ValueError(f"unsupported simulation backend option: {unknown[0]}")


def finite_values(values, length: int, label: str) -> list[float]:
    result = [float(value) for value in values]
    if len(result) != length or not all(math.isfinite(value) for value in result):
        raise ValueError(f"{label} must contain {length} finite values")
    return result


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class SimTimestampGuard:
    """Detect a simulator time rewind while allowing repeated timestamps."""

    def __init__(self) -> None:
        self._last_stamp_sec: float | None = None
        self._lock = threading.Lock()

    def observe(self, stamp) -> bool:
        current = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        with self._lock:
            jumped_back = (self._last_stamp_sec is not None and
                           current < self._last_stamp_sec - 1e-9)
            self._last_stamp_sec = current
            return jumped_back


class SimRosNode:
    """A device-owned ROS endpoint. Its ROS time follows the simulator clock."""

    def __init__(self, device_id: str, suffix: str) -> None:
        if not rclpy.ok():
            raise RuntimeError("ROS 2 must be initialized before opening a ros2_sim backend")
        safe_id = re.sub(r"[^A-Za-z0-9_]", "_", device_id)
        self.node = Node(f"nexus_sim_{safe_id}_{suffix}", parameter_overrides=[
            Parameter("use_sim_time", value=True),
        ])
        self.executor = MultiThreadedExecutor(num_threads=2)
        self.executor.add_node(self.node)
        self._spin_thread = threading.Thread(
            target=self.executor.spin, name=f"nexus-sim-{safe_id}-{suffix}", daemon=True
        )
        self._spin_thread.start()

    def close_ros(self) -> None:
        self.executor.shutdown(timeout_sec=1.0)
        self._spin_thread.join(timeout=1.0)
        self.executor.remove_node(self.node)
        self.node.destroy_node()

    def wait_for(self, condition, timeout_sec: float = 5.0) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if condition():
                return True
            time.sleep(0.02)
        return bool(condition())
