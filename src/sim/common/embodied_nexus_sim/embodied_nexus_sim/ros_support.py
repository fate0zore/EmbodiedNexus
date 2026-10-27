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
    """从选项中读取并规范化 ROS 话题名称。

    Args:
        options: 选项，作为执行 configured_topic 对应的处理流程所需的输入。 类型为 dict[str, Any]。
        key: 按键，作为执行 configured_topic 对应的处理流程所需的输入。 类型为 str。
        default: 获取默认，作为执行 configured_topic 对应的处理流程所需的输入。 类型为 str。

    Returns:
        本次处理产生的结果值。 返回类型为 str。
    """
    topic = str(options.get(key, default)).strip()
    if not topic.startswith("/") or "//" in topic:
        raise ValueError(f"backend_options.{key} must be an absolute ROS topic")
    return topic


def validate_options(options: dict[str, Any], allowed: set[str]) -> None:
    """验证选项。

    Args:
        options: 选项，作为验证选项所需的输入。 类型为 dict[str, Any]。
        allowed: allowed，作为验证选项所需的输入。 类型为 set[str]。

    Returns:
        无返回值。
    """
    unknown = sorted(set(options) - allowed)
    if unknown:
        raise ValueError(f"unsupported simulation backend option: {unknown[0]}")


def finite_values(values, length: int, label: str) -> list[float]:
    """解析指定长度的数值序列并校验全部为有限数。

    Args:
        values: 要应用、比较或转换的一组输入值。
        length: length，作为执行 finite_values 对应的处理流程所需的输入。 类型为 int。
        label: label，作为执行 finite_values 对应的处理流程所需的输入。 类型为 str。

    Returns:
        本次处理产生的结果值。 返回类型为 list[float]。
    """
    result = [float(value) for value in values]
    if len(result) != length or not all(math.isfinite(value) for value in result):
        raise ValueError(f"{label} must contain {length} finite values")
    return result


def yaw_from_quaternion(q) -> float:
    """从四元数计算绕 Z 轴的偏航角。

    Args:
        q: q，作为执行 yaw_from_quaternion 对应的处理流程所需的输入。

    Returns:
        本次处理产生的结果值。 返回类型为 float。
    """
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class SimTimestampGuard:
    """Detect a simulator time rewind while allowing repeated timestamps."""

    def __init__(self) -> None:
        """初始化对象并保存构造参数或依赖。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        self._last_stamp_sec: float | None = None
        self._lock = threading.Lock()

    def observe(self, stamp) -> bool:
        """记录时钟样本并检测时钟跳变。

        Args:
            stamp: 时间戳，作为执行 observe 对应的处理流程所需的输入。

        Returns:
            布尔值，表示执行 observe 对应的处理流程是否成功或成立。
        """
        current = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        with self._lock:
            jumped_back = (self._last_stamp_sec is not None and
                           current < self._last_stamp_sec - 1e-9)
            self._last_stamp_sec = current
            return jumped_back


class SimRosNode:
    """A device-owned ROS endpoint. Its ROS time follows the simulator clock."""

    def __init__(self, device_id: str, suffix: str) -> None:
        """初始化对象并保存构造参数或依赖。

        Args:
            device_id: 设备标识符，作为初始化对象并保存构造参数或依赖所需的输入。 类型为 str。
            suffix: suffix，作为初始化对象并保存构造参数或依赖所需的输入。 类型为 str。

        Returns:
            无返回值。
        """
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
        """关闭ros。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        self.executor.shutdown(timeout_sec=1.0)
        self._spin_thread.join(timeout=1.0)
        self.executor.remove_node(self.node)
        self.node.destroy_node()

    def wait_for(self, condition, timeout_sec: float = 5.0) -> bool:
        """等待的。

        Args:
            condition: condition，作为等待的所需的输入。
            timeout_sec: 本次调用的最长等待时长，单位为秒。 类型为 float。

        Returns:
            布尔值，表示等待的是否成功或成立。
        """
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if condition():
                return True
            time.sleep(0.02)
        return bool(condition())
