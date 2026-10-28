from __future__ import annotations

import math
import threading
import time

from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from embodied_nexus_gripper import GripperFeedback

from ...ros_support import SimRosNode, SimTimestampGuard, configured_topic, validate_options


class Ros2SimGripperBackend:
    """MS42DC motor-frame adapter driven by measured gripper joint state."""

    feedback_is_simulated = True

    def __init__(self, config: dict) -> None:
        """初始化对象并保存构造参数或依赖。

        Args:
            config: 设备或平台配置对象，提供本次操作所需参数。 类型为 dict。

        Returns:
            无返回值。
        """
        self.config = config
        options = config["backend_options"]
        validate_options(options, {"namespace", "joint_states_topic", "gripper_trajectory_topic",
                                   "gripper_joint_names", "finger_travel_rad", "speed_at_150_rad_sec"})
        root = str(options.get("namespace", "/sim/arachne")).rstrip("/")
        if not root.startswith("/"):
            raise ValueError("backend_options.namespace must be an absolute ROS namespace")
        self.state_topic = configured_topic(options, "joint_states_topic", f"{root}/joint_states")
        self.command_topic = configured_topic(options, "gripper_trajectory_topic", f"{root}/gripper_controller/joint_trajectory")
        self.joints = list(options.get("gripper_joint_names", ["ms42dc_left_finger_joint", "ms42dc_right_finger_joint"]))
        if len(self.joints) != 2 or len(set(self.joints)) != 2:
            raise ValueError("backend_options.gripper_joint_names must contain two unique joints")
        self.travel_rad = float(options.get("finger_travel_rad", 0.6))
        self.speed_at_150 = float(options.get("speed_at_150_rad_sec", 0.6))
        if not math.isfinite(self.travel_rad) or self.travel_rad <= 0 or not math.isfinite(self.speed_at_150) or self.speed_at_150 <= 0:
            raise ValueError("gripper simulation travel and speed must be finite and positive")
        self.feedback_is_simulated = True
        self.opened = False
        self._sample = None
        self._receipt = 0.0
        self._lock = threading.RLock()
        self._stamp_guard = SimTimestampGuard()
        self._clock_jump_pending = False

    def open(self) -> None:
        """打开设备连接或目标资源。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        self._ros = SimRosNode(self.config["device_id"], "gripper")
        self._publisher = self._ros.node.create_publisher(JointTrajectory, self.command_topic, 10)
        self._subscription = self._ros.node.create_subscription(JointState, self.state_topic, self._on_state, 10)
        if not self._ros.wait_for(lambda: self._receipt > 0.0, 10.0):
            self._ros.close_ros()
            raise TimeoutError(f"no simulated gripper feedback on {self.state_topic}")
        self.opened = True

    def _on_state(self, msg: JointState) -> None:
        """处理状态事件。

        Args:
            msg: 待处理的消息对象。 类型为 JointState。

        Returns:
            无返回值。
        """
        by_name = {name: index for index, name in enumerate(msg.name)}
        if any(name not in by_name for name in self.joints):
            return
        indices = [by_name[name] for name in self.joints]
        if len(msg.position) <= max(indices):
            return
        sample = ([float(msg.position[index]) for index in indices],
                  [float(msg.velocity[index]) if len(msg.velocity) > index else 0.0 for index in indices])
        with self._lock:
            if self._stamp_guard.observe(msg.header.stamp):
                self._clock_jump_pending = True
            self._sample = sample
            self._receipt = time.monotonic()

    def _state(self):
        """构造设备状态响应。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。
        """
        with self._lock:
            if self._sample is None or time.monotonic() - self._receipt > float(self.config["feedback"]["stale_sec"]):
                raise TimeoutError("simulated gripper joint feedback is missing or stale")
            return self._sample

    def _feedback(self) -> GripperFeedback:
        """反馈当前设备或操作。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。 返回类型为 GripperFeedback。
        """
        positions, velocities = self._state()
        speed = max(abs(value) for value in velocities)
        moving = speed > 0.01
        return GripperFeedback(moving=moving, reached=not moving, stationary=not moving)

    def read_feedback(self):
        """读取反馈。

        Args:
            无参数。

        Returns:
            查询或读取到的数据；未找到时的处理方式由实现决定。
        """
        if not self.opened:
            raise OSError("simulated gripper is disconnected")
        with self._lock:
            clock_jump, self._clock_jump_pending = self._clock_jump_pending, False
        if clock_jump:
            raise RuntimeError("simulated ROS clock moved backwards")
        return self._feedback()

    def _publish(self, positions, duration: float) -> None:
        """发布当前设备或操作。

        Args:
            positions: positions，作为发布当前设备或操作所需的输入。
            duration: duration，作为发布当前设备或操作所需的输入。 类型为 float。

        Returns:
            无返回值。
        """
        message = JointTrajectory()
        message.header.stamp = self._ros.node.get_clock().now().to_msg()
        message.joint_names = self.joints
        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in positions]
        point.time_from_start = Duration(sec=int(duration), nanosec=int((duration % 1) * 1e9))
        message.points = [point]
        self._publisher.publish(message)

    def command(self, direction: int, angle_tenths: int, speed_tenths: int) -> None:
        """执行或转发设备控制命令。

        Args:
            direction: direction，作为执行 command 对应的处理流程所需的输入。 类型为 int。
            angle_tenths: 角度tenths，作为执行 command 对应的处理流程所需的输入。 类型为 int。
            speed_tenths: 速度tenths，作为执行 command 对应的处理流程所需的输入。 类型为 int。

        Returns:
            无返回值。
        """
        if not self.opened:
            raise OSError("simulated gripper is disconnected")
        current, _ = self._state()
        fraction = min(1.0, int(angle_tenths) / max(int(self.config["motion"]["close_angle_tenths"]), 1))
        delta = self.travel_rad * fraction * (1.0 if int(direction) else -1.0)
        target = max(0.0, min(self.travel_rad, current[0] + delta))
        actual_delta = abs(target - current[0])
        speed = self.speed_at_150 * max(int(speed_tenths), 1) / 150.0
        self._publish([target, -target], max(0.15, actual_delta / speed))

    def stop(self) -> None:
        """停止设备当前动作。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        current, _ = self._state()
        self._publish(current, 0.05)

    def close(self) -> None:
        """关闭连接并释放相关资源。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        if self.opened:
            try:
                self.stop()
            except Exception:
                pass
        self.opened = False
        if hasattr(self, "_ros"):
            self._ros.close_ros()
