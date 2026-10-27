from __future__ import annotations

import threading
import time

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

from .ros_support import (SimRosNode, SimTimestampGuard, configured_topic,
                          validate_options, yaw_from_quaternion)


class Ros2SimBaseBackend:
    """Scout backend using standard simulated velocity and odometry topics."""

    def __init__(self, config: dict) -> None:
        """初始化对象并保存构造参数或依赖。

        Args:
            config: 设备或平台配置对象，提供本次操作所需参数。 类型为 dict。

        Returns:
            无返回值。
        """
        self.config = config
        options = config["backend_options"]
        validate_options(options, {"namespace", "cmd_vel_topic", "odom_topic"})
        root = str(options.get("namespace", "/sim/arachne")).rstrip("/")
        if not root.startswith("/"):
            raise ValueError("backend_options.namespace must be an absolute ROS namespace")
        self.command_topic = configured_topic(options, "cmd_vel_topic", f"{root}/base_controller/cmd_vel")
        self.odom_topic = configured_topic(options, "odom_topic", f"{root}/base_controller/odom")
        self.connected = False
        self.enabled = False
        self._receipt = 0.0
        self._odom = None
        self._lock = __import__("threading").RLock()
        self._stamp_guard = SimTimestampGuard()
        self._clock_jump_pending = False
        self._clock_recovery_active = False
        self._clock_recovery_stationary_samples = 0
        self._clock_recovery_event = threading.Event()
        self._clock_recovery_thread = None
        self._clock_recovery_thread_lock = threading.Lock()

    def open(self) -> None:
        """打开设备连接或目标资源。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        self._ros = SimRosNode(self.config["device_id"], "base")
        self._publisher = self._ros.node.create_publisher(TwistStamped, self.command_topic, 10)
        self._subscription = self._ros.node.create_subscription(Odometry, self.odom_topic, self._on_odom, 10)
        if not self._ros.wait_for(lambda: self._receipt > 0.0, 10.0):
            self._ros.close_ros()
            raise TimeoutError(f"no simulated base odometry on {self.odom_topic}")
        self.connected = True

    def _on_odom(self, msg: Odometry) -> None:
        """处理odom事件。

        Args:
            msg: 待处理的消息对象。 类型为 Odometry。

        Returns:
            无返回值。
        """
        start_recovery = False
        with self._lock:
            clock_jump = self._stamp_guard.observe(msg.header.stamp)
            if clock_jump:
                self._clock_jump_pending = True
                self._clock_recovery_active = True
                self._clock_recovery_stationary_samples = 0
                self._clock_recovery_event.clear()
                start_recovery = True
            elif self._clock_recovery_active:
                motion = msg.twist.twist
                if abs(float(motion.linear.x)) <= 0.01 and abs(float(motion.angular.z)) <= 0.01:
                    self._clock_recovery_stationary_samples += 1
                    if self._clock_recovery_stationary_samples >= 3:
                        self._clock_recovery_active = False
                        self._clock_recovery_event.set()
                else:
                    self._clock_recovery_stationary_samples = 0
            self._odom = msg
            self._receipt = time.monotonic()
        if start_recovery:
            with self._clock_recovery_thread_lock:
                if self._clock_recovery_thread is None or not self._clock_recovery_thread.is_alive():
                    self._clock_recovery_thread = threading.Thread(
                        target=self._keep_zero_after_clock_jump,
                        name=f"{self.config['device_id']}-clock-recovery-stop",
                        daemon=True,
                    )
                    self._clock_recovery_thread.start()

    def _keep_zero_after_clock_jump(self) -> None:
        """检测仿真时钟回跳并保持底盘零速度。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        while not self._clock_recovery_event.wait(0.1):
            try:
                self.stop()
            except Exception:
                pass

    def request_control_mode(self) -> None:
        """请求控制权mode。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        self.enabled = True

    def set_velocity(self, linear: float, angular: float) -> None:
        """设置速度。

        Args:
            linear: 线速度，作为设置速度所需的输入。 类型为 float。
            angular: 角速度，作为设置速度所需的输入。 类型为 float。

        Returns:
            无返回值。
        """
        linear = float(linear)
        angular = float(angular)
        if not self.connected and (linear != 0.0 or angular != 0.0):
            raise ConnectionError("simulated base is disconnected")
        message = TwistStamped()
        message.header.stamp = self._ros.node.get_clock().now().to_msg()
        message.header.frame_id = str(self.config["frames"]["base"])
        message.twist.linear.x = linear
        message.twist.angular.z = angular
        self._publisher.publish(message)

    def stop(self) -> None:
        """停止设备当前动作。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        self.set_velocity(0.0, 0.0)

    def poll(self):
        """轮询当前设备或操作。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。
        """
        with self._lock:
            clock_jump, self._clock_jump_pending = self._clock_jump_pending, False
            msg, age = self._odom, time.monotonic() - self._receipt if self._receipt else float("inf")
        if clock_jump:
            self.connected = False
            return None, None, False, "simulated ROS clock moved backwards"
        stale = float(self.config["feedback"]["stale_sec"])
        if msg is None or age > stale:
            self.connected = False
            return None, None, False, "simulated base odometry is missing or stale"
        self.connected = True
        system = {"vehicle_state": 0, "control_mode": 1 if self.enabled else 0,
                  "battery_voltage": None, "error_code": 0}
        motion = {"linear_mps": float(msg.twist.twist.linear.x),
                  "angular_rps": float(msg.twist.twist.angular.z), "lateral_mps": 0.0}
        return system, motion, True, ""

    def read_odometry(self):
        """读取odometry。

        Args:
            无参数。

        Returns:
            查询或读取到的数据；未找到时的处理方式由实现决定。
        """
        with self._lock:
            msg, receipt = self._odom, self._receipt
            stale = float(self.config["feedback"]["stale_sec"])
            if msg is None or not receipt or time.monotonic() - receipt > stale:
                return None
            p = msg.pose.pose.position
            return {"x_m": float(p.x), "y_m": float(p.y),
                    "yaw_rad": yaw_from_quaternion(msg.pose.pose.orientation),
                    "stamp_sec": float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9}

    def close(self) -> None:
        """关闭连接并释放相关资源。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        self._clock_recovery_event.set()
        recovery_thread = self._clock_recovery_thread
        if recovery_thread is not None and recovery_thread is not threading.current_thread():
            recovery_thread.join(timeout=1.0)
        if hasattr(self, "_publisher"):
            try:
                self.stop()
            except Exception:
                pass
        self.connected = False
        if hasattr(self, "_ros"):
            self._ros.close_ros()
