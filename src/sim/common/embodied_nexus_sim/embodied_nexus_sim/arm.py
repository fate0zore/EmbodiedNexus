from __future__ import annotations

import threading
import time

from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .ros_support import (SimRosNode, SimTimestampGuard, configured_topic,
                          finite_values, validate_options)


class Ros2SimArmBackend:
    """Aubo-compatible backend backed by measured joint state and trajectories."""

    capabilities = ("joint_state", "move_joints", "joint_velocity", "power", "stop")

    def __init__(self, config: dict) -> None:
        self.config = config
        options = config["backend_options"]
        validate_options(options, {"namespace", "joint_states_topic", "arm_trajectory_topic"})
        root = str(options.get("namespace", "/sim/arachne")).rstrip("/")
        if not root.startswith("/"):
            raise ValueError("backend_options.namespace must be an absolute ROS namespace")
        self.joints = list(config["joints"]["names"])
        self.state_topic = configured_topic(options, "joint_states_topic", f"{root}/joint_states")
        self.command_topic = configured_topic(options, "arm_trajectory_topic", f"{root}/arm_controller/joint_trajectory")
        self.connected = False
        self.enabled = False
        self.mode = "Running"
        self._sample = None
        self._receipt = 0.0
        self._lock = threading.RLock()
        self._servo = False
        self._stamp_guard = SimTimestampGuard()
        self._clock_jump_pending = False

    def open(self) -> None:
        self._ros = SimRosNode(self.config["device_id"], "arm")
        self._publisher = self._ros.node.create_publisher(JointTrajectory, self.command_topic, 10)
        self._subscription = self._ros.node.create_subscription(JointState, self.state_topic, self._on_state, 10)
        if not self._ros.wait_for(lambda: self._receipt > 0.0, 10.0):
            self._ros.close_ros()
            raise TimeoutError(f"no simulated arm feedback on {self.state_topic}")
        self.connected = True

    def _on_state(self, msg: JointState) -> None:
        by_name = {name: index for index, name in enumerate(msg.name)}
        if any(name not in by_name for name in self.joints):
            return
        indices = [by_name[name] for name in self.joints]
        if len(msg.position) <= max(indices):
            return
        velocity = [float(msg.velocity[index]) if len(msg.velocity) > index else 0.0 for index in indices]
        sample = {"positions_rad": [float(msg.position[index]) for index in indices],
                  "velocities_rad_sec": velocity, "mode": self.mode, "safety_mode": "Normal",
                  "teach_enabled": None, "payload": None, "tcp_offset": None,
                  "hardware_estop": None, "feedback_timestamp_source": "simulation_clock",
                  "stamp_sec": float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9}
        with self._lock:
            if self._stamp_guard.observe(msg.header.stamp):
                self._clock_jump_pending = True
            self._sample = sample
            self._receipt = time.monotonic()

    def _fresh(self):
        with self._lock:
            if self._sample is None or time.monotonic() - self._receipt > float(self.config["feedback"]["stale_sec"]):
                raise TimeoutError("simulated joint state is missing or stale")
            sample = dict(self._sample)
        return sample

    def poll(self):
        with self._lock:
            clock_jump, self._clock_jump_pending = self._clock_jump_pending, False
        if clock_jump:
            self.connected = False
            self._servo = False
            raise RuntimeError("simulated ROS clock moved backwards")
        try:
            sample = self._fresh()
        except Exception:
            self.connected = False
            raise
        self.connected = True
        sample["mode"] = self.mode
        return sample

    def _publish(self, positions, duration: float) -> None:
        values = finite_values(positions, 6, "joint target")
        message = JointTrajectory()
        message.header.stamp = self._ros.node.get_clock().now().to_msg()
        message.joint_names = self.joints
        point = JointTrajectoryPoint()
        point.positions = values
        point.time_from_start = Duration(sec=int(duration), nanosec=int((duration % 1) * 1e9))
        message.points = [point]
        self._publisher.publish(message)

    def execute(self, method: str, params: list | None = None):
        params = params or []
        if method == "MotionControl.setServoMode":
            self._servo = bool(params[0])
            return 0
        if method == "MotionControl.isServoModeEnabled":
            return self._servo
        if method == "MotionControl.setServoModeSelect":
            return 0
        if method == "MotionControl.moveJoint":
            target = finite_values(params[0], 6, "joint target")
            speed = max(0.02, min(float(params[2]), float(self.config["motion"]["max_speed_rad_sec"])))
            current = self._fresh()["positions_rad"]
            duration = max(0.15, max(abs(a - b) for a, b in zip(target, current)) / speed)
            self._publish(target, duration)
            return 0
        if method == "MotionControl.servoJoint":
            if not self._servo:
                raise RuntimeError("simulated joint velocity stream is not enabled")
            target = finite_values(params[0], 6, "joint velocity target")
            self._publish(target, max(0.05, float(params[3])))
            return 0
        if method == "MotionControl.stopJoint":
            self.stop()
            return 0
        if method == "RobotManage.poweron":
            self.mode = "Idle"
            return 0
        if method == "RobotManage.startup":
            self.mode = "Running"
            return 0
        if method in {"RobotManage.poweroff", "RobotManage.powerOff"}:
            self.mode = "PowerOff"
            return 0
        if method in {"RobotManage.freedrive", "RobotManage.backdrive", "RobotManage.handguideMode"}:
            raise NotImplementedError("teach mode is not simulated")
        if method == "RobotManage.exitHandguideMode":
            return 0
        raise NotImplementedError(f"Aubo operation {method} is not supported by simulation")

    def read_optional(self, _suffix: str):
        return None

    def stop(self) -> bool:
        try:
            self._publish(self._fresh()["positions_rad"], 0.05)
            self._servo = False
            return True
        except Exception:
            return False

    def close(self) -> None:
        self.stop()
        self.connected = False
        if hasattr(self, "_ros"):
            self._ros.close_ros()
