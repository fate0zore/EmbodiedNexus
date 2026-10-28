from __future__ import annotations

import math
import sys
import threading
import time
import unittest
import uuid
from importlib.util import find_spec
from pathlib import Path

if find_spec("rclpy") is None:
    raise unittest.SkipTest("simulator backend contract tests require ROS 2 Humble")

workspace = Path(__file__).resolve().parents[5]
sys.path[0:0] = [
    str(workspace / "src/sim/common/embodied_nexus_sim"),
    str(workspace / "src/base/embodied_nexus_base"),
    str(workspace / "src/arm/embodied_nexus_arm"),
    str(workspace / "src/gripper/embodied_nexus_gripper"),
]

import rclpy
from builtin_interfaces.msg import Time as RosTime
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

from embodied_nexus_sim.arm import Ros2SimArmBackend
from embodied_nexus_sim.base import Ros2SimBaseBackend
from embodied_nexus_sim.gripper import Ros2SimGripperBackend
from embodied_nexus_sim.ros_support import SimTimestampGuard


def _wait(predicate, timeout=3.0):
    """等待当前设备或操作。

    Args:
        predicate: predicate，作为等待当前设备或操作所需的输入。
        timeout: 本次网络或服务调用的最长等待时长，通常以秒为单位。

    Returns:
        本次处理产生的结果值。
    """
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return bool(predicate())


class RosBackendContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        """设置upclass。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """释放测试类创建的共享进程和资源。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        """为当前测试准备所需对象和资源。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        suffix = uuid.uuid4().hex[:8]
        self.root = f"/sim/backend_contract_{suffix}"
        self.node = Node(f"backend_contract_{suffix}")
        self.executor = MultiThreadedExecutor(num_threads=2)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        self.backend = None

    def tearDown(self):
        """清理当前测试创建的对象和资源。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        if self.backend is not None:
            self.backend.close()
        self.executor.shutdown(timeout_sec=1.0)
        self.thread.join(timeout=1.0)
        self.executor.remove_node(self.node)
        self.node.destroy_node()

    def test_base_twist_and_odometry_keep_measured_pose_and_stale_state(self):
        """验证底盘twist和odometrykeepmeasured位姿和stale状态。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        odom_topic = f"{self.root}/base/odom"
        command_topic = f"{self.root}/base/cmd_vel"
        odom_pub = self.node.create_publisher(Odometry, odom_topic, 10)
        commands = []
        command_received = threading.Event()
        self.node.create_subscription(
            TwistStamped, command_topic,
            lambda message: (commands.append(message), command_received.set()), 10)
        publish_state = {"enabled": True}

        def publish_odom():
            """发布odom。

            Args:
                无参数。

            Returns:
                无返回值。
            """
            if not publish_state["enabled"]:
                return
            message = Odometry()
            message.header.stamp = RosTime(sec=7, nanosec=250_000_000)
            message.header.frame_id = "odom"
            message.child_frame_id = "base_footprint"
            message.pose.pose.position.x = 1.25
            message.pose.pose.position.y = -0.4
            message.pose.pose.orientation.z = math.sin(0.3)
            message.pose.pose.orientation.w = math.cos(0.3)
            message.twist.twist.linear.x = 0.12
            message.twist.twist.angular.z = 0.2
            odom_pub.publish(message)

        self.timer = self.node.create_timer(0.02, publish_odom)
        config = {"device_id": "base_sim", "backend_options": {
            "namespace": self.root, "odom_topic": odom_topic, "cmd_vel_topic": command_topic,
        }, "feedback": {"stale_sec": 0.15}, "frames": {"base": "base_link"}}
        self.backend = Ros2SimBaseBackend(config)
        self.backend.open()
        self.backend.request_control_mode()
        self.backend.set_velocity(0.15, -0.25)
        self.assertTrue(command_received.wait(2.0))
        self.assertAlmostEqual(commands[-1].twist.linear.x, 0.15)
        system, motion, connected, error = self.backend.poll()
        pose = self.backend.read_odometry()
        self.assertTrue(connected, error)
        self.assertEqual(system["control_mode"], 1)
        self.assertAlmostEqual(motion["angular_rps"], 0.2)
        self.assertAlmostEqual(pose["x_m"], 1.25)
        self.assertAlmostEqual(pose["yaw_rad"], 0.6)
        self.assertEqual(pose["stamp_sec"], 7.25)
        publish_state["enabled"] = False
        self.assertTrue(_wait(lambda: self.backend.poll()[2] is False, timeout=0.6))
        self.assertIsNone(self.backend.read_odometry())
        with self.assertRaisesRegex(ConnectionError, "disconnected"):
            self.backend.set_velocity(0.1, 0.0)
        previous_commands = len(commands)
        self.backend.stop()
        self.assertTrue(_wait(lambda: len(commands) > previous_commands))
        self.assertEqual(commands[-1].twist.linear.x, 0.0)
        self.assertEqual(commands[-1].twist.angular.z, 0.0)

    def test_sim_timestamp_guard_detects_only_backward_time(self):
        """验证simtimestampguarddetectsonlybackward时间。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        guard = SimTimestampGuard()
        self.assertFalse(guard.observe(RosTime(sec=10, nanosec=0)))
        self.assertFalse(guard.observe(RosTime(sec=10, nanosec=0)))
        self.assertFalse(guard.observe(RosTime(sec=10, nanosec=100)))
        self.assertTrue(guard.observe(RosTime(sec=2, nanosec=0)))
        self.assertFalse(guard.observe(RosTime(sec=2, nanosec=100)))

    def test_base_clock_rewind_disconnects_feedback_but_allows_zero_velocity(self):
        """验证底盘clockrewinddisconnects反馈butallowszero速度。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。
        """
        odom_topic = f"{self.root}/rewind/odom"
        command_topic = f"{self.root}/rewind/cmd_vel"
        odom_pub = self.node.create_publisher(Odometry, odom_topic, 10)
        commands = []
        self.node.create_subscription(TwistStamped, command_topic, commands.append, 10)
        stamp = {"value": RosTime(sec=10)}

        def publish_odom():
            """发布odom。

            Args:
                无参数。

            Returns:
                无返回值。
            """
            message = Odometry()
            message.header.stamp = stamp["value"]
            message.pose.pose.orientation.w = 1.0
            odom_pub.publish(message)

        self.timer = self.node.create_timer(0.02, publish_odom)
        config = {"device_id": "base_rewind_sim", "backend_options": {
            "namespace": self.root, "odom_topic": odom_topic, "cmd_vel_topic": command_topic,
        }, "feedback": {"stale_sec": 0.5}, "frames": {"base": "base_link"}}
        self.backend = Ros2SimBaseBackend(config)
        self.backend.open()
        self.backend.set_velocity(0.1, 0.0)
        stamp["value"] = RosTime(sec=2)
        observed = {}

        def clock_rewind_seen():
            """检查测试期间是否检测到仿真时钟回跳。

            Args:
                无参数。

            Returns:
                本次处理产生的结果值。
            """
            result = self.backend.poll()
            if not result[2] and "moved backwards" in result[3]:
                observed["result"] = result
                return True
            return False

        self.assertTrue(_wait(clock_rewind_seen))
        with self.assertRaisesRegex(ConnectionError, "disconnected"):
            self.backend.set_velocity(0.1, 0.0)
        previous_commands = len(commands)
        self.backend.stop()
        self.assertTrue(_wait(lambda: len(commands) > previous_commands))
        self.assertEqual(commands[-1].twist.linear.x, 0.0)
        self.assertEqual(commands[-1].twist.angular.z, 0.0)

    def test_arm_joint_state_is_measured_and_trajectory_commands_are_stamped(self):
        """验证机械臂关节状态ismeasured和trajectorycommandsarestamped。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        state_topic = f"{self.root}/joint_states"
        command_topic = f"{self.root}/arm_controller/joint_trajectory"
        names = ["aubo_shoulder_joint", "aubo_upperArm_joint", "aubo_foreArm_joint",
                 "aubo_wrist1_joint", "aubo_wrist2_joint", "aubo_wrist3_joint"]
        state_pub = self.node.create_publisher(JointState, state_topic, 10)
        commands = []
        command_received = threading.Event()
        self.node.create_subscription(
            JointTrajectory, command_topic,
            lambda message: (commands.append(message), command_received.set()), 10)

        def publish_joint_state():
            """发布关节状态。

            Args:
                无参数。

            Returns:
                无返回值。
            """
            message = JointState()
            message.header.stamp = RosTime(sec=3)
            message.name = names
            message.position = [0.1] * 6
            message.velocity = [0.0] * 6
            state_pub.publish(message)

        self.timer = self.node.create_timer(0.02, publish_joint_state)
        config = {"device_id": "arm_sim", "backend_options": {
            "namespace": self.root, "joint_states_topic": state_topic,
            "arm_trajectory_topic": command_topic,
        }, "joints": {"names": names}, "feedback": {"stale_sec": 0.5},
           "motion": {"max_speed_rad_sec": 0.25}}
        self.backend = Ros2SimArmBackend(config)
        self.backend.open()
        state = self.backend.poll()
        self.assertEqual(state["positions_rad"], [0.1] * 6)
        self.assertEqual(state["feedback_timestamp_source"], "simulation_clock")
        self.assertEqual(self.backend.execute("MotionControl.moveJoint", [[0.2] * 6, 0.4, 0.2]), 0)
        self.assertTrue(command_received.wait(2.0))
        self.assertEqual(commands[-1].joint_names, names)
        self.assertEqual(list(commands[-1].points[0].positions), [0.2] * 6)
        self.assertEqual(self.backend.capabilities[-1], "stop")

    def test_gripper_motor_mapping_uses_actual_joint_feedback(self):
        """验证夹爪控制电机mappingusesactual关节反馈。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        state_topic = f"{self.root}/joint_states"
        command_topic = f"{self.root}/gripper_controller/joint_trajectory"
        names = ["ms42dc_left_finger_joint", "ms42dc_right_finger_joint"]
        state_pub = self.node.create_publisher(JointState, state_topic, 10)
        commands = []
        command_received = threading.Event()
        self.node.create_subscription(
            JointTrajectory, command_topic,
            lambda message: (commands.append(message), command_received.set()), 10)
        measured = {"position": [0.0, 0.0], "velocity": [0.0, 0.0]}

        def publish_joint_state():
            """发布关节状态。

            Args:
                无参数。

            Returns:
                无返回值。
            """
            message = JointState()
            message.name = names
            message.position = list(measured["position"])
            message.velocity = list(measured["velocity"])
            state_pub.publish(message)

        self.timer = self.node.create_timer(0.02, publish_joint_state)
        config = {"device_id": "gripper_sim", "backend_options": {
            "namespace": self.root, "joint_states_topic": state_topic,
            "gripper_trajectory_topic": command_topic, "gripper_joint_names": names,
            "finger_travel_rad": 0.6, "speed_at_150_rad_sec": 0.6,
        }, "feedback": {"stale_sec": 0.5, "moving_value": 0, "reached_value": 1},
           "motion": {"close_angle_tenths": 19656}, "protocol": {"device_id": 1}}
        self.backend = Ros2SimGripperBackend(config)
        self.backend.open()
        self.backend.command(direction=1, angle_tenths=19656, speed_tenths=150)
        self.assertTrue(command_received.wait(2.0))
        self.assertEqual(list(commands[-1].points[0].positions), [0.6, -0.6])
        measured["position"] = [0.3, -0.3]
        measured["velocity"] = [0.5, -0.5]
        feedback = None
        self.assertTrue(_wait(lambda: (setattr(self, "feedback", self.backend.read_feedback()) or
                                     self.feedback.moving)))
        feedback = self.feedback
        self.assertFalse(feedback.reached)
        self.assertFalse(feedback.stationary)
        self.assertTrue(self.backend.feedback_is_simulated)


if __name__ == "__main__":
    unittest.main()
