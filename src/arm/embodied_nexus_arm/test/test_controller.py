from __future__ import annotations

import copy
import time
import unittest
from pathlib import Path

from embodied_nexus_arm.backend import MockBackend
from embodied_nexus_arm.config import load_config, validate_config
from embodied_nexus_arm.controller import ArmController


CONFIG = Path(__file__).resolve().parents[1] / "config" / "aubo_i5_mock.yaml"


class ArmControllerTests(unittest.TestCase):
    def setUp(self):
        """为当前测试准备所需对象和资源。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        self.config = load_config(CONFIG)
        self.backend = MockBackend(self.config)
        self.controller = ArmController(self.config, self.backend)
        self.controller.poll_once()

    def tearDown(self):
        """清理当前测试创建的对象和资源。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        self.controller.shutdown()

    def lease_and_enable(self):
        """租约和启用。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。
        """
        lease, _ = self.controller.acquire_control("test")
        self.controller.enable_motion(lease, True)
        return lease

    def test_config_checks_joint_mapping_and_unknown_fields(self):
        """验证配置checks关节mapping和未知项fields。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        broken = copy.deepcopy(self.config)
        broken["joints"]["names"][1] = broken["joints"]["names"][0]
        with self.assertRaises(ValueError):
            validate_config(broken)
        broken = copy.deepcopy(self.config)
        broken["extra"] = True
        with self.assertRaises(ValueError):
            validate_config(broken)

    def test_simulator_profile_uses_the_pluggable_ros_backend_contract(self):
        """验证simulatorprofileusesthepluggableros后端contract。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        sim = load_config(Path(__file__).resolve().parents[1] / "config" / "aubo_i5_sim.yaml")
        self.assertEqual(sim["backend"], "ros2_sim")
        self.assertEqual(sim["backend_options"]["arm_trajectory_topic"],
                         "/sim/arachne/arm_controller/joint_trajectory")
        self.assertEqual(len(sim["joints"]["names"]), 6)

    def test_move_uses_measured_feedback_and_reports_acceptance(self):
        """验证运动usesmeasured反馈和reportsacceptance。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        lease = self.lease_and_enable()
        target = [0.08, 0, 0, 0, 0, 0]
        result = self.controller.move_joints(lease, target, speed=0.25, timeout=2.0)
        self.assertTrue(result["success"])
        self.assertTrue(result["controller_accepted"])
        self.assertTrue(result["stationary_confirmed"])
        self.assertLessEqual(result["max_error_rad"], self.config["motion"]["goal_tolerance_rad"])

    def test_cancel_requests_stop_and_confirms_measured_stationary_state(self):
        """验证取消requests停止和confirmsmeasuredstationary状态。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        lease = self.lease_and_enable()
        result = self.controller.move_joints(
            lease, [0.5, 0, 0, 0, 0, 0], speed=0.25, timeout=2.0,
            cancel_requested=lambda: True)
        self.assertFalse(result["success"])
        self.assertTrue(result["controller_accepted"])
        self.assertTrue(result["stop_requested"])
        self.assertTrue(result["stationary_confirmed"])

    def test_state_reports_missing_or_moving_feedback_as_unknown_or_not_stationary(self):
        """验证状态reportsmissingormoving反馈as未知项ornotstationary。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        initial = self.controller.state()
        self.assertTrue(initial["stationary_confirmed"])
        self.assertEqual(initial["feedback_timestamp_source"], "host_receive")
        self.assertIsNotNone(initial["stamp_sec"])
        self.backend.velocities = [0.1, 0, 0, 0, 0, 0]
        self.controller.poll_once()
        self.assertFalse(self.controller.state()["stationary_confirmed"])
        self.backend.config["mock"]["disconnect"] = True
        self.controller.poll_once()
        state = self.controller.state()
        self.assertFalse(state["feedback_valid"])
        self.assertIsNone(state["positions_rad"])
        self.assertIsNone(state["hardware_estop"])
        self.assertIsNone(state["stationary_confirmed"])

    def test_unsafe_or_stale_state_cannot_enable(self):
        """验证unsafeorstale状态cannot启用。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        lease, _ = self.controller.acquire_control("test")
        self.backend.safety = "EmergencyStop"
        with self.assertRaises(RuntimeError):
            self.controller.enable_motion(lease, True)
        self.assertFalse(self.controller.motion_enabled)

    def test_velocity_watchdog_stops_and_disables_motion(self):
        """验证速度watchdogstops和disablesmotion。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        lease = self.lease_and_enable()
        self.controller.config["motion"]["watchdog_sec"] = 0.04
        self.controller.set_joint_velocity(lease, [0.1, 0, 0, 0, 0, 0])
        deadline = self.controller.lease_deadline
        while time.monotonic() < deadline:
            time.sleep(0.005)
        self.controller.tick()
        self.assertFalse(self.controller.motion_enabled)
        self.assertTrue(self.controller.state()["stop_requested"])
        self.assertEqual(self.backend.velocities, [0.0] * 6)

    def test_velocity_updates_keep_servo_stream_continuous_until_zero(self):
        """验证速度updateskeep伺服数据流continuousuntilzero。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        lease = self.lease_and_enable()
        self.backend.command_log.clear()
        first = self.controller.set_joint_velocity(lease, [0.1, 0, 0, 0, 0, 0])
        second = self.controller.set_joint_velocity(lease, [0.1, 0, 0, 0, 0, 0])
        self.assertTrue(first["controller_accepted"])
        self.assertTrue(second["controller_accepted"])
        self.assertEqual(sum(name == "MotionControl.stopJoint" for name, _ in self.backend.command_log), 1)
        self.assertEqual(sum(name == "MotionControl.servoJoint" for name, _ in self.backend.command_log), 2)
        self.assertTrue(self.backend._servo_mode_enabled)
        stream_state = self.controller.state()
        self.assertTrue(stream_state["continuous_control_active"])
        self.assertIsNotNone(stream_state["joint_velocity_command_age_sec"])
        self.assertIsNotNone(stream_state["joint_velocity_command_stamp_sec"])
        zero = self.controller.set_joint_velocity(lease, [0] * 6)
        self.assertTrue(zero["controller_accepted"])
        self.assertTrue(zero["stationary_confirmed"])
        self.assertFalse(self.backend._servo_mode_enabled)
        self.assertTrue(self.controller.motion_enabled)

    def test_velocity_limit_is_enforced_and_busy_servo_command_is_retryable(self):
        """验证速度limitisenforced和busy伺服commandisretryable。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。
        """
        lease = self.lease_and_enable()
        with self.assertRaises(ValueError):
            self.controller.set_joint_velocity(lease, [0.26, 0, 0, 0, 0, 0])
        original_execute = self.backend.execute
        busy = {"next": True}

        def return_busy_once(method, params=None):
            """模拟首次调用返回忙碌状态的控制器响应。

            Args:
                method: HTTP 请求方法，例如 GET 或 POST。
                params: params，作为执行 return_busy_once 对应的处理流程所需的输入。

            Returns:
                本次处理产生的结果值。
            """
            if method == "MotionControl.servoJoint" and busy["next"]:
                busy["next"] = False
                return 2
            return original_execute(method, params)

        self.backend.execute = return_busy_once
        response = self.controller.set_joint_velocity(lease, [0.1, 0, 0, 0, 0, 0])
        self.assertFalse(response["controller_accepted"])
        self.assertTrue(response["retry_next_cycle"])
        self.assertFalse(self.controller.state()["controller_accepted"])

    def test_joint_move_timeout_stops_and_does_not_report_success(self):
        """验证关节运动超时时间stops和doesnotreportsuccess。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        lease = self.lease_and_enable()
        with self.assertRaises(TimeoutError):
            self.controller.move_joints(lease, [0.4, 0, 0, 0, 0, 0], speed=0.1, timeout=0.01)
        state = self.controller.state()
        self.assertEqual(state["operation_state"], "failed")
        self.assertTrue(state["stop_requested"])
        self.assertTrue(state["stationary_confirmed"])

    def test_lease_conflict_expiry_and_stop_failure(self):
        """验证租约conflictexpiry和停止failure。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        lease, _ = self.controller.acquire_control("first", ttl_sec=0.05)
        with self.assertRaises(RuntimeError):
            self.controller.acquire_control("second")
        time.sleep(0.06)
        self.controller.tick()
        self.assertFalse(self.controller.lease_id)
        self.backend.config["mock"]["fail_stop"] = True
        result = self.controller.stop("injected stop error")
        self.assertFalse(result["controller_accepted"])
        self.assertTrue(result["stationary_confirmed"])

    def test_teach_mode_excludes_joint_motion(self):
        """验证示教modeexcludes关节motion。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        lease = self.lease_and_enable()
        self.controller.set_teach_mode(lease, True)
        with self.assertRaises(RuntimeError):
            self.controller.move_joints(lease, [0.1] * 6)
        self.controller.set_teach_mode(lease, False)


if __name__ == "__main__":
    unittest.main()
