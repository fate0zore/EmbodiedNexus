from __future__ import annotations

import json
import math
import time
from pathlib import Path

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from embodied_nexus_arm_interfaces.action import ExecuteArmTrajectory, MoveArmJoints
from embodied_nexus_arm_interfaces.srv import (
    AcquireArmControl, ApplyArmConfiguration, GetArmConfiguration, GetArmInfo,
    GetArmState, ReleaseArmControl, RenewArmControl, SetArmJointVelocity,
    SetArmLifecycle, SetArmMotionEnabled, SetArmTeachMode, StopArm,
    ValidateArmConfiguration,
)

from .config import load_config
from .controller import ArmController


class ArmNode(Node):
    def __init__(self, config_path: str | None = None) -> None:
        """初始化对象并保存构造参数或依赖。

        Args:
            config_path: 配置文件路径，指向要加载的设备配置。 类型为 str | None。

        Returns:
            无返回值。
        """
        super().__init__("embodied_nexus_arm")
        self.declare_parameter("config_file", config_path or str(Path(__file__).resolve().parents[1] / "config" / "aubo_i5_mock.yaml"))
        self.declare_parameter("interface_mode", "")
        self.config = load_config(str(self.get_parameter("config_file").value))
        mode = str(self.get_parameter("interface_mode").value).strip()
        if mode:
            self.config["interface_mode"] = mode
        self.controller = ArmController(self.config)
        self.device_id = self.config["device_id"]
        self.prefix = f"/embodied_nexus/arms/{self.device_id}"
        self.group = ReentrantCallbackGroup()
        self.state_pub = self.create_publisher(String, f"{self.prefix}/_internal/state_json", 10)
        self.joints_pub = self.create_publisher(JointState, f"{self.prefix}/joint_states", 10)
        self._arm_services = [
            self.create_service(GetArmInfo, f"{self.prefix}/info", self._info, callback_group=self.group),
            self.create_service(GetArmState, f"{self.prefix}/get_state", self._state, callback_group=self.group),
            self.create_service(GetArmConfiguration, f"{self.prefix}/config/get", self._get_config, callback_group=self.group),
            self.create_service(ValidateArmConfiguration, f"{self.prefix}/config/validate", self._validate, callback_group=self.group),
            self.create_service(ApplyArmConfiguration, f"{self.prefix}/config/apply", self._apply, callback_group=self.group),
            self.create_service(AcquireArmControl, f"{self.prefix}/control/acquire", self._acquire, callback_group=self.group),
            self.create_service(RenewArmControl, f"{self.prefix}/control/renew", self._renew, callback_group=self.group),
            self.create_service(ReleaseArmControl, f"{self.prefix}/control/release", self._release, callback_group=self.group),
            self.create_service(SetArmMotionEnabled, f"{self.prefix}/motion/enable", self._enable, callback_group=self.group),
            self.create_service(SetArmJointVelocity, f"{self.prefix}/joint_velocity", self._velocity, callback_group=self.group),
            self.create_service(SetArmLifecycle, f"{self.prefix}/lifecycle", self._lifecycle, callback_group=self.group),
            self.create_service(SetArmTeachMode, f"{self.prefix}/teach_mode", self._teach, callback_group=self.group),
            self.create_service(StopArm, f"{self.prefix}/stop", self._stop, callback_group=self.group),
        ]
        self.move_action = ActionServer(self, MoveArmJoints, f"{self.prefix}/move_joints",
                                        execute_callback=self._move_joints,
                                        goal_callback=self._move_goal,
                                        cancel_callback=self._cancel_goal,
                                        callback_group=self.group)
        self.trajectory_action = ActionServer(
            self, ExecuteArmTrajectory, f"{self.prefix}/execute_trajectory",
            execute_callback=self._execute_trajectory,
            goal_callback=self._trajectory_goal,
            cancel_callback=self._cancel_goal,
            callback_group=self.group,
        )
        hz = float(self.config["feedback"]["rate_hz"])
        self.timer = self.create_timer(1.0 / hz, self._tick, callback_group=self.group,
                                       clock=Clock(clock_type=ClockType.STEADY_TIME))

    @staticmethod
    def _json(value) -> str:
        """将输入值转换为可写入 JSON 的数据。

        Args:
            value: 要应用、比较或转换的输入值。

        Returns:
            本次处理产生的结果值。 返回类型为 str。
        """
        return json.dumps(value, ensure_ascii=False, allow_nan=False, separators=(",", ":"))

    @staticmethod
    def _ok(response, error=""):
        """构造表示操作成功的响应。

        Args:
            response: 服务端或被调用操作返回的响应对象。
            error: error，作为构造表示操作成功的响应所需的输入。

        Returns:
            本次处理产生的结果值。
        """
        response.success = not bool(error)
        response.error = str(error)
        return response

    def _info(self, _request, response):
        """构造设备信息响应。

        Args:
            _request: 请求，作为构造设备信息响应所需的输入。
            response: 服务端或被调用操作返回的响应对象。

        Returns:
            本次处理产生的结果值。
        """
        try:
            response.descriptor_json = self._json(self.controller.info())
            return self._ok(response)
        except Exception as exc:
            return self._ok(response, exc)

    def _state(self, _request, response):
        """构造设备状态响应。

        Args:
            _request: 请求，作为构造设备状态响应所需的输入。
            response: 服务端或被调用操作返回的响应对象。

        Returns:
            本次处理产生的结果值。
        """
        try:
            response.state_json = self._json(self.controller.state())
            return self._ok(response)
        except Exception as exc:
            return self._ok(response, exc)

    def _get_config(self, _request, response):
        """读取设备配置并构造服务响应。

        Args:
            _request: 请求，作为读取设备配置并构造服务响应所需的输入。
            response: 服务端或被调用操作返回的响应对象。

        Returns:
            查询或读取到的数据；未找到时的处理方式由实现决定。
        """
        try:
            response.config_json = self._json(self.controller.configuration())
            return self._ok(response)
        except Exception as exc:
            return self._ok(response, exc)

    def _validate(self, request, response):
        """验证当前设备或操作。

        Args:
            request: 封装调用输入字段的请求对象。
            response: 服务端或被调用操作返回的响应对象。

        Returns:
            本次处理产生的结果值。
        """
        try:
            response.config_json = self._json(self.controller.validate_configuration(json.loads(request.patch_json)))
            response.valid = True
            response.error = ""
        except Exception as exc:
            response.valid = False
            response.error = str(exc)
        return response

    def _apply(self, request, response):
        """应用当前设备或操作。

        Args:
            request: 封装调用输入字段的请求对象。
            response: 服务端或被调用操作返回的响应对象。

        Returns:
            本次处理产生的结果值。
        """
        try:
            response.config_json = self._json(self.controller.apply_configuration(
                json.loads(request.patch_json), request.lease_id))
            return self._ok(response)
        except Exception as exc:
            return self._ok(response, exc)

    def _acquire(self, request, response):
        """申请设备控制权。

        Args:
            request: 封装调用输入字段的请求对象。
            response: 服务端或被调用操作返回的响应对象。

        Returns:
            本次处理产生的结果值。
        """
        try:
            response.lease_id, response.expires_in_sec = self.controller.acquire_control(
                request.owner, request.ttl_sec or None)
            return self._ok(response)
        except Exception as exc:
            return self._ok(response, exc)

    def _renew(self, request, response):
        """续租当前设备控制权。

        Args:
            request: 封装调用输入字段的请求对象。
            response: 服务端或被调用操作返回的响应对象。

        Returns:
            本次处理产生的结果值。
        """
        try:
            response.expires_in_sec = self.controller.renew_control(request.lease_id)
            return self._ok(response)
        except Exception as exc:
            return self._ok(response, exc)

    def _release(self, request, response):
        """释放当前设备控制租约。

        Args:
            request: 封装调用输入字段的请求对象。
            response: 服务端或被调用操作返回的响应对象。

        Returns:
            本次处理产生的结果值。
        """
        try:
            result = self.controller.release_control(request.lease_id)
            response.stop_requested = result["stop_requested"]
            response.controller_accepted = result["controller_accepted"]
            response.stationary_confirmed = result["stationary_confirmed"]
            response.success = True
            response.error = result["error"]
            return response
        except Exception as exc:
            return self._ok(response, exc)

    def _enable(self, request, response):
        """更新设备运动使能状态。

        Args:
            request: 封装调用输入字段的请求对象。
            response: 服务端或被调用操作返回的响应对象。

        Returns:
            本次处理产生的结果值。
        """
        try:
            response.enabled = self.controller.enable_motion(request.lease_id, request.enabled)["enabled"]
            return self._ok(response)
        except Exception as exc:
            response.enabled = False
            return self._ok(response, exc)

    def _velocity(self, request, response):
        """更新设备速度指令。

        Args:
            request: 封装调用输入字段的请求对象。
            response: 服务端或被调用操作返回的响应对象。

        Returns:
            本次处理产生的结果值。
        """
        try:
            result = self.controller.set_joint_velocity(request.lease_id, request.velocity_rad_sec)
            response.controller_accepted = result["controller_accepted"]
            response.retry_next_cycle = result.get("retry_next_cycle", False)
            response.commanded_positions_rad = result.get("commanded_positions_rad", [])
            response.stationary_confirmed = result.get("stationary_confirmed", False)
            return self._ok(response)
        except Exception as exc:
            return self._ok(response, exc)

    def _lifecycle(self, request, response):
        """设置设备生命周期状态。

        Args:
            request: 封装调用输入字段的请求对象。
            response: 服务端或被调用操作返回的响应对象。

        Returns:
            本次处理产生的结果值。
        """
        try:
            result = self.controller.set_lifecycle(request.lease_id, request.command)
            response.operation_state = result["operation_state"]
            return self._ok(response)
        except Exception as exc:
            response.operation_state = "failed"
            return self._ok(response, exc)

    def _teach(self, request, response):
        """切换机械臂示教模式。

        Args:
            request: 封装调用输入字段的请求对象。
            response: 服务端或被调用操作返回的响应对象。

        Returns:
            本次处理产生的结果值。
        """
        try:
            result = self.controller.set_teach_mode(request.lease_id, request.enabled)
            response.teach_state = result["teach_state"]
            return self._ok(response)
        except Exception as exc:
            response.teach_state = "unknown"
            return self._ok(response, exc)

    def _stop(self, request, response):
        """停止设备当前动作。

        Args:
            request: 封装调用输入字段的请求对象。
            response: 服务端或被调用操作返回的响应对象。

        Returns:
            本次处理产生的结果值。
        """
        try:
            result = self.controller.stop(request.reason or "ROS stop requested")
            response.stop_requested = result["stop_requested"]
            response.controller_accepted = result["controller_accepted"]
            response.stationary_confirmed = result["stationary_confirmed"]
            response.hardware_estop = result["hardware_estop"]
            response.success = True
            response.error = result["error"]
            return response
        except Exception as exc:
            return self._ok(response, exc)

    def _move_goal(self, request):
        """校验运动目标并启动动作。

        Args:
            request: 封装调用输入字段的请求对象。

        Returns:
            本次处理产生的结果值。
        """
        try:
            self.controller._validate_vector(request.positions_rad, "positions_rad")
            self.controller._validate_vector([request.speed_rad_sec or self.config["motion"]["max_speed_rad_sec"]] * len(self.config["joints"]["names"]),
                                           "speed_rad_sec")
            self.controller._require_lease(request.lease_id)
            return GoalResponse.ACCEPT if self.controller.motion_enabled else GoalResponse.REJECT
        except Exception as exc:
            self.get_logger().warning(f"Rejected arm joint goal: {exc}")
            return GoalResponse.REJECT

    def _trajectory_goal(self, request):
        """Validate a trajectory goal before accepting it for execution.

        Args:
            request: Action goal containing the lease and timed joint waypoints.

        Returns:
            GoalResponse.ACCEPT when a simulation trajectory can be executed.
        """
        try:
            if self.config.get("backend") != "ros2_sim":
                raise NotImplementedError("trajectory execution is currently supported only by ros2_sim")
            if not request.trajectory.points:
                raise ValueError("trajectory must contain at least one waypoint")
            self.controller._require_lease(request.lease_id)
            if not self.controller.motion_enabled:
                raise PermissionError("arm motion is disabled")
            return GoalResponse.ACCEPT
        except Exception as exc:
            self.get_logger().warning(f"Rejected arm trajectory goal: {exc}")
            return GoalResponse.REJECT

    @staticmethod
    def _cancel_goal(_goal_handle):
        """取消正在执行的动作目标。

        Args:
            _goal_handle: goal处理，作为取消正在执行的动作目标所需的输入。

        Returns:
            本次处理产生的结果值。
        """
        return CancelResponse.ACCEPT

    def _move_joints(self, goal_handle):
        """执行机械臂关节运动。

        Args:
            goal_handle: goal处理，作为执行机械臂关节运动所需的输入。

        Returns:
            本次处理产生的结果值。
        """
        request = goal_handle.request
        result = MoveArmJoints.Result()
        try:
            outcome = self.controller.move_joints(
                request.lease_id, request.positions_rad, request.speed_rad_sec,
                request.acceleration_rad_sec2, request.timeout_sec,
                cancel_requested=lambda: goal_handle.is_cancel_requested,
                feedback=lambda state, error: self._publish_action_feedback(goal_handle, state, error))
            result.success = outcome["success"]
            result.message = outcome["message"]
            result.max_error_rad = outcome["max_error_rad"]
            result.controller_accepted = outcome["controller_accepted"]
            result.stop_requested = outcome["stop_requested"]
            result.stationary_confirmed = outcome["stationary_confirmed"]
            result.hardware_estop = outcome["hardware_estop"]
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
            elif result.success:
                goal_handle.succeed()
            else:
                goal_handle.abort()
        except Exception as exc:
            result.success = False
            result.message = str(exc)
            result.max_error_rad = 0.0
            result.controller_accepted = self.controller.controller_accepted
            result.stop_requested = self.controller.stop_requested
            result.stationary_confirmed = self.controller.stationary_confirmed
            result.hardware_estop = bool(self.controller.feedback.get("hardware_estop", False))
            goal_handle.canceled() if goal_handle.is_cancel_requested else goal_handle.abort()
        return result

    def _execute_trajectory(self, goal_handle):
        """Execute a lease-protected, time-parameterized arm trajectory.

        Args:
            goal_handle: Accepted trajectory action goal.

        Returns:
            Action result describing execution, stop, and feedback status.
        """
        request = goal_handle.request
        result = ExecuteArmTrajectory.Result()
        trajectory = request.trajectory
        points = list(trajectory.points)
        joint_names = list(trajectory.joint_names)
        duration_sec = 0.0
        peak_velocity = {name: 0.0 for name in joint_names}
        peak_acceleration = {name: 0.0 for name in joint_names}
        if points:
            last_time = points[-1].time_from_start
            duration_sec = float(last_time.sec) + float(last_time.nanosec) * 1e-9
        for point in points:
            for index, name in enumerate(joint_names):
                if index < len(point.velocities):
                    peak_velocity[name] = max(peak_velocity[name], abs(float(point.velocities[index])))
                if index < len(point.accelerations):
                    peak_acceleration[name] = max(
                        peak_acceleration[name], abs(float(point.accelerations[index])))
        self.get_logger().info(
            f"Starting arm trajectory device={self.device_id} points={len(points)} "
            f"duration_sec={duration_sec:.4f} requested_speed_rad_sec={request.speed_rad_sec:.4f} "
            f"requested_acceleration_rad_sec2={request.acceleration_rad_sec2:.4f} "
            f"timeout_sec={request.timeout_sec:.3f} "
            f"peak_velocity_rad_sec={peak_velocity} peak_acceleration_rad_sec2={peak_acceleration}")
        last_feedback_index = -10
        last_feedback_log_time = 0.0
        final_feedback_index = max(
            0, len(points) - 1 - int(bool(points and
                points[0].time_from_start.sec == 0 and points[0].time_from_start.nanosec == 0)))

        def publish_progress(state, error, point_index):
            """Publish trajectory feedback and log sparse waypoint progress.

            Args:
                state: Execution state reported by the arm controller.
                error: Maximum measured error to the currently tracked waypoint.
                point_index: Zero-based timed-waypoint index after any start point is removed.

            Returns:
                None.
            """
            nonlocal last_feedback_index, last_feedback_log_time
            self._publish_trajectory_feedback(goal_handle, state, error, point_index)
            now = time.monotonic()
            if ((point_index == 0 and last_feedback_index != 0) or
                    point_index >= last_feedback_index + 10 or
                    (point_index == final_feedback_index and point_index != last_feedback_index) or
                    now - last_feedback_log_time >= 2.0):
                current = self.controller.state()
                snapshot = {key: current.get(key) for key in (
                    "stamp_sec", "feedback_timestamp_source", "feedback_valid", "feedback_age_sec",
                    "joint_names", "positions_rad", "velocities_rad_sec")}
                self.get_logger().info(
                    f"Arm trajectory progress device={self.device_id} "
                    f"waypoint_index={point_index}/{final_feedback_index} "
                    f"tracked_waypoint_error_rad={float(error):.5f} "
                    f"feedback={json.dumps(snapshot, sort_keys=True)}")
                last_feedback_index = point_index
                last_feedback_log_time = now

        try:
            outcome = self.controller.execute_trajectory(
                request.lease_id, request.trajectory, request.speed_rad_sec,
                request.acceleration_rad_sec2, request.timeout_sec,
                cancel_requested=lambda: goal_handle.is_cancel_requested,
                feedback=publish_progress,
            )
            result.success = outcome["success"]
            result.message = outcome["message"]
            result.max_error_rad = outcome["max_error_rad"]
            result.controller_accepted = outcome["controller_accepted"]
            result.stop_requested = outcome["stop_requested"]
            result.stationary_confirmed = outcome["stationary_confirmed"]
            result.hardware_estop = outcome["hardware_estop"]
            self.get_logger().info(
                f"Arm trajectory finished device={self.device_id} success={result.success} "
                f"message={result.message!r} stop_requested={result.stop_requested} "
                f"final_max_error_rad={result.max_error_rad:.6f} "
                f"stationary_confirmed={result.stationary_confirmed}")
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
            elif result.success:
                goal_handle.succeed()
            else:
                goal_handle.abort()
        except Exception as exc:
            result.success = False
            result.message = str(exc)
            result.max_error_rad = float(self.controller.last_trajectory_error_rad)
            result.controller_accepted = self.controller.controller_accepted
            result.stop_requested = self.controller.stop_requested
            result.stationary_confirmed = self.controller.stationary_confirmed
            result.hardware_estop = bool(self.controller.feedback.get("hardware_estop", False))
            self.get_logger().error(
                f"Arm trajectory failed device={self.device_id} "
                f"last_max_error_rad={result.max_error_rad:.6f} "
                f"controller_accepted={result.controller_accepted} "
                f"stop_requested={result.stop_requested}: {result.message}")
            goal_handle.canceled() if goal_handle.is_cancel_requested else goal_handle.abort()
        return result

    @staticmethod
    def _publish_trajectory_feedback(goal_handle, state, error, point_index):
        """Publish current trajectory phase, waypoint, and joint error.

        Args:
            goal_handle: Accepted trajectory action goal.
            state: Current execution state label.
            error: Maximum measured joint error in radians.
            point_index: Current zero-based trajectory waypoint index.

        Returns:
            None.
        """
        feedback = ExecuteArmTrajectory.Feedback()
        feedback.state = str(state)
        feedback.point_index = int(point_index)
        feedback.max_error_rad = float(error if math.isfinite(error) else 0.0)
        goal_handle.publish_feedback(feedback)

    @staticmethod
    def _publish_action_feedback(goal_handle, state, error):
        """发布动作执行反馈。

        Args:
            goal_handle: goal处理，作为发布动作执行反馈所需的输入。
            state: 状态，作为发布动作执行反馈所需的输入。
            error: error，作为发布动作执行反馈所需的输入。

        Returns:
            无返回值。
        """
        feedback = MoveArmJoints.Feedback()
        feedback.state = str(state)
        feedback.max_error_rad = float(error if math.isfinite(error) else 0.0)
        goal_handle.publish_feedback(feedback)

    def _tick(self):
        """执行一次周期性状态更新。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        self.controller.tick()
        state = self.controller.state()
        self.state_pub.publish(String(data=self._json(state)))
        positions, velocities = state.get("positions_rad"), state.get("velocities_rad_sec")
        if state["feedback_valid"] and positions is not None and velocities is not None:
            message = JointState()
            message.header.stamp = self.get_clock().now().to_msg()
            message.name = list(self.config["joints"]["names"])
            message.position = list(positions)
            message.velocity = list(velocities)
            self.joints_pub.publish(message)

    def destroy_node(self):
        """销毁节点。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。
        """
        try:
            self.controller.shutdown()
        finally:
            self.move_action.destroy()
            return super().destroy_node()


def main(args=None):
    """解析启动参数并运行程序入口。

    Args:
        args: 额外的位置参数集合。

    Returns:
        无返回值。
    """
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = ArmNode()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown(timeout_sec=4.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
