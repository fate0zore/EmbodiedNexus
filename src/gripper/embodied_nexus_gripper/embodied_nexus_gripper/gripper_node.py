from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.parameter import Parameter
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from embodied_nexus_gripper_interfaces.action import MoveGripper
from embodied_nexus_gripper_interfaces.msg import GripperState
from embodied_nexus_gripper_interfaces.srv import (
    AcquireGripperControl, ApplyGripperConfiguration, GetGripperConfiguration,
    GetGripperInfo, GetGripperState, ReleaseGripperControl,
    RenewGripperControl, SetGripperMotionEnabled, StopGripper,
    ValidateGripperConfiguration,
)

from .config import load_config
from .controller import GripperController, MotionResult


def _default_config_path() -> str:
    """获取默认配置文件路径。

    Args:
        无参数。

    Returns:
        构造或创建完成的对象或资源。 返回类型为 str。
    """
    try:
        from ament_index_python.packages import get_package_share_directory
        return str(Path(get_package_share_directory("embodied_nexus_gripper")) / "config" / "ms42dc.yaml")
    except Exception:
        return str(Path(__file__).resolve().parents[1] / "config" / "ms42dc.yaml")


class GripperNode(Node):
    def __init__(self, config_path: str | None = None) -> None:
        """初始化对象并保存构造参数或依赖。

        Args:
            config_path: 配置文件路径，指向要加载的设备配置。 类型为 str | None。

        Returns:
            无返回值。
        """
        overrides = [Parameter("config_file", value=config_path)] if config_path else None
        super().__init__("embodied_nexus_gripper", parameter_overrides=overrides)
        self.declare_parameter("config_file", _default_config_path())
        self.declare_parameter("interface_mode", "")
        self.declare_parameter("web.bind_host", "")
        self.declare_parameter("web.port", 0)
        self.declare_parameter("web.auth_token_env", "")
        config = load_config(str(self.get_parameter("config_file").value))
        mode = str(self.get_parameter("interface_mode").value).strip()
        if mode:
            config["interface_mode"] = mode
        self.controller = GripperController(config)
        if not self.controller.state()["connected"]:
            self.get_logger().warning(f"gripper backend is not connected: {self.controller.state()['last_error']}")
        self.callback_group = ReentrantCallbackGroup()
        self.prefix = f"/embodied_nexus/grippers/{config['device_id']}"
        self.state_pub = self.create_publisher(GripperState, f"{self.prefix}/state", 10)

        self.create_service(GetGripperInfo, f"{self.prefix}/info", self._get_info, callback_group=self.callback_group)
        self.create_service(GetGripperState, f"{self.prefix}/get_state", self._get_state, callback_group=self.callback_group)
        self.create_service(GetGripperConfiguration, f"{self.prefix}/config", self._get_config, callback_group=self.callback_group)
        self.create_service(ValidateGripperConfiguration, f"{self.prefix}/config/validate", self._validate_config, callback_group=self.callback_group)
        self.create_service(ApplyGripperConfiguration, f"{self.prefix}/config/apply", self._apply_config, callback_group=self.callback_group)
        self.create_service(AcquireGripperControl, f"{self.prefix}/control/acquire", self._acquire_control, callback_group=self.callback_group)
        self.create_service(RenewGripperControl, f"{self.prefix}/control/renew", self._renew_control, callback_group=self.callback_group)
        self.create_service(ReleaseGripperControl, f"{self.prefix}/control/release", self._release_control, callback_group=self.callback_group)
        self.create_service(SetGripperMotionEnabled, f"{self.prefix}/motion/enable", self._set_motion_enabled, callback_group=self.callback_group)
        self.create_service(StopGripper, f"{self.prefix}/stop", self._stop, callback_group=self.callback_group)
        self.action_server = ActionServer(
            self, MoveGripper, f"{self.prefix}/move",
            execute_callback=self._execute_move,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self.callback_group,
        )
        self.timer = self.create_timer(float(config["feedback"]["poll_interval_sec"]), self._publish_state,
                                       callback_group=self.callback_group,
                                       clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(
            f"gripper {config['device_id']} model={config['model']} backend={config['backend']} "
            f"state_topic={self.prefix}/state"
        )

    @staticmethod
    def _json(value: Any) -> str:
        """将输入值转换为可写入 JSON 的数据。

        Args:
            value: 要应用、比较或转换的输入值。 类型为 Any。

        Returns:
            本次处理产生的结果值。 返回类型为 str。
        """
        return json.dumps(value, ensure_ascii=False, separators=(",", ":"))

    def _get_info(self, _request, response):
        """获取信息。

        Args:
            _request: 请求，作为获取信息所需的输入。
            response: 服务端或被调用操作返回的响应对象。

        Returns:
            查询或读取到的数据；未找到时的处理方式由实现决定。
        """
        response.success = True
        response.error = ""
        response.descriptor_json = self._json(self.controller.descriptor())
        return response

    def _get_state(self, _request, response):
        """获取状态。

        Args:
            _request: 请求，作为获取状态所需的输入。
            response: 服务端或被调用操作返回的响应对象。

        Returns:
            查询或读取到的数据；未找到时的处理方式由实现决定。
        """
        self._publish_state()
        self._fill_state(response.state)
        return response

    def _fill_state(self, message: GripperState):
        """将内部设备状态复制到 ROS 状态消息。

        Args:
            message: 待处理的消息对象。 类型为 GripperState。

        Returns:
            本次处理产生的结果值。
        """
        state = self.controller.state()
        message.header.stamp = self.get_clock().now().to_msg()
        message.device_id = state["device_id"]
        message.model = state["model"]
        message.backend = state["backend"]
        message.lifecycle_state = state["lifecycle_state"]
        message.connected = state["connected"]
        message.motion_enabled = state["motion_enabled"]
        message.operation_id = state["operation_id"]
        message.operation_state = state["operation_state"]
        message.last_error = state["operation_error"] or state["last_error"]
        message.lease_owner = state["lease_owner"]
        message.config_version = state["config_version"]
        message.command_target_known = state["command_target_known"]
        message.command_target_degrees = float(state["command_target_degrees"] or 0.0)
        message.feedback_valid = state["feedback_valid"]
        message.feedback_sequence = state["feedback_sequence"]
        message.feedback_moving = state["feedback_moving"]
        message.feedback_reached = state["feedback_reached"]
        message.feedback_stationary = state["feedback_stationary"]
        message.feedback_age_sec = float(
            state["feedback_age_sec"] if state["feedback_age_sec"] is not None else -1.0
        )
        message.stop_sent = state["stop_sent"]
        message.hardware_stop_confirmed = state["hardware_stop_confirmed"]
        return message

    def _publish_state(self) -> None:
        """发布状态。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        self.controller.poll_once()
        self.state_pub.publish(self._fill_state(GripperState()))

    def _get_config(self, _request, response):
        """读取设备配置并构造服务响应。

        Args:
            _request: 请求，作为读取设备配置并构造服务响应所需的输入。
            response: 服务端或被调用操作返回的响应对象。

        Returns:
            查询或读取到的数据；未找到时的处理方式由实现决定。
        """
        response.success = True
        response.error = ""
        response.config_json = self._json(self.controller.configuration())
        return response

    def _validate_config(self, request, response):
        """验证配置。

        Args:
            request: 封装调用输入字段的请求对象。
            response: 服务端或被调用操作返回的响应对象。

        Returns:
            布尔值，表示验证配置是否成功或成立。
        """
        try:
            patch = json.loads(request.patch_json)
            response.config_json = self._json(self.controller.validate_configuration(patch))
            response.valid = True
            response.error = ""
        except Exception as exc:
            response.valid = False
            response.error = str(exc)
            response.config_json = "{}"
        return response

    def _apply_config(self, request, response):
        """应用配置。

        Args:
            request: 封装调用输入字段的请求对象。
            response: 服务端或被调用操作返回的响应对象。

        Returns:
            本次处理产生的结果值。
        """
        try:
            patch = json.loads(request.patch_json)
            response.config_json = self._json(self.controller.apply_configuration(patch, request.lease_id))
            response.success = True
            response.error = ""
        except Exception as exc:
            response.success = False
            response.error = str(exc)
            response.config_json = "{}"
        return response

    def _acquire_control(self, request, response):
        """获取控制权。

        Args:
            request: 封装调用输入字段的请求对象。
            response: 服务端或被调用操作返回的响应对象。

        Returns:
            本次处理产生的结果值。
        """
        try:
            response.lease_id, response.expires_in_sec = self.controller.acquire_control(
                request.owner, request.ttl_sec or None
            )
            response.success = True
            response.error = ""
        except Exception as exc:
            response.success = False
            response.error = str(exc)
            response.lease_id = ""
            response.expires_in_sec = 0.0
        return response

    def _renew_control(self, request, response):
        """续租控制权。

        Args:
            request: 封装调用输入字段的请求对象。
            response: 服务端或被调用操作返回的响应对象。

        Returns:
            本次处理产生的结果值。
        """
        try:
            response.expires_in_sec = self.controller.renew_control(request.lease_id)
            response.success = True
            response.error = ""
        except Exception as exc:
            response.success = False
            response.error = str(exc)
            response.expires_in_sec = 0.0
        return response

    def _release_control(self, request, response):
        """释放控制权。

        Args:
            request: 封装调用输入字段的请求对象。
            response: 服务端或被调用操作返回的响应对象。

        Returns:
            本次处理产生的结果值。
        """
        try:
            self.controller.release_control(request.lease_id)
            response.success = True
            response.error = ""
        except Exception as exc:
            response.success = False
            response.error = str(exc)
        return response

    def _set_motion_enabled(self, request, response):
        """设置motionenabled。

        Args:
            request: 封装调用输入字段的请求对象。
            response: 服务端或被调用操作返回的响应对象。

        Returns:
            本次处理产生的结果值。
        """
        try:
            self.controller.set_motion_enabled(request.lease_id, request.enabled)
            response.success = True
            response.error = ""
        except Exception as exc:
            response.success = False
            response.error = str(exc)
        return response

    def _stop(self, request, response):
        """停止设备当前动作。

        Args:
            request: 封装调用输入字段的请求对象。
            response: 服务端或被调用操作返回的响应对象。

        Returns:
            本次处理产生的结果值。
        """
        result = self.controller.stop(request.reason)
        response.success = result["success"]
        response.stop_sent = result["stop_sent"]
        response.hardware_stop_confirmed = result["hardware_stop_confirmed"]
        response.error = result["error"]
        return response

    def _goal_callback(self, request):
        """校验底盘运动目标并返回接受或拒绝结果。

        Args:
            request: 封装调用输入字段的请求对象。

        Returns:
            本次处理产生的结果值。
        """
        if not request.client_operation_id:
            self.get_logger().warning("Rejecting gripper motion goal without a client_operation_id")
            return GoalResponse.REJECT
        try:
            self.controller.reserve_motion_goal(
                request.lease_id, request.command, request.angle_degrees, request.client_operation_id
            )
            return GoalResponse.ACCEPT
        except Exception as exc:
            self.get_logger().warning(f"Rejecting gripper motion goal: {exc}")
            return GoalResponse.REJECT

    def _cancel_callback(self, goal_handle):
        """取消callback。

        Args:
            goal_handle: goal处理，作为取消callback所需的输入。

        Returns:
            本次处理产生的结果值。
        """
        self.controller.cancel_reserved_goal(goal_handle.request.client_operation_id)
        return CancelResponse.ACCEPT

    def _execute_move(self, goal_handle):
        """执行运动。

        Args:
            goal_handle: goal处理，作为执行运动所需的输入。

        Returns:
            本次处理产生的结果值。
        """
        request = goal_handle.request
        result = MoveGripper.Result()

        def publish(state: dict) -> None:
            """发布当前设备或操作。

            Args:
                state: 状态，作为发布当前设备或操作所需的输入。 类型为 dict。

            Returns:
                无返回值。
            """
            feedback = MoveGripper.Feedback()
            feedback.operation_id = str(request.client_operation_id)
            feedback.state = str(state["operation_state"])
            feedback.feedback_age_sec = float(
                state["feedback_age_sec"] if state["feedback_age_sec"] is not None else -1.0
            )
            goal_handle.publish_feedback(feedback)

        try:
            outcome: MotionResult = self.controller.move(
                request.lease_id, request.command, request.angle_degrees,
                cancel_requested=lambda: goal_handle.is_cancel_requested,
                feedback_callback=publish,
                operation_id=request.client_operation_id,
            )
            result.success = outcome.success
            result.operation_id = outcome.operation_id
            result.error = outcome.error
            result.stop_sent = outcome.stop_sent
            result.hardware_stop_confirmed = outcome.hardware_stop_confirmed
            if outcome.cancelled:
                goal_handle.canceled()
            elif outcome.success:
                goal_handle.succeed()
            else:
                goal_handle.abort()
        except Exception as exc:
            result.success = False
            result.operation_id = request.client_operation_id
            result.error = str(exc)
            stop = self.controller.stop("motion action failed")
            result.stop_sent = stop["stop_sent"]
            result.hardware_stop_confirmed = stop["hardware_stop_confirmed"]
            goal_handle.abort()
        return result

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
            return super().destroy_node()


def main(argv=None) -> None:
    """解析启动参数并运行程序入口。

    Args:
        argv: argv，作为解析启动参数并运行程序入口所需的输入。

    Returns:
        无返回值。
    """
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--config", default=None)
    args, ros_args = parser.parse_known_args(argv)
    rclpy.init(args=ros_args)
    node = GripperNode(args.config)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
