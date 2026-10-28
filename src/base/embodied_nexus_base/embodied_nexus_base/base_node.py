from __future__ import annotations

import json
import math
import time
from pathlib import Path

import rclpy
from rclpy.clock import Clock, ClockType
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from embodied_nexus_base_interfaces.action import MoveBaseDistance, TurnBaseRelative
from embodied_nexus_base_interfaces.msg import BaseState
from embodied_nexus_base_interfaces.srv import (
    AcquireBaseControl, ApplyBaseConfiguration, GetBaseConfiguration, GetBaseInfo,
    GetBaseState, ReleaseBaseControl, RenewBaseControl, SetBaseMotionEnabled,
    SetBaseVelocity, StopBase, ValidateBaseConfiguration,
)

from .config import load_config
from .controller import BaseController, relative_distance_progress, unwrapped_yaw_delta


def default_config_path() -> str:
    """返回该设备的默认配置文件路径。

    Args:
        无参数。

    Returns:
        构造或创建完成的对象或资源。 返回类型为 str。
    """
    try:
        from ament_index_python.packages import get_package_share_directory
        return str(Path(get_package_share_directory("embodied_nexus_base")) / "config" / "scout_mock.yaml")
    except Exception:
        return str(Path(__file__).resolve().parents[1] / "config" / "scout_mock.yaml")


class BaseNode(Node):
    def __init__(self, config_path: str | None = None) -> None:
        """初始化对象并保存构造参数或依赖。

        Args:
            config_path: 配置文件路径，指向要加载的设备配置。 类型为 str | None。

        Returns:
            无返回值。
        """
        super().__init__("embodied_nexus_base")
        self.declare_parameter("config_file", config_path or default_config_path())
        self.declare_parameter("interface_mode", "")
        self.config = load_config(str(self.get_parameter("config_file").value))
        mode = str(self.get_parameter("interface_mode").value).strip()
        if mode:
            self.config["interface_mode"] = mode
        self.controller = BaseController(self.config)
        self.device_id = self.config["device_id"]
        self.prefix = f"/embodied_nexus/bases/{self.device_id}"
        self.state_pub = self.create_publisher(BaseState, f"{self.prefix}/state", 10)
        self.state_json_pub = self.create_publisher(String, f"{self.prefix}/_internal/state_json", 10)
        self.odom_pub = self.create_publisher(Odometry, f"{self.prefix}/odom", 20)
        self.tf_broadcaster = TransformBroadcaster(self) if self.config["frames"]["publish_tf"] else None
        self._services = [
            self.create_service(GetBaseInfo, f"{self.prefix}/info", self._info),
            self.create_service(GetBaseState, f"{self.prefix}/get_state", self._state),
            self.create_service(GetBaseConfiguration, f"{self.prefix}/config/get", self._get_config),
            self.create_service(ValidateBaseConfiguration, f"{self.prefix}/config/validate", self._validate),
            self.create_service(ApplyBaseConfiguration, f"{self.prefix}/config/apply", self._apply),
            self.create_service(AcquireBaseControl, f"{self.prefix}/control/acquire", self._acquire),
            self.create_service(RenewBaseControl, f"{self.prefix}/control/renew", self._renew),
            self.create_service(ReleaseBaseControl, f"{self.prefix}/control/release", self._release),
            self.create_service(SetBaseMotionEnabled, f"{self.prefix}/motion/enable", self._enable),
            self.create_service(SetBaseVelocity, f"{self.prefix}/velocity", self._velocity),
            self.create_service(StopBase, f"{self.prefix}/stop", self._stop),
        ]
        self.distance_action = ActionServer(
            self, MoveBaseDistance, f"{self.prefix}/move_relative",
            execute_callback=self._move_distance, goal_callback=self._distance_goal,
            cancel_callback=self._cancel_action)
        self.turn_action = ActionServer(
            self, TurnBaseRelative, f"{self.prefix}/turn_relative",
            execute_callback=self._turn_relative, goal_callback=self._turn_goal,
            cancel_callback=self._cancel_action)
        hz = max(float(self.config["motion"]["send_rate_hz"]), 1.0)
        self.timer = self.create_timer(1.0 / hz, self._tick,
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
    def _ok(response, success=True, error=""):
        """构造表示操作成功的响应。

        Args:
            response: 服务端或被调用操作返回的响应对象。
            success: success，作为构造表示操作成功的响应所需的输入。
            error: error，作为构造表示操作成功的响应所需的输入。

        Returns:
            本次处理产生的结果值。
        """
        response.success = success
        response.error = error
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
            return self._ok(response, False, str(exc))

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
            return self._ok(response, False, str(exc))

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
            return self._ok(response, False, str(exc))

    def _validate(self, request, response):
        """验证当前设备或操作。

        Args:
            request: 封装调用输入字段的请求对象。
            response: 服务端或被调用操作返回的响应对象。

        Returns:
            本次处理产生的结果值。
        """
        try:
            candidate = self.controller.validate_configuration(json.loads(request.patch_json))
            response.valid = True
            response.config_json = self._json(candidate)
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
            return self._ok(response, False, str(exc))

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
            return self._ok(response, False, str(exc))

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
            return self._ok(response, False, str(exc))

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
            response.stop_sent = result["stop_requested"]
            response.zero_velocity_written = result["zero_velocity_written"]
            return self._ok(response, True, result["error"])
        except Exception as exc:
            return self._ok(response, False, str(exc))

    def _enable(self, request, response):
        """更新设备运动使能状态。

        Args:
            request: 封装调用输入字段的请求对象。
            response: 服务端或被调用操作返回的响应对象。

        Returns:
            本次处理产生的结果值。
        """
        try:
            result = self.controller.enable_motion(request.lease_id, request.enabled)
            response.enabled = result["enabled"]
            response.control_mode_confirmed = result["control_mode_confirmed"]
            response.stop_requested = result.get("stop_requested", False)
            response.zero_velocity_written = result.get("zero_velocity_written", False)
            return self._ok(response, True, result.get("error", ""))
        except Exception as exc:
            response.enabled = False
            response.control_mode_confirmed = False
            return self._ok(response, False, str(exc))

    def _velocity(self, request, response):
        """更新设备速度指令。

        Args:
            request: 封装调用输入字段的请求对象。
            response: 服务端或被调用操作返回的响应对象。

        Returns:
            本次处理产生的结果值。
        """
        try:
            self.controller.set_velocity(request.lease_id, request.linear_mps, request.angular_rps)
            return self._ok(response)
        except Exception as exc:
            return self._ok(response, False, str(exc))

    def _stop(self, request, response):
        """停止设备当前动作。

        Args:
            request: 封装调用输入字段的请求对象。
            response: 服务端或被调用操作返回的响应对象。

        Returns:
            本次处理产生的结果值。
        """
        try:
            result = self.controller.stop(request.reason or "stop requested")
            response.stop_requested = result["stop_requested"]
            response.zero_velocity_written = result["zero_velocity_written"]
            response.stationary_confirmed = result["stationary_confirmed"]
            response.hardware_estop = result["hardware_estop"]
            return self._ok(response, True, result["error"])
        except Exception as exc:
            return self._ok(response, False, str(exc))

    def _distance_goal(self, request):
        """处理底盘距离运动动作的目标请求。

        Args:
            request: 封装调用输入字段的请求对象。

        Returns:
            本次处理产生的结果值。
        """
        try:
            distance = self.controller._finite(request.distance_m, "distance_m")
            speed = self.controller._finite(request.speed_mps, "speed_mps")
            if distance == 0 or abs(distance) > 100.0:
                return GoalResponse.REJECT
            if speed < 0 or speed > self.config["motion"]["max_linear_mps"]:
                return GoalResponse.REJECT
            self.controller._require_lease(request.lease_id)
            return GoalResponse.ACCEPT
        except Exception:
            return GoalResponse.REJECT

    def _turn_goal(self, request):
        """处理底盘转向动作的目标请求。

        Args:
            request: 封装调用输入字段的请求对象。

        Returns:
            本次处理产生的结果值。
        """
        try:
            angle = self.controller._finite(request.angle_rad, "angle_rad")
            speed = self.controller._finite(request.speed_rps, "speed_rps")
            if angle == 0 or abs(angle) > 2 * math.pi * 20:
                return GoalResponse.REJECT
            if speed < 0 or speed > self.config["motion"]["max_angular_rps"]:
                return GoalResponse.REJECT
            self.controller._require_lease(request.lease_id)
            return GoalResponse.ACCEPT
        except Exception:
            return GoalResponse.REJECT

    @staticmethod
    def _cancel_action(_goal_handle):
        """取消动作。

        Args:
            _goal_handle: goal处理，作为取消动作所需的输入。

        Returns:
            本次处理产生的结果值。
        """
        return CancelResponse.ACCEPT

    def _move_distance(self, goal_handle):
        """运动distance。

        Args:
            goal_handle: goal处理，作为运动distance所需的输入。

        Returns:
            本次处理产生的结果值。
        """
        request = goal_handle.request
        result = MoveBaseDistance.Result()
        operation_id = ""
        success = False
        progress = 0.0
        message = ""
        motion = self.config["motion"]
        tolerance = float(motion["position_tolerance_m"])
        speed = request.speed_mps or float(motion["default_linear_mps"])
        target = float(request.distance_m)
        deadline = time.monotonic() + float(motion["operation_timeout_sec"])
        try:
            operation_id, start = self.controller.begin_operation(request.lease_id)
            while rclpy.ok() and time.monotonic() < deadline:
                if goal_handle.is_cancel_requested:
                    message = "cancelled"
                    break
                state = self.controller.state()
                if not state["motion_feedback_valid"] or not state["pose_estimate_valid"]:
                    raise RuntimeError("odometry feedback became stale")
                if not state["motion_enabled"]:
                    raise RuntimeError("base motion was stopped")
                progress = relative_distance_progress(
                    start, (float(state["x_m"]), float(state["y_m"])))
                remaining = target - progress
                feedback = MoveBaseDistance.Feedback()
                feedback.progress_m = progress
                feedback.remaining_m = remaining
                goal_handle.publish_feedback(feedback)
                if ((target > 0 and progress >= target - tolerance) or
                        (target < 0 and progress <= target + tolerance)):
                    success = True
                    message = "distance reached within configured tolerance"
                    break
                self.controller.set_operation_velocity(operation_id, math.copysign(speed, target), 0.0)
                time.sleep(0.05)
            if not message:
                message = "operation timeout" if time.monotonic() >= deadline else "ROS is shutting down"
        except Exception as exc:
            message = str(exc)
        finally:
            if operation_id:
                terminal = "succeeded" if success else "cancelled" if goal_handle.is_cancel_requested else "failed"
                stopped = self.controller.finish_operation_with_zero(operation_id, terminal)
            else:
                stopped = {"stop_requested": False, "zero_velocity_written": False}
        result.success = success
        if success and not stopped["zero_velocity_written"]:
            result.success = False
            message = f"{message}; zero velocity write failed"
        result.message = message
        result.progress_m = progress
        result.stop_requested = stopped["stop_requested"]
        result.zero_velocity_written = stopped["zero_velocity_written"]
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
        elif success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def _turn_relative(self, goal_handle):
        """执行底盘相对转向动作并检查完成状态。

        Args:
            goal_handle: goal处理，作为执行 _turn_relative 对应的处理流程所需的输入。

        Returns:
            本次处理产生的结果值。
        """
        request = goal_handle.request
        result = TurnBaseRelative.Result()
        operation_id = ""
        success = False
        progress = 0.0
        message = ""
        motion = self.config["motion"]
        tolerance = math.radians(float(motion["yaw_tolerance_deg"]))
        speed = request.speed_rps or float(motion["default_angular_rps"])
        target = float(request.angle_rad)
        deadline = time.monotonic() + float(motion["operation_timeout_sec"])
        previous_yaw = None
        try:
            operation_id, start = self.controller.begin_operation(request.lease_id)
            previous_yaw = start[2]
            while rclpy.ok() and time.monotonic() < deadline:
                if goal_handle.is_cancel_requested:
                    message = "cancelled"
                    break
                state = self.controller.state()
                if not state["motion_feedback_valid"] or not state["pose_estimate_valid"]:
                    raise RuntimeError("odometry feedback became stale")
                if not state["motion_enabled"]:
                    raise RuntimeError("base motion was stopped")
                yaw = float(state["yaw_rad"])
                delta = unwrapped_yaw_delta(yaw, previous_yaw)
                progress += delta
                previous_yaw = yaw
                remaining = target - progress
                feedback = TurnBaseRelative.Feedback()
                feedback.progress_rad = progress
                feedback.remaining_rad = remaining
                goal_handle.publish_feedback(feedback)
                if ((target > 0 and progress >= target - tolerance) or
                        (target < 0 and progress <= target + tolerance)):
                    success = True
                    message = "relative turn reached within configured tolerance"
                    break
                self.controller.set_operation_velocity(operation_id, 0.0, math.copysign(speed, target))
                time.sleep(0.05)
            if not message:
                message = "operation timeout" if time.monotonic() >= deadline else "ROS is shutting down"
        except Exception as exc:
            message = str(exc)
        finally:
            if operation_id:
                terminal = "succeeded" if success else "cancelled" if goal_handle.is_cancel_requested else "failed"
                stopped = self.controller.finish_operation_with_zero(operation_id, terminal)
            else:
                stopped = {"stop_requested": False, "zero_velocity_written": False}
        result.success = success
        if success and not stopped["zero_velocity_written"]:
            result.success = False
            message = f"{message}; zero velocity write failed"
        result.message = message
        result.progress_rad = progress
        result.stop_requested = stopped["stop_requested"]
        result.zero_velocity_written = stopped["zero_velocity_written"]
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
        elif success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def _tick(self):
        """执行一次周期性状态更新。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        self.controller.tick()
        state = self.controller.state()
        message = BaseState()
        message.header.stamp = self.get_clock().now().to_msg()
        mapping = {"device_id": "device_id", "model": "model", "backend": "backend",
                   "lifecycle_state": "lifecycle_state", "connected": "connected",
                   "motion_enabled": "motion_enabled", "operation_id": "operation_id",
                   "operation_state": "operation_state", "lease_owner": "lease_owner",
                   "last_error": "last_error", "system_feedback_valid": "system_feedback_valid",
                   "system_feedback_age_sec": "system_feedback_age_sec",
                   "motion_feedback_valid": "motion_feedback_valid",
                   "motion_feedback_age_sec": "motion_feedback_age_sec",
                   "pose_estimate_valid": "pose_estimate_valid", "stop_requested": "stop_requested",
                   "zero_velocity_written": "zero_velocity_written",
                   "stationary_confirmed": "stationary_confirmed", "hardware_estop": "hardware_estop"}
        mapping["fault_active"] = "fault_active"
        mapping["control_mode_confirmed"] = "control_mode_confirmed"
        for target, source in mapping.items():
            value = state[source]
            if value is not None:
                setattr(message, target, value)
        for key in ("battery_voltage", "measured_linear_mps", "measured_angular_rps",
                    "x_m", "y_m", "yaw_rad"):
            value = state[key]
            if value is not None:
                setattr(message, key, value)
        odom = Odometry()
        odom.header = message.header
        odom.header.frame_id = self.config["frames"]["odom"]
        odom.child_frame_id = self.config["frames"]["base"]
        if state["pose_estimate_valid"]:
            odom.pose.pose.position.x = state["x_m"]
            odom.pose.pose.position.y = state["y_m"]
            odom.pose.pose.orientation.z = math.sin(state["yaw_rad"] / 2.0)
            odom.pose.pose.orientation.w = math.cos(state["yaw_rad"] / 2.0)
        if state["motion_feedback_valid"]:
            odom.twist.twist.linear.x = state["measured_linear_mps"]
            odom.twist.twist.angular.z = state["measured_angular_rps"]
        self.state_pub.publish(message)
        self.state_json_pub.publish(String(data=self._json(state)))
        self.odom_pub.publish(odom)
        if self.tf_broadcaster is not None and state["pose_estimate_valid"]:
            transform = TransformStamped()
            transform.header = odom.header
            transform.child_frame_id = odom.child_frame_id
            transform.transform.translation.x = state["x_m"]
            transform.transform.translation.y = state["y_m"]
            transform.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)

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
            self.distance_action.destroy()
            self.turn_action.destroy()
            return super().destroy_node()


def main(args=None):
    """解析启动参数并运行程序入口。

    Args:
        args: 额外的位置参数集合。

    Returns:
        无返回值。
    """
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = BaseNode()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown(timeout_sec=3.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
