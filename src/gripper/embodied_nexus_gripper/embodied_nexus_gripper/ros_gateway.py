from __future__ import annotations

import json
import threading
import time
import uuid
from pathlib import Path
from typing import Any

from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from action_msgs.msg import GoalStatus

from embodied_nexus_gripper_interfaces.action import MoveGripper
from embodied_nexus_gripper_interfaces.srv import (
    AcquireGripperControl, ApplyGripperConfiguration, GetGripperConfiguration,
    GetGripperInfo, GetGripperState, ReleaseGripperControl,
    RenewGripperControl, SetGripperMotionEnabled, StopGripper,
    ValidateGripperConfiguration,
)

from .config import load_config


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


class GripperRosGateway(Node):
    """ROS client facade for the HTTP gateway; it never opens hardware."""

    def __init__(self, device_id: str | None = None, config_path: str | None = None) -> None:
        """初始化对象并保存构造参数或依赖。

        Args:
            device_id: 设备标识符，作为初始化对象并保存构造参数或依赖所需的输入。 类型为 str | None。
            config_path: 配置文件路径，指向要加载的设备配置。 类型为 str | None。

        Returns:
            无返回值。
        """
        overrides = [Parameter("config_file", value=config_path)] if config_path else None
        super().__init__("embodied_nexus_gripper_web", parameter_overrides=overrides)
        self.declare_parameter("config_file", _default_config_path())
        self.declare_parameter("interface_mode", "")
        self.declare_parameter("web.bind_host", "")
        self.declare_parameter("web.port", 0)
        self.declare_parameter("web.auth_token_env", "")
        self._config_data = load_config(str(self.get_parameter("config_file").value))
        mode = str(self.get_parameter("interface_mode").value).strip()
        if mode:
            self._config_data["interface_mode"] = mode
        for key in ("bind_host", "port", "auth_token_env"):
            value = self.get_parameter(f"web.{key}").value
            if value:
                self._config_data["web"][key] = value
        self.device_id = device_id or self._config_data["device_id"]
        self.prefix = f"/embodied_nexus/grippers/{self.device_id}"
        self._service_specs = {
            "info": (GetGripperInfo, f"{self.prefix}/info"),
            "state": (GetGripperState, f"{self.prefix}/get_state"),
            "config": (GetGripperConfiguration, f"{self.prefix}/config"),
            "validate": (ValidateGripperConfiguration, f"{self.prefix}/config/validate"),
            "apply": (ApplyGripperConfiguration, f"{self.prefix}/config/apply"),
            "acquire": (AcquireGripperControl, f"{self.prefix}/control/acquire"),
            "renew": (RenewGripperControl, f"{self.prefix}/control/renew"),
            "release": (ReleaseGripperControl, f"{self.prefix}/control/release"),
            "enable": (SetGripperMotionEnabled, f"{self.prefix}/motion/enable"),
            "stop": (StopGripper, f"{self.prefix}/stop"),
        }
        self._service_clients = {
            key: self.create_client(kind, name) for key, (kind, name) in self._service_specs.items()
        }
        self.action_client = ActionClient(self, MoveGripper, f"{self.prefix}/move")
        self._operations: dict[str, dict[str, Any]] = {}
        self._goal_handles: dict[str, Any] = {}
        self._operation_lock = threading.RLock()

    def _call(self, name: str, request: Any, timeout: float = 5.0):
        """调用 ROS 服务并等待调用结果。

        Args:
            name: 名称，作为调用 ROS 服务并等待调用结果所需的输入。 类型为 str。
            request: 封装调用输入字段的请求对象。 类型为 Any。
            timeout: 本次网络或服务调用的最长等待时长，通常以秒为单位。 类型为 float。

        Returns:
            本次处理产生的结果值。
        """
        client = self._service_clients[name]
        if not client.wait_for_service(timeout_sec=timeout):
            raise TimeoutError(f"ROS service {self._service_specs[name][1]} is unavailable")
        event = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _future: event.set())
        if not event.wait(timeout):
            raise TimeoutError(f"ROS service {self._service_specs[name][1]} timed out")
        response = future.result()
        if response is None:
            raise RuntimeError(f"ROS service {client.srv_name} returned no response")
        return response

    @staticmethod
    def _require(response: Any, success: bool, error: str = "") -> Any:
        """检查当前设备或操作。

        Args:
            response: 服务端或被调用操作返回的响应对象。 类型为 Any。
            success: success，作为检查当前设备或操作所需的输入。 类型为 bool。
            error: error，作为检查当前设备或操作所需的输入。 类型为 str。

        Returns:
            本次处理产生的结果值。 返回类型为 Any。
        """
        if not success:
            raise RuntimeError(error or "gripper request failed")
        return response

    def info(self) -> dict:
        """获取设备能力与静态信息。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。 返回类型为 dict。
        """
        response = self._call("info", GetGripperInfo.Request())
        self._require(response, response.success, response.error)
        return json.loads(response.descriptor_json)

    def state(self) -> dict:
        """获取设备当前状态。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。 返回类型为 dict。
        """
        response = self._call("state", GetGripperState.Request())
        message = response.state
        stamp = message.header.stamp
        reached = self._config_data["feedback"]["reached_value"]
        simulated_stop_feedback = (
            self._config_data["backend"] == "ros2_sim" and not message.motion_enabled and
            message.feedback_valid and message.feedback_speed_raw == 0 and
            reached is not None and message.feedback_reached == int(reached)
        )
        return {
            "device_id": message.device_id, "backend": message.backend,
            "lifecycle_state": message.lifecycle_state, "connected": message.connected,
            "motion_enabled": message.motion_enabled, "operation_id": message.operation_id,
            "operation_state": message.operation_state, "last_error": message.last_error,
            "lease_owner": message.lease_owner, "config_version": message.config_version,
            "command_target_known": message.command_target_known,
            "command_target_degrees": message.command_target_degrees if message.command_target_known else None,
            "feedback_valid": message.feedback_valid, "feedback_sequence": message.feedback_sequence,
            "feedback_device_id": message.feedback_device_id, "feedback_reached": message.feedback_reached,
            "feedback_speed_raw": message.feedback_speed_raw, "feedback_angle_raw": message.feedback_angle_raw,
            "feedback_age_sec": message.feedback_age_sec if message.feedback_age_sec >= 0 else None,
            "stop_sent": message.stop_sent, "hardware_stop_confirmed": message.hardware_stop_confirmed,
            "stop_feedback_confirmed": bool(message.hardware_stop_confirmed or simulated_stop_feedback),
            "stamp": {"sec": stamp.sec, "nanosec": stamp.nanosec},
        }

    def config(self) -> dict:
        """获取当前配置。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。 返回类型为 dict。
        """
        response = self._call("config", GetGripperConfiguration.Request())
        self._require(response, response.success, response.error)
        return json.loads(response.config_json)

    def validate_config(self, patch: dict) -> dict:
        """验证配置。

        Args:
            patch: patch，作为验证配置所需的输入。 类型为 dict。

        Returns:
            布尔值，表示验证配置是否成功或成立。 返回类型为 dict。
        """
        request = ValidateGripperConfiguration.Request()
        request.patch_json = json.dumps(patch)
        response = self._call("validate", request)
        self._require(response, response.valid, response.error)
        return json.loads(response.config_json)

    def apply_config(self, lease_id: str, patch: dict) -> dict:
        """应用配置。

        Args:
            lease_id: 租约标识符，作为应用配置所需的输入。 类型为 str。
            patch: patch，作为应用配置所需的输入。 类型为 dict。

        Returns:
            本次处理产生的结果值。 返回类型为 dict。
        """
        request = ApplyGripperConfiguration.Request()
        request.lease_id, request.patch_json = lease_id, json.dumps(patch)
        response = self._call("apply", request)
        self._require(response, response.success, response.error)
        return json.loads(response.config_json)

    def acquire(self, owner: str, ttl_sec: float | None = None) -> dict:
        """申请设备控制权。

        Args:
            owner: owner，作为申请设备控制权所需的输入。 类型为 str。
            ttl_sec: ttlsec，作为申请设备控制权所需的输入。 类型为 float | None。

        Returns:
            本次处理产生的结果值。 返回类型为 dict。
        """
        request = AcquireGripperControl.Request()
        request.owner, request.ttl_sec = owner, float(ttl_sec or 0.0)
        response = self._call("acquire", request)
        self._require(response, response.success, response.error)
        return {"lease_id": response.lease_id, "expires_in_sec": response.expires_in_sec}

    def renew(self, lease_id: str) -> dict:
        """续租当前设备控制权。

        Args:
            lease_id: 租约标识符，作为续租当前设备控制权所需的输入。 类型为 str。

        Returns:
            本次处理产生的结果值。 返回类型为 dict。
        """
        request = RenewGripperControl.Request()
        request.lease_id = lease_id
        response = self._call("renew", request)
        self._require(response, response.success, response.error)
        return {"expires_in_sec": response.expires_in_sec}

    def release(self, lease_id: str) -> dict:
        """释放当前设备控制租约。

        Args:
            lease_id: 租约标识符，作为释放当前设备控制租约所需的输入。 类型为 str。

        Returns:
            本次处理产生的结果值。 返回类型为 dict。
        """
        request = ReleaseGripperControl.Request()
        request.lease_id = lease_id
        response = self._call("release", request)
        self._require(response, response.success, response.error)
        return {"released": True}

    def enable(self, lease_id: str, enabled: bool) -> dict:
        """更新设备运动使能状态。

        Args:
            lease_id: 租约标识符，作为更新设备运动使能状态所需的输入。 类型为 str。
            enabled: 表示对应功能是否启用的布尔值。 类型为 bool。

        Returns:
            本次处理产生的结果值。 返回类型为 dict。
        """
        request = SetGripperMotionEnabled.Request()
        request.lease_id, request.enabled = lease_id, bool(enabled)
        response = self._call("enable", request)
        self._require(response, response.success, response.error)
        return {"motion_enabled": bool(enabled)}

    def stop(self, reason: str = "HTTP stop requested") -> dict:
        """停止设备当前动作。

        Args:
            reason: 原因，作为停止设备当前动作所需的输入。 类型为 str。

        Returns:
            本次处理产生的结果值。 返回类型为 dict。
        """
        request = StopGripper.Request()
        request.reason = reason
        response = self._call("stop", request)
        return {"success": response.success, "stop_sent": response.stop_sent,
                "hardware_stop_confirmed": response.hardware_stop_confirmed, "error": response.error}

    def start_operation(self, lease_id: str, command: str, angle_degrees: float = 0.0) -> dict:
        """启动操作。

        Args:
            lease_id: 租约标识符，作为启动操作所需的输入。 类型为 str。
            command: command，作为启动操作所需的输入。 类型为 str。
            angle_degrees: 角度degrees，作为启动操作所需的输入。 类型为 float。

        Returns:
            本次处理产生的结果值。 返回类型为 dict。
        """
        if not self.action_client.wait_for_server(timeout_sec=2.0):
            raise TimeoutError(f"ROS action server {self.prefix}/move is unavailable")
        operation_id = str(uuid.uuid4())
        goal = MoveGripper.Goal()
        goal.lease_id = lease_id
        goal.client_operation_id = operation_id
        goal.command = command
        goal.angle_degrees = float(angle_degrees)
        with self._operation_lock:
            if len(self._operations) >= 128:
                completed = sorted(
                    (item for item in self._operations.values()
                     if item.get("state") in {"succeeded", "failed", "cancelled"}),
                    key=lambda item: item.get("created_at", 0.0),
                )
                for old in completed[:max(1, len(self._operations) - 96)]:
                    self._operations.pop(old["operation_id"], None)
                    self._goal_handles.pop(old["operation_id"], None)
            self._operations[operation_id] = {
                "operation_id": operation_id, "state": "submitting", "success": None,
                "error": "", "stop_sent": False, "hardware_stop_confirmed": False,
                "created_at": time.time(), "cancel_on_accept": False,
            }
        send_future = self.action_client.send_goal_async(
            goal, feedback_callback=lambda msg: self._on_feedback(operation_id, msg)
        )
        send_future.add_done_callback(lambda future: self._on_goal_response(operation_id, future))
        return {"operation_id": operation_id, "state": "submitting"}

    def _on_feedback(self, operation_id: str, wrapped: Any) -> None:
        """处理反馈事件。

        Args:
            operation_id: 操作标识符，作为处理反馈事件所需的输入。 类型为 str。
            wrapped: wrapped，作为处理反馈事件所需的输入。 类型为 Any。

        Returns:
            无返回值。
        """
        message = wrapped.feedback
        with self._operation_lock:
            operation = self._operations.get(operation_id)
            if operation:
                operation["state"] = message.state or "running"
                operation["feedback_age_sec"] = message.feedback_age_sec if message.feedback_age_sec >= 0 else None

    def _on_goal_response(self, operation_id: str, future: Any) -> None:
        """处理goal响应事件。

        Args:
            operation_id: 操作标识符，作为处理goal响应事件所需的输入。 类型为 str。
            future: 异步服务调用返回的 Future 对象。 类型为 Any。

        Returns:
            无返回值。
        """
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                with self._operation_lock:
                    self._operations[operation_id].update(state="failed", success=False, error="motion goal rejected")
                return
            with self._operation_lock:
                self._goal_handles[operation_id] = goal_handle
                self._operations[operation_id]["state"] = "accepted"
                cancel_on_accept = self._operations[operation_id]["cancel_on_accept"]
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(lambda result: self._on_result(operation_id, result))
            if cancel_on_accept:
                goal_handle.cancel_goal_async()
        except Exception as exc:
            with self._operation_lock:
                self._operations[operation_id].update(state="failed", success=False, error=str(exc))

    def _on_result(self, operation_id: str, future: Any) -> None:
        """处理结果事件。

        Args:
            operation_id: 操作标识符，作为处理结果事件所需的输入。 类型为 str。
            future: 异步服务调用返回的 Future 对象。 类型为 Any。

        Returns:
            无返回值。
        """
        try:
            wrapped = future.result()
            result = wrapped.result
            success = bool(result.success)
            state = "succeeded" if success else ("cancelled" if wrapped.status == GoalStatus.STATUS_CANCELED else "failed")
            with self._operation_lock:
                self._operations[operation_id].update(
                    state=state, success=success, error=result.error,
                    stop_sent=result.stop_sent, hardware_stop_confirmed=result.hardware_stop_confirmed,
                )
        except Exception as exc:
            with self._operation_lock:
                self._operations[operation_id].update(state="failed", success=False, error=str(exc))

    def operation(self, operation_id: str) -> dict:
        """操作当前设备或操作。

        Args:
            operation_id: 操作标识符，作为操作当前设备或操作所需的输入。 类型为 str。

        Returns:
            本次处理产生的结果值。 返回类型为 dict。
        """
        with self._operation_lock:
            operation = self._operations.get(operation_id)
            if operation is None:
                raise KeyError(operation_id)
            return dict(operation)

    def cancel_operation(self, operation_id: str) -> dict:
        """取消操作。

        Args:
            operation_id: 操作标识符，作为取消操作所需的输入。 类型为 str。

        Returns:
            本次处理产生的结果值。 返回类型为 dict。
        """
        with self._operation_lock:
            goal_handle = self._goal_handles.get(operation_id)
            operation = self._operations.get(operation_id)
        if operation is None:
            raise KeyError(operation_id)
        if goal_handle is None:
            if operation["state"] != "submitting":
                return {"cancel_requested": False, "state": operation["state"]}
            with self._operation_lock:
                self._operations[operation_id]["cancel_on_accept"] = True
                self._operations[operation_id]["state"] = "cancelling"
            return {"cancel_requested": True, "state": "cancelling"}
        event = threading.Event()
        future = goal_handle.cancel_goal_async()
        future.add_done_callback(lambda _future: event.set())
        if not event.wait(5.0):
            raise TimeoutError("timed out requesting gripper operation cancellation")
        with self._operation_lock:
            self._operations[operation_id]["state"] = "cancelling"
        return {"cancel_requested": True, "state": "cancelling"}
