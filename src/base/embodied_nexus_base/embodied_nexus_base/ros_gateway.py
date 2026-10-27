from __future__ import annotations

import json
import os
import threading
import time
import uuid
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from typing import Any

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

from embodied_nexus_base_interfaces.action import MoveBaseDistance, TurnBaseRelative
from embodied_nexus_base_interfaces.srv import (
    AcquireBaseControl, ApplyBaseConfiguration, GetBaseConfiguration, GetBaseInfo,
    GetBaseState, ReleaseBaseControl, RenewBaseControl, SetBaseMotionEnabled,
    SetBaseVelocity, StopBase, ValidateBaseConfiguration,
)

from .base_node import default_config_path
from .config import load_config
from .web_server import BaseWebServer


class BaseRosGateway(Node):
    """Authenticated HTTP/WebSocket facade; all device access stays in BaseNode."""

    def __init__(self, config_path: str | None = None) -> None:
        """初始化对象并保存构造参数或依赖。

        Args:
            config_path: 配置文件路径，指向要加载的设备配置。 类型为 str | None。

        Returns:
            无返回值。
        """
        super().__init__("embodied_nexus_base_web")
        self.declare_parameter("config_file", config_path or default_config_path())
        self.declare_parameter("web.bind_host", "")
        self.declare_parameter("web.port", 0)
        self.declare_parameter("web.auth_token_env", "")
        self.config = load_config(str(self.get_parameter("config_file").value))
        for key in ("bind_host", "port", "auth_token_env"):
            value = self.get_parameter(f"web.{key}").value
            if value:
                self.config["web"][key] = value
        self.device_id = self.config["device_id"]
        self.prefix = f"/embodied_nexus/bases/{self.device_id}"
        self._service_types = {
            "info": (GetBaseInfo, f"{self.prefix}/info"),
            "state": (GetBaseState, f"{self.prefix}/get_state"),
            "config": (GetBaseConfiguration, f"{self.prefix}/config/get"),
            "validate": (ValidateBaseConfiguration, f"{self.prefix}/config/validate"),
            "apply": (ApplyBaseConfiguration, f"{self.prefix}/config/apply"),
            "acquire": (AcquireBaseControl, f"{self.prefix}/control/acquire"),
            "renew": (RenewBaseControl, f"{self.prefix}/control/renew"),
            "release": (ReleaseBaseControl, f"{self.prefix}/control/release"),
            "enable": (SetBaseMotionEnabled, f"{self.prefix}/motion/enable"),
            "velocity": (SetBaseVelocity, f"{self.prefix}/velocity"),
            "stop": (StopBase, f"{self.prefix}/stop"),
        }
        self._service_clients = {key: self.create_client(kind, name)
                                 for key, (kind, name) in self._service_types.items()}
        self.actions = {
            "distance": ActionClient(self, MoveBaseDistance, f"{self.prefix}/move_relative"),
            "turn": ActionClient(self, TurnBaseRelative, f"{self.prefix}/turn_relative"),
        }
        self._operations: dict[str, dict[str, Any]] = {}
        self._operations_lock = threading.RLock()
        self._workers = ThreadPoolExecutor(max_workers=4, thread_name_prefix="base-http-operation")
        self._state_lock = threading.Lock()
        self._cached_state: dict | None = None
        self._cached_state_at = 0.0
        self.create_subscription(String, f"{self.prefix}/_internal/state_json", self._on_state, 10)
        web = self.config["web"]
        token = os.environ.get(str(web["auth_token_env"]), "")
        self.web = BaseWebServer(self, str(web["bind_host"]), int(web["port"]), token)
        self.web.start()
        self.get_logger().info(f"base Web API listening on {web['bind_host']}:{web['port']}")

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
        service_name = self._service_types[name][1]
        if not client.wait_for_service(timeout_sec=timeout):
            raise TimeoutError(f"ROS service {service_name} is unavailable")
        done = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _future: done.set())
        if not done.wait(timeout):
            raise TimeoutError(f"ROS service {service_name} timed out")
        response = future.result()
        if response is None:
            raise RuntimeError(f"ROS service {service_name} returned no response")
        return response

    @staticmethod
    def _require(response, success: bool, error: str = ""):
        """检查当前设备或操作。

        Args:
            response: 服务端或被调用操作返回的响应对象。
            success: success，作为检查当前设备或操作所需的输入。 类型为 bool。
            error: error，作为检查当前设备或操作所需的输入。 类型为 str。

        Returns:
            本次处理产生的结果值。
        """
        if not success:
            if "lease" in error.lower() or "held" in error.lower():
                raise PermissionError(error)
            if "timeout" in error.lower():
                raise TimeoutError(error)
            if "invalid" in error.lower() or "must " in error.lower() or "exceed" in error.lower():
                raise ValueError(error)
            raise RuntimeError(error or "base request failed")
        return response

    def info(self) -> dict:
        """获取设备能力与静态信息。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。 返回类型为 dict。
        """
        response = self._call("info", GetBaseInfo.Request())
        self._require(response, response.success, response.error)
        return json.loads(response.descriptor_json)

    def status(self) -> dict:
        """获取服务当前运行状态。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。 返回类型为 dict。
        """
        with self._state_lock:
            cached = dict(self._cached_state) if self._cached_state is not None else None
            age = time.monotonic() - self._cached_state_at
        if cached is not None:
            cached["gateway_state_age_sec"] = max(age, 0.0)
            if age > 0.5:
                cached.update({"connected": False, "motion_enabled": False,
                               "system_feedback_valid": False, "motion_feedback_valid": False,
                               "last_error": "base state stream is stale"})
            return cached
        response = self._call("state", GetBaseState.Request())
        self._require(response, response.success, response.error)
        return json.loads(response.state_json)

    def _on_state(self, message: String) -> None:
        """处理状态事件。

        Args:
            message: 待处理的消息对象。 类型为 String。

        Returns:
            无返回值。
        """
        try:
            state = json.loads(message.data)
        except (TypeError, ValueError):
            return
        with self._state_lock:
            self._cached_state = state
            self._cached_state_at = time.monotonic()

    def get_config(self) -> dict:
        """获取配置。

        Args:
            无参数。

        Returns:
            查询或读取到的数据；未找到时的处理方式由实现决定。 返回类型为 dict。
        """
        response = self._call("config", GetBaseConfiguration.Request())
        self._require(response, response.success, response.error)
        return json.loads(response.config_json)

    def validate_config(self, patch: dict) -> dict:
        """验证配置。

        Args:
            patch: patch，作为验证配置所需的输入。 类型为 dict。

        Returns:
            布尔值，表示验证配置是否成功或成立。 返回类型为 dict。
        """
        request = ValidateBaseConfiguration.Request()
        request.patch_json = json.dumps(patch, allow_nan=False)
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
        request = ApplyBaseConfiguration.Request()
        request.lease_id = lease_id
        request.patch_json = json.dumps(patch, allow_nan=False)
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
        request = AcquireBaseControl.Request()
        request.owner = owner
        request.ttl_sec = float(ttl_sec or 0.0)
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
        request = RenewBaseControl.Request()
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
        request = ReleaseBaseControl.Request()
        request.lease_id = lease_id
        response = self._call("release", request)
        self._require(response, response.success, response.error)
        return {"released": True, "stop_sent": response.stop_sent,
                "zero_velocity_written": response.zero_velocity_written,
                "error": response.error}

    def enable(self, lease_id: str, enabled: bool) -> dict:
        """更新设备运动使能状态。

        Args:
            lease_id: 租约标识符，作为更新设备运动使能状态所需的输入。 类型为 str。
            enabled: 表示对应功能是否启用的布尔值。 类型为 bool。

        Returns:
            本次处理产生的结果值。 返回类型为 dict。
        """
        request = SetBaseMotionEnabled.Request()
        request.lease_id, request.enabled = lease_id, bool(enabled)
        response = self._call("enable", request)
        self._require(response, response.success, response.error)
        return {"enabled": response.enabled,
                "control_mode_confirmed": response.control_mode_confirmed,
                "stop_requested": response.stop_requested,
                "zero_velocity_written": response.zero_velocity_written,
                "error": response.error}

    def set_velocity(self, lease_id: str, linear_mps: float, angular_rps: float) -> dict:
        """设置速度。

        Args:
            lease_id: 租约标识符，作为设置速度所需的输入。 类型为 str。
            linear_mps: 线速度mps，作为设置速度所需的输入。 类型为 float。
            angular_rps: 角速度rps，作为设置速度所需的输入。 类型为 float。

        Returns:
            本次处理产生的结果值。 返回类型为 dict。
        """
        request = SetBaseVelocity.Request()
        request.lease_id = lease_id
        request.linear_mps = float(linear_mps)
        request.angular_rps = float(angular_rps)
        response = self._call("velocity", request)
        self._require(response, response.success, response.error)
        return {"accepted": True}

    def stop(self, reason: str) -> dict:
        """停止设备当前动作。

        Args:
            reason: 原因，作为停止设备当前动作所需的输入。 类型为 str。

        Returns:
            本次处理产生的结果值。 返回类型为 dict。
        """
        request = StopBase.Request()
        request.reason = str(reason)
        response = self._call("stop", request)
        return {"stop_requested": response.stop_requested,
                "zero_velocity_written": response.zero_velocity_written,
                "stationary_confirmed": response.stationary_confirmed,
                "hardware_estop": response.hardware_estop,
                "error": response.error}

    def create_operation(self, values: dict[str, Any]) -> dict:
        """创建操作。

        Args:
            values: 要应用、比较或转换的一组输入值。 类型为 dict[str, Any]。

        Returns:
            构造或创建完成的对象或资源。 返回类型为 dict。
        """
        kind = values.get("type")
        if kind not in self.actions:
            raise ValueError("operation type must be distance or turn")
        operation_id = uuid.uuid4().hex
        record = {"operation_id": operation_id, "type": kind, "state": "submitting",
                  "success": None, "message": "", "progress": 0.0,
                  "created_at": time.time(), "goal_handle": None, "cancel_requested": False}
        with self._operations_lock:
            self._operations[operation_id] = record
        self._workers.submit(self._run_operation, operation_id, dict(values))
        return {"operation_id": operation_id, "state": "submitting"}

    def _run_operation(self, operation_id: str, values: dict[str, Any]) -> None:
        """运行操作。

        Args:
            operation_id: 操作标识符，作为运行操作所需的输入。 类型为 str。
            values: 要应用、比较或转换的一组输入值。 类型为 dict[str, Any]。

        Returns:
            无返回值。
        """
        kind = values["type"]
        client = self.actions[kind]
        try:
            with self._operations_lock:
                if self._operations.get(operation_id, {}).get("cancel_requested"):
                    self._operations[operation_id]["state"] = "cancelled"
                    return
            if not client.wait_for_server(timeout_sec=5.0):
                raise TimeoutError(f"ROS action {kind} is unavailable")
            with self._operations_lock:
                if self._operations.get(operation_id, {}).get("cancel_requested"):
                    self._operations[operation_id]["state"] = "cancelled"
                    return
            if kind == "distance":
                goal = MoveBaseDistance.Goal()
                goal.lease_id = str(values.get("lease_id", ""))
                goal.distance_m = float(values.get("distance_m", 0.0))
                goal.speed_mps = float(values.get("speed_mps", 0.0))
            else:
                goal = TurnBaseRelative.Goal()
                goal.lease_id = str(values.get("lease_id", ""))
                goal.angle_rad = float(values.get("angle_rad", 0.0))
                goal.speed_rps = float(values.get("speed_rps", 0.0))
            sent = client.send_goal_async(goal, feedback_callback=
                                          lambda update: self._operation_feedback(operation_id, update))
            goal_handle = self._wait_future(sent, 6.0)
            if not goal_handle.accepted:
                raise RuntimeError("base rejected the relative operation goal")
            with self._operations_lock:
                record = self._operations.get(operation_id)
                if record:
                    record["goal_handle"] = goal_handle
                    record["state"] = "running"
                    if record["cancel_requested"]:
                        record["state"] = "cancelling"
                        goal_handle.cancel_goal_async()
            try:
                wrapped = self._wait_future(goal_handle.get_result_async(), 45.0)
            except Exception:
                goal_handle.cancel_goal_async()
                try:
                    self.stop("HTTP operation monitor failed")
                except Exception:
                    pass
                raise
            outcome = wrapped.result
            terminal = "succeeded" if wrapped.status == GoalStatus.STATUS_SUCCEEDED and outcome.success else (
                "cancelled" if wrapped.status == GoalStatus.STATUS_CANCELED else "failed")
            with self._operations_lock:
                record = self._operations.get(operation_id)
                if record:
                    record.update({"state": terminal, "success": bool(outcome.success),
                                   "message": outcome.message,
                                   "progress": outcome.progress_m if kind == "distance" else outcome.progress_rad,
                                   "stop_requested": outcome.stop_requested,
                                   "zero_velocity_written": outcome.zero_velocity_written,
                                   "goal_handle": None})
        except Exception as exc:
            with self._operations_lock:
                record = self._operations.get(operation_id)
                if record:
                    record.update({"state": "failed", "success": False,
                                   "message": str(exc), "goal_handle": None})

    @staticmethod
    def _wait_future(future, timeout: float):
        """等待异步服务调用完成或超时。

        Args:
            future: 异步服务调用返回的 Future 对象。
            timeout: 本次网络或服务调用的最长等待时长，通常以秒为单位。 类型为 float。

        Returns:
            本次处理产生的结果值。
        """
        done = threading.Event()
        future.add_done_callback(lambda _future: done.set())
        if not done.wait(timeout):
            raise TimeoutError("ROS action request timed out")
        result = future.result()
        if result is None:
            raise RuntimeError("ROS action returned no result")
        return result

    def _operation_feedback(self, operation_id: str, wrapped) -> None:
        """操作反馈。

        Args:
            operation_id: 操作标识符，作为操作反馈所需的输入。 类型为 str。
            wrapped: wrapped，作为操作反馈所需的输入。

        Returns:
            无返回值。
        """
        feedback = wrapped.feedback
        value = getattr(feedback, "progress_m", getattr(feedback, "progress_rad", 0.0))
        with self._operations_lock:
            record = self._operations.get(operation_id)
            if record and record["state"] not in {"cancelled", "failed", "succeeded"}:
                record["progress"] = float(value)
                record["state"] = "running"

    def get_operation(self, operation_id: str) -> dict:
        """获取操作。

        Args:
            operation_id: 操作标识符，作为获取操作所需的输入。 类型为 str。

        Returns:
            查询或读取到的数据；未找到时的处理方式由实现决定。 返回类型为 dict。
        """
        with self._operations_lock:
            record = self._operations.get(operation_id)
            if record is None:
                raise KeyError(f"unknown base operation {operation_id}")
            return {key: value for key, value in record.items() if key != "goal_handle"}

    def cancel_operation(self, operation_id: str) -> dict:
        """取消操作。

        Args:
            operation_id: 操作标识符，作为取消操作所需的输入。 类型为 str。

        Returns:
            本次处理产生的结果值。 返回类型为 dict。
        """
        with self._operations_lock:
            record = self._operations.get(operation_id)
            if record is None:
                raise KeyError(f"unknown base operation {operation_id}")
            handle = record.get("goal_handle")
            if handle is None:
                if record["state"] == "submitting":
                    record["cancel_requested"] = True
                    record["state"] = "cancelled"
                    record["message"] = "cancelled before ROS goal submission"
                elif record["state"] in {"succeeded", "failed", "cancelled"}:
                    return self.get_operation(operation_id)
            else:
                record["state"] = "cancelling"
                handle.cancel_goal_async()
            return self.get_operation(operation_id)

    def close(self) -> None:
        """关闭连接并释放相关资源。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        self.web.close()
        self._workers.shutdown(wait=False, cancel_futures=True)
        super().destroy_node()


def main(args=None):
    """解析启动参数并运行程序入口。

    Args:
        args: 额外的位置参数集合。

    Returns:
        无返回值。
    """
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = BaseRosGateway()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown(timeout_sec=2.0)
        node.close()
        if rclpy.ok():
            rclpy.shutdown()
