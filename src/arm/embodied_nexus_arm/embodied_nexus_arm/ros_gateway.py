from __future__ import annotations

import json
import os
import threading
import time
import uuid
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
from typing import Any

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

from embodied_nexus_arm_interfaces.action import MoveArmJoints
from embodied_nexus_arm_interfaces.srv import (
    AcquireArmControl, ApplyArmConfiguration, GetArmConfiguration, GetArmInfo,
    GetArmState, ReleaseArmControl, RenewArmControl, SetArmJointVelocity,
    SetArmLifecycle, SetArmMotionEnabled, SetArmTeachMode, StopArm,
    ValidateArmConfiguration,
)

from .config import load_config
from .web_server import ArmWebServer


class ArmRosGateway(Node):
    def __init__(self, config_path: str | None = None) -> None:
        """初始化对象并保存构造参数或依赖。

        Args:
            config_path: 配置文件路径，指向要加载的设备配置。 类型为 str | None。

        Returns:
            无返回值。
        """
        super().__init__("embodied_nexus_arm_web_gateway")
        self.declare_parameter("config_file", config_path or str(Path(__file__).resolve().parents[1] / "config" / "aubo_i5_mock.yaml"))
        self.declare_parameter("web.bind_host", "")
        self.declare_parameter("web.port", 0)
        self.declare_parameter("web.auth_token_env", "")
        self.config = load_config(str(self.get_parameter("config_file").value))
        for key in ("bind_host", "port", "auth_token_env"):
            value = self.get_parameter(f"web.{key}").value
            if value:
                self.config["web"][key] = value
        self.device_id = self.config["device_id"]
        self.prefix = f"/embodied_nexus/arms/{self.device_id}"
        self._operations: dict[str, dict[str, Any]] = {}
        self._lock = threading.RLock()
        self._workers = ThreadPoolExecutor(max_workers=4, thread_name_prefix="arm-web-operation")
        self._state: dict[str, Any] = {}
        self.executor = None
        self.service_clients = {
            "info": self.create_client(GetArmInfo, f"{self.prefix}/info"),
            "state": self.create_client(GetArmState, f"{self.prefix}/get_state"),
            "config": self.create_client(GetArmConfiguration, f"{self.prefix}/config/get"),
            "validate": self.create_client(ValidateArmConfiguration, f"{self.prefix}/config/validate"),
            "apply": self.create_client(ApplyArmConfiguration, f"{self.prefix}/config/apply"),
            "acquire": self.create_client(AcquireArmControl, f"{self.prefix}/control/acquire"),
            "renew": self.create_client(RenewArmControl, f"{self.prefix}/control/renew"),
            "release": self.create_client(ReleaseArmControl, f"{self.prefix}/control/release"),
            "enable": self.create_client(SetArmMotionEnabled, f"{self.prefix}/motion/enable"),
            "velocity": self.create_client(SetArmJointVelocity, f"{self.prefix}/joint_velocity"),
            "lifecycle": self.create_client(SetArmLifecycle, f"{self.prefix}/lifecycle"),
            "teach": self.create_client(SetArmTeachMode, f"{self.prefix}/teach_mode"),
            "stop": self.create_client(StopArm, f"{self.prefix}/stop"),
        }
        self.action = ActionClient(self, MoveArmJoints, f"{self.prefix}/move_joints")
        self.create_subscription(String, f"{self.prefix}/_internal/state_json", self._on_state, 10)
        web = self.config["web"]
        token = os.environ.get(web["auth_token_env"], "")
        self.web = ArmWebServer(self, web["bind_host"], int(web["port"]), token)
        self.web.start()

    def _on_state(self, message: String) -> None:
        """处理状态事件。

        Args:
            message: 待处理的消息对象。 类型为 String。

        Returns:
            无返回值。
        """
        try:
            with self._lock:
                self._state = json.loads(message.data)
        except json.JSONDecodeError:
            return

    def _call(self, name: str, request: Any, timeout: float = 5.0):
        """调用 ROS 服务并等待调用结果。

        Args:
            name: 名称，作为调用 ROS 服务并等待调用结果所需的输入。 类型为 str。
            request: 封装调用输入字段的请求对象。 类型为 Any。
            timeout: 本次网络或服务调用的最长等待时长，通常以秒为单位。 类型为 float。

        Returns:
            本次处理产生的结果值。
        """
        client = self.service_clients[name]
        if not client.wait_for_service(timeout_sec=min(timeout, 3.0)):
            raise TimeoutError(f"arm ROS service {name} is unavailable")
        future = client.call_async(request)
        response = self._wait_future(future, timeout)
        if not response.success:
            exc = PermissionError if "lease" in response.error.lower() else RuntimeError
            raise exc(response.error)
        return response

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
            raise TimeoutError("arm ROS request timed out")
        result = future.result()
        if result is None:
            raise RuntimeError("arm ROS request returned no result")
        return result

    def info(self):
        """获取设备能力与静态信息。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。
        """
        return json.loads(self._call("info", GetArmInfo.Request()).descriptor_json)

    def status(self):
        """获取服务当前运行状态。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。
        """
        with self._lock:
            state = dict(self._state)
        if not state:
            response = self._call("state", GetArmState.Request())
            state = json.loads(response.state_json)
        return state

    def get_config(self):
        """获取配置。

        Args:
            无参数。

        Returns:
            查询或读取到的数据；未找到时的处理方式由实现决定。
        """
        return json.loads(self._call("config", GetArmConfiguration.Request()).config_json)

    def validate_config(self, patch):
        """验证配置。

        Args:
            patch: patch，作为验证配置所需的输入。

        Returns:
            布尔值，表示验证配置是否成功或成立。
        """
        req = ValidateArmConfiguration.Request()
        req.patch_json = json.dumps(patch, allow_nan=False)
        response = self.service_clients["validate"].call_async(req)
        result = self._wait_future(response, 5.0)
        if not result.valid:
            raise ValueError(result.error)
        return json.loads(result.config_json)

    def apply_config(self, lease_id, patch):
        """应用配置。

        Args:
            lease_id: 租约标识符，作为应用配置所需的输入。
            patch: patch，作为应用配置所需的输入。

        Returns:
            本次处理产生的结果值。
        """
        req = ApplyArmConfiguration.Request()
        req.lease_id, req.patch_json = str(lease_id), json.dumps(patch, allow_nan=False)
        return json.loads(self._call("apply", req).config_json)

    def acquire(self, owner, ttl_sec=None):
        """申请设备控制权。

        Args:
            owner: owner，作为申请设备控制权所需的输入。
            ttl_sec: ttlsec，作为申请设备控制权所需的输入。

        Returns:
            本次处理产生的结果值。
        """
        req = AcquireArmControl.Request()
        req.owner, req.ttl_sec = str(owner), float(ttl_sec or 0.0)
        result = self._call("acquire", req)
        return {"lease_id": result.lease_id, "expires_in_sec": result.expires_in_sec}

    def renew(self, lease_id):
        """续租当前设备控制权。

        Args:
            lease_id: 租约标识符，作为续租当前设备控制权所需的输入。

        Returns:
            本次处理产生的结果值。
        """
        req = RenewArmControl.Request()
        req.lease_id = str(lease_id)
        return {"expires_in_sec": self._call("renew", req).expires_in_sec}

    def release(self, lease_id):
        """释放当前设备控制租约。

        Args:
            lease_id: 租约标识符，作为释放当前设备控制租约所需的输入。

        Returns:
            本次处理产生的结果值。
        """
        req = ReleaseArmControl.Request()
        req.lease_id = str(lease_id)
        result = self._call("release", req)
        return {"released": True, "stop_requested": result.stop_requested,
                "controller_accepted": result.controller_accepted,
                "stationary_confirmed": result.stationary_confirmed}

    def enable(self, lease_id, enabled):
        """更新设备运动使能状态。

        Args:
            lease_id: 租约标识符，作为更新设备运动使能状态所需的输入。
            enabled: 表示对应功能是否启用的布尔值。

        Returns:
            本次处理产生的结果值。
        """
        req = SetArmMotionEnabled.Request()
        req.lease_id, req.enabled = str(lease_id), bool(enabled)
        result = self._call("enable", req)
        return {"enabled": result.enabled}

    def set_velocity(self, lease_id, velocity):
        """设置速度。

        Args:
            lease_id: 租约标识符，作为设置速度所需的输入。
            velocity: 速度，作为设置速度所需的输入。

        Returns:
            本次处理产生的结果值。
        """
        if not isinstance(velocity, (tuple, list)) or len(velocity) != len(self.config["joints"]["names"]):
            raise ValueError("velocity_rad_sec must match configured joint count")
        req = SetArmJointVelocity.Request()
        req.lease_id = str(lease_id)
        req.velocity_rad_sec = [float(v) for v in velocity]
        result = self._call("velocity", req)
        return {"controller_accepted": result.controller_accepted,
                "retry_next_cycle": result.retry_next_cycle,
                "commanded_positions_rad": list(result.commanded_positions_rad),
                "stationary_confirmed": result.stationary_confirmed}

    def lifecycle(self, lease_id, command):
        """生命周期当前设备或操作。

        Args:
            lease_id: 租约标识符，作为生命周期当前设备或操作所需的输入。
            command: command，作为生命周期当前设备或操作所需的输入。

        Returns:
            本次处理产生的结果值。
        """
        req = SetArmLifecycle.Request()
        req.lease_id, req.command = str(lease_id), str(command)
        result = self._call("lifecycle", req, timeout=60.0)
        return {"operation_state": result.operation_state}

    def teach(self, lease_id, enabled):
        """示教当前设备或操作。

        Args:
            lease_id: 租约标识符，作为示教当前设备或操作所需的输入。
            enabled: 表示对应功能是否启用的布尔值。

        Returns:
            本次处理产生的结果值。
        """
        req = SetArmTeachMode.Request()
        req.lease_id, req.enabled = str(lease_id), bool(enabled)
        result = self._call("teach", req, timeout=6.0)
        return {"teach_state": result.teach_state}

    def stop(self, reason):
        """停止设备当前动作。

        Args:
            reason: 原因，作为停止设备当前动作所需的输入。

        Returns:
            本次处理产生的结果值。
        """
        req = StopArm.Request()
        req.reason = str(reason)
        result = self._call("stop", req, timeout=4.0)
        return {"stop_requested": result.stop_requested,
                "controller_accepted": result.controller_accepted,
                "stationary_confirmed": result.stationary_confirmed,
                "hardware_estop": result.hardware_estop}

    def create_operation(self, values: dict[str, Any]):
        """创建操作。

        Args:
            values: 要应用、比较或转换的一组输入值。 类型为 dict[str, Any]。

        Returns:
            构造或创建完成的对象或资源。
        """
        if values.get("type") != "move_joints":
            raise ValueError("operation type must be move_joints")
        target = values.get("positions_rad")
        if not isinstance(target, list) or len(target) != len(self.config["joints"]["names"]):
            raise ValueError("positions_rad must match configured joint count")
        operation_id = uuid.uuid4().hex
        record = {"operation_id": operation_id, "type": "move_joints", "state": "submitting",
                  "success": None, "message": "", "progress": None,
                  "created_at": time.time(), "goal_handle": None, "cancel_requested": False}
        with self._lock:
            self._operations[operation_id] = record
        self._workers.submit(self._run_operation, operation_id, dict(values))
        return {"operation_id": operation_id, "state": "submitting"}

    def _run_operation(self, operation_id, values):
        """运行操作。

        Args:
            operation_id: 操作标识符，作为运行操作所需的输入。
            values: 要应用、比较或转换的一组输入值。

        Returns:
            无返回值。
        """
        try:
            if not self.action.wait_for_server(timeout_sec=5.0):
                raise TimeoutError("arm move_joints action is unavailable")
            with self._lock:
                record = self._operations[operation_id]
                if record["cancel_requested"]:
                    record["state"] = "cancelled"
                    return
            goal = MoveArmJoints.Goal()
            goal.lease_id = str(values.get("lease_id", ""))
            goal.positions_rad = [float(v) for v in values["positions_rad"]]
            goal.speed_rad_sec = float(values.get("speed_rad_sec", 0.0))
            goal.acceleration_rad_sec2 = float(values.get("acceleration_rad_sec2", 0.0))
            goal.timeout_sec = float(values.get("timeout_sec", 0.0))
            sent = self.action.send_goal_async(goal, feedback_callback=lambda update: self._feedback(operation_id, update))
            handle = self._wait_future(sent, 6.0)
            if not handle.accepted:
                raise RuntimeError("arm rejected the joint target")
            with self._lock:
                record = self._operations[operation_id]
                record["goal_handle"] = handle
                record["state"] = "running"
                if record["cancel_requested"]:
                    record["state"] = "cancelling"
                    handle.cancel_goal_async()
            wrapped = self._wait_future(handle.get_result_async(), float(goal.timeout_sec or 30.0) + 10.0)
            outcome = wrapped.result
            terminal = "succeeded" if wrapped.status == GoalStatus.STATUS_SUCCEEDED and outcome.success else (
                "cancelled" if wrapped.status == GoalStatus.STATUS_CANCELED else "failed")
            with self._lock:
                record = self._operations[operation_id]
                record.update({"state": terminal, "success": bool(outcome.success),
                               "message": outcome.message, "progress": outcome.max_error_rad,
                               "controller_accepted": outcome.controller_accepted,
                               "stop_requested": outcome.stop_requested,
                               "stationary_confirmed": outcome.stationary_confirmed,
                               "hardware_estop": outcome.hardware_estop, "goal_handle": None})
        except Exception as exc:
            with self._lock:
                record = self._operations.get(operation_id)
                if record:
                    record.update({"state": "failed", "success": False,
                                   "message": str(exc), "goal_handle": None})

    def _feedback(self, operation_id, wrapped):
        """反馈当前设备或操作。

        Args:
            operation_id: 操作标识符，作为反馈当前设备或操作所需的输入。
            wrapped: wrapped，作为反馈当前设备或操作所需的输入。

        Returns:
            无返回值。
        """
        with self._lock:
            record = self._operations.get(operation_id)
            if record and record["state"] not in {"succeeded", "failed", "cancelled"}:
                record.update({"state": "running", "progress": wrapped.feedback.max_error_rad})

    def get_operation(self, operation_id):
        """获取操作。

        Args:
            operation_id: 操作标识符，作为获取操作所需的输入。

        Returns:
            查询或读取到的数据；未找到时的处理方式由实现决定。
        """
        with self._lock:
            record = self._operations.get(operation_id)
            if record is None:
                raise KeyError(f"unknown arm operation {operation_id}")
            return {key: value for key, value in record.items() if key != "goal_handle"}

    def cancel_operation(self, operation_id):
        """取消操作。

        Args:
            operation_id: 操作标识符，作为取消操作所需的输入。

        Returns:
            本次处理产生的结果值。
        """
        with self._lock:
            record = self._operations.get(operation_id)
            if record is None:
                raise KeyError(f"unknown arm operation {operation_id}")
            handle = record.get("goal_handle")
            if handle is None:
                if record["state"] == "submitting":
                    record["cancel_requested"] = True
                    record["state"] = "cancelled"
                return self.get_operation(operation_id)
            record["state"] = "cancelling"
            handle.cancel_goal_async()
            return self.get_operation(operation_id)

    def close(self):
        """关闭连接并释放相关资源。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        self.web.close()
        with self._lock:
            handles = [record.get("goal_handle") for record in self._operations.values()
                       if record.get("state") in {"submitting", "accepted", "running", "cancelling"}]
        for handle in handles:
            if handle is not None:
                try:
                    handle.cancel_goal_async()
                except Exception as exc:
                    self.get_logger().warning(f"Could not cancel arm operation during gateway shutdown: {exc}")
        try:
            client = self.service_clients["stop"]
            if client.wait_for_service(timeout_sec=1.0):
                request = StopArm.Request()
                request.reason = "arm gateway exiting"
                future = client.call_async(request)
                if self.executor is not None:
                    self.executor.spin_until_future_complete(future, timeout_sec=4.0)
                if not future.done():
                    self.get_logger().error("Arm stop request timed out during gateway shutdown")
                else:
                    result = future.result()
                    if result is None or not result.controller_accepted:
                        error = getattr(result, "error", "no stop response")
                        self.get_logger().error(f"Arm controller did not confirm stop during gateway shutdown: {error}")
            else:
                self.get_logger().error("Arm stop service unavailable during gateway shutdown")
        except Exception as exc:
            self.get_logger().error(f"Arm stop request failed during gateway shutdown: {exc}")
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
    node = ArmRosGateway()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    node.executor = executor
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        executor.shutdown(timeout_sec=3.0)
        if rclpy.ok():
            rclpy.shutdown()
