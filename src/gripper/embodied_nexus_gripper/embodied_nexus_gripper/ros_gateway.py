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
    try:
        from ament_index_python.packages import get_package_share_directory
        return str(Path(get_package_share_directory("embodied_nexus_gripper")) / "config" / "ms42dc.yaml")
    except Exception:
        return str(Path(__file__).resolve().parents[1] / "config" / "ms42dc.yaml")


class GripperRosGateway(Node):
    """ROS client facade for the HTTP gateway; it never opens hardware."""

    def __init__(self, device_id: str | None = None, config_path: str | None = None) -> None:
        overrides = [Parameter("config_file", value=config_path)] if config_path else None
        super().__init__("embodied_nexus_gripper_web", parameter_overrides=overrides)
        self.declare_parameter("config_file", _default_config_path())
        self.declare_parameter("interface_mode", "")
        self.declare_parameter("web.bind_host", "")
        self.declare_parameter("web.port", 0)
        self.declare_parameter("web.auth_token_env", "")
        self.config = load_config(str(self.get_parameter("config_file").value))
        mode = str(self.get_parameter("interface_mode").value).strip()
        if mode:
            self.config["interface_mode"] = mode
        for key in ("bind_host", "port", "auth_token_env"):
            value = self.get_parameter(f"web.{key}").value
            if value:
                self.config["web"][key] = value
        self.device_id = device_id or self.config["device_id"]
        self.prefix = f"/embodied_nexus/grippers/{self.device_id}"
        self._services = {
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
        self.clients = {key: self.create_client(kind, name) for key, (kind, name) in self._services.items()}
        self.action_client = ActionClient(self, MoveGripper, f"{self.prefix}/move")
        self._operations: dict[str, dict[str, Any]] = {}
        self._goal_handles: dict[str, Any] = {}
        self._operation_lock = threading.RLock()

    def _call(self, name: str, request: Any, timeout: float = 5.0):
        client = self.clients[name]
        if not client.wait_for_service(timeout_sec=timeout):
            raise TimeoutError(f"ROS service {self._services[name][1]} is unavailable")
        event = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _future: event.set())
        if not event.wait(timeout):
            raise TimeoutError(f"ROS service {self._services[name][1]} timed out")
        response = future.result()
        if response is None:
            raise RuntimeError(f"ROS service {client.srv_name} returned no response")
        return response

    @staticmethod
    def _require(response: Any, success: bool, error: str = "") -> Any:
        if not success:
            raise RuntimeError(error or "gripper request failed")
        return response

    def info(self) -> dict:
        response = self._call("info", GetGripperInfo.Request())
        self._require(response, response.success, response.error)
        return json.loads(response.descriptor_json)

    def state(self) -> dict:
        response = self._call("state", GetGripperState.Request())
        message = response.state
        stamp = message.header.stamp
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
            "stamp": {"sec": stamp.sec, "nanosec": stamp.nanosec},
        }

    def config(self) -> dict:
        response = self._call("config", GetGripperConfiguration.Request())
        self._require(response, response.success, response.error)
        return json.loads(response.config_json)

    def validate_config(self, patch: dict) -> dict:
        request = ValidateGripperConfiguration.Request()
        request.patch_json = json.dumps(patch)
        response = self._call("validate", request)
        self._require(response, response.valid, response.error)
        return json.loads(response.config_json)

    def apply_config(self, lease_id: str, patch: dict) -> dict:
        request = ApplyGripperConfiguration.Request()
        request.lease_id, request.patch_json = lease_id, json.dumps(patch)
        response = self._call("apply", request)
        self._require(response, response.success, response.error)
        return json.loads(response.config_json)

    def acquire(self, owner: str, ttl_sec: float | None = None) -> dict:
        request = AcquireGripperControl.Request()
        request.owner, request.ttl_sec = owner, float(ttl_sec or 0.0)
        response = self._call("acquire", request)
        self._require(response, response.success, response.error)
        return {"lease_id": response.lease_id, "expires_in_sec": response.expires_in_sec}

    def renew(self, lease_id: str) -> dict:
        request = RenewGripperControl.Request()
        request.lease_id = lease_id
        response = self._call("renew", request)
        self._require(response, response.success, response.error)
        return {"expires_in_sec": response.expires_in_sec}

    def release(self, lease_id: str) -> dict:
        request = ReleaseGripperControl.Request()
        request.lease_id = lease_id
        response = self._call("release", request)
        self._require(response, response.success, response.error)
        return {"released": True}

    def enable(self, lease_id: str, enabled: bool) -> dict:
        request = SetGripperMotionEnabled.Request()
        request.lease_id, request.enabled = lease_id, bool(enabled)
        response = self._call("enable", request)
        self._require(response, response.success, response.error)
        return {"motion_enabled": bool(enabled)}

    def stop(self, reason: str = "HTTP stop requested") -> dict:
        request = StopGripper.Request()
        request.reason = reason
        response = self._call("stop", request)
        return {"success": response.success, "stop_sent": response.stop_sent,
                "hardware_stop_confirmed": response.hardware_stop_confirmed, "error": response.error}

    def start_operation(self, lease_id: str, command: str, angle_degrees: float = 0.0) -> dict:
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
        message = wrapped.feedback
        with self._operation_lock:
            operation = self._operations.get(operation_id)
            if operation:
                operation["state"] = message.state or "running"
                operation["feedback_age_sec"] = message.feedback_age_sec if message.feedback_age_sec >= 0 else None

    def _on_goal_response(self, operation_id: str, future: Any) -> None:
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
        with self._operation_lock:
            operation = self._operations.get(operation_id)
            if operation is None:
                raise KeyError(operation_id)
            return dict(operation)

    def cancel_operation(self, operation_id: str) -> dict:
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
