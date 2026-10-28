from __future__ import annotations

import math
import threading
import time
import uuid
from typing import Any

from .backend import create_backend, create_model_policy
from .config import validate_patch


def relative_distance_progress(start: tuple[float, float, float], current: tuple[float, float]) -> float:
    """Project estimated displacement onto the operation's starting heading.

    Args:
        start: 启动，作为沿动作开始时的朝向投影估计位移进度所需的输入。 类型为 tuple[float, float, float]。
        current: current，作为沿动作开始时的朝向投影估计位移进度所需的输入。 类型为 tuple[float, float]。

    Returns:
        本次处理产生的结果值。 返回类型为 float。
    """
    return ((current[0] - start[0]) * math.cos(start[2]) +
            (current[1] - start[1]) * math.sin(start[2]))


def unwrapped_yaw_delta(current: float, previous: float) -> float:
    """Return the signed shortest yaw increment across the ±pi boundary.

    Args:
        current: current，作为计算跨越 ±π 边界时连续的偏航角增量所需的输入。 类型为 float。
        previous: previous，作为计算跨越 ±π 边界时连续的偏航角增量所需的输入。 类型为 float。

    Returns:
        本次处理产生的结果值。 返回类型为 float。
    """
    return math.atan2(math.sin(current - previous), math.cos(current - previous))


class BaseController:
    """ROS-independent control rules shared by the hardware node and tests."""

    def __init__(self, config: dict[str, Any], backend=None) -> None:
        """初始化对象并保存构造参数或依赖。

        Args:
            config: 设备或平台配置对象，提供本次操作所需参数。 类型为 dict[str, Any]。
            backend: 后端，作为初始化对象并保存构造参数或依赖所需的输入。

        Returns:
            无返回值。
        """
        self.config = config
        self.backend = backend or create_backend(config)
        self.model_policy = create_model_policy(config)
        self.lock = threading.RLock()
        self.backend.open()
        now = time.monotonic()
        self.lifecycle_state = "ready"
        self.connected = bool(getattr(self.backend, "connected", True))
        self.motion_enabled = False
        self.last_error = ""
        self.lease_id = ""
        self.lease_owner = ""
        self.lease_deadline = 0.0
        self.lease_ttl = 0.0
        self.target_linear = self.target_angular = 0.0
        self.command_time = 0.0
        self.operation_id = ""
        self.operation_state = "idle"
        self.system: dict | None = None
        self.motion: dict | None = None
        self.system_time = self.motion_time = 0.0
        self.last_tick = now
        self.x = self.y = self.yaw = 0.0
        self.pose_valid = False
        self.odometry_source = "integrated_velocity_estimate"
        self.odometry_time = 0.0
        self.odometry_stamp_sec = 0.0
        self.stop_requested = False
        self.zero_velocity_written = False
        self.stationary_confirmed = False

    def info(self) -> dict:
        """获取设备能力与静态信息。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。 返回类型为 dict。
        """
        return {"device_id": self.config["device_id"], "device_type": "base",
                "model": self.config["model"], "backend": self.config["backend"],
                "interface_version": "1.0", "capabilities": ["state", "odometry", "set_velocity",
                "move_relative", "turn_relative", "stop"],
                "odometry_source": self.odometry_source}

    def state(self) -> dict:
        """获取设备当前状态。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。 返回类型为 dict。
        """
        with self.lock:
            now = time.monotonic()
            stale = float(self.config["feedback"]["stale_sec"])
            system_age = max(now - self.system_time, 0.0) if self.system_time else None
            motion_age = max(now - self.motion_time, 0.0) if self.motion_time else None
            system_valid = self.system is not None and system_age is not None and system_age <= stale
            motion_valid = self.motion is not None and motion_age is not None and motion_age <= stale
            measured_linear = float(self.motion["linear_mps"]) if motion_valid else None
            measured_angular = float(self.motion["angular_rps"]) if motion_valid else None
            model_state = self.model_policy.state_fields(self.system)
            stationary = bool(motion_valid and abs(measured_linear) < 0.01 and abs(measured_angular) < 0.01)
            self.stationary_confirmed = stationary
            timestamp = self.odometry_stamp_sec if self.odometry_source == "simulator_odometry" and self.odometry_stamp_sec else time.time()
            return {"stamp_sec": timestamp,
                    "timestamp_source": "simulation_clock" if self.odometry_source == "simulator_odometry" else "gateway_snapshot",
                    "device_id": self.config["device_id"], "model": self.config["model"],
                    "backend": self.config["backend"], "lifecycle_state": self.lifecycle_state,
                    "connected": bool(self.connected), "motion_enabled": self.motion_enabled,
                    "operation_id": self.operation_id, "operation_state": self.operation_state,
                    "lease_owner": self.lease_owner, "last_error": self.last_error,
                    "system_feedback_valid": system_valid, "system_feedback_age_sec": system_age,
                    **model_state,
                    "motion_feedback_valid": motion_valid, "motion_feedback_age_sec": motion_age,
                    "measured_linear_mps": measured_linear, "measured_angular_rps": measured_angular,
                    "pose_estimate_valid": self.pose_valid and motion_valid,
                    "x_m": self.x if self.pose_valid else None, "y_m": self.y if self.pose_valid else None,
                    "yaw_rad": self.yaw if self.pose_valid else None,
                    "odometry_source": self.odometry_source,
                    "target_linear_mps": self.target_linear if self.motion_enabled else 0.0,
                    "target_angular_rps": self.target_angular if self.motion_enabled else 0.0,
                    "stop_requested": self.stop_requested,
                    "zero_velocity_written": self.zero_velocity_written,
                    "stationary_confirmed": stationary}

    def configuration(self) -> dict:
        """获取当前设备配置。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。 返回类型为 dict。
        """
        import copy
        with self.lock:
            return copy.deepcopy(self.config)

    def validate_configuration(self, patch: dict) -> dict:
        """验证配置。

        Args:
            patch: patch，作为验证配置所需的输入。 类型为 dict。

        Returns:
            布尔值，表示验证配置是否成功或成立。 返回类型为 dict。
        """
        with self.lock:
            return validate_patch(self.config, patch)

    def apply_configuration(self, patch: dict, lease_id: str) -> dict:
        """应用配置。

        Args:
            patch: patch，作为应用配置所需的输入。 类型为 dict。
            lease_id: 租约标识符，作为应用配置所需的输入。 类型为 str。

        Returns:
            本次处理产生的结果值。 返回类型为 dict。
        """
        with self.lock:
            self._require_lease(lease_id)
            if self.motion_enabled or self.operation_id:
                raise RuntimeError("base must be stopped before applying configuration")
            self.config = validate_patch(self.config, patch)
            if hasattr(self.backend, "config"):
                self.backend.config = self.config
            return self.configuration()

    def acquire_control(self, owner: str, ttl_sec: float | None = None) -> tuple[str, float]:
        """获取控制权。

        Args:
            owner: owner，作为获取控制权所需的输入。 类型为 str。
            ttl_sec: ttlsec，作为获取控制权所需的输入。 类型为 float | None。

        Returns:
            本次处理产生的结果值。 返回类型为 tuple[str, float]。
        """
        with self.lock:
            self._expire_lease()
            if self.lease_id:
                raise RuntimeError(f"base control lease is already held by {self.lease_owner}")
            ttl = float(ttl_sec or self.config["control"]["lease_ttl_sec"])
            maximum = float(self.config["control"]["max_lease_ttl_sec"])
            if not math.isfinite(ttl) or ttl <= 0 or ttl > maximum:
                raise ValueError(f"lease ttl must be in (0, {maximum:g}] seconds")
            if not owner or len(owner) > 128:
                raise ValueError("owner must contain 1 to 128 characters")
            self.lease_id = uuid.uuid4().hex
            self.lease_owner = owner
            self.lease_ttl = ttl
            self.lease_deadline = time.monotonic() + ttl
            return self.lease_id, ttl

    def renew_control(self, lease_id: str) -> float:
        """续租控制权。

        Args:
            lease_id: 租约标识符，作为续租控制权所需的输入。 类型为 str。

        Returns:
            本次处理产生的结果值。 返回类型为 float。
        """
        with self.lock:
            self._require_lease(lease_id)
            self.lease_deadline = time.monotonic() + self.lease_ttl
            return self.lease_ttl

    def release_control(self, lease_id: str) -> dict:
        """释放控制权。

        Args:
            lease_id: 租约标识符，作为释放控制权所需的输入。 类型为 str。

        Returns:
            本次处理产生的结果值。 返回类型为 dict。
        """
        with self.lock:
            self._require_lease(lease_id)
        result = self.stop("control lease released")
        with self.lock:
            self.motion_enabled = False
            self._clear_lease()
        return result

    def enable_motion(self, lease_id: str, enabled: bool) -> dict:
        """启用motion。

        Args:
            lease_id: 租约标识符，作为启用motion所需的输入。 类型为 str。
            enabled: 表示对应功能是否启用的布尔值。 类型为 bool。

        Returns:
            本次处理产生的结果值。 返回类型为 dict。
        """
        if not enabled:
            with self.lock:
                self._require_lease(lease_id)
                self.motion_enabled = False
            return self.stop("motion disabled") | {"enabled": False, "control_mode_confirmed": True}
        with self.lock:
            self._require_lease(lease_id)
            if not self.connected:
                raise RuntimeError("base is not connected")
        self.poll_once()
        state = self.state()
        if not state["system_feedback_valid"] or not state["motion_feedback_valid"]:
            raise RuntimeError("fresh system and motion feedback are required before enabling motion")
        if not self.model_policy.motion_permitted(state):
            raise RuntimeError("base state or error does not permit motion")
        if not getattr(self.backend, "enabled", False):
            self.backend.request_control_mode()
        deadline = time.monotonic() + float(self.config["feedback"]["control_mode_timeout_sec"])
        while time.monotonic() < deadline:
            self.poll_once()
            current = self.state()
            if self.model_policy.control_confirmed(current):
                with self.lock:
                    self._require_lease(lease_id)
                    if not self.model_policy.motion_permitted(current):
                        raise RuntimeError("base reported a fault while enabling motion")
                    self.motion_enabled = True
                    self.stop_requested = False
                    self.zero_velocity_written = False
                    self.last_error = ""
                return {"enabled": True, "control_mode_confirmed": True}
            time.sleep(0.02)
        raise TimeoutError("base did not confirm control mode")

    def set_velocity(self, lease_id: str, linear: float, angular: float) -> None:
        """设置速度。

        Args:
            lease_id: 租约标识符，作为设置速度所需的输入。 类型为 str。
            linear: 线速度，作为设置速度所需的输入。 类型为 float。
            angular: 角速度，作为设置速度所需的输入。 类型为 float。

        Returns:
            无返回值。
        """
        linear = self._finite(linear, "linear_mps")
        angular = self._finite(angular, "angular_rps")
        motion = self.config["motion"]
        if abs(linear) > motion["max_linear_mps"] or abs(angular) > motion["max_angular_rps"]:
            raise ValueError("requested velocity exceeds configured base limits")
        with self.lock:
            self._require_lease(lease_id)
            if not self.motion_enabled:
                raise RuntimeError("base motion is disabled")
            if self.operation_id:
                raise RuntimeError("a relative base operation is active")
            self.target_linear, self.target_angular = linear, angular
            self.command_time = time.monotonic()
            self.stop_requested = False
            self.zero_velocity_written = False

    def set_operation_velocity(self, operation_id: str, linear: float, angular: float) -> None:
        """设置操作速度。

        Args:
            operation_id: 操作标识符，作为设置操作速度所需的输入。 类型为 str。
            linear: 线速度，作为设置操作速度所需的输入。 类型为 float。
            angular: 角速度，作为设置操作速度所需的输入。 类型为 float。

        Returns:
            无返回值。
        """
        with self.lock:
            if not operation_id or operation_id != self.operation_id or not self.motion_enabled:
                raise RuntimeError("relative operation is no longer active")
            self.target_linear, self.target_angular = linear, angular
            self.command_time = time.monotonic()
            self.stop_requested = False
            self.zero_velocity_written = False

    def begin_operation(self, lease_id: str) -> tuple[str, tuple[float, float, float]]:
        """为有效控制租约登记一个运行中的设备操作。

        Args:
            lease_id: 租约标识符，作为执行 begin_operation 对应的处理流程所需的输入。 类型为 str。

        Returns:
            本次处理产生的结果值。 返回类型为 tuple[str, tuple[float, float, float]]。
        """
        with self.lock:
            self._require_lease(lease_id)
            if not self.motion_enabled:
                raise RuntimeError("base motion is disabled")
            if self.operation_id:
                raise RuntimeError("another relative base operation is active")
            if not self.pose_valid or not self.state()["motion_feedback_valid"]:
                raise RuntimeError("fresh estimated odometry is required for relative motion")
            self.operation_id = uuid.uuid4().hex
            self.operation_state = "running"
            return self.operation_id, (self.x, self.y, self.yaw)

    def finish_operation(self, operation_id: str, state: str) -> None:
        """结束指定操作并记录最终状态。

        Args:
            operation_id: 操作标识符，作为执行 finish_operation 对应的处理流程所需的输入。 类型为 str。
            state: 状态，作为执行 finish_operation 对应的处理流程所需的输入。 类型为 str。

        Returns:
            无返回值。
        """
        with self.lock:
            if self.operation_id == operation_id:
                self.operation_state = state
                self.operation_id = ""
                self.target_linear = self.target_angular = 0.0
                self.command_time = 0.0

    def finish_operation_with_zero(self, operation_id: str, state: str) -> dict:
        """Write zero at an action boundary while leaving a healthy lease enabled.

        Args:
            operation_id: 操作标识符，作为在动作边界发送零速度并保留有效控制租约所需的输入。 类型为 str。
            state: 状态，作为在动作边界发送零速度并保留有效控制租约所需的输入。 类型为 str。

        Returns:
            本次处理产生的结果值。 返回类型为 dict。
        """
        with self.lock:
            if not operation_id or operation_id != self.operation_id:
                return {"stop_requested": False, "zero_velocity_written": False,
                        "stationary_confirmed": False, "hardware_estop": False,
                        "error": "operation no longer owns the base"}
            self.target_linear = self.target_angular = 0.0
            self.command_time = 0.0
            self.operation_state = state
            self.operation_id = ""
        error = ""
        written = False
        try:
            self.backend.stop()
            written = True
        except Exception as exc:
            error = str(exc)
            with self.lock:
                self.motion_enabled = False
                self.last_error = error
                self.operation_state = "failed"
        with self.lock:
            self.zero_velocity_written = written
        state_data = self.state()
        return {"stop_requested": True, "zero_velocity_written": written,
                "stationary_confirmed": state_data["stationary_confirmed"],
                "hardware_estop": state_data["hardware_estop"], "error": error}

    def stop(self, reason: str = "stop requested") -> dict:
        """停止设备当前动作。

        Args:
            reason: 原因，作为停止设备当前动作所需的输入。 类型为 str。

        Returns:
            本次处理产生的结果值。 返回类型为 dict。
        """
        with self.lock:
            self.stop_requested = True
            self.target_linear = self.target_angular = 0.0
            self.command_time = 0.0
            self.motion_enabled = False
        written = False
        error = ""
        try:
            self.backend.stop()
            written = True
        except Exception as exc:
            error = str(exc)
            with self.lock:
                self.last_error = f"{reason}: {error}"
        with self.lock:
            self.zero_velocity_written = written
        state = self.state()
        return {"stop_requested": True, "zero_velocity_written": written,
                "stationary_confirmed": state["stationary_confirmed"],
                "hardware_estop": state["hardware_estop"], "error": error}

    def tick(self) -> None:
        """更新当前设备或操作。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        self.poll_once()
        now = time.monotonic()
        with self.lock:
            self._expire_lease()
            state = self.state()
            unsafe = self.motion_enabled and (
                not self.connected or not state["system_feedback_valid"] or
                not state["motion_feedback_valid"] or
                not self.model_policy.motion_permitted(state)
            )
            stale_command = self.motion_enabled and self.command_time and (
                now - self.command_time > float(self.config["motion"]["command_timeout_sec"]))
            should_write = self.motion_enabled
            if stale_command:
                self.target_linear = self.target_angular = 0.0
                self.command_time = now
            linear = self.target_linear if self.motion_enabled else 0.0
            angular = self.target_angular if self.motion_enabled else 0.0
        if unsafe:
            self.stop("feedback, lease or vehicle state became unsafe")
            return
        if should_write:
            try:
                self.backend.set_velocity(linear, angular)
                with self.lock:
                    self.zero_velocity_written = linear == 0.0 and angular == 0.0
                    self.last_error = ""
            except Exception as exc:
                with self.lock:
                    self.connected = False
                    self.last_error = str(exc)
                self.stop("velocity write failed")

    def poll_once(self) -> None:
        """读取并验证一条新样本，更新设备状态。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        system, motion, connected, error = self.backend.poll()
        odometry = self.backend.read_odometry() if hasattr(self.backend, "read_odometry") else None
        now = time.monotonic()
        with self.lock:
            self.connected = connected
            if error:
                self.last_error = error
            if system is not None:
                self.system = system
                self.system_time = now
            if motion is not None:
                self.motion = motion
                self.motion_time = now
                linear, angular = float(motion["linear_mps"]), float(motion["angular_rps"])
                dt = max(0.0, min(now - self.last_tick, 0.25))
                self.last_tick = now
                self.x += linear * math.cos(self.yaw) * dt
                self.y += linear * math.sin(self.yaw) * dt
                self.yaw = math.atan2(math.sin(self.yaw + angular * dt), math.cos(self.yaw + angular * dt))
                self.pose_valid = True
            if odometry is not None:
                self.x, self.y, self.yaw = (float(odometry[key]) for key in ("x_m", "y_m", "yaw_rad"))
                self.pose_valid = True
                self.odometry_time = now
                self.odometry_stamp_sec = float(odometry.get("stamp_sec", 0.0))
                self.odometry_source = "simulator_odometry"

    def shutdown(self) -> None:
        """关闭当前设备或操作。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        try:
            self.stop("base service shutting down")
        finally:
            try:
                self.backend.close()
            finally:
                self.lifecycle_state = "closed"

    def _require_lease(self, lease_id: str) -> None:
        """确认请求持有有效的控制租约。

        Args:
            lease_id: 租约标识符，作为确认请求持有有效的控制租约所需的输入。 类型为 str。

        Returns:
            无返回值。
        """
        self._expire_lease()
        if not self.lease_id or lease_id != self.lease_id or time.monotonic() >= self.lease_deadline:
            raise PermissionError("valid base control lease required")

    def _expire_lease(self) -> bool:
        """检查控制租约是否到期并在到期时释放控制权。

        Args:
            无参数。

        Returns:
            布尔值，表示执行 _expire_lease 对应的处理流程是否成功或成立。
        """
        if self.lease_id and time.monotonic() >= self.lease_deadline:
            was_active = self.motion_enabled or self.target_linear != 0.0 or self.target_angular != 0.0
            self.motion_enabled = False
            self.target_linear = self.target_angular = 0.0
            self.command_time = 0.0
            self._clear_lease()
            if was_active:
                self.stop_requested = True
                try:
                    self.backend.stop()
                    self.zero_velocity_written = True
                except Exception as exc:
                    self.zero_velocity_written = False
                    self.last_error = f"control lease expired: {exc}"
            return True
        return False

    def _clear_lease(self) -> None:
        """清除租约。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        self.lease_id = self.lease_owner = ""
        self.lease_deadline = self.lease_ttl = 0.0

    @staticmethod
    def _finite(value: float, name: str) -> float:
        """校验有限数值输入。

        Args:
            value: 要应用、比较或转换的输入值。 类型为 float。
            name: 名称，作为执行 _finite 对应的处理流程所需的输入。 类型为 str。

        Returns:
            本次处理产生的结果值。 返回类型为 float。
        """
        if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
            raise ValueError(f"{name} must be a finite number")
        return float(value)
