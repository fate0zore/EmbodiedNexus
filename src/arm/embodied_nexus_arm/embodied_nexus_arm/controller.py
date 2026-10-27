from __future__ import annotations

import copy
import math
import threading
import time
import uuid
from typing import Any

from .backend import AuboBackend, MockBackend, create_backend


SAFE_MODES = {"normal", "reducedmode"}


class ArmController:
    """ROS-independent Aubo lifecycle, state and motion policy."""

    def __init__(self, config: dict[str, Any], backend=None) -> None:
        """初始化对象并保存构造参数或依赖。

        Args:
            config: 设备或平台配置对象，提供本次操作所需参数。 类型为 dict[str, Any]。
            backend: 后端，作为初始化对象并保存构造参数或依赖所需的输入。

        Returns:
            无返回值。
        """
        self.config = copy.deepcopy(config)
        self.backend = backend or create_backend(config)
        self.lock = threading.RLock()
        self.backend.open()
        self.connected = bool(getattr(self.backend, "connected", True))
        self.lifecycle_state = "ready" if self.connected else "disconnected"
        self.motion_enabled = False
        self.last_error = ""
        self.lease_id = ""
        self.lease_owner = ""
        self.lease_deadline = 0.0
        self.lease_ttl = 0.0
        self.active_operation = ""
        self.operation_state = "idle"
        self.cancel_event = threading.Event()
        self.velocity_command_time = 0.0
        self.servo_mode_active = False
        self.teach_enabled = False
        self.stop_requested = False
        self.controller_accepted = False
        self.stationary_confirmed = False
        self.feedback: dict[str, Any] = {}
        self.feedback_time = 0.0
        self.payload = self.backend.read_optional("RobotConfig.getPayload")
        self.tcp_offset = self.backend.read_optional("RobotConfig.getTcpOffset")

    def info(self) -> dict[str, Any]:
        """获取设备能力与静态信息。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any]。
        """
        capabilities = ["joint_state", "move_joints", "joint_velocity", "power", "teach_mode", "stop"]
        if hasattr(self.backend, "capabilities"):
            capabilities = list(self.backend.capabilities)
        return {"device_id": self.config["device_id"], "device_type": "arm",
                "model": self.config["model"], "backend": self.config["backend"],
                "interface_version": "1.0", "capabilities": capabilities,
                "joint_names": list(self.config["joints"]["names"]),
                "units": {"position": "rad", "velocity": "rad/s", "acceleration": "rad/s^2"}}

    def state(self) -> dict[str, Any]:
        """获取设备当前状态。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any]。
        """
        with self.lock:
            now = time.monotonic()
            wall_now = time.time()
            age = max(0.0, now - self.feedback_time) if self.feedback_time else None
            fresh = age is not None and age <= float(self.config["feedback"]["stale_sec"])
            fb = copy.deepcopy(self.feedback)
            velocities = fb.get("velocities_rad_sec") if fresh else None
            stationary = (None if velocities is None else bool(velocities) and all(
                abs(float(value)) <= 0.01 for value in velocities))
            return {"device_id": self.config["device_id"], "model": self.config["model"],
                    "backend": self.config["backend"], "lifecycle_state": self.lifecycle_state,
                    "connected": self.connected, "feedback_valid": bool(fresh and fb),
                    "feedback_age_sec": age,
                    "stamp_sec": fb.get("stamp_sec", wall_now - age if age is not None else None),
                    "joint_names": list(self.config["joints"]["names"]),
                    "feedback_timestamp_source": fb.get("feedback_timestamp_source", "host_receive"),
                    "positions_rad": fb.get("positions_rad"),
                    "velocities_rad_sec": fb.get("velocities_rad_sec"),
                    "mode": fb.get("mode"), "safety_mode": fb.get("safety_mode"),
                    "hardware_estop": fb.get("hardware_estop") if fresh else None,
                    "teach_enabled": fb.get("teach_enabled"), "motion_enabled": self.motion_enabled,
                    "lease_owner": self.lease_owner, "operation_id": self.active_operation,
                    "operation_state": self.operation_state, "last_error": self.last_error,
                    "stop_requested": self.stop_requested,
                    "controller_accepted": self.controller_accepted,
                    "stationary_confirmed": stationary,
                    "continuous_control_active": self.servo_mode_active,
                    "joint_velocity_command_age_sec": (
                        max(0.0, now - self.velocity_command_time)
                        if self.velocity_command_time else None
                    ),
                    "joint_velocity_command_stamp_sec": (
                        wall_now - max(0.0, now - self.velocity_command_time)
                        if self.velocity_command_time else None
                    ),
                    "payload": self.payload, "tcp_offset": self.tcp_offset,
                    "joint_limits_valid": self._effective_limits() is not None}

    def configuration(self) -> dict[str, Any]:
        """获取当前设备配置。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any]。
        """
        with self.lock:
            return copy.deepcopy(self.config)

    def validate_configuration(self, patch: dict[str, Any]) -> dict[str, Any]:
        """验证配置。

        Args:
            patch: patch，作为验证配置所需的输入。 类型为 dict[str, Any]。

        Returns:
            布尔值，表示验证配置是否成功或成立。 返回类型为 dict[str, Any]。
        """
        if not isinstance(patch, dict):
            raise ValueError("configuration patch must be a mapping")
        if set(patch) - {"motion"} or ("motion" in patch and set(patch["motion"]) - {"max_speed_rad_sec"}):
            raise ValueError("only motion.max_speed_rad_sec may be adjusted at runtime")
        candidate = copy.deepcopy(self.config)
        candidate["motion"].update(patch.get("motion", {}))
        value = float(candidate["motion"]["max_speed_rad_sec"])
        if not math.isfinite(value) or value <= 0 or value > self.config["motion"]["max_speed_rad_sec"]:
            raise ValueError("runtime speed limit must be positive and cannot be increased")
        return candidate

    def apply_configuration(self, patch: dict[str, Any], lease_id: str) -> dict[str, Any]:
        """应用配置。

        Args:
            patch: patch，作为应用配置所需的输入。 类型为 dict[str, Any]。
            lease_id: 租约标识符，作为应用配置所需的输入。 类型为 str。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any]。
        """
        with self.lock:
            self._require_lease(lease_id)
            if self.active_operation or self.teach_enabled or self.motion_enabled:
                raise RuntimeError("arm must be stopped and disabled before applying configuration")
            self.config = self.validate_configuration(patch)
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
            self._expire_lease_locked()
            if self.lease_id:
                raise RuntimeError(f"arm control lease is already held by {self.lease_owner}")
            ttl = float(ttl_sec or self.config["control"]["lease_ttl_sec"])
            maximum = float(self.config["control"]["max_lease_ttl_sec"])
            if not math.isfinite(ttl) or ttl <= 0 or ttl > maximum:
                raise ValueError(f"lease ttl must be in (0, {maximum:g}] seconds")
            if not owner or len(owner) > 128:
                raise ValueError("owner must contain 1 to 128 characters")
            self.lease_id, self.lease_owner = uuid.uuid4().hex, owner
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

    def release_control(self, lease_id: str) -> dict[str, Any]:
        """释放控制权。

        Args:
            lease_id: 租约标识符，作为释放控制权所需的输入。 类型为 str。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any]。
        """
        with self.lock:
            self._require_lease(lease_id)
        result = self.stop("control lease released")
        with self.lock:
            self.motion_enabled = False
            self._clear_lease_locked()
        return result

    def enable_motion(self, lease_id: str, enabled: bool) -> dict[str, Any]:
        """启用motion。

        Args:
            lease_id: 租约标识符，作为启用motion所需的输入。 类型为 str。
            enabled: 表示对应功能是否启用的布尔值。 类型为 bool。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any]。
        """
        with self.lock:
            self._require_lease(lease_id)
            if not enabled:
                self.motion_enabled = False
                self.cancel_event.set()
            elif self.active_operation or self.teach_enabled:
                raise RuntimeError("arm is busy with another operation")
        if not enabled:
            return self.stop("motion disabled") | {"enabled": False}
        self.poll_once()
        state = self.state()
        if not state["feedback_valid"]:
            raise RuntimeError("fresh arm feedback is required before enabling motion")
        if not self.connected or str(state["mode"]).lower() != "running":
            raise RuntimeError(f"Aubo must be Running before enabling motion (mode={state['mode']})")
        if str(state["safety_mode"]).lower() not in SAFE_MODES:
            raise RuntimeError(f"Aubo safety state does not permit motion: {state['safety_mode']}")
        if self._effective_limits() is None:
            raise RuntimeError("controller joint limits are unknown; motion remains disabled")
        with self.lock:
            self._require_lease(lease_id)
            self.motion_enabled = True
            self.cancel_event.clear()
            self.last_error = ""
        return {"enabled": True}

    def set_joint_velocity(self, lease_id: str, values: list[float]) -> dict[str, Any]:
        """设置关节速度。

        Args:
            lease_id: 租约标识符，作为设置关节速度所需的输入。 类型为 str。
            values: 要应用、比较或转换的一组输入值。 类型为 list[float]。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any]。
        """
        velocity = self._validate_vector(values, "velocity_rad_sec")
        limit = float(self.config["motion"]["max_speed_rad_sec"])
        if any(abs(value) > limit for value in velocity):
            raise ValueError("requested joint velocity exceeds configured arm limit")
        self._require_motion_ready(lease_id)
        measured = self.state()["positions_rad"]
        limits = self._effective_limits()
        if limits is None:
            raise RuntimeError("controller joint limits are unknown; motion is disabled")
        for index, value in enumerate(velocity):
            if (value > 0 and measured[index] >= limits[1][index]) or (
                    value < 0 and measured[index] <= limits[0][index]):
                raise ValueError(f"joint velocity points beyond limit for {self.config['joints']['names'][index]}")
        with self.lock:
            self._require_lease(lease_id)
            if not self.motion_enabled or self.active_operation or self.teach_enabled:
                raise RuntimeError("arm velocity control requires an enabled idle arm")
            previous = self.velocity_command_time > 0
        if all(abs(value) <= 1.0e-4 for value in velocity):
            accepted = self._halt_velocity_stream()
            if not accepted:
                raise RuntimeError("Aubo did not confirm joint stop")
            state = self.state()
            return {"controller_accepted": True, "stationary_confirmed": state["stationary_confirmed"]}
        if not previous:
            if not self.backend.stop():
                raise RuntimeError("Aubo did not confirm pre-velocity stop")
            result = self.backend.execute("MotionControl.setServoMode", [True])
            if result not in (0, None):
                self.backend.stop()
                raise RuntimeError(f"Aubo servo mode rejected command: result={result}")
            with self.lock:
                self.servo_mode_active = True
        try:
            enabled = self.backend.execute("MotionControl.isServoModeEnabled")
            if enabled is False:
                raise RuntimeError("Aubo servo mode did not become active")
            command_period = float(self.config["motion"]["servo_joint_command_period_sec"])
            target = [float(q) + float(v) * command_period for q, v in zip(measured, velocity)]
            for index, value in enumerate(target):
                if not limits[0][index] <= value <= limits[1][index]:
                    raise ValueError(f"joint velocity step exceeds limit for {self.config['joints']['names'][index]}")
            servo = self.config["motion"]
            result = self.backend.execute("MotionControl.servoJoint", [
                target,
                float(servo["servo_joint_velocity_rad_sec"]),
                float(servo["servo_joint_acceleration_rad_sec2"]),
                float(servo["servo_joint_period_sec"]),
                float(servo["servo_joint_lookahead_sec"]),
                int(servo["servo_joint_gain"]),
            ])
            if result not in (0, None, 2):
                raise RuntimeError(f"Aubo servoJoint rejected command: result={result}")
        except Exception:
            self.stop("joint servo command failed")
            raise
        with self.lock:
            self.velocity_command_time = time.monotonic()
            self.controller_accepted = result in (0, None)
        return {
            "controller_accepted": result in (0, None),
            "retry_next_cycle": result == 2,
            "commanded_positions_rad": target,
        }

    def move_joints(self, lease_id: str, positions: list[float], speed: float = 0.0,
                    acceleration: float = 0.0, timeout: float = 0.0,
                    cancel_requested=None, feedback=None) -> dict[str, Any]:
        """运动joints。

        Args:
            lease_id: 租约标识符，作为运动joints所需的输入。 类型为 str。
            positions: positions，作为运动joints所需的输入。 类型为 list[float]。
            speed: 速度，作为运动joints所需的输入。 类型为 float。
            acceleration: acceleration，作为运动joints所需的输入。 类型为 float。
            timeout: 本次网络或服务调用的最长等待时长，通常以秒为单位。 类型为 float。
            cancel_requested: 取消requested，作为运动joints所需的输入。
            feedback: 反馈，作为运动joints所需的输入。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any]。
        """
        target = self._validate_vector(positions, "positions_rad")
        state = self.state()
        if not state["feedback_valid"]:
            raise RuntimeError("fresh joint feedback is required before motion")
        limits = self._effective_limits()
        if limits is None:
            raise RuntimeError("controller joint limits are unknown; motion is disabled")
        for i, value in enumerate(target):
            if not limits[0][i] <= value <= limits[1][i]:
                raise ValueError(f"joint {self.config['joints']['names'][i]} target exceeds effective limits")
        max_speed = float(self.config["motion"]["max_speed_rad_sec"])
        speed = float(speed or max_speed)
        acceleration = float(acceleration or self.config["motion"]["max_acceleration_rad_sec2"])
        if not math.isfinite(speed) or not 0 < speed <= max_speed:
            raise ValueError(f"speed_rad_sec must be in (0, {max_speed:g}]")
        if not math.isfinite(acceleration) or not 0 < acceleration <= self.config["motion"]["max_acceleration_rad_sec2"]:
            raise ValueError("acceleration_rad_sec2 exceeds configured limit")
        timeout = float(timeout or self.config["motion"]["operation_timeout_sec"])
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("timeout_sec must be finite and positive")
        with self.lock:
            self._require_lease(lease_id)
            if not self.motion_enabled or self.active_operation or self.teach_enabled:
                raise RuntimeError("joint motion requires an enabled idle arm")
            operation_id = uuid.uuid4().hex
            self.active_operation = operation_id
            self.operation_state = "running"
            self.cancel_event.clear()
            self.velocity_command_time = 0.0
            self.controller_accepted = False
            self.stop_requested = False
            self.stationary_confirmed = False
        stop_requested = False
        accepted = False
        last_error = max(abs(a - b) for a, b in zip(target, state["positions_rad"]))
        stable_since = None
        terminal_state = "failed"
        try:
            self._require_motion_ready(lease_id)
            if self.servo_mode_active:
                self._leave_servo_mode()
            self.backend.execute("MotionControl.setServoModeSelect", [0])
            if not self.backend.stop():
                raise RuntimeError("Aubo did not confirm pre-move stop")
            result = self.backend.execute("MotionControl.moveJoint", [
                target, acceleration, speed, 0.0, 0.0])
            if result not in (0, None):
                raise RuntimeError(f"Aubo moveJoint rejected command: result={result}")
            accepted = True
            with self.lock:
                self.controller_accepted = True
            deadline = time.monotonic() + timeout
            stable_sec = float(self.config["motion"]["stable_sec"])
            tolerance = float(self.config["motion"]["goal_tolerance_rad"])
            while time.monotonic() < deadline:
                self.poll_once()
                if (cancel_requested and cancel_requested()) or self.cancel_event.is_set():
                    stop_requested = True
                    break
                self._require_motion_ready(lease_id)
                current_state = self.state()
                if not current_state["feedback_valid"] or current_state["positions_rad"] is None:
                    raise RuntimeError("joint feedback became stale during motion")
                last_error = max(abs(a - b) for a, b in zip(target, current_state["positions_rad"]))
                velocities = current_state["velocities_rad_sec"]
                measured_stationary = velocities is not None and all(
                    abs(float(value)) <= 0.01 for value in velocities)
                if feedback:
                    feedback("moving", last_error)
                if last_error <= tolerance and measured_stationary:
                    stable_since = stable_since or time.monotonic()
                    if time.monotonic() - stable_since >= stable_sec:
                        break
                else:
                    stable_since = None
                time.sleep(0.04)
            else:
                raise TimeoutError("Aubo joint target did not reach tolerance before timeout")
            if stop_requested:
                self.backend.stop()
                self.poll_once()
                stopped_state = self.state()
                terminal_state = "cancelled"
                return {"success": False, "message": "cancelled", "max_error_rad": last_error,
                        "controller_accepted": accepted, "stop_requested": True,
                        "stationary_confirmed": stopped_state["stationary_confirmed"] is True,
                        "hardware_estop": bool(stopped_state.get("hardware_estop"))}
            if last_error > tolerance:
                raise RuntimeError("Aubo joint target failed to settle")
            terminal_state = "succeeded"
            return {"success": True, "message": "joint target reached", "max_error_rad": last_error,
                    "controller_accepted": accepted, "stop_requested": False,
                    "stationary_confirmed": True, "hardware_estop": False}
        except Exception:
            stop_requested = True
            self.backend.stop()
            # Refresh measured feedback after the stop request. Some backends,
            # including the deterministic mock, can confirm the stop immediately.
            self.poll_once()
            raise
        finally:
            with self.lock:
                if self.active_operation == operation_id:
                    self.active_operation = ""
                    self.operation_state = terminal_state
                    self.stop_requested = stop_requested
                    velocities = self.feedback.get("velocities_rad_sec")
                    self.stationary_confirmed = bool(velocities) and all(
                        abs(float(v)) <= 0.01 for v in velocities)

    def set_lifecycle(self, lease_id: str, command: str) -> dict[str, Any]:
        """设置生命周期。

        Args:
            lease_id: 租约标识符，作为设置生命周期所需的输入。 类型为 str。
            command: command，作为设置生命周期所需的输入。 类型为 str。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any]。
        """
        if command not in {"power_on", "startup", "power_off"}:
            raise ValueError("command must be power_on, startup or power_off")
        with self.lock:
            self._require_lease(lease_id)
            if self.active_operation or self.teach_enabled:
                raise RuntimeError("arm is busy")
            self.motion_enabled = False
            self.active_operation = "lifecycle"
            self.operation_state = "running"
            self.cancel_event.clear()
        try:
            if self.cancel_event.is_set():
                raise RuntimeError("Aubo lifecycle operation cancelled")
            if command == "power_off":
                if not self.backend.stop():
                    raise RuntimeError("Aubo did not confirm stop before power off")
                result = self._power_off()
                expected = {"PowerOff", "Poweroff", "poweroff"}
            elif command == "power_on":
                current = self.state().get("mode")
                if self.cancel_event.is_set():
                    raise RuntimeError("Aubo lifecycle operation cancelled")
                result = 0 if current in {"Idle", "Running"} else self._power_on()
                expected = {"Idle", "Running"}
            else:
                state = self.state()
                if state.get("mode") not in {"Idle", "Running"}:
                    if self.cancel_event.is_set():
                        raise RuntimeError("Aubo lifecycle operation cancelled")
                    self._power_on()
                    self._wait_for_mode({"Idle", "Running"})
                state = self.state()
                if self.cancel_event.is_set():
                    raise RuntimeError("Aubo lifecycle operation cancelled")
                result = 0 if state.get("mode") == "Running" else self.backend.execute("RobotManage.startup")
                expected = {"Running"}
            if result not in (0, None):
                raise RuntimeError(f"Aubo {command} rejected command: result={result}")
            self._wait_for_mode(expected)
            return {"success": True, "operation_state": "succeeded"}
        finally:
            with self.lock:
                self.active_operation = ""
                self.operation_state = "idle"

    def _power_on(self):
        """启动机械臂并等待控制模式就绪。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。
        """
        return self.backend.execute("RobotManage.poweron")

    def _wait_for_mode(self, expected: set[str]) -> None:
        """等待机械臂进入指定控制模式。

        Args:
            expected: expected，作为等待机械臂进入指定控制模式所需的输入。 类型为 set[str]。

        Returns:
            无返回值。
        """
        deadline = time.monotonic() + max(45.0, float(self.config["motion"]["operation_timeout_sec"]))
        while time.monotonic() < deadline:
            if self.cancel_event.is_set():
                raise RuntimeError("Aubo lifecycle operation cancelled")
            self.poll_once()
            if self.feedback.get("mode") in expected:
                return
            time.sleep(0.1)
        raise TimeoutError(f"Aubo did not reach mode {sorted(expected)} (last={self.feedback.get('mode')})")

    def _power_off(self):
        """关闭机械臂驱动并停止运动。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。
        """
        error = None
        for method in ("RobotManage.poweroff", "RobotManage.powerOff"):
            try:
                return self.backend.execute(method)
            except Exception as exc:
                error = exc
        raise RuntimeError(f"Aubo power_off RPC unavailable: {error}")

    def set_teach_mode(self, lease_id: str, enabled: bool) -> dict[str, Any]:
        """设置示教mode。

        Args:
            lease_id: 租约标识符，作为设置示教mode所需的输入。 类型为 str。
            enabled: 表示对应功能是否启用的布尔值。 类型为 bool。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any]。
        """
        with self.lock:
            self._require_lease(lease_id)
            if self.active_operation not in {"", "teach"}:
                raise RuntimeError("arm is busy with another operation")
            if enabled:
                state = self.state()
                if not self.motion_enabled or not state["feedback_valid"] or not self._safe_state(state):
                    raise RuntimeError("teach mode requires an enabled arm in fresh Running/Normal state")
                self.active_operation = "teach"
        method = self.config["teach"]["method"]
        if method == "handguide":
            result = self.backend.execute("RobotManage.handguideMode", [[], []]) if enabled else self.backend.execute("RobotManage.exitHandguideMode")
        else:
            result = self.backend.execute(f"RobotManage.{method}", [bool(enabled)])
        if result not in (0, None):
            raise RuntimeError(f"Aubo teach mode rejected command: result={result}")
        with self.lock:
            self.teach_enabled = bool(enabled)
            self.active_operation = "teach" if enabled else ""
            self.operation_state = "running" if enabled else "idle"
            if enabled:
                self.motion_enabled = False
        return {"success": True, "teach_state": "enabled" if enabled else "disabled"}

    def stop(self, reason: str = "stop requested") -> dict[str, Any]:
        """停止设备当前动作。

        Args:
            reason: 原因，作为停止设备当前动作所需的输入。 类型为 str。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any]。
        """
        with self.lock:
            self.stop_requested = True
            self.cancel_event.set()
            self.motion_enabled = False
            self.velocity_command_time = 0.0
        accepted = bool(self.backend.stop())
        if self.servo_mode_active:
            try:
                servo_result = self.backend.execute("MotionControl.setServoMode", [False])
                accepted = accepted and servo_result in (0, None)
            except Exception:
                accepted = False
            with self.lock:
                self.servo_mode_active = False
        if self.teach_enabled:
            try:
                self._send_teach(False)
            except Exception as exc:
                self.last_error = f"stop sent but teach mode exit failed: {exc}"
            else:
                self.teach_enabled = False
                self.active_operation = ""
        try:
            self.poll_once()
        except Exception:
            pass
        with self.lock:
            velocities = self.feedback.get("velocities_rad_sec") or []
            stationary = bool(velocities) and all(abs(float(v)) <= 0.01 for v in velocities)
            self.stationary_confirmed = stationary
            estop = bool(self.feedback.get("hardware_estop", False))
        return {"stop_requested": True, "controller_accepted": accepted,
                "stationary_confirmed": stationary, "hardware_estop": estop,
                "reason": str(reason), "error": "" if accepted else getattr(self.backend, "last_error", "stop failed")}

    def _leave_servo_mode(self) -> None:
        """Stop once when changing from streaming servo targets to moveJoint.

        Args:
            无参数。

        Returns:
            无返回值。
        """
        if not self.backend.stop():
            raise RuntimeError("Aubo did not confirm stop before leaving servo mode")
        result = self.backend.execute("MotionControl.setServoMode", [False])
        if result not in (0, None):
            raise RuntimeError(f"Aubo could not leave servo mode: result={result}")
        with self.lock:
            self.servo_mode_active = False
            self.velocity_command_time = 0.0

    def _halt_velocity_stream(self) -> bool:
        """Halt a zero-velocity command while keeping the lease enabled.

        Args:
            无参数。

        Returns:
            布尔值，表示在速度流结束时发送零速度指令是否成功或成立。
        """
        accepted = bool(self.backend.stop())
        if self.servo_mode_active:
            try:
                result = self.backend.execute("MotionControl.setServoMode", [False])
                accepted = accepted and result in (0, None)
            except Exception:
                accepted = False
        with self.lock:
            self.velocity_command_time = 0.0
            self.servo_mode_active = False
            self.controller_accepted = accepted
        return accepted

    def poll_once(self) -> None:
        """读取并验证一条新样本，更新设备状态。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        try:
            feedback = self.backend.poll()
            with self.lock:
                self.feedback = feedback
                self.feedback_time = time.monotonic()
                self.connected = True
                self.lifecycle_state = "ready"
                if self.last_error:
                    self.last_error = ""
        except Exception as exc:
            with self.lock:
                self.connected = False
                self.lifecycle_state = "disconnected"
                self.last_error = str(exc)
                self.feedback_time = 0.0
                self.feedback = {}
                should_stop = self.motion_enabled or bool(self.active_operation)
                self.motion_enabled = False
                self.cancel_event.set()
            if should_stop:
                self.backend.stop()

    def tick(self) -> None:
        """更新当前设备或操作。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        expired = False
        watchdog = False
        with self.lock:
            if self.lease_id and time.monotonic() >= self.lease_deadline:
                expired = True
                self._clear_lease_locked()
                self.motion_enabled = False
                self.stop_requested = True
                self.cancel_event.set()
            if self.velocity_command_time and time.monotonic() - self.velocity_command_time > self.config["motion"]["watchdog_sec"]:
                watchdog = True
                self.velocity_command_time = 0.0
                self.motion_enabled = False
                self.stop_requested = True
        if expired or watchdog:
            self.backend.stop()
            if self.servo_mode_active:
                try:
                    self.backend.execute("MotionControl.setServoMode", [False])
                except Exception:
                    pass
                self.servo_mode_active = False
            if self.teach_enabled:
                try:
                    self._send_teach(False)
                    self.teach_enabled = False
                    self.active_operation = ""
                except Exception:
                    pass
        self.poll_once()

    def shutdown(self) -> None:
        """关闭当前设备或操作。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        should_stop = self.motion_enabled or bool(self.active_operation) or self.teach_enabled
        self.motion_enabled = False
        self.cancel_event.set()
        if should_stop:
            self.backend.stop()
        if self.teach_enabled:
            try:
                self._send_teach(False)
            except Exception:
                pass
        with self.lock:
            self._clear_lease_locked()
        self.backend.close()

    def _require_motion_ready(self, lease_id: str) -> None:
        """检查运动就绪。

        Args:
            lease_id: 租约标识符，作为检查运动就绪所需的输入。 类型为 str。

        Returns:
            无返回值。
        """
        with self.lock:
            self._require_lease(lease_id)
            if not self.motion_enabled:
                raise RuntimeError("arm motion has been disabled")
        state = self.state()
        if not state["feedback_valid"] or not self._safe_state(state):
            raise RuntimeError("Aubo mode, safety or feedback no longer permits motion")

    def _send_teach(self, enabled: bool) -> Any:
        """发送示教。

        Args:
            enabled: 表示对应功能是否启用的布尔值。 类型为 bool。

        Returns:
            本次处理产生的结果值。 返回类型为 Any。
        """
        method = self.config["teach"]["method"]
        if method == "handguide":
            return self.backend.execute("RobotManage.handguideMode", [[], []]) if enabled else self.backend.execute("RobotManage.exitHandguideMode")
        return self.backend.execute(f"RobotManage.{method}", [bool(enabled)])

    @staticmethod
    def _safe_state(state: dict[str, Any]) -> bool:
        """筛选并返回可安全公开的机械臂状态字段。

        Args:
            state: 状态，作为执行 _safe_state 对应的处理流程所需的输入。 类型为 dict[str, Any]。

        Returns:
            布尔值，表示执行 _safe_state 对应的处理流程是否成功或成立。
        """
        return (str(state.get("mode", "")).lower() == "running" and
                str(state.get("safety_mode", "")).lower() in SAFE_MODES)

    def _effective_limits(self):
        """根据设备和配置计算当前有效的关节速度限制。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。
        """
        local_min = [float(v) for v in self.config["joints"]["min_rad"]]
        local_max = [float(v) for v in self.config["joints"]["max_rad"]]
        if isinstance(self.backend, AuboBackend):
            hardware_min, hardware_max = self.backend.hardware_min, self.backend.hardware_max
            if hardware_min is None or hardware_max is None:
                return None
            local_min = [max(a, b) for a, b in zip(local_min, hardware_min)]
            local_max = [min(a, b) for a, b in zip(local_max, hardware_max)]
        return (local_min, local_max) if all(a < b for a, b in zip(local_min, local_max)) else None

    @staticmethod
    def _validate_vector(values: list[float], label: str) -> list[float]:
        """验证vector。

        Args:
            values: 要应用、比较或转换的一组输入值。 类型为 list[float]。
            label: label，作为验证vector所需的输入。 类型为 str。

        Returns:
            布尔值，表示验证vector是否成功或成立。 返回类型为 list[float]。
        """
        if isinstance(values, (str, bytes)):
            raise ValueError(f"{label} must contain six values")
        try:
            result = [float(value) for value in values]
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{label} must contain six numeric values") from exc
        if len(result) != 6:
            raise ValueError(f"{label} must contain six values")
        if not all(math.isfinite(value) for value in result):
            raise ValueError(f"{label} must contain finite values")
        return result

    def _require_lease(self, lease_id: str) -> None:
        """确认请求持有有效的控制租约。

        Args:
            lease_id: 租约标识符，作为确认请求持有有效的控制租约所需的输入。 类型为 str。

        Returns:
            无返回值。
        """
        self._expire_lease_locked()
        if not lease_id or lease_id != self.lease_id:
            raise PermissionError("valid arm control lease is required")

    def _expire_lease_locked(self) -> None:
        """在锁保护下检查并过期控制租约。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        if self.lease_id and time.monotonic() >= self.lease_deadline:
            self._clear_lease_locked()
            self.motion_enabled = False
            self.cancel_event.set()

    def _clear_lease_locked(self) -> None:
        """在锁保护下清除控制租约。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        self.lease_id = self.lease_owner = ""
        self.lease_deadline = self.lease_ttl = 0.0
