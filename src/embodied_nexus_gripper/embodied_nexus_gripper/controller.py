from __future__ import annotations

import copy
import threading
import time
import uuid
from dataclasses import dataclass
from typing import Callable

from .backend import Backend, create_backend
from .config import finite_number, public_config, validate_config, validate_patch
from .protocol import Feedback, ProtocolError


@dataclass
class MotionResult:
    success: bool
    operation_id: str
    error: str = ""
    cancelled: bool = False
    stop_sent: bool = False
    hardware_stop_confirmed: bool = False


class GripperController:
    """Thread-safe, ROS-independent MS42DC state and motion controller."""

    def __init__(self, config: dict, backend: Backend | None = None) -> None:
        validate_config(config)
        self.config = copy.deepcopy(config)
        self._config_version = 1
        self.backend = backend or create_backend(self.config)
        self._lock = threading.RLock()
        self._poll_lock = threading.Lock()
        self._connected = False
        self._motion_enabled = False
        self._last_error = ""
        self._feedback: Feedback | None = None
        self._feedback_at = 0.0
        self._feedback_sequence = 0
        self._last_connect_attempt = 0.0
        self._lease_id = ""
        self._lease_owner = ""
        self._lease_deadline = 0.0
        self._lease_ttl = 0.0
        self._operation_id = ""
        self._operation_state = "idle"
        self._operation_error = ""
        self._operation_target: float | None = None
        self._operation_stop_event: threading.Event | None = None
        self._reserved_goal_id = ""
        self._stop_sent = False
        self._hardware_stop_confirmed = False
        self._try_connect()

    def _try_connect(self) -> None:
        try:
            self.backend.open()
        except Exception as exc:
            with self._lock:
                self._connected = False
                self._last_error = str(exc)
                self._last_connect_attempt = time.monotonic()
        else:
            with self._lock:
                self._connected = True
                self._last_error = ""
                self._last_connect_attempt = time.monotonic()

    def poll_once(self) -> bool:
        """Read and validate one new sample; return true only for a fresh sample."""
        with self._poll_lock:
            with self._lock:
                connected = self._connected
                retry_after = float(self.config["serial"]["reconnect_backoff_sec"])
                should_retry = not connected and time.monotonic() - self._last_connect_attempt >= retry_after
            if not connected:
                if should_retry:
                    self._try_connect()
                with self._lock:
                    if not self._connected:
                        return False
            try:
                feedback = self.backend.read_feedback()
                if feedback is None:
                    with self._lock:
                        self._last_error = "feedback timeout: no complete response frame"
                    return False
            except ProtocolError as exc:
                with self._lock:
                    self._last_error = str(exc)
                return False
            except Exception as exc:
                with self._lock:
                    self._last_error = str(exc)
                    self._connected = False
                    self._motion_enabled = False
                    self._clear_lease_locked()
                try:
                    self.backend.close()
                except Exception:
                    pass
                return False
            with self._lock:
                self._feedback = feedback
                self._feedback_at = time.monotonic()
                self._feedback_sequence += 1
                if self._operation_state not in {"running", "stopping"}:
                    self._last_error = ""
                return True

    def state(self) -> dict:
        with self._lock:
            age = None if not self._feedback_at else max(time.monotonic() - self._feedback_at, 0.0)
            if not self._connected:
                lifecycle = "faulted" if self._last_error else "connecting"
            elif self._operation_state in {"running", "stopping"}:
                lifecycle = "running"
            else:
                lifecycle = "ready"
            feedback = self._feedback
            return {
                "device_id": self.config["device_id"],
                "backend": self.config["backend"],
                "lifecycle_state": lifecycle,
                "connected": self._connected,
                "motion_enabled": self._motion_enabled,
                "operation_id": self._operation_id,
                "operation_state": self._operation_state,
                "operation_error": self._operation_error,
                "last_error": self._last_error,
                "config_version": self._config_version,
                "command_target_known": self._operation_target is not None,
                "command_target_degrees": self._operation_target,
                "feedback_valid": bool(feedback and age is not None and age <= float(self.config["feedback"]["stale_sec"])),
                "feedback_sequence": self._feedback_sequence,
                "feedback_device_id": feedback.device_id if feedback else 0,
                "feedback_reached": feedback.reached if feedback else 0,
                "feedback_speed_raw": feedback.speed_raw if feedback else 0,
                "feedback_angle_raw": feedback.angle_raw if feedback else 0,
                "feedback_age_sec": age,
                "lease_owner": self._lease_owner if self._lease_valid_locked() else "",
                "stop_sent": self._stop_sent,
                "hardware_stop_confirmed": self._hardware_stop_confirmed,
            }

    def descriptor(self) -> dict:
        return {
            "device_id": self.config["device_id"], "model": "ms42dc",
            "driver": "embodied_nexus_gripper", "driver_version": "0.1.0",
            "interface_version": "0.1.0",
            "capabilities": [
                {"name": "open", "parameters": [], "units": None},
                {"name": "close", "parameters": [], "units": None},
                {"name": "move_relative", "parameters": [{
                    "name": "angle_degrees", "unit": "deg",
                    "min": -int(self.config["motion"]["max_relative_angle_tenths"]) / 10.0,
                    "max": int(self.config["motion"]["max_relative_angle_tenths"]) / 10.0,
                }], "units": None},
                {"name": "stop", "parameters": [], "units": None},
                {"name": "enable_motion", "parameters": [], "units": None},
            ],
            "feedback": {
                "raw_motor_angle": True,
                "opening_width_m": None, "finger_position": None, "force_n": None,
                "completion_confirmed": bool(self.config["feedback"]["confirmed"]),
            },
        }

    def configuration(self) -> dict:
        with self._lock:
            result = public_config(self.config)
            result["config_version"] = self._config_version
            return result

    def validate_configuration(self, patch: dict) -> dict:
        with self._lock:
            if "mock" in patch and self.config["backend"] != "mock":
                raise ValueError("mock fault injection fields require backend=mock")
            result = public_config(validate_patch(self.config, patch))
            result["config_version"] = self._config_version
            return result

    def apply_configuration(self, patch: dict, lease_id: str) -> dict:
        with self._lock:
            self._require_lease_locked(lease_id)
            if self._operation_state == "running" or self._reserved_goal_id:
                raise RuntimeError("configuration cannot change while a motion is active")
            candidate = validate_patch(self.config, patch)
            for section in ("control", "feedback"):
                if section in patch:
                    for key in patch[section]:
                        if key not in {"operation_timeout_sec", "lease_ttl_sec", "lease_renew_interval_sec", "speed_tenths", "stale_sec"}:
                            raise ValueError(f"configuration field requires restart or is immutable: {section}.{key}")
            if "mock" in patch and self.config["backend"] != "mock":
                raise ValueError("mock fault injection fields require backend=mock")
            self.config = candidate
            self._config_version += 1
            if hasattr(self.backend, "config"):
                self.backend.config = candidate
            result = public_config(candidate)
            result["config_version"] = self._config_version
            return result

    def acquire_control(self, owner: str, ttl_sec: float | None = None) -> tuple[str, float]:
        owner = str(owner).strip()
        if not owner:
            raise ValueError("control owner must not be empty")
        with self._lock:
            self._expire_lease_locked()
            if self._lease_id:
                raise RuntimeError(f"control is already held by {self._lease_owner}")
            if self._operation_state == "running" or self._reserved_goal_id:
                raise RuntimeError("cannot transfer control while a motion is active")
            ttl = float(self.config["control"]["lease_ttl_sec"] if ttl_sec is None else finite_number(ttl_sec, "ttl_sec"))
            if not 0 < ttl <= float(self.config["control"]["max_lease_ttl_sec"]):
                raise ValueError("ttl_sec is outside the configured lease range")
            self._lease_id = str(uuid.uuid4())
            self._lease_owner = owner
            self._lease_ttl = ttl
            self._lease_deadline = time.monotonic() + ttl
            return self._lease_id, ttl

    def renew_control(self, lease_id: str) -> float:
        with self._lock:
            self._require_lease_locked(lease_id)
            self._lease_deadline = time.monotonic() + self._lease_ttl
            return self._lease_ttl

    def release_control(self, lease_id: str) -> None:
        with self._lock:
            self._require_lease_locked(lease_id)
            active = self._operation_state == "running"
        if active:
            self.stop("control lease released")
        with self._lock:
            self._motion_enabled = False
            self._clear_lease_locked()

    def set_motion_enabled(self, lease_id: str, enabled: bool) -> None:
        if enabled:
            self.poll_once()
            with self._lock:
                self._require_lease_locked(lease_id)
                if not self._connected:
                    raise RuntimeError("gripper is not connected")
                if not self.config["feedback"]["confirmed"]:
                    raise RuntimeError("MS42DC feedback semantics are not confirmed in configuration")
                if self.config["feedback"]["reached_value"] is None:
                    raise RuntimeError("feedback.reached_value must be configured before motion is enabled")
                if not self.state()["feedback_valid"]:
                    raise RuntimeError("a fresh valid MS42DC feedback sample is required before motion is enabled")
                self._motion_enabled = True
                return
        with self._lock:
            self._require_lease_locked(lease_id)
            active = self._operation_state == "running"
            self._motion_enabled = False
        if active:
            self.stop("motion disabled")

    def _lease_valid_locked(self, lease_id: str | None = None) -> bool:
        self._expire_lease_locked()
        return bool(self._lease_id and time.monotonic() < self._lease_deadline and
                    (lease_id is None or self._lease_id == lease_id))

    def _expire_lease_locked(self) -> bool:
        if self._lease_id and time.monotonic() >= self._lease_deadline:
            self._clear_lease_locked()
            self._motion_enabled = False
            return True
        return False

    def _clear_lease_locked(self) -> None:
        self._lease_id = self._lease_owner = ""
        self._lease_deadline = self._lease_ttl = 0.0

    def _require_lease_locked(self, lease_id: str) -> None:
        if not self._lease_valid_locked(lease_id):
            raise PermissionError("valid control lease required")

    def _motion_parameters(self, command: str, angle_degrees: float) -> tuple[int, int, float]:
        command = str(command).strip().lower()
        if command not in {"open", "close", "relative"}:
            raise ValueError("command must be open, close, or relative")
        requested = finite_number(angle_degrees, "angle_degrees")
        if command == "open":
            direction = 0
            angle_raw = int(self.config["motion"]["open_angle_tenths"])
            target_degrees = -angle_raw / 10.0
        elif command == "close":
            direction = 1
            angle_raw = int(self.config["motion"]["close_angle_tenths"])
            target_degrees = angle_raw / 10.0
        else:
            if requested == 0:
                raise ValueError("relative angle_degrees must be nonzero")
            angle_raw = int(round(abs(requested) * 10.0))
            maximum = int(self.config["motion"]["max_relative_angle_tenths"])
            if angle_raw < 1 or angle_raw > maximum:
                raise ValueError(f"relative angle must be in [{-maximum / 10.0:g}, {maximum / 10.0:g}] degrees")
            direction = 1 if requested > 0 else 0
            target_degrees = requested
        if angle_raw < 1:
            raise ValueError(f"{command} angle is configured as zero")
        return direction, angle_raw, target_degrees

    def reserve_motion_goal(self, lease_id: str, command: str, angle_degrees: float, operation_id: str) -> None:
        with self._lock:
            self._require_lease_locked(lease_id)
            if not self._motion_enabled:
                raise PermissionError("motion is disabled")
            if not self._connected:
                raise RuntimeError("gripper is not connected")
            if self._operation_state == "running" or self._reserved_goal_id:
                raise RuntimeError("another gripper operation is active")
            self._motion_parameters(command, angle_degrees)
            self._reserved_goal_id = operation_id

    def cancel_reserved_goal(self, operation_id: str) -> None:
        with self._lock:
            if self._reserved_goal_id == operation_id:
                self._reserved_goal_id = ""

    def move(
        self, lease_id: str, command: str, angle_degrees: float = 0.0,
        cancel_requested: Callable[[], bool] | None = None,
        feedback_callback: Callable[[dict], None] | None = None,
        operation_id: str = "",
    ) -> MotionResult:
        direction, angle_raw, target_degrees = self._motion_parameters(command, angle_degrees)
        operation_id = operation_id.strip() or str(uuid.uuid4())
        stop_event = threading.Event()
        with self._lock:
            if self._reserved_goal_id and self._reserved_goal_id != operation_id:
                raise RuntimeError("another gripper operation is active")
            if self._reserved_goal_id == operation_id:
                self._reserved_goal_id = ""
            self._require_lease_locked(lease_id)
            if not self._motion_enabled:
                raise PermissionError("motion is disabled")
            if not self._connected:
                raise RuntimeError("gripper is not connected")
            if self._operation_state == "running":
                raise RuntimeError("another gripper operation is active")
            self._operation_id = operation_id
            self._operation_state = "running"
            self._operation_error = ""
            self._operation_target = target_degrees
            self._operation_stop_event = stop_event
            self._stop_sent = False
            self._hardware_stop_confirmed = False

        # Serialize against polling and establish a feedback sequence boundary
        # immediately before writing the command. Pre-command state can never
        # satisfy this operation's completion condition.
        try:
            with self._poll_lock:
                with self._lock:
                    self._require_lease_locked(lease_id)
                    baseline = self._feedback_sequence
                self.backend.command(direction, angle_raw, int(self.config["control"]["speed_tenths"]))
        except Exception as exc:
            with self._lock:
                self._operation_state = "failed"
                self._operation_error = str(exc)
                self._last_error = str(exc)
                self._motion_enabled = False
            stop = self.stop("motion command write failed")
            return MotionResult(False, operation_id, str(exc), False,
                                stop["stop_sent"], stop["hardware_stop_confirmed"])

        started = time.monotonic()
        deadline = started + float(self.config["control"]["operation_timeout_sec"])
        moving_seen = False
        last_reported = -1
        while time.monotonic() < deadline:
            if cancel_requested and cancel_requested():
                stop_result = self.stop("operation cancelled")
                with self._lock:
                    self._operation_state = "cancelled"
                    self._operation_error = "operation cancelled"
                return MotionResult(False, operation_id, "operation cancelled", True,
                                    stop_result["stop_sent"], stop_result["hardware_stop_confirmed"])
            with self._lock:
                lease_ok = self._lease_valid_locked(lease_id)
                enabled = self._motion_enabled
                stopped = stop_event.is_set()
            if stopped:
                with self._lock:
                    self._operation_state = "cancelled"
                    self._operation_error = "operation stopped"
                    return MotionResult(False, operation_id, "operation stopped", True,
                                        self._stop_sent, self._hardware_stop_confirmed)
            if not lease_ok or not enabled:
                stop_result = self.stop("control lease expired or motion disabled")
                message = "control lease expired or motion disabled"
                with self._lock:
                    self._operation_state = "failed"
                    self._operation_error = message
                return MotionResult(False, operation_id, message, False,
                                    stop_result["stop_sent"], stop_result["hardware_stop_confirmed"])

            self.poll_once()
            with self._lock:
                feedback = self._feedback
                sequence = self._feedback_sequence
                age = None if not self._feedback_at else time.monotonic() - self._feedback_at
                current_state = copy.deepcopy(self.state())
            new_sample = False
            if sequence > baseline and sequence != last_reported and feedback is not None:
                new_sample = True
                last_reported = sequence
                feedback_cfg = self.config["feedback"]
                if feedback_cfg["moving_value"] is not None and feedback.reached == int(feedback_cfg["moving_value"]):
                    moving_seen = True
                transition_ok = not bool(feedback_cfg["require_moving_transition"]) or moving_seen
                with self._lock:
                    still_authorized = self._lease_valid_locked(lease_id) and self._motion_enabled
                if feedback.reached == int(feedback_cfg["reached_value"]) and transition_ok and still_authorized:
                    with self._lock:
                        self._operation_state = "succeeded"
                        self._operation_error = ""
                    return MotionResult(True, operation_id)
            if age is not None and age > float(self.config["feedback"]["stale_sec"]):
                message = "feedback became stale before the operation completed"
                stop_result = self.stop(message)
                with self._lock:
                    self._operation_state = "failed"
                    self._operation_error = message
                return MotionResult(False, operation_id, message, False,
                                    stop_result["stop_sent"], stop_result["hardware_stop_confirmed"])
            if feedback_callback and new_sample:
                feedback_callback(current_state)
            time.sleep(float(self.config["feedback"]["poll_interval_sec"]))

        message = "operation timed out waiting for a fresh confirmed reached feedback"
        stop_result = self.stop(message)
        with self._lock:
            self._operation_state = "failed"
            self._operation_error = message
        return MotionResult(False, operation_id, message, False,
                            stop_result["stop_sent"], stop_result["hardware_stop_confirmed"])

    def stop(self, reason: str = "stop requested") -> dict:
        reason = str(reason or "stop requested")
        with self._lock:
            event = self._operation_stop_event
            if event:
                event.set()
            self._motion_enabled = False
            active_operation = bool(self._operation_id and self._operation_state == "running")
            if active_operation:
                self._operation_state = "stopping"
                self._operation_error = reason
        sent = confirmed = False
        error = ""
        with self._poll_lock:
            try:
                with self._lock:
                    connected = self._connected
                if not connected:
                    raise OSError("gripper is not connected; stop could not be sent")
                self.backend.stop()
                sent = True
                feedback = self.backend.read_feedback()
                if feedback is not None:
                    with self._lock:
                        self._feedback = feedback
                        self._feedback_at = time.monotonic()
                        self._feedback_sequence += 1
                        cfg = self.config["feedback"]
                        expected = cfg["reached_value"]
                        confirmed = feedback.speed_raw == 0 and expected is not None and feedback.reached == int(expected)
            except Exception as exc:
                error = str(exc)
        with self._lock:
            self._stop_sent = sent
            self._hardware_stop_confirmed = confirmed
            if error:
                self._last_error = error
            elif sent:
                self._last_error = ""
            if self._operation_state == "stopping":
                self._operation_state = "cancelled"
        return {"success": sent, "stop_sent": sent, "hardware_stop_confirmed": confirmed, "error": error}

    def shutdown(self) -> None:
        try:
            if self._connected:
                self.stop("device shutting down")
        finally:
            try:
                self.backend.close()
            finally:
                with self._lock:
                    self._connected = False
                    self._motion_enabled = False
                    self._clear_lease_locked()

\n