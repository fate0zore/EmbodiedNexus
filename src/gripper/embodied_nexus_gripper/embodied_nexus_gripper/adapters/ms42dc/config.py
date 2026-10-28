from __future__ import annotations

import re
from typing import Any


DEFAULT_CONFIG: dict[str, Any] = {
    "device_id": "gripper_main",
    "model": "ms42dc",
    "backend": "serial",
    "backend_options": None,
    "interface_mode": "both",
    "serial": {"port": "/dev/motor_serial", "baudrate": 115200,
               "timeout_sec": 0.25, "reconnect_backoff_sec": 1.0},
    "protocol": {"device_id": 1, "sub_divide": 32, "mode": 2},
    "motion": {"open_angle_tenths": 19656, "close_angle_tenths": 19656,
               "max_relative_angle_tenths": 65535},
    "feedback": {"poll_interval_sec": 0.05, "stale_sec": 1.0,
                 "reached_value": 1, "moving_value": 0,
                 "require_moving_transition": True, "confirmed": False},
    "control": {"operation_timeout_sec": 10.0, "lease_ttl_sec": 2.0,
                "lease_renew_interval_sec": 0.5, "max_lease_ttl_sec": 30.0,
                "speed_tenths": 150},
    "mock": {"move_duration_sec": 0.2, "fail_feedback": False,
             "fail_stop": False, "disconnect": False},
    "web": {"bind_host": "0.0.0.0", "port": 8776,
            "auth_token_env": "EMBODIED_NEXUS_GRIPPER_TOKEN"},
}

_RUNTIME_PATHS = {
    "control.operation_timeout_sec", "control.lease_ttl_sec",
    "control.lease_renew_interval_sec", "control.speed_tenths",
    "feedback.stale_sec",
    "mock.fail_feedback", "mock.fail_stop", "mock.disconnect",
}
_ID_RE = re.compile(r"^[A-Za-z0-9_-]+$")


def validate_config(config: dict[str, Any]) -> None:
    """验证配置。

    Args:
        config: 设备或平台配置对象，提供本次操作所需参数。 类型为 dict[str, Any]。

    Returns:
        无返回值。
    """
    from ...config import _integer, finite_number

    def reject_unknown(value: dict[str, Any], schema: dict[str, Any], prefix: str = "") -> None:
        """拒绝配置中的未知字段。

        Args:
            value: 要应用、比较或转换的输入值。 类型为 dict[str, Any]。
            schema: schema，作为拒绝配置中的未知字段所需的输入。 类型为 dict[str, Any]。
            prefix: prefix，作为拒绝配置中的未知字段所需的输入。 类型为 str。

        Returns:
            无返回值。
        """
        for key, item in value.items():
            path = f"{prefix}.{key}" if prefix else key
            if key not in schema:
                raise ValueError(f"unknown configuration field: {path}")
            if isinstance(item, dict):
                if key == "backend_options" and config.get("backend") == "ros2_sim":
                    continue
                if not isinstance(schema[key], dict):
                    raise ValueError(f"configuration field {path} must not be a mapping")
                reject_unknown(item, schema[key], path)
    reject_unknown(config, DEFAULT_CONFIG)
    device_id = config.get("device_id", "")
    if not isinstance(device_id, str):
        raise ValueError("device_id must be a string")
    if not _ID_RE.fullmatch(device_id):
        raise ValueError("device_id may contain only letters, digits, underscore, and hyphen")
    if config.get("model") != "ms42dc":
        raise ValueError("model must be ms42dc; other gripper models use separate adapters")
    if config.get("backend") not in {"serial", "mock", "ros2_sim"}:
        raise ValueError("backend must be serial, mock, or ros2_sim")
    if config["backend"] == "ros2_sim" and not isinstance(config.get("backend_options"), dict):
        raise ValueError("ros2_sim requires backend_options")
    if config.get("interface_mode") not in {"ros2", "web", "both"}:
        raise ValueError("interface_mode must be ros2, web, or both")
    serial_cfg, protocol, motion = config["serial"], config["protocol"], config["motion"]
    if config["backend"] == "serial" and (not isinstance(serial_cfg["port"], str) or not serial_cfg["port"].strip()):
        raise ValueError("serial.port must not be empty for the serial backend")
    if _integer(serial_cfg["baudrate"], "serial.baudrate") < 1:
        raise ValueError("serial.baudrate must be positive")
    if not 0 < finite_number(serial_cfg["timeout_sec"], "serial.timeout_sec") <= 2.0:
        raise ValueError("serial.timeout_sec must be in (0, 2]")
    if finite_number(serial_cfg["reconnect_backoff_sec"], "serial.reconnect_backoff_sec") <= 0:
        raise ValueError("serial.reconnect_backoff_sec must be positive")
    for key, maximum in (("device_id", 255), ("sub_divide", 255), ("mode", 255)):
        if not 0 <= _integer(protocol[key], f"protocol.{key}") <= maximum:
            raise ValueError(f"protocol.{key} must be in [0, {maximum}]")
    for key in ("open_angle_tenths", "close_angle_tenths", "max_relative_angle_tenths"):
        if not 0 <= _integer(motion[key], f"motion.{key}") <= 65535:
            raise ValueError(f"motion.{key} must be in [0, 65535]")
    if int(motion["max_relative_angle_tenths"]) < 1:
        raise ValueError("motion.max_relative_angle_tenths must be positive")
    feedback = config["feedback"]
    for key in ("poll_interval_sec", "stale_sec"):
        if finite_number(feedback[key], f"feedback.{key}") <= 0:
            raise ValueError(f"feedback.{key} must be positive")
    for key in ("reached_value", "moving_value"):
        value = feedback.get(key)
        if value is not None and not 0 <= _integer(value, f"feedback.{key}") <= 255:
            raise ValueError(f"feedback.{key} must be null or in [0, 255]")
    for key in ("require_moving_transition", "confirmed"):
        if not isinstance(feedback[key], bool):
            raise ValueError(f"feedback.{key} must be a boolean")
    if bool(feedback["require_moving_transition"]) and feedback["moving_value"] is None:
        raise ValueError("feedback.moving_value is required when transition checking is enabled")
    if feedback["confirmed"] and feedback["reached_value"] is None:
        raise ValueError("feedback.reached_value is required when feedback semantics are confirmed")
    control = config["control"]
    for key in ("operation_timeout_sec", "lease_ttl_sec", "lease_renew_interval_sec", "max_lease_ttl_sec"):
        if finite_number(control[key], f"control.{key}") <= 0:
            raise ValueError(f"control.{key} must be positive")
    if control["lease_renew_interval_sec"] >= control["lease_ttl_sec"]:
        raise ValueError("control.lease_renew_interval_sec must be less than lease_ttl_sec")
    if control["lease_ttl_sec"] > control["max_lease_ttl_sec"]:
        raise ValueError("control.lease_ttl_sec cannot exceed control.max_lease_ttl_sec")
    if not 0 <= _integer(control["speed_tenths"], "control.speed_tenths") <= 65535:
        raise ValueError("control.speed_tenths must be in [0, 65535]")
    if finite_number(config["mock"]["move_duration_sec"], "mock.move_duration_sec") <= 0:
        raise ValueError("mock.move_duration_sec must be positive")
    for key in ("fail_feedback", "fail_stop", "disconnect"):
        if not isinstance(config["mock"][key], bool):
            raise ValueError(f"mock.{key} must be a boolean")
    if not 1 <= _integer(config["web"]["port"], "web.port") <= 65535:
        raise ValueError("web.port must be in [1, 65535]")
    for key in ("bind_host", "auth_token_env"):
        if not isinstance(config["web"][key], str) or not config["web"][key].strip():
            raise ValueError(f"web.{key} must be a nonempty string")
    if not re.fullmatch(r"[A-Z][A-Z0-9_]*", config["web"]["auth_token_env"]):
        raise ValueError("web.auth_token_env must be an uppercase environment variable name")
