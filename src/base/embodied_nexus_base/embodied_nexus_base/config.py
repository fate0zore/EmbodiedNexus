from __future__ import annotations

import copy
import json
import math
import re
from pathlib import Path
from typing import Any

import yaml


DEFAULT_CONFIG: dict[str, Any] = {
    "device_id": "base_main", "model": "scout_2", "backend": "mock",
    "interface_mode": "both",
    "serial": {"port": "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
               "usb_baudrate": 2000000, "can_bitrate": 500000,
               "frame_type": "standard", "adapter_mode": "normal"},
    "frames": {"odom": "odom", "base": "base_link", "publish_tf": False},
    "motion": {"max_linear_mps": 1.5, "max_angular_rps": 1.5,
               "default_linear_mps": 0.16, "default_angular_rps": 0.60,
               "position_tolerance_m": 0.02, "yaw_tolerance_deg": 0.5,
               "send_rate_hz": 50.0, "command_timeout_sec": 0.5,
               "operation_timeout_sec": 30.0},
    "feedback": {"stale_sec": 1.0, "control_mode_timeout_sec": 1.0},
    "control": {"lease_ttl_sec": 2.0, "max_lease_ttl_sec": 30.0},
    "mock": {"disconnect": False, "fail_command": False, "fail_stop": False,
             "vehicle_state": 0, "control_mode": 1, "battery_voltage": 24.0,
             "error_code": 0},
    "web": {"bind_host": "0.0.0.0", "port": 8778,
            "auth_token_env": "EMBODIED_NEXUS_BASE_TOKEN"},
}

RUNTIME_PATHS = {"motion.command_timeout_sec", "motion.operation_timeout_sec",
                 "feedback.stale_sec", "mock.disconnect", "mock.fail_command",
                 "mock.fail_stop", "mock.vehicle_state", "mock.control_mode",
                 "mock.battery_voltage", "mock.error_code"}
_ID_RE = re.compile(r"^[A-Za-z0-9_-]+$")
_ENV_RE = re.compile(r"^[A-Z][A-Z0-9_]*$")


def _merge(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    result = copy.deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = _merge(result[key], value)
        else:
            result[key] = value
    return result


def load_config(path: str | Path | None = None) -> dict[str, Any]:
    config = copy.deepcopy(DEFAULT_CONFIG)
    if path:
        loaded = yaml.safe_load(Path(path).expanduser().read_text(encoding="utf-8")) or {}
        if not isinstance(loaded, dict):
            raise ValueError("base config root must be a YAML mapping")
        config = _merge(config, loaded)
    validate_config(config)
    return config


def validate_config(config: dict[str, Any]) -> None:
    _reject_unknown(config, DEFAULT_CONFIG)
    if not isinstance(config.get("device_id"), str) or not _ID_RE.fullmatch(config["device_id"]):
        raise ValueError("device_id must contain only letters, digits, '_' or '-'")
    if config.get("model") != "scout_2":
        raise ValueError("model must be scout_2")
    if config.get("backend") not in {"mock", "scout_waveshare_serial"}:
        raise ValueError("backend must be mock or scout_waveshare_serial")
    if config.get("interface_mode") not in {"ros2", "web", "both"}:
        raise ValueError("interface_mode must be ros2, web or both")
    serial = config["serial"]
    if not isinstance(serial.get("port"), str) or not serial["port"]:
        raise ValueError("serial.port must be a non-empty string")
    if isinstance(serial.get("usb_baudrate"), bool) or not isinstance(serial.get("usb_baudrate"), int) or serial["usb_baudrate"] <= 0:
        raise ValueError("serial.usb_baudrate must be a positive integer")
    if serial.get("can_bitrate") not in {1000000, 800000, 500000, 400000, 250000, 200000, 125000, 100000, 50000, 20000, 10000, 5000}:
        raise ValueError("serial.can_bitrate is unsupported")
    if serial.get("frame_type") not in {"standard", "extended"}:
        raise ValueError("serial.frame_type must be standard or extended")
    frames = config["frames"]
    if not frames.get("odom") or not frames.get("base"):
        raise ValueError("frame names must be non-empty")
    motion = config["motion"]
    positive = ("max_linear_mps", "max_angular_rps", "default_linear_mps", "default_angular_rps",
                "position_tolerance_m", "yaw_tolerance_deg", "send_rate_hz",
                "command_timeout_sec", "operation_timeout_sec")
    for name in positive:
        value = motion[name]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"motion.{name} must be a finite positive number")
    if motion["default_linear_mps"] > motion["max_linear_mps"] or motion["default_angular_rps"] > motion["max_angular_rps"]:
        raise ValueError("default speeds cannot exceed their configured maximum")
    if motion["max_linear_mps"] > 32.767 or motion["max_angular_rps"] > 32.767:
        raise ValueError("Scout velocity limits must fit the signed millivalue protocol fields")
    for group, field in (("feedback", "stale_sec"), ("feedback", "control_mode_timeout_sec"),
                         ("control", "lease_ttl_sec"), ("control", "max_lease_ttl_sec")):
        value = config[group][field]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"{group}.{field} must be finite and positive")
    if config["control"]["lease_ttl_sec"] > config["control"]["max_lease_ttl_sec"]:
        raise ValueError("control.lease_ttl_sec exceeds its maximum")
    if isinstance(config["web"]["port"], bool) or not isinstance(config["web"]["port"], int) or not 1 <= config["web"]["port"] <= 65535:
        raise ValueError("web.port must be between 1 and 65535")
    if not _ENV_RE.fullmatch(str(config["web"]["auth_token_env"])):
        raise ValueError("web.auth_token_env must be an environment variable name")


def _reject_unknown(value: dict[str, Any], schema: dict[str, Any], prefix: str = "") -> None:
    unknown = sorted(set(value) - set(schema))
    if unknown:
        label = f"{prefix}." if prefix else ""
        raise ValueError(f"unknown configuration field {label}{unknown[0]}")
    for key, child_schema in schema.items():
        child = value.get(key)
        if child is not None and isinstance(child_schema, dict) and isinstance(child, dict):
            _reject_unknown(child, child_schema, f"{prefix}.{key}" if prefix else key)


def validate_patch(config: dict[str, Any], patch: dict[str, Any]) -> dict[str, Any]:
    if not isinstance(patch, dict):
        raise ValueError("configuration patch must be a mapping")
    candidate = _merge(config, patch)
    validate_config(candidate)
    changed = _leaf_paths(config, candidate)
    unsupported = sorted(changed - RUNTIME_PATHS)
    if unsupported:
        raise ValueError("configuration fields require restart: " + ", ".join(unsupported))
    # Security and movement limits are fixed at process start and cannot be raised live.
    for path in ("motion.max_linear_mps", "motion.max_angular_rps"):
        if _get(candidate, path) > _get(config, path):
            raise ValueError(f"{path} cannot be increased at runtime")
    return candidate


def _leaf_paths(before: dict, after: dict, prefix: str = "") -> set[str]:
    changed: set[str] = set()
    for key in before.keys() | after.keys():
        path = f"{prefix}.{key}" if prefix else key
        old, new = before.get(key), after.get(key)
        if isinstance(old, dict) and isinstance(new, dict):
            changed |= _leaf_paths(old, new, path)
        elif old != new:
            changed.add(path)
    return changed


def _get(value: dict, path: str):
    for part in path.split("."):
        value = value[part]
    return value


def public_config(config: dict[str, Any]) -> dict[str, Any]:
    return copy.deepcopy(config)


def config_json(config: dict[str, Any]) -> str:
    return json.dumps(public_config(config), ensure_ascii=False, allow_nan=False, indent=2)
