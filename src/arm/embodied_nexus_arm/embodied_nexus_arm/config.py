from __future__ import annotations

import copy
import json
import math
import re
from pathlib import Path
from typing import Any

import yaml


def default_config_path() -> Path:
    return Path(__file__).resolve().parents[1] / "config" / "aubo_i5_mock.yaml"


def load_config(path: str | Path | None = None) -> dict[str, Any]:
    config = yaml.safe_load(Path(path or default_config_path()).read_text(encoding="utf-8"))
    if not isinstance(config, dict):
        raise ValueError("arm configuration must be a mapping")
    validate_config(config)
    return config


def validate_config(config: dict[str, Any]) -> None:
    schema = {"device_id": str, "model": str, "backend": str, "interface_mode": str,
              "rpc": dict, "joints": dict, "motion": dict, "feedback": dict,
              "control": dict, "teach": dict, "web": dict, "mock": dict}
    _reject_unknown(config, schema)
    if not config.get("device_id") or config.get("model") != "aubo_i5":
        raise ValueError("device_id is required and model must be aubo_i5")
    if config.get("backend") not in {"mock", "aubo_jsonrpc"}:
        raise ValueError("backend must be mock or aubo_jsonrpc")
    if config.get("interface_mode") not in {"ros2", "web", "both"}:
        raise ValueError("interface_mode must be ros2, web or both")
    joints = config["joints"]
    if set(joints) != {"names", "min_rad", "max_rad"}:
        raise ValueError("joints requires names, min_rad and max_rad")
    names, lower, upper = joints["names"], joints["min_rad"], joints["max_rad"]
    if not isinstance(names, list) or len(names) != 6 or len(set(names)) != 6 or not all(names):
        raise ValueError("joints.names must contain six unique nonempty names")
    if len(lower) != 6 or len(upper) != 6:
        raise ValueError("joint position limits must contain six values")
    for low, high in zip(lower, upper):
        if not math.isfinite(float(low)) or not math.isfinite(float(high)) or float(low) >= float(high):
            raise ValueError("each joint lower limit must be finite and below its upper limit")
    for group, fields in {
        "rpc": ("host", "port", "timeout_sec", "reconnect_sec"),
        "motion": ("max_speed_rad_sec", "max_acceleration_rad_sec2",
                   "speed_joint_acceleration_rad_sec2", "stop_acceleration_rad_sec2",
                   "speed_joint_duration_sec", "watchdog_sec", "operation_timeout_sec",
                   "goal_tolerance_rad", "stable_sec"),
        "feedback": ("rate_hz", "stale_sec"),
        "control": ("lease_ttl_sec", "max_lease_ttl_sec"),
        "teach": ("method",), "web": ("bind_host", "port", "auth_token_env"),
    }.items():
        if not isinstance(config.get(group), dict) or set(config[group]) != set(fields):
            raise ValueError(f"{group} configuration fields must be {', '.join(fields)}")
    if int(config["rpc"]["port"]) not in range(1, 65536):
        raise ValueError("rpc.port must be between 1 and 65535")
    for group, field in (("rpc", "timeout_sec"), ("rpc", "reconnect_sec"),
                         ("feedback", "rate_hz"), ("feedback", "stale_sec"),
                         ("control", "lease_ttl_sec"), ("control", "max_lease_ttl_sec")):
        _positive(config[group][field], f"{group}.{field}")
    for field, value in config["motion"].items():
        _positive(value, f"motion.{field}")
    if config["control"]["lease_ttl_sec"] > config["control"]["max_lease_ttl_sec"]:
        raise ValueError("control.lease_ttl_sec exceeds its maximum")
    if config["teach"]["method"] not in {"freedrive", "backdrive", "handguide"}:
        raise ValueError("teach.method must be freedrive, backdrive or handguide")
    web = config["web"]
    if int(web["port"]) not in range(1, 65536) or not re.fullmatch(r"[A-Z][A-Z0-9_]*", str(web["auth_token_env"])):
        raise ValueError("invalid web port or auth_token_env")
    if config["backend"] == "mock":
        mock = config.get("mock")
        if not isinstance(mock, dict) or set(mock) != {"disconnect", "fail_command", "fail_stop", "mode", "safety"}:
            raise ValueError("mock backend requires disconnect, fail_command, fail_stop, mode and safety")
    elif "mock" in config:
        raise ValueError("mock settings are not allowed for the real backend")


def _positive(value: Any, label: str) -> None:
    if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or value <= 0:
        raise ValueError(f"{label} must be finite and positive")


def _reject_unknown(value: dict, schema: dict, prefix: str = "") -> None:
    unknown = sorted(set(value) - set(schema))
    if unknown:
        raise ValueError(f"unknown configuration field {prefix + '.' if prefix else ''}{unknown[0]}")


def config_json(config: dict[str, Any]) -> str:
    return json.dumps(copy.deepcopy(config), ensure_ascii=False, allow_nan=False, indent=2)
