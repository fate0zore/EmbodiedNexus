from __future__ import annotations

import copy
import math
import re
from pathlib import Path
from typing import Any
from urllib.parse import unquote

import yaml


DEFAULT_CORE = {
    "bind_host": "0.0.0.0", "port": 8780,
    "auth_token_env": "EMBODIED_NEXUS_CORE_TOKEN",
    "start_timeout_sec": 30.0, "shutdown_timeout_sec": 5.0,
    "poll_interval_sec": 1.0, "gateway_unavailable_sec": 3.0,
}
DEVICE_TYPES = {
    "base": ("embodied_nexus_base", "base.launch.py", "device_id", "bases"),
    "arm": ("embodied_nexus_arm", "arm.launch.py", "device_id", "arms"),
    "gripper": ("embodied_nexus_gripper", "gripper.launch.py", "device_id", "grippers"),
    "camera": ("embodied_nexus_camera", "camera.launch.py", "camera_id", "cameras"),
    "lidar": ("embodied_nexus_lidar", "lidar.launch.py", "device_id", "lidars"),
}
_ID = re.compile(r"^[A-Za-z][A-Za-z0-9_-]*$")
_ENV = re.compile(r"^[A-Z][A-Z0-9_]*$")


def default_config_path() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory("embodied_nexus_core")) / "config" / "platform_mock.yaml"
    except Exception:
        return Path(__file__).resolve().parents[1] / "config" / "platform_mock.yaml"


def resolve_config_reference(reference: str, platform_path: Path) -> Path:
    if not isinstance(reference, str) or not reference.strip():
        raise ValueError("device config must be a non-empty path")
    value = reference.strip()
    if value.startswith("package://"):
        rest = value[len("package://"):]
        package, separator, relative = rest.partition("/")
        if not separator or not package or not relative:
            raise ValueError(f"invalid package config reference: {value}")
        try:
            from ament_index_python.packages import get_package_share_directory
            root = Path(get_package_share_directory(package)).resolve()
        except Exception as exc:
            raise ValueError(f"ROS package is not installed: {package}") from exc
        candidate = (root / unquote(relative)).resolve()
        if not candidate.is_relative_to(root):
            raise ValueError(f"package config path escapes package share: {value}")
        return candidate
    path = Path(value).expanduser()
    return (path if path.is_absolute() else platform_path.parent / path).resolve()


def load_platform(path: str | Path | None = None, *, check_packages: bool = True) -> dict[str, Any]:
    platform_path = Path(path or default_config_path()).expanduser().resolve()
    raw = yaml.safe_load(platform_path.read_text(encoding="utf-8"))
    if not isinstance(raw, dict) or set(raw) != {"core", "devices"}:
        raise ValueError("platform root must contain exactly 'core' and 'devices'")
    core = dict(DEFAULT_CORE)
    if not isinstance(raw["core"], dict) or set(raw["core"]) - set(DEFAULT_CORE):
        raise ValueError("core config contains unknown fields")
    core.update(raw["core"])
    if not isinstance(core["bind_host"], str) or not core["bind_host"].strip():
        raise ValueError("core.bind_host must be a non-empty host")
    if isinstance(core["port"], bool) or not isinstance(core["port"], int) or not 1 <= core["port"] <= 65535:
        raise ValueError("core.port must be in [1, 65535]")
    if not _ENV.fullmatch(str(core["auth_token_env"])):
        raise ValueError("core.auth_token_env must be an environment variable name")
    for name in ("start_timeout_sec", "shutdown_timeout_sec", "poll_interval_sec", "gateway_unavailable_sec"):
        value = core[name]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"core.{name} must be positive")
    if not isinstance(raw["devices"], list):
        raise ValueError("devices must be a list")
    seen: set[str] = set()
    devices = []
    for index, item in enumerate(raw["devices"]):
        if not isinstance(item, dict) or set(item) - {"id", "type", "config", "enabled"}:
            raise ValueError(f"devices[{index}] has an invalid shape")
        device_id, kind = item.get("id"), item.get("type")
        if not isinstance(device_id, str) or not _ID.fullmatch(device_id):
            raise ValueError(f"devices[{index}].id must use letters, digits, '_' or '-'")
        if device_id in seen:
            raise ValueError(f"duplicate device id: {device_id}")
        seen.add(device_id)
        if not isinstance(kind, str) or kind not in DEVICE_TYPES:
            raise ValueError(f"devices[{index}].type is unsupported: {kind}")
        enabled = item.get("enabled", True)
        if not isinstance(enabled, bool):
            raise ValueError(f"devices[{index}].enabled must be boolean")
        config_path = resolve_config_reference(item.get("config"), platform_path)
        if not config_path.is_file():
            raise ValueError(f"device config does not exist: {config_path}")
        device_config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
        identity_key = DEVICE_TYPES[kind][2]
        if not isinstance(device_config, dict) or device_config.get(identity_key) != device_id:
            raise ValueError(f"{config_path} must declare {identity_key}: {device_id}")
        if check_packages:
            _validate_ros_package(kind)
        devices.append({"id": device_id, "type": kind, "config": str(config_path),
                        "enabled": enabled})
    return {"config_file": str(platform_path), "core": core, "devices": devices}


def _validate_ros_package(kind: str) -> None:
    package, launch_file = DEVICE_TYPES[kind][:2]
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory(package))
    except Exception as exc:
        raise ValueError(f"ROS package is not installed: {package}") from exc
    if not (share / "launch" / launch_file).is_file():
        raise ValueError(f"ROS launch file is not installed: {package}/{launch_file}")


def public_platform(config: dict[str, Any]) -> dict[str, Any]:
    safe = copy.deepcopy(config)
    safe["core"].pop("auth_token_env", None)
    return safe
