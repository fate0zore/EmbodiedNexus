from __future__ import annotations

import copy
import json
import math
from pathlib import Path
import re
from typing import Any

import yaml


DEFAULT_CONFIG: dict[str, Any] = {
    "camera_id": "camera_front",
    "interface_mode": "both",
    "source": {
        "backend": "gemini_v4l2",
        "color_device": "/dev/v4l/by-id/usb-Orbbec_R__Orbbec_Gemini_335_CP0E853000EY-video-index0",
        "depth_device": "/dev/video0",
        "color_topic": "/camera/color/image_raw",
        "depth_topic": "/camera/depth/image_raw",
        "color_info_topic": "/camera/color/camera_info",
        "depth_info_topic": "/camera/depth/camera_info",
    },
    "capture": {
        "color_width": 640,
        "color_height": 480,
        "color_fps": 30.0,
        "color_fourcc": "YUYV",
        "depth_width": 640,
        "depth_height": 480,
        "depth_fps": 5.0,
        "depth_fourcc": "Z16 ",
        "capture_timeout_sec": 4.0,
        "reconnect_backoff_sec": 1.0,
        "color_controls": {},
        "depth_controls": {},
        "color_frame_id": "camera_color_optical_frame",
        "depth_frame_id": "camera_depth_optical_frame",
    },
    "streams": {
        "publish_color": True,
        "publish_depth": True,
        "publish_depth_color": False,
        "publish_points": True,
        "color_output_fps": 30.0,
        "depth_output_fps": 5.0,
        "web_color_fps": 15.0,
        "web_depth_fps": 5.0,
        "jpeg_quality": 85,
        "frame_stale_sec": 1.0,
        "max_clients": 8,
    },
    "pointcloud": {
        "source": "depth",
        "input_topic": "/camera/points",
        "target_frame": "",
        "min_depth_m": 0.05,
        "max_depth_m": 5.0,
        "stride": 4,
        "max_points": 50000,
        "max_pair_delta_ms": 50.0,
        "output_fps": 5.0,
        "roi": [],
    },
    "calibration": {
        "factory_file": "",
        "custom_file": "",
        "history_file": "config/gemini335_intrinsics/intrinsics_history.json",
        "depth_scale_m": 0.001,
        "depth_to_color_translation_unit": "mm",
        "require_color_calibration_for_cloud": True,
        "allow_resolution_scaling": False,
    },
    "calibration_session": {
        "root": "recordings/camera_calibration",
        "board_image_path": "",
        "board_width_m": 0.210,
        "board_height_m": 0.297,
        "min_usable_frames": 10,
        "max_rms_px": 0.5,
        "max_outlier_px": 2.0,
    },
    "web": {
        "bind_host": "0.0.0.0",
        "port": 8765,
        "auth_token_env": "EMBODIED_NEXUS_CAMERA_TOKEN",
    },
}

_RECONFIGURABLE_PATHS = {
    "streams.color_output_fps",
    "streams.depth_output_fps",
    "streams.web_color_fps",
    "streams.web_depth_fps",
    "streams.jpeg_quality",
    "pointcloud.min_depth_m",
    "pointcloud.max_depth_m",
    "pointcloud.stride",
    "pointcloud.max_points",
    "pointcloud.output_fps",
    "pointcloud.roi",
}


def _merge(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    result = copy.deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = _merge(result[key], value)
        else:
            result[key] = value
    return result


def load_config(path: str | Path | None) -> dict[str, Any]:
    config = copy.deepcopy(DEFAULT_CONFIG)
    if path:
        config_path = Path(path).expanduser().resolve()
        loaded = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
        if not isinstance(loaded, dict):
            raise ValueError("camera config root must be a YAML mapping")
        config = _merge(config, loaded)
        for key in ("factory_file", "custom_file"):
            value = str(config["calibration"].get(key, "")).strip()
            if value and not Path(value).expanduser().is_absolute():
                config["calibration"][key] = str((config_path.parent / value).resolve())
    validate_config(config)
    return config


def validate_config(config: dict[str, Any]) -> None:
    if config.get("interface_mode") not in {"ros2", "web", "both"}:
        raise ValueError("interface_mode must be ros2, web, or both")
    if config.get("source", {}).get("backend") not in {"gemini_v4l2", "ros2_topics", "mock"}:
        raise ValueError("source.backend must be gemini_v4l2, ros2_topics, or mock")
    if config.get("pointcloud", {}).get("source") not in {"depth", "ros2_topic"}:
        raise ValueError("pointcloud.source must be depth or ros2_topic")
    camera_id = str(config.get("camera_id", ""))
    if not camera_id or any(ch not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-" for ch in camera_id):
        raise ValueError("camera_id may contain only letters, digits, underscore, and hyphen")
    source = config["source"]
    if source["backend"] == "ros2_topics":
        outputs = {
            f"/embodied_nexus/cameras/{camera_id}/color/image_raw",
            f"/embodied_nexus/cameras/{camera_id}/depth/image_raw",
            f"/embodied_nexus/cameras/{camera_id}/points",
        }
        inputs = {source["color_topic"], source["depth_topic"], config["pointcloud"]["input_topic"]}
        inputs.update({source["color_info_topic"], source["depth_info_topic"]})
        if outputs & inputs:
            raise ValueError("ROS 2 input topics must differ from this camera's output topics")
    capture = config["capture"]
    for name in ("color_width", "color_height", "depth_width", "depth_height"):
        if int(capture[name]) < 1:
            raise ValueError(f"capture.{name} must be positive")
    for name in ("color_fps", "depth_fps"):
        if float(capture[name]) <= 0:
            raise ValueError(f"capture.{name} must be positive")
    for name in ("capture_timeout_sec", "reconnect_backoff_sec"):
        if float(capture[name]) <= 0:
            raise ValueError(f"capture.{name} must be positive")
    streams = config["streams"]
    if not 1 <= int(streams["jpeg_quality"]) <= 100:
        raise ValueError("streams.jpeg_quality must be in [1, 100]")
    if int(streams["max_clients"]) < 1:
        raise ValueError("streams.max_clients must be positive")
    cloud = config["pointcloud"]
    if float(cloud["min_depth_m"]) < 0 or float(cloud["max_depth_m"]) <= float(cloud["min_depth_m"]):
        raise ValueError("pointcloud depth limits are invalid")
    if int(cloud["stride"]) < 1 or int(cloud["max_points"]) < 1 or float(cloud["output_fps"]) <= 0:
        raise ValueError("pointcloud stride and max_points must be positive")
    if float(cloud["max_pair_delta_ms"]) <= 0:
        raise ValueError("pointcloud.max_pair_delta_ms must be positive")
    roi = cloud.get("roi", [])
    if not isinstance(roi, (list, tuple)) or (roi and len(roi) != 4):
        raise ValueError("pointcloud.roi must be empty or [x, y, width, height]")
    if roi:
        if any(isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)) or int(value) != value for value in roi):
            raise ValueError("pointcloud.roi values must be finite integers")
        x, y, width, height = (int(value) for value in roi)
        if x < 0 or y < 0 or width < 1 or height < 1:
            raise ValueError("pointcloud.roi origin must be nonnegative and dimensions must be positive")
    for key in ("color_controls", "depth_controls"):
        controls = capture.get(key, {})
        if not isinstance(controls, dict):
            raise ValueError(f"capture.{key} must be a mapping of V4L2 control names to numeric values")
        for name, value in controls.items():
            if not re.fullmatch(r"[A-Za-z0-9_-]+", str(name)):
                raise ValueError(f"capture.{key} contains an invalid V4L2 control name: {name}")
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
                raise ValueError(f"capture.{key}.{name} must be a finite numeric value")
    web = config["web"]
    if not 1 <= int(web["port"]) <= 65535:
        raise ValueError("web.port must be a valid TCP port")


def apply_runtime_patch(config: dict[str, Any], patch: dict[str, Any]) -> dict[str, Any]:
    """Apply only documented hot-reconfigurable leaves; reject partial updates."""
    candidate = copy.deepcopy(config)

    def visit(prefix: str, values: dict[str, Any], target: dict[str, Any]) -> None:
        for key, value in values.items():
            path = f"{prefix}.{key}" if prefix else key
            if isinstance(value, dict):
                if key not in target or not isinstance(target[key], dict):
                    raise ValueError(f"unknown configuration field: {path}")
                visit(path, value, target[key])
            else:
                if path not in _RECONFIGURABLE_PATHS:
                    raise ValueError(f"configuration field requires restart or is immutable: {path}")
                target[key] = value

    visit("", patch, candidate)
    validate_config(candidate)
    return candidate


def public_config(config: dict[str, Any]) -> dict[str, Any]:
    safe = copy.deepcopy(config)
    safe.get("web", {}).pop("auth_token", None)
    safe.get("web", {}).pop("token", None)
    return safe


def config_json(config: dict[str, Any]) -> str:
    return json.dumps(public_config(config), ensure_ascii=False, indent=2)
