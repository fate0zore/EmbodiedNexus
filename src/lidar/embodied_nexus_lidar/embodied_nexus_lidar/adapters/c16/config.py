from __future__ import annotations

import ipaddress
import math
import re
from typing import Any


DEFAULT_CONFIG: dict[str, Any] = {
    "device_id": "lidar_main",
    "model": "lslidar_c16",
    "backend": "mock",
    "interface_mode": "both",
    "source": {
        "pointcloud_topic": "/lslidar_point_cloud",
        "output_topic": "points",
        "scan_output_topic": "scan",
    },
    "driver": {
        "device_ip": "192.168.1.200",
        "msop_port": 2368,
        "difop_port": 2369,
        "command_port": 2368,
        "pcl_type": False,
        "add_multicast": False,
        "group_ip": "224.1.1.2",
        "use_time_service": False,
        "min_range_m": 0.15,
        "max_range_m": 200.0,
        "angle_disable_min": 0,
        "angle_disable_max": 0,
        "distance_unit": 0.4,
        "horizontal_angle_resolution_deg": 0.18,
        "frame_id": "lidar_link",
        "coordinate_opt": False,
    },
    "filter": {
        "target_frame": "",
        "min_range_m": 0.15,
        "max_range_m": 200.0,
        "min_height_m": -10.0,
        "max_height_m": 10.0,
    },
    "scan": {
        "min_height_m": -0.35,
        "max_height_m": 0.35,
        "angle_min_rad": -math.pi,
        "angle_max_rad": math.pi,
        "angle_increment_rad": math.pi / 180.0,
        "range_min_m": 0.15,
        "range_max_m": 18.0,
        "use_inf": True,
    },
    "streams": {
        "stale_sec": 1.0,
        "web_points_fps": 5.0,
        "web_scan_fps": 10.0,
        "max_clients": 8,
    },
    "control": {
        "difop_stale_sec": 2.0,
        "lease_ttl_sec": 2.0,
        "lease_renew_interval_sec": 0.5,
        "max_lease_ttl_sec": 30.0,
    },
    "mock": {"disconnect": False, "fail_stop": False, "fail_command": False},
    "web": {
        "bind_host": "0.0.0.0",
        "port": 8777,
        "auth_token_env": "EMBODIED_NEXUS_LIDAR_TOKEN",
    },
}

RUNTIME_PATHS = {
    "filter.min_range_m", "filter.max_range_m", "filter.min_height_m", "filter.max_height_m",
    "scan.min_height_m", "scan.max_height_m", "scan.angle_min_rad", "scan.angle_max_rad",
    "scan.angle_increment_rad", "scan.range_min_m", "scan.range_max_m", "scan.use_inf",
    "streams.stale_sec", "streams.web_points_fps", "streams.web_scan_fps", "streams.max_clients",
    "control.difop_stale_sec", "mock.disconnect", "mock.fail_stop", "mock.fail_command",
}
_ID_RE = re.compile(r"^[A-Za-z0-9_-]+$")
_ENV_RE = re.compile(r"^[A-Z][A-Z0-9_]*$")


def validate_config(config: dict[str, Any]) -> None:
    """验证配置。

    Args:
        config: 设备或平台配置对象，提供本次操作所需参数。 类型为 dict[str, Any]。

    Returns:
        无返回值。
    """
    from ...config import _finite

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
                if not isinstance(schema[key], dict):
                    raise ValueError(f"configuration field {path} must not be a mapping")
                reject_unknown(item, schema[key], path)

    reject_unknown(config, DEFAULT_CONFIG)
    if not isinstance(config["device_id"], str) or not _ID_RE.fullmatch(config["device_id"]):
        raise ValueError("device_id may contain only letters, digits, underscore, and hyphen")
    if config["model"] != "lslidar_c16":
        raise ValueError("model must be lslidar_c16")
    if config["backend"] not in {"c16", "ros2_topics", "mock"}:
        raise ValueError("backend must be c16, ros2_topics, or mock")
    if config["backend"] != "mock" and any(config["mock"].values()):
        raise ValueError("mock fault injection fields require backend=mock")
    if config["interface_mode"] not in {"ros2", "web", "both"}:
        raise ValueError("interface_mode must be ros2, web, or both")
    src = config["source"]
    for key in ("pointcloud_topic", "output_topic", "scan_output_topic"):
        if not isinstance(src[key], str) or not src[key].strip() or "//" in src[key]:
            raise ValueError(f"source.{key} must be a valid nonempty ROS topic")
    output_topics = {src["output_topic"], src["scan_output_topic"]}
    if src["pointcloud_topic"] in output_topics:
        raise ValueError("source.pointcloud_topic cannot equal an output topic")
    driver = config["driver"]
    for key in ("device_ip", "group_ip"):
        if not isinstance(driver[key], str):
            raise ValueError(f"driver.{key} must be a valid IP address")
        try:
            address = ipaddress.IPv4Address(driver[key])
        except ValueError as exc:
            raise ValueError(f"driver.{key} must be a valid IP address") from exc
        if key == "group_ip" and not address.is_multicast:
            raise ValueError("driver.group_ip must be a multicast IP address")
        if key == "device_ip" and (address.is_multicast or address.is_unspecified):
            raise ValueError("driver.device_ip must be a unicast IPv4 address")
    if not isinstance(driver["frame_id"], str) or not driver["frame_id"].strip():
        raise ValueError("driver.frame_id must be a nonempty string")
    for key in ("msop_port", "difop_port", "command_port"):
        value = driver[key]
        if isinstance(value, bool) or not isinstance(value, int) or not 1 <= value <= 65535:
            raise ValueError(f"driver.{key} must be in [1, 65535]")
    for key in ("pcl_type", "add_multicast", "use_time_service", "coordinate_opt"):
        if not isinstance(driver[key], bool):
            raise ValueError(f"driver.{key} must be a boolean")
    if driver["pcl_type"]:
        raise ValueError("driver.pcl_type must be false to preserve C16 XYZIRT point fields")
    if driver["use_time_service"]:
        raise ValueError("driver.use_time_service must remain false until a hardware clock source is verified")
    for key in ("min_range_m", "max_range_m", "distance_unit", "horizontal_angle_resolution_deg"):
        _finite(driver[key], f"driver.{key}")
    if driver["min_range_m"] < 0 or driver["max_range_m"] <= driver["min_range_m"]:
        raise ValueError("driver range limits are invalid")
    if driver["distance_unit"] <= 0 or driver["horizontal_angle_resolution_deg"] <= 0:
        raise ValueError("driver distance and angular resolutions must be positive")
    if not 0 <= driver["angle_disable_min"] <= 36000 or not 0 <= driver["angle_disable_max"] <= 36000:
        raise ValueError("driver angle clipping values must be in [0, 36000]")
    ranges = config["filter"]
    for key, value in ranges.items():
        if key == "target_frame":
            if not isinstance(value, str):
                raise ValueError("filter.target_frame must be a string")
        else:
            _finite(value, f"filter.{key}")
    if not 0 <= ranges["min_range_m"] < ranges["max_range_m"]:
        raise ValueError("filter range must satisfy 0 <= min_range_m < max_range_m")
    if ranges["min_height_m"] >= ranges["max_height_m"]:
        raise ValueError("filter height range is invalid")
    scan = config["scan"]
    for key in ("min_height_m", "max_height_m", "angle_min_rad", "angle_max_rad", "angle_increment_rad", "range_min_m", "range_max_m"):
        _finite(scan[key], f"scan.{key}")
    if scan["min_height_m"] >= scan["max_height_m"]:
        raise ValueError("scan height range is invalid")
    if not -math.pi <= scan["angle_min_rad"] < scan["angle_max_rad"] <= math.pi:
        raise ValueError("scan angle range must be ordered and within [-pi, pi]")
    if not 0 < scan["angle_increment_rad"] <= math.pi:
        raise ValueError("scan.angle_increment_rad must be in (0, pi]")
    if not 0 <= scan["range_min_m"] < scan["range_max_m"]:
        raise ValueError("scan range is invalid")
    if not isinstance(scan["use_inf"], bool):
        raise ValueError("scan.use_inf must be a boolean")
    for section, keys in (("streams", ("stale_sec", "web_points_fps", "web_scan_fps")),
                          ("control", ("difop_stale_sec", "lease_ttl_sec", "lease_renew_interval_sec", "max_lease_ttl_sec"))):
        for key in keys:
            if _finite(config[section][key], f"{section}.{key}") <= 0:
                raise ValueError(f"{section}.{key} must be positive")
    control = config["control"]
    if control["lease_renew_interval_sec"] >= control["lease_ttl_sec"]:
        raise ValueError("control.lease_renew_interval_sec must be less than lease_ttl_sec")
    if control["lease_ttl_sec"] > control["max_lease_ttl_sec"]:
        raise ValueError("control.lease_ttl_sec cannot exceed control.max_lease_ttl_sec")
    for section, key, minimum, maximum in (("streams", "max_clients", 1, 128),):
        value = config[section][key]
        if isinstance(value, bool) or not isinstance(value, int) or not minimum <= value <= maximum:
            raise ValueError(f"{section}.{key} must be in [{minimum}, {maximum}]")
    for key in ("disconnect", "fail_stop", "fail_command"):
        if not isinstance(config["mock"][key], bool):
            raise ValueError(f"mock.{key} must be a boolean")
    web = config["web"]
    if not isinstance(web["port"], int) or isinstance(web["port"], bool) or not 1 <= web["port"] <= 65535:
        raise ValueError("web.port must be in [1, 65535]")
    if not isinstance(web["bind_host"], str) or not web["bind_host"].strip():
        raise ValueError("web.bind_host must be nonempty")
    if not isinstance(web["auth_token_env"], str) or not _ENV_RE.fullmatch(web["auth_token_env"]):
        raise ValueError("web.auth_token_env must be an uppercase environment variable")
