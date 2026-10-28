from __future__ import annotations

import copy
import json
import math
import re
from pathlib import Path
from typing import Any

import yaml


def default_config_path() -> Path:
    """返回该设备的默认配置文件路径。

    Args:
        无参数。

    Returns:
        构造或创建完成的对象或资源。 返回类型为 Path。
    """
    return Path(__file__).resolve().parents[1] / "config" / "aubo_i5_mock.yaml"


def load_config(path: str | Path | None = None) -> dict[str, Any]:
    """加载配置。

    Args:
        path: 目标文件、资源或接口的路径。 类型为 str | Path | None。

    Returns:
        查询或读取到的数据；未找到时的处理方式由实现决定。 返回类型为 dict[str, Any]。
    """
    config = yaml.safe_load(Path(path or default_config_path()).read_text(encoding="utf-8"))
    if not isinstance(config, dict):
        raise ValueError("arm configuration must be a mapping")
    validate_config(config)
    return config


def validate_config(config: dict[str, Any]) -> None:
    """Validate common joint identity and the selected model configuration.

    Args:
        config: Effective device configuration.

    Returns:
        None.
    """
    from .model_registry import resolve_model
    if not isinstance(config, dict) or not isinstance(config.get("model"), str):
        raise ValueError("arm configuration must specify a model")
    if not config.get("device_id"):
        raise ValueError("device_id is required")
    joints = config.get("joints")
    if not isinstance(joints, dict) or set(joints) != {"names", "min_rad", "max_rad"}:
        raise ValueError("joints requires names, min_rad and max_rad")
    names, lower, upper = joints["names"], joints["min_rad"], joints["max_rad"]
    if not isinstance(names, list) or not names or any(not isinstance(n, str) or not n for n in names) or len(set(names)) != len(names):
        raise ValueError("joints.names must contain unique nonempty names")
    if not isinstance(lower, list) or not isinstance(upper, list) or len(lower) != len(names) or len(upper) != len(names):
        raise ValueError("joint position limits must match joints.names")
    for low, high in zip(lower, upper):
        if not math.isfinite(float(low)) or not math.isfinite(float(high)) or float(low) >= float(high):
            raise ValueError("each joint lower limit must be finite and below its upper limit")
    resolve_model(config["model"]).validate_config(config)


def _positive(value: Any, label: str) -> None:
    """校验输入数值为正并返回浮点数。

    Args:
        value: 要应用、比较或转换的输入值。 类型为 Any。
        label: label，作为校验输入数值为正并返回浮点数所需的输入。 类型为 str。

    Returns:
        无返回值。
    """
    if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or value <= 0:
        raise ValueError(f"{label} must be finite and positive")


def _reject_unknown(value: dict, schema: dict, prefix: str = "") -> None:
    """拒绝配置中的未知字段。

    Args:
        value: 要应用、比较或转换的输入值。 类型为 dict。
        schema: schema，作为拒绝配置中的未知字段所需的输入。 类型为 dict。
        prefix: prefix，作为拒绝配置中的未知字段所需的输入。 类型为 str。

    Returns:
        无返回值。
    """
    unknown = sorted(set(value) - set(schema))
    if unknown:
        raise ValueError(f"unknown configuration field {prefix + '.' if prefix else ''}{unknown[0]}")


def config_json(config: dict[str, Any]) -> str:
    """配置JSON。

    Args:
        config: 设备或平台配置对象，提供本次操作所需参数。 类型为 dict[str, Any]。

    Returns:
        本次处理产生的结果值。 返回类型为 str。
    """
    return json.dumps(copy.deepcopy(config), ensure_ascii=False, allow_nan=False, indent=2)
