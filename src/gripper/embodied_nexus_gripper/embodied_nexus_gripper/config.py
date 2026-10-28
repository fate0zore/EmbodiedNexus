from __future__ import annotations

import copy
import json
import math
import re
from pathlib import Path
from typing import Any

import yaml


from .adapters.ms42dc.config import DEFAULT_CONFIG, _RUNTIME_PATHS


def _merge(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    """合并当前设备或操作。

    Args:
        base: 底盘，作为合并当前设备或操作所需的输入。 类型为 dict[str, Any]。
        override: override，作为合并当前设备或操作所需的输入。 类型为 dict[str, Any]。

    Returns:
        本次处理产生的结果值。 返回类型为 dict[str, Any]。
    """
    result = copy.deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = _merge(result[key], value)
        else:
            result[key] = value
    return result


def load_config(path: str | Path | None = None) -> dict[str, Any]:
    """加载配置。

    Args:
        path: 目标文件、资源或接口的路径。 类型为 str | Path | None。

    Returns:
        查询或读取到的数据；未找到时的处理方式由实现决定。 返回类型为 dict[str, Any]。
    """
    loaded = {}
    if path:
        config_path = Path(path).expanduser().resolve()
        loaded = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
        if not isinstance(loaded, dict):
            raise ValueError("gripper config root must be a YAML mapping")
    from .model_registry import resolve_model
    model = loaded.get("model", DEFAULT_CONFIG["model"])
    config = _merge(resolve_model(model).default_config(), loaded)
    validate_config(config)
    return config


def validate_config(config: dict[str, Any]) -> None:
    """Validate a gripper using its selected model provider.

    Args:
        config: Effective device configuration.

    Returns:
        None.
    """
    from .model_registry import resolve_model
    if not isinstance(config, dict) or not isinstance(config.get("model"), str):
        raise ValueError("gripper configuration must specify a model")
    device_id = config.get("device_id")
    if not isinstance(device_id, str) or not re.fullmatch(r"[A-Za-z0-9_-]+", device_id):
        raise ValueError("invalid gripper device_id")
    resolve_model(config["model"]).validate_config(config)


def _integer(value: Any, label: str) -> int:
    """解析整数型配置值并检查其有效性。

    Args:
        value: 要应用、比较或转换的输入值。 类型为 Any。
        label: label，作为执行 _integer 对应的处理流程所需的输入。 类型为 str。

    Returns:
        本次处理产生的结果值。 返回类型为 int。
    """
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{label} must be an integer")
    return value


def validate_patch(config: dict[str, Any], patch: dict[str, Any]) -> dict[str, Any]:
    """验证patch。

    Args:
        config: 设备或平台配置对象，提供本次操作所需参数。 类型为 dict[str, Any]。
        patch: patch，作为验证patch所需的输入。 类型为 dict[str, Any]。

    Returns:
        布尔值，表示验证patch是否成功或成立。 返回类型为 dict[str, Any]。
    """
    if not isinstance(patch, dict):
        raise ValueError("configuration patch must be a JSON object")
    if not patch:
        raise ValueError("configuration patch must not be empty")
    candidate = copy.deepcopy(config)
    from .model_registry import resolve_model
    runtime_paths = resolve_model(config["model"]).runtime_paths()

    def visit(prefix: str, values: dict[str, Any], target: dict[str, Any]) -> None:
        """递归遍历配置结构并处理其节点。

        Args:
            prefix: prefix，作为执行 visit 对应的处理流程所需的输入。 类型为 str。
            values: 要应用、比较或转换的一组输入值。 类型为 dict[str, Any]。
            target: 目标，作为执行 visit 对应的处理流程所需的输入。 类型为 dict[str, Any]。

        Returns:
            无返回值。
        """
        if not values:
            raise ValueError(f"configuration patch section {prefix or '<root>'} must not be empty")
        for key, value in values.items():
            path = f"{prefix}.{key}" if prefix else key
            if isinstance(value, dict):
                if key not in target or not isinstance(target[key], dict):
                    raise ValueError(f"unknown configuration field: {path}")
                visit(path, value, target[key])
            else:
                if path not in runtime_paths:
                    raise ValueError(f"configuration field requires restart or is immutable: {path}")
                target[key] = value
    visit("", patch, candidate)
    validate_config(candidate)
    return candidate


def public_config(config: dict[str, Any]) -> dict[str, Any]:
    """公开配置。

    Args:
        config: 设备或平台配置对象，提供本次操作所需参数。 类型为 dict[str, Any]。

    Returns:
        本次处理产生的结果值。 返回类型为 dict[str, Any]。
    """
    result = copy.deepcopy(config)
    result["web"].pop("token", None)
    result["web"].pop("auth_token", None)
    return result


def config_json(config: dict[str, Any]) -> str:
    """配置JSON。

    Args:
        config: 设备或平台配置对象，提供本次操作所需参数。 类型为 dict[str, Any]。

    Returns:
        本次处理产生的结果值。 返回类型为 str。
    """
    return json.dumps(public_config(config), ensure_ascii=False, indent=2)


def finite_number(value: Any, label: str) -> float:
    """解析并校验有限数值。

    Args:
        value: 要应用、比较或转换的输入值。 类型为 Any。
        label: label，作为执行 finite_number 对应的处理流程所需的输入。 类型为 str。

    Returns:
        本次处理产生的结果值。 返回类型为 float。
    """
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        raise ValueError(f"{label} must be a finite number")
    return float(value)
