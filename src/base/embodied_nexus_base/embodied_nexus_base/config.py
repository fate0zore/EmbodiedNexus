from __future__ import annotations

import copy
import json
import math
import re
from pathlib import Path
from typing import Any

import yaml


from .adapters.scout_2.config import DEFAULT_CONFIG, RUNTIME_PATHS


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
        loaded = yaml.safe_load(Path(path).expanduser().read_text(encoding="utf-8")) or {}
        if not isinstance(loaded, dict):
            raise ValueError("base config root must be a YAML mapping")
    from .model_registry import resolve_model
    model = loaded.get("model", DEFAULT_CONFIG["model"])
    config = _merge(resolve_model(model).default_config(), loaded)
    validate_config(config)
    return config


def validate_config(config: dict[str, Any]) -> None:
    """Validate a base using the selected model provider.

    Args:
        config: Effective device configuration.

    Returns:
        None.
    """
    from .model_registry import resolve_model
    if not isinstance(config, dict) or not isinstance(config.get("model"), str):
        raise ValueError("base configuration must specify a model")
    device_id = config.get("device_id")
    if not isinstance(device_id, str) or not re.fullmatch(r"[A-Za-z0-9_-]+", device_id):
        raise ValueError("invalid base device_id")
    resolve_model(config["model"]).validate_config(config)


def _reject_unknown(value: dict[str, Any], schema: dict[str, Any], prefix: str = "") -> None:
    """拒绝配置中的未知字段。

    Args:
        value: 要应用、比较或转换的输入值。 类型为 dict[str, Any]。
        schema: schema，作为拒绝配置中的未知字段所需的输入。 类型为 dict[str, Any]。
        prefix: prefix，作为拒绝配置中的未知字段所需的输入。 类型为 str。

    Returns:
        无返回值。
    """
    unknown = sorted(set(value) - set(schema))
    if unknown:
        label = f"{prefix}." if prefix else ""
        raise ValueError(f"unknown configuration field {label}{unknown[0]}")
    for key, child_schema in schema.items():
        child = value.get(key)
        if child is not None and isinstance(child_schema, dict) and isinstance(child, dict):
            _reject_unknown(child, child_schema, f"{prefix}.{key}" if prefix else key)


def validate_patch(config: dict[str, Any], patch: dict[str, Any]) -> dict[str, Any]:
    """验证patch。

    Args:
        config: 设备或平台配置对象，提供本次操作所需参数。 类型为 dict[str, Any]。
        patch: patch，作为验证patch所需的输入。 类型为 dict[str, Any]。

    Returns:
        布尔值，表示验证patch是否成功或成立。 返回类型为 dict[str, Any]。
    """
    if not isinstance(patch, dict):
        raise ValueError("configuration patch must be a mapping")
    candidate = _merge(config, patch)
    validate_config(candidate)
    changed = _leaf_paths(config, candidate)
    from .model_registry import resolve_model
    unsupported = sorted(changed - resolve_model(config["model"]).runtime_paths())
    if unsupported:
        raise ValueError("configuration fields require restart: " + ", ".join(unsupported))
    # Security and movement limits are fixed at process start and cannot be raised live.
    for path in ("motion.max_linear_mps", "motion.max_angular_rps"):
        if _get(candidate, path) > _get(config, path):
            raise ValueError(f"{path} cannot be increased at runtime")
    return candidate


def _leaf_paths(before: dict, after: dict, prefix: str = "") -> set[str]:
    """递归比较配置并收集发生变化的叶节点路径。

    Args:
        before: before，作为执行 _leaf_paths 对应的处理流程所需的输入。 类型为 dict。
        after: after，作为执行 _leaf_paths 对应的处理流程所需的输入。 类型为 dict。
        prefix: prefix，作为执行 _leaf_paths 对应的处理流程所需的输入。 类型为 str。

    Returns:
        本次处理产生的结果值。 返回类型为 set[str]。
    """
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
    """获取当前设备或操作。

    Args:
        value: 要应用、比较或转换的输入值。 类型为 dict。
        path: 目标文件、资源或接口的路径。 类型为 str。

    Returns:
        本次处理产生的结果值。
    """
    for part in path.split("."):
        value = value[part]
    return value


def public_config(config: dict[str, Any]) -> dict[str, Any]:
    """公开配置。

    Args:
        config: 设备或平台配置对象，提供本次操作所需参数。 类型为 dict[str, Any]。

    Returns:
        本次处理产生的结果值。 返回类型为 dict[str, Any]。
    """
    return copy.deepcopy(config)


def config_json(config: dict[str, Any]) -> str:
    """配置JSON。

    Args:
        config: 设备或平台配置对象，提供本次操作所需参数。 类型为 dict[str, Any]。

    Returns:
        本次处理产生的结果值。 返回类型为 str。
    """
    return json.dumps(public_config(config), ensure_ascii=False, allow_nan=False, indent=2)
