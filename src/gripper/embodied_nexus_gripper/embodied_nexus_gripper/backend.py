from __future__ import annotations

from typing import Any, Protocol


class Backend(Protocol):
    def open(self) -> None:
        """打开设备连接或目标资源。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        ...
    def close(self) -> None:
        """关闭连接并释放相关资源。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        ...
    def stop(self) -> None:
        """停止设备当前动作。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        ...
    def read_feedback(self) -> Any | None:
        """读取反馈。

        Args:
            无参数。

        Returns:
            型号适配器提供的反馈，或 None。
        """
        ...


def create_model_operations(config: dict):
    """Create the operation adapter for a configured gripper model.

    Args:
        config: Validated gripper configuration.

    Returns:
        Model-specific motion and feedback adapter.
    """
    from .model_registry import resolve_model
    return resolve_model(config["model"]).create_operations()


def create_backend(config: dict) -> Backend:
    """创建后端。

    Args:
        config: 设备或平台配置对象，提供本次操作所需参数。 类型为 dict。

    Returns:
        构造或创建完成的对象或资源。 返回类型为 Backend。
    """
    from .model_registry import resolve_model
    return resolve_model(config["model"]).create_backend(config)
