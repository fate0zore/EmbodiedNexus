from __future__ import annotations

from typing import Any, Protocol


class ArmBackend(Protocol):
    """Common connection and feedback contract for arm backends."""

    connected: bool

    def joint_limits(self) -> tuple[list[float], list[float]] | None:
        """读取后端提供的关节限制。

        Returns:
            最小和最大关节角列表；限制不可用时返回 None。
        """
        ...

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
    def poll(self) -> dict[str, Any]:
        """轮询当前设备或操作。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any]。
        """
        ...
    def stop(self) -> bool:
        """停止设备当前动作。

        Args:
            无参数。

        Returns:
            布尔值，表示停止设备当前动作是否成功或成立。
        """
        ...


def create_model_operations(config: dict[str, Any], backend: ArmBackend):
    """选择机械臂型号的操作适配器。

    Args:
        config: 设备配置，包含型号标识。
        backend: 设备或仿真后端。

    Returns:
        对应型号的操作适配器。
    """
    from .model_registry import resolve_model
    return resolve_model(config["model"]).create_operations(backend)


def create_backend(config: dict[str, Any]):
    """创建后端。

    Args:
        config: 设备或平台配置对象，提供本次操作所需参数。 类型为 dict[str, Any]。

    Returns:
        构造或创建完成的对象或资源。
    """
    from .model_registry import resolve_model
    return resolve_model(config["model"]).create_backend(config)
