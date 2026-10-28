from __future__ import annotations

from typing import Any, Protocol


class BaseBackend(Protocol):
    """Public backend contract implemented by native and simulator adapters."""

    connected: bool

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
    def request_control_mode(self) -> None:
        """请求控制权mode。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        ...
    def set_velocity(self, linear: float, angular: float) -> None:
        """设置速度。

        Args:
            linear: 线速度，作为设置速度所需的输入。 类型为 float。
            angular: 角速度，作为设置速度所需的输入。 类型为 float。

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
    def poll(self) -> tuple[dict | None, dict | None, bool, str]:
        """轮询当前设备或操作。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。 返回类型为 tuple[dict | None, dict | None, bool, str]。
        """
        ...


def create_model_policy(config: dict[str, Any]):
    """Select the system feedback policy for a configured base model.

    Args:
        config: Effective device configuration.

    Returns:
        Model-specific feedback and safety policy.
    """
    from .model_registry import resolve_model
    return resolve_model(config["model"]).create_policy()


def create_backend(config: dict[str, Any], serial_factory=None):
    """创建后端。

    Args:
        config: 设备或平台配置对象，提供本次操作所需参数。 类型为 dict[str, Any]。
        serial_factory: serialfactory，作为创建后端所需的输入。

    Returns:
        构造或创建完成的对象或资源。
    """
    from .model_registry import resolve_model
    return resolve_model(config["model"]).create_backend(config, serial_factory=serial_factory)
