from __future__ import annotations

import math
import threading
import time
from typing import Any


class MockBackend:
    def __init__(self, config: dict[str, Any]) -> None:
        """初始化对象并保存构造参数或依赖。

        Args:
            config: 设备或平台配置对象，提供本次操作所需参数。 类型为 dict[str, Any]。

        Returns:
            无返回值。
        """
        self.config = config
        self.connected = True
        self.enabled = False
        self.command = (0.0, 0.0)
        self.velocity = (0.0, 0.0)
        self.x = self.y = self.yaw = 0.0
        self._last = time.monotonic()
        self._lock = threading.RLock()

    def open(self) -> None:
        """打开设备连接或目标资源。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        return

    def close(self) -> None:
        """关闭连接并释放相关资源。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        self.set_velocity(0.0, 0.0)
        self.connected = False

    def request_control_mode(self) -> None:
        """请求控制权mode。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        self.enabled = True

    def set_velocity(self, linear: float, angular: float) -> None:
        """设置速度。

        Args:
            linear: 线速度，作为设置速度所需的输入。 类型为 float。
            angular: 角速度，作为设置速度所需的输入。 类型为 float。

        Returns:
            无返回值。
        """
        if self.config["mock"]["fail_command"]:
            raise OSError("mock command write failed")
        self.command = (float(linear), float(angular))

    def stop(self) -> None:
        """停止设备当前动作。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        if self.config["mock"]["fail_stop"]:
            raise OSError("mock stop write failed")
        self.command = (0.0, 0.0)

    def poll(self) -> tuple[dict | None, dict | None, bool, str]:
        """轮询当前设备或操作。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。 返回类型为 tuple[dict | None, dict | None, bool, str]。
        """
        with self._lock:
            now = time.monotonic()
            dt = min(max(now - self._last, 0.0), 0.25)
            self._last = now
            self.connected = not self.config["mock"]["disconnect"]
            if not self.connected:
                self.velocity = (0.0, 0.0)
                return None, None, False, "mock backend is disconnected"
            self.velocity = self.command if self.enabled else (0.0, 0.0)
            linear, angular = self.velocity
            self.x += linear * math.cos(self.yaw) * dt
            self.y += linear * math.sin(self.yaw) * dt
            self.yaw = (self.yaw + angular * dt + math.pi) % (2 * math.pi) - math.pi
            mock = self.config["mock"]
            system = {"vehicle_state": int(mock["vehicle_state"]),
                      "control_mode": int(mock["control_mode"] if self.enabled else 0),
                      "battery_voltage": float(mock["battery_voltage"]),
                      "error_code": int(mock["error_code"])}
            motion = {"linear_mps": linear, "angular_rps": angular, "lateral_mps": 0.0}
            return system, motion, True, ""
