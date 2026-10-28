from __future__ import annotations

import math
import threading
import time
from typing import Any

from .protocol import MS42DCFeedback


class MockBackend:
    def __init__(self, config: dict) -> None:
        """初始化对象并保存构造参数或依赖。

        Args:
            config: 设备或平台配置对象，提供本次操作所需参数。 类型为 dict。

        Returns:
            无返回值。
        """
        self.config = config
        self.opened = False
        self._lock = threading.Lock()
        self._moving = False
        self._direction = 0
        self._target = 0
        self._started = 0.0
        self._speed = 0
        self._angle = 0

    def open(self) -> None:
        """打开设备连接或目标资源。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        with self._lock:
            if self.config["mock"]["disconnect"]:
                raise OSError("mock backend injected a disconnect")
            self.opened = True

    def close(self) -> None:
        """关闭连接并释放相关资源。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        with self._lock:
            self.opened = False
            self._moving = False

    def _check(self) -> None:
        """检查当前设备或操作。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        if self.config["mock"]["disconnect"]:
            self.opened = False
        if not self.opened:
            raise OSError("mock gripper is disconnected")

    def command(self, direction: int, angle_tenths: int, speed_tenths: int) -> None:
        """执行或转发设备控制命令。

        Args:
            direction: direction，作为执行 command 对应的处理流程所需的输入。 类型为 int。
            angle_tenths: 角度tenths，作为执行 command 对应的处理流程所需的输入。 类型为 int。
            speed_tenths: 速度tenths，作为执行 command 对应的处理流程所需的输入。 类型为 int。

        Returns:
            无返回值。
        """
        with self._lock:
            self._check()
            self._direction = direction
            self._target = angle_tenths
            self._started = time.monotonic()
            self._speed = speed_tenths
            self._moving = bool(speed_tenths and angle_tenths)
            if not self._moving:
                self._speed = 0

    def stop(self) -> None:
        """停止设备当前动作。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        with self._lock:
            self._check()
            if self.config["mock"]["fail_stop"]:
                raise OSError("mock backend injected a stop failure")
            self._moving = False
            self._speed = 0
            self._angle = min(0xFFFFFFFF, max(0, self._angle + (self._target if self._direction else -self._target)))

    def read_feedback(self) -> MS42DCFeedback | None:
        """读取反馈。

        Args:
            无参数。

        Returns:
            查询或读取到的数据；未找到时的处理方式由实现决定。 返回类型为 MS42DCFeedback | None。
        """
        with self._lock:
            self._check()
            if self.config["mock"]["fail_feedback"]:
                return None
            if self._moving:
                if time.monotonic() - self._started >= float(self.config["mock"]["move_duration_sec"]):
                    self._moving = False
                    self._speed = 0
                    self._angle = min(0xFFFFFFFF, max(0,
                        self._angle + (self._target if self._direction else -self._target)))
                    self._target = 0
            state = self.config["feedback"]
            reached = state["moving_value"] if self._moving else state["reached_value"]
            return MS42DCFeedback(
                device_id=int(self.config["protocol"]["device_id"]),
                reached=int(reached if reached is not None else 0),
                speed_raw=int(self._speed), angle_raw=int(self._angle),
            )
