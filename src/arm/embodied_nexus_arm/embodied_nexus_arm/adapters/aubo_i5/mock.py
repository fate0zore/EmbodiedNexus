from __future__ import annotations

import math
import threading
import time
from typing import Any


class MockBackend:
    def joint_limits(self) -> tuple[list[float], list[float]]:
        """返回 mock 配置的关节限制。

        Returns:
            最小和最大关节角列表。
        """
        return (list(self.config["joints"]["min_rad"]), list(self.config["joints"]["max_rad"]))

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
        self.mode = str(config["mock"]["mode"])
        self.safety = str(config["mock"]["safety"])
        self.positions = [0.0] * len(self.config["joints"]["names"])
        self.velocities = [0.0] * len(self.config["joints"]["names"])
        self.target: list[float] | None = None
        self.last_update = time.monotonic()
        self.controller_accepted = False
        self._teach_enabled = False
        self._servo_mode_enabled = False
        self.command_log: list[tuple[str, list[Any]]] = []

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
        self.stop()
        self.connected = False

    def poll(self) -> dict[str, Any]:
        """轮询当前设备或操作。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any]。
        """
        if self.config["mock"]["disconnect"]:
            self.connected = False
            raise ConnectionError("mock Aubo backend is disconnected")
        self.connected = True
        now = time.monotonic()
        dt = min(max(now - self.last_update, 0.0), 0.25)
        self.last_update = now
        if self.target is not None:
            max_step = self.config["motion"]["max_speed_rad_sec"] * dt
            for i, target in enumerate(self.target):
                delta = target - self.positions[i]
                step = max(-max_step, min(max_step, delta))
                self.positions[i] += step
                self.velocities[i] = step / dt if dt else 0.0
            if all(abs(a - b) <= 1.0e-4 for a, b in zip(self.positions, self.target)):
                self.target = None
                self.velocities = [0.0] * len(self.config["joints"]["names"])
        elif not any(self.velocities):
            self.velocities = [0.0] * len(self.config["joints"]["names"])
        return {"positions_rad": list(self.positions), "velocities_rad_sec": list(self.velocities),
                "mode": self.mode, "safety_mode": self.safety, "teach_enabled": self._teach_enabled,
                "payload": None, "tcp_offset": None, "hardware_estop": self.safety.lower() in {"protectivestop", "emergency"}}

    def execute(self, method: str, params: list[Any] | None = None) -> Any:
        """执行当前设备或操作。

        Args:
            method: HTTP 请求方法，例如 GET 或 POST。 类型为 str。
            params: params，作为执行当前设备或操作所需的输入。 类型为 list[Any] | None。

        Returns:
            本次处理产生的结果值。 返回类型为 Any。
        """
        params = params or []
        self.command_log.append((method, list(params)))
        if self.config["mock"]["fail_command"] and method not in {"MotionControl.stopJoint"}:
            raise OSError("mock Aubo command failed")
        if method == "MotionControl.moveJoint":
            self.target = list(params[0])
            self.controller_accepted = True
            return 0
        if method == "MotionControl.speedJoint":
            self.velocities = list(params[0])
            self.target = None
            self.controller_accepted = True
            return 0
        if method == "MotionControl.setServoMode":
            self._servo_mode_enabled = bool(params[0])
            if not self._servo_mode_enabled:
                self.velocities = [0.0] * len(self.config["joints"]["names"])
                self.target = None
            return 0
        if method == "MotionControl.isServoModeEnabled":
            return self._servo_mode_enabled
        if method == "MotionControl.servoJoint":
            if not self._servo_mode_enabled:
                raise RuntimeError("mock servo mode is not enabled")
            self.target = list(params[0])
            self.velocities = [0.0] * len(self.config["joints"]["names"])
            self.controller_accepted = True
            return 0
        if method == "MotionControl.stopJoint":
            if self.config["mock"]["fail_stop"]:
                raise OSError("mock Aubo stop failed")
            self.velocities = [0.0] * len(self.config["joints"]["names"])
            self.target = None
            return 0
        if method == "RobotManage.poweron":
            self.mode = "Idle"
            return 0
        if method == "RobotManage.startup":
            self.mode = "Running"
            return 0
        if method in {"RobotManage.poweroff", "RobotManage.powerOff"}:
            self.mode = "PowerOff"
            return 0
        if method == "RobotManage.freedrive":
            self._teach_enabled = bool(params[0])
            return 0
        if method == "RobotManage.backdrive":
            self._teach_enabled = bool(params[0])
            return 0
        if method == "RobotManage.handguideMode":
            self._teach_enabled = True
            return 0
        if method == "RobotManage.exitHandguideMode":
            self._teach_enabled = False
            return 0
        return 0

    def read_optional(self, _suffix: str) -> Any:
        """读取可选项。

        Args:
            _suffix: suffix，作为读取可选项所需的输入。 类型为 str。

        Returns:
            查询或读取到的数据；未找到时的处理方式由实现决定。 返回类型为 Any。
        """
        return None

    def stop(self) -> bool:
        """停止设备当前动作。

        Args:
            无参数。

        Returns:
            布尔值，表示停止设备当前动作是否成功或成立。
        """
        try:
            self.execute("MotionControl.stopJoint", [self.config["motion"]["stop_acceleration_rad_sec2"]])
            return True
        except Exception:
            return False
