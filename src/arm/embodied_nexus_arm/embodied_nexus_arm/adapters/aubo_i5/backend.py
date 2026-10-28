from __future__ import annotations

import codecs
import json
import math
import os
import socket
import threading
import time
from json import JSONDecoder
from pathlib import Path
from typing import Any, Protocol



from .rpc import AuboJsonRpc

class AuboBackend:
    def joint_limits(self) -> tuple[list[float], list[float]] | None:
        """返回从 Aubo 控制器读取的关节限制。

        Returns:
            最小和最大关节角列表；控制器限制未知时返回 None。
        """
        if self.hardware_min is None or self.hardware_max is None:
            return None
        return self.hardware_min, self.hardware_max

    KNOWN_PROCESSES = ("arachne_hardware", "teach_panel", "grasp_task_server",
                       "aubo_sdk_velocity_bridge", "aubo_move_joint_action_server")
    OWNER_FILE = Path("/tmp/arachne_aubo_control_owner")
    TEACH_FILE = Path("/tmp/arachne_aubo_teach_mode")

    def __init__(self, config: dict[str, Any]) -> None:
        """初始化对象并保存构造参数或依赖。

        Args:
            config: 设备或平台配置对象，提供本次操作所需参数。 类型为 dict[str, Any]。

        Returns:
            无返回值。
        """
        self.config = config
        rpc = config["rpc"]
        self.rpc = AuboJsonRpc(rpc["host"], rpc["port"], rpc["timeout_sec"])
        self.connected = False
        self.hardware_min: list[float] | None = None
        self.hardware_max: list[float] | None = None
        self.payload: Any = None
        self.tcp_offset: Any = None
        self.last_error = ""

    def _check_existing_owner(self) -> None:
        """检查existingowner。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        proc = Path("/proc")
        if proc.is_dir():
            for entry in proc.iterdir():
                if not entry.name.isdigit() or int(entry.name) in {os.getpid(), os.getppid()}:
                    continue
                try:
                    argv = (entry / "cmdline").read_bytes().replace(b"\0", b" ").decode(errors="ignore")
                except OSError:
                    continue
                if any(name in argv for name in self.KNOWN_PROCESSES):
                    raise RuntimeError(f"Arachne process {entry.name} may already control the Aubo arm")
        if self.OWNER_FILE.exists() or self.TEACH_FILE.exists():
            raise RuntimeError("Arachne Aubo control marker exists; refusing to connect")

    def open(self) -> None:
        """打开设备连接或目标资源。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        self._check_existing_owner()
        self.rpc.connect()
        self.connected = True
        try:
            lows = self.rpc.robot_call("RobotConfig.getJointMinPositions")
            highs = self.rpc.robot_call("RobotConfig.getJointMaxPositions")
            if len(lows) == 6 and len(highs) == 6:
                self.hardware_min = [float(v) for v in lows]
                self.hardware_max = [float(v) for v in highs]
        except Exception as exc:
            self.last_error = f"controller joint limits unavailable: {exc}"
        self.payload = self.read_optional("RobotConfig.getPayload")
        self.tcp_offset = self.read_optional("RobotConfig.getTcpOffset")

    def close(self) -> None:
        """关闭连接并释放相关资源。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        self.rpc.close()
        self.connected = False

    def poll(self) -> dict[str, Any]:
        """轮询当前设备或操作。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any]。
        """
        if not self.connected:
            self.open()
        try:
            positions = self.rpc.robot_call("RobotState.getJointPositions")
            velocities = self.rpc.robot_call("RobotState.getJointSpeeds")
            mode = self.rpc.robot_call("RobotState.getRobotModeType")
            safety = self.rpc.robot_call("RobotState.getSafetyModeType")
            teach = self._read_teach_status()
            self.last_error = ""
            return {"positions_rad": _six(positions), "velocities_rad_sec": _six(velocities),
                    "mode": str(mode), "safety_mode": str(safety), "teach_enabled": teach,
                    "payload": self.payload, "tcp_offset": self.tcp_offset,
                    "hardware_estop": str(safety).lower() in {"emergencystop", "protectivestop"}}
        except Exception as exc:
            self.last_error = str(exc)
            self.connected = False
            self.rpc.close()
            raise

    def _read_teach_status(self) -> Any:
        """读取示教状态。

        Args:
            无参数。

        Returns:
            查询或读取到的数据；未找到时的处理方式由实现决定。 返回类型为 Any。
        """
        method = self.config["teach"]["method"]
        suffix = {"freedrive": "RobotManage.isFreedriveEnabled",
                  "backdrive": "RobotManage.isBackdriveEnabled",
                  "handguide": "RobotManage.getHandguideStatus"}[method]
        try:
            return self.rpc.robot_call(suffix)
        except Exception as exc:
            return f"unknown: {exc}"

    def execute(self, method: str, params: list[Any] | None = None) -> Any:
        """执行当前设备或操作。

        Args:
            method: HTTP 请求方法，例如 GET 或 POST。 类型为 str。
            params: params，作为执行当前设备或操作所需的输入。 类型为 list[Any] | None。

        Returns:
            本次处理产生的结果值。 返回类型为 Any。
        """
        if not self.connected:
            raise ConnectionError("Aubo RPC is disconnected")
        return self.rpc.robot_call(method, params)

    def read_optional(self, suffix: str) -> Any:
        """读取可选项。

        Args:
            suffix: suffix，作为读取可选项所需的输入。 类型为 str。

        Returns:
            查询或读取到的数据；未找到时的处理方式由实现决定。 返回类型为 Any。
        """
        try:
            return self.rpc.robot_call(suffix)
        except Exception:
            return None

    def stop(self) -> bool:
        """停止设备当前动作。

        Args:
            无参数。

        Returns:
            布尔值，表示停止设备当前动作是否成功或成立。
        """
        try:
            result = self.execute("MotionControl.stopJoint", [self.config["motion"]["stop_acceleration_rad_sec2"]])
            return result in (0, None)
        except Exception as exc:
            self.last_error = str(exc)
            return False


def _six(values: Any) -> list[float]:
    """校验并规范化六元素关节向量。

    Args:
        values: 要应用、比较或转换的一组输入值。 类型为 Any。

    Returns:
        本次处理产生的结果值。 返回类型为 list[float]。
    """
    if not isinstance(values, (list, tuple)) or len(values) != 6:
        raise ValueError("Aubo returned an invalid six-joint vector")
    result = [float(value) for value in values]
    if not all(math.isfinite(value) for value in result):
        raise ValueError("Aubo returned a non-finite joint value")
    return result
