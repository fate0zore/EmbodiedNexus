from __future__ import annotations

import os
import threading
import time
from pathlib import Path
from typing import Protocol

from .protocol import Feedback, ProtocolError, build_motion_frame, build_state_request_frame, parse_feedback_frame


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
    def command(self, direction: int, angle_tenths: int, speed_tenths: int) -> None:
        """执行或转发设备控制命令。

        Args:
            direction: direction，作为执行 command 对应的处理流程所需的输入。 类型为 int。
            angle_tenths: 角度tenths，作为执行 command 对应的处理流程所需的输入。 类型为 int。
            speed_tenths: 速度tenths，作为执行 command 对应的处理流程所需的输入。 类型为 int。

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
    def read_feedback(self) -> Feedback | None:
        """读取反馈。

        Args:
            无参数。

        Returns:
            查询或读取到的数据；未找到时的处理方式由实现决定。 返回类型为 Feedback | None。
        """
        ...


def _check_known_gripper_process(port: str) -> None:
    """Fail safely when a known Arachne driver is already running on Linux.

    Args:
        port: 网络服务使用的端口号。 类型为 str。

    Returns:
        无返回值。
    """
    proc = Path("/proc")
    if not proc.is_dir():
        return
    known = {"ms42dc_direct_serial_driver", "ms42dc_official_bridge", "ms42dc_step_motor_node"}
    own_pid = os.getpid()
    for entry in proc.iterdir():
        if not entry.name.isdigit() or int(entry.name) == own_pid:
            continue
        try:
            argv = (entry / "cmdline").read_bytes().replace(b"\0", b" ").decode(errors="ignore")
        except (OSError, PermissionError):
            continue
        if any(name in argv for name in known) or "step_motor motor_node" in argv:
            raise RuntimeError(
                f"known MS42DC driver process {entry.name} may own {port}; stop the old stack first"
            )


class MS42DCSerialBackend:
    """Exclusive serial backend for the MS42DC protocol."""

    def __init__(self, config: dict) -> None:
        """初始化对象并保存构造参数或依赖。

        Args:
            config: 设备或平台配置对象，提供本次操作所需参数。 类型为 dict。

        Returns:
            无返回值。
        """
        self.config = config
        self._serial = None
        self._lock = threading.Lock()

    def open(self) -> None:
        """打开设备连接或目标资源。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        with self._lock:
            if self._serial is not None and self._serial.is_open:
                return
            try:
                import serial
            except ImportError as exc:
                raise RuntimeError("pyserial is required for backend=serial; install python3-serial") from exc
            port = str(self.config["serial"]["port"])
            _check_known_gripper_process(port)
            kwargs = {
                "port": port,
                "baudrate": int(self.config["serial"]["baudrate"]),
                "timeout": float(self.config["serial"]["timeout_sec"]),
                "write_timeout": float(self.config["serial"]["timeout_sec"]),
            }
            if os.name == "posix":
                kwargs["exclusive"] = True
            self._serial = serial.Serial(**kwargs)

    def close(self) -> None:
        """关闭连接并释放相关资源。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        with self._lock:
            if self._serial is not None:
                try:
                    self._serial.close()
                finally:
                    self._serial = None

    def _write(self, frame: bytes) -> None:
        """写入当前设备或操作。

        Args:
            frame: 坐标系，作为写入当前设备或操作所需的输入。 类型为 bytes。

        Returns:
            无返回值。
        """
        if self._serial is None or not self._serial.is_open:
            raise OSError("MS42DC serial port is not open")
        self._serial.write(frame)
        self._serial.flush()

    def command(self, direction: int, angle_tenths: int, speed_tenths: int) -> None:
        """执行或转发设备控制命令。

        Args:
            direction: direction，作为执行 command 对应的处理流程所需的输入。 类型为 int。
            angle_tenths: 角度tenths，作为执行 command 对应的处理流程所需的输入。 类型为 int。
            speed_tenths: 速度tenths，作为执行 command 对应的处理流程所需的输入。 类型为 int。

        Returns:
            无返回值。
        """
        frame = build_motion_frame(
            device_id=int(self.config["protocol"]["device_id"]),
            mode=int(self.config["protocol"]["mode"]), direction=direction,
            sub_divide=int(self.config["protocol"]["sub_divide"]),
            angle_tenths=angle_tenths, speed_tenths=speed_tenths,
        )
        with self._lock:
            self._write(frame)

    def stop(self) -> None:
        # A zero-speed, zero-angle motion frame is the stop command used by Arachne.
        """停止设备当前动作。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        self.command(direction=0, angle_tenths=0, speed_tenths=0)

    def read_feedback(self) -> Feedback | None:
        """读取反馈。

        Args:
            无参数。

        Returns:
            查询或读取到的数据；未找到时的处理方式由实现决定。 返回类型为 Feedback | None。
        """
        device_id = int(self.config["protocol"]["device_id"])
        with self._lock:
            if self._serial is None or not self._serial.is_open:
                raise OSError("MS42DC serial port is not open")
            self._serial.reset_input_buffer()
            self._write(build_state_request_frame(device_id))
            response = self._serial.read(9)
        if len(response) != 9:
            return None
        return parse_feedback_frame(response, device_id)


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

    def read_feedback(self) -> Feedback | None:
        """读取反馈。

        Args:
            无参数。

        Returns:
            查询或读取到的数据；未找到时的处理方式由实现决定。 返回类型为 Feedback | None。
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
            return Feedback(
                device_id=int(self.config["protocol"]["device_id"]),
                reached=int(reached if reached is not None else 0),
                speed_raw=int(self._speed), angle_raw=int(self._angle),
            )


def create_backend(config: dict) -> Backend:
    """创建后端。

    Args:
        config: 设备或平台配置对象，提供本次操作所需参数。 类型为 dict。

    Returns:
        构造或创建完成的对象或资源。 返回类型为 Backend。
    """
    if config["backend"] == "mock":
        return MockBackend(config)
    if config["backend"] == "serial":
        return MS42DCSerialBackend(config)
    if config["backend"] == "ros2_sim":
        from importlib.metadata import entry_points

        providers = entry_points().select(group="embodied_nexus_gripper.backends", name="ros2_sim")
        provider = next(iter(providers), None)
        if provider is None:
            raise RuntimeError("backend=ros2_sim requires the embodied_nexus_sim ROS package")
        return provider.load()(config)
    raise ValueError(f"unsupported gripper backend {config['backend']!r}")
