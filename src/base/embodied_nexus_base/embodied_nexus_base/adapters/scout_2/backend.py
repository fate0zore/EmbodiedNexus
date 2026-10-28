from __future__ import annotations

import os
import math
import threading
import time
from pathlib import Path
from typing import Any, Protocol

from .protocol import (SCOUT_CONTROL_MODE_ID, SCOUT_MOTION_COMMAND_ID, ScoutFeedback,
                       decode_scout_frames, decode_variable_frames, encode_config,
                       encode_variable_frame, int16_to_bytes)


class ScoutWaveshareSerialBackend:
    """Scout 2.0 USB-CAN-A backend, based on the Arachne Waveshare driver."""

    _KNOWN_DRIVERS = ("scout_waveshare_serial_driver", "scout_base_node", "scout_base.launch.py")

    def __init__(self, config: dict[str, Any], serial_factory=None) -> None:
        """初始化对象并保存构造参数或依赖。

        Args:
            config: 设备或平台配置对象，提供本次操作所需参数。 类型为 dict[str, Any]。
            serial_factory: serialfactory，作为初始化对象并保存构造参数或依赖所需的输入。

        Returns:
            无返回值。
        """
        self.config = config
        self._serial_factory = serial_factory
        self._serial = None
        self._buffer = b""
        self._lock = threading.RLock()
        self.connected = False
        self.latest_system: dict | None = None
        self.latest_motion: dict | None = None
        self.control_mode_requested = False

    def _check_existing_driver(self) -> None:
        """检查existingdriver。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        proc = Path("/proc")
        if not proc.is_dir():
            return
        for entry in proc.iterdir():
            if not entry.name.isdigit() or int(entry.name) in {os.getpid(), os.getppid()}:
                continue
            try:
                argv = (entry / "cmdline").read_bytes().replace(b"\0", b" ").decode(errors="ignore")
            except OSError:
                continue
            if any(driver in argv for driver in self._KNOWN_DRIVERS):
                raise RuntimeError(f"existing Scout driver process {entry.name} may own the CAN adapter")

    def open(self) -> None:
        """打开设备连接或目标资源。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        if self._serial is not None:
            return
        self._check_existing_driver()
        if self._serial_factory is None:
            try:
                import serial
            except ImportError as exc:
                raise RuntimeError("Scout serial backend requires pyserial") from exc
            factory = serial.Serial
        else:
            factory = self._serial_factory
        settings = self.config["serial"]
        kwargs = {"baudrate": int(settings["usb_baudrate"]), "timeout": 0,
                  "write_timeout": 0.2}
        if os.name == "posix":
            kwargs["exclusive"] = True
        self._serial = factory(str(settings["port"]), **kwargs)
        self.connected = True
        # This configures the USB-CAN adapter only. It does not switch Scout control mode.
        self._write(encode_config(int(settings["can_bitrate"]), str(settings["frame_type"]),
                                  mode=str(settings["adapter_mode"])))

    def close(self) -> None:
        """关闭连接并释放相关资源。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        with self._lock:
            if self._serial is not None:
                self._serial.close()
            self._serial = None
            self.connected = False

    def _write(self, payload: bytes) -> None:
        """写入当前设备或操作。

        Args:
            payload: 消息载荷或接口响应内容。 类型为 bytes。

        Returns:
            无返回值。
        """
        if self._serial is None:
            raise OSError("Scout serial adapter is not open")
        written = self._serial.write(payload)
        if written is not None and written != len(payload):
            raise OSError(f"short serial write ({written}/{len(payload)} bytes)")

    def request_control_mode(self) -> None:
        """请求控制权mode。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        with self._lock:
            self._write(encode_variable_frame(SCOUT_CONTROL_MODE_ID, bytes([0x01]),
                                              str(self.config["serial"]["frame_type"])))
            self.control_mode_requested = True

    def set_velocity(self, linear: float, angular: float) -> None:
        """设置速度。

        Args:
            linear: 线速度，作为设置速度所需的输入。 类型为 float。
            angular: 角速度，作为设置速度所需的输入。 类型为 float。

        Returns:
            无返回值。
        """
        payload = (int16_to_bytes(round(linear * 1000.0)) +
                   int16_to_bytes(round(angular * 1000.0)) + int16_to_bytes(0) + int16_to_bytes(0))
        with self._lock:
            self._write(encode_variable_frame(SCOUT_MOTION_COMMAND_ID, payload,
                                              str(self.config["serial"]["frame_type"])))

    def stop(self) -> None:
        """停止设备当前动作。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        self.set_velocity(0.0, 0.0)

    def poll(self) -> tuple[dict | None, dict | None, bool, str]:
        """轮询当前设备或操作。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。 返回类型为 tuple[dict | None, dict | None, bool, str]。
        """
        with self._lock:
            if self._serial is None:
                return None, None, False, "Scout serial adapter is not open"
            try:
                chunks = []
                while True:
                    chunk = self._serial.read(4096)
                    if not chunk:
                        break
                    chunks.append(chunk)
                self._buffer += b"".join(chunks)
                frames, remainder = decode_variable_frames(self._buffer)
                # Keep only a possible partial frame; discard corrupt/noise tails after 128 bytes.
                self._buffer = remainder[-128:]
                feedback = decode_scout_frames(frames)
                if feedback.system is not None:
                    self.latest_system = feedback.system
                if feedback.motion is not None:
                    self.latest_motion = feedback.motion
                return feedback.system, feedback.motion, True, ""
            except Exception as exc:
                self.connected = False
                return None, None, False, str(exc)
