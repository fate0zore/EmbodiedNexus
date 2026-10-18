from __future__ import annotations

import os
import math
import threading
import time
from pathlib import Path
from typing import Any

from .protocol import (SCOUT_CONTROL_MODE_ID, SCOUT_MOTION_COMMAND_ID, ScoutFeedback,
                       decode_scout_frames, decode_variable_frames, encode_config,
                       encode_variable_frame, int16_to_bytes)


class MockBackend:
    def __init__(self, config: dict[str, Any]) -> None:
        self.config = config
        self.connected = True
        self.enabled = False
        self.command = (0.0, 0.0)
        self.velocity = (0.0, 0.0)
        self.x = self.y = self.yaw = 0.0
        self._last = time.monotonic()
        self._lock = threading.RLock()

    def open(self) -> None:
        return

    def close(self) -> None:
        self.set_velocity(0.0, 0.0)
        self.connected = False

    def request_control_mode(self) -> None:
        self.enabled = True

    def set_velocity(self, linear: float, angular: float) -> None:
        if self.config["mock"]["fail_command"]:
            raise OSError("mock command write failed")
        self.command = (float(linear), float(angular))

    def stop(self) -> None:
        if self.config["mock"]["fail_stop"]:
            raise OSError("mock stop write failed")
        self.command = (0.0, 0.0)

    def poll(self) -> tuple[dict | None, dict | None, bool, str]:
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


class ScoutWaveshareSerialBackend:
    """Scout 2.0 USB-CAN-A backend, based on the Arachne Waveshare driver."""

    _KNOWN_DRIVERS = ("scout_waveshare_serial_driver", "scout_base_node", "scout_base.launch.py")

    def __init__(self, config: dict[str, Any], serial_factory=None) -> None:
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
        with self._lock:
            if self._serial is not None:
                self._serial.close()
            self._serial = None
            self.connected = False

    def _write(self, payload: bytes) -> None:
        if self._serial is None:
            raise OSError("Scout serial adapter is not open")
        written = self._serial.write(payload)
        if written is not None and written != len(payload):
            raise OSError(f"short serial write ({written}/{len(payload)} bytes)")

    def request_control_mode(self) -> None:
        with self._lock:
            self._write(encode_variable_frame(SCOUT_CONTROL_MODE_ID, bytes([0x01]),
                                              str(self.config["serial"]["frame_type"])))
            self.control_mode_requested = True

    def set_velocity(self, linear: float, angular: float) -> None:
        payload = (int16_to_bytes(round(linear * 1000.0)) +
                   int16_to_bytes(round(angular * 1000.0)) + int16_to_bytes(0) + int16_to_bytes(0))
        with self._lock:
            self._write(encode_variable_frame(SCOUT_MOTION_COMMAND_ID, payload,
                                              str(self.config["serial"]["frame_type"])))

    def stop(self) -> None:
        self.set_velocity(0.0, 0.0)

    def poll(self) -> tuple[dict | None, dict | None, bool, str]:
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


def create_backend(config: dict[str, Any], serial_factory=None):
    if config["backend"] == "mock":
        return MockBackend(config)
    if config["backend"] == "scout_waveshare_serial":
        return ScoutWaveshareSerialBackend(config, serial_factory=serial_factory)
    raise ValueError(f"unsupported base backend {config['backend']!r}")
