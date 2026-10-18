from __future__ import annotations

import os
import threading
import time
from pathlib import Path
from typing import Protocol

from .protocol import Feedback, ProtocolError, build_motion_frame, build_state_request_frame, parse_feedback_frame


class Backend(Protocol):
    def open(self) -> None: ...
    def close(self) -> None: ...
    def command(self, direction: int, angle_tenths: int, speed_tenths: int) -> None: ...
    def stop(self) -> None: ...
    def read_feedback(self) -> Feedback | None: ...


def _check_known_gripper_process(port: str) -> None:
    """Fail safely when a known Arachne driver is already running on Linux."""
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
        self.config = config
        self._serial = None
        self._lock = threading.Lock()

    def open(self) -> None:
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
        with self._lock:
            if self._serial is not None:
                try:
                    self._serial.close()
                finally:
                    self._serial = None

    def _write(self, frame: bytes) -> None:
        if self._serial is None or not self._serial.is_open:
            raise OSError("MS42DC serial port is not open")
        self._serial.write(frame)
        self._serial.flush()

    def command(self, direction: int, angle_tenths: int, speed_tenths: int) -> None:
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
        self.command(direction=0, angle_tenths=0, speed_tenths=0)

    def read_feedback(self) -> Feedback | None:
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
        with self._lock:
            if self.config["mock"]["disconnect"]:
                raise OSError("mock backend injected a disconnect")
            self.opened = True

    def close(self) -> None:
        with self._lock:
            self.opened = False
            self._moving = False

    def _check(self) -> None:
        if self.config["mock"]["disconnect"]:
            self.opened = False
        if not self.opened:
            raise OSError("mock gripper is disconnected")

    def command(self, direction: int, angle_tenths: int, speed_tenths: int) -> None:
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
        with self._lock:
            self._check()
            if self.config["mock"]["fail_stop"]:
                raise OSError("mock backend injected a stop failure")
            self._moving = False
            self._speed = 0
            self._angle = min(0xFFFFFFFF, max(0, self._angle + (self._target if self._direction else -self._target)))

    def read_feedback(self) -> Feedback | None:
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
    if config["backend"] == "mock":
        return MockBackend(config)
    return MS42DCSerialBackend(config)
