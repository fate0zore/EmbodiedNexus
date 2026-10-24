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


class ArmBackend(Protocol):
    """Public Aubo operation and feedback contract for pluggable backends."""

    connected: bool

    def open(self) -> None: ...
    def close(self) -> None: ...
    def poll(self) -> dict[str, Any]: ...
    def execute(self, method: str, params: list[Any] | None = None) -> Any: ...
    def read_optional(self, suffix: str) -> Any: ...
    def stop(self) -> bool: ...


class AuboJsonRpc:
    """Synchronous Aubo JSON-RPC client that frames fragmented TCP responses."""

    def __init__(self, host: str, port: int, timeout: float) -> None:
        self.host, self.port, self.timeout = host, int(port), float(timeout)
        self.robot_name = "rob1"
        self._socket: socket.socket | None = None
        self._buffer = ""
        self._decoder = JSONDecoder()
        self._text_decoder = codecs.getincrementaldecoder("utf-8")()
        self._request_id = 0
        self._lock = threading.RLock()

    def connect(self) -> None:
        self.close()
        self._socket = socket.create_connection((self.host, self.port), timeout=self.timeout)
        self._socket.settimeout(self.timeout)
        names = self.call("getRobotNames")
        if isinstance(names, list) and names:
            self.robot_name = str(names[0])

    def close(self) -> None:
        sock, self._socket = self._socket, None
        self._buffer = ""
        self._text_decoder.reset()
        if sock is not None:
            sock.close()

    def call(self, method: str, params: list[Any] | None = None) -> Any:
        with self._lock:
            if self._socket is None:
                raise ConnectionError("Aubo RPC is disconnected")
            self._request_id += 1
            request_id = self._request_id
            payload = json.dumps({"jsonrpc": "2.0", "method": method,
                                  "params": params or [], "id": request_id},
                                 separators=(",", ":"), allow_nan=False).encode("utf-8")
            try:
                self._socket.sendall(payload)
                response = self._read_response(request_id)
            except (OSError, ValueError, json.JSONDecodeError):
                self.close()
                raise
            if response.get("error") not in (None, "", "None", "null"):
                raise RuntimeError(f"{method} failed: {response['error']}")
            return response.get("result")

    def robot_call(self, suffix: str, params: list[Any] | None = None) -> Any:
        return self.call(f"{self.robot_name}.{suffix}", params)

    def _read_response(self, request_id: int) -> dict[str, Any]:
        deadline = time.monotonic() + self.timeout
        while True:
            stripped = self._buffer.lstrip()
            if stripped:
                try:
                    value, end = self._decoder.raw_decode(stripped)
                except json.JSONDecodeError:
                    value = None
                if value is not None:
                    self._buffer = stripped[end:]
                    if not isinstance(value, dict):
                        raise ValueError("Aubo RPC response must be a JSON object")
                    if value.get("id") != request_id:
                        raise ValueError(f"unexpected Aubo RPC response id {value.get('id')}")
                    return value
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(f"Aubo RPC response timed out for request {request_id}")
            self._socket.settimeout(remaining)
            chunk = self._socket.recv(4096)
            if not chunk:
                raise ConnectionError("Aubo RPC closed the connection before replying")
            self._buffer += self._text_decoder.decode(chunk, final=False)


class MockBackend:
    def __init__(self, config: dict[str, Any]) -> None:
        self.config = config
        self.connected = True
        self.enabled = False
        self.mode = str(config["mock"]["mode"])
        self.safety = str(config["mock"]["safety"])
        self.positions = [0.0] * 6
        self.velocities = [0.0] * 6
        self.target: list[float] | None = None
        self.last_update = time.monotonic()
        self.controller_accepted = False
        self._teach_enabled = False
        self._servo_mode_enabled = False
        self.command_log: list[tuple[str, list[Any]]] = []

    def open(self) -> None:
        return

    def close(self) -> None:
        self.stop()
        self.connected = False

    def poll(self) -> dict[str, Any]:
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
                self.velocities = [0.0] * 6
        elif not any(self.velocities):
            self.velocities = [0.0] * 6
        return {"positions_rad": list(self.positions), "velocities_rad_sec": list(self.velocities),
                "mode": self.mode, "safety_mode": self.safety, "teach_enabled": self._teach_enabled,
                "payload": None, "tcp_offset": None, "hardware_estop": self.safety.lower() in {"protectivestop", "emergency"}}

    def execute(self, method: str, params: list[Any] | None = None) -> Any:
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
                self.velocities = [0.0] * 6
                self.target = None
            return 0
        if method == "MotionControl.isServoModeEnabled":
            return self._servo_mode_enabled
        if method == "MotionControl.servoJoint":
            if not self._servo_mode_enabled:
                raise RuntimeError("mock servo mode is not enabled")
            self.target = list(params[0])
            self.velocities = [0.0] * 6
            self.controller_accepted = True
            return 0
        if method == "MotionControl.stopJoint":
            if self.config["mock"]["fail_stop"]:
                raise OSError("mock Aubo stop failed")
            self.velocities = [0.0] * 6
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
        return None

    def stop(self) -> bool:
        try:
            self.execute("MotionControl.stopJoint", [self.config["motion"]["stop_acceleration_rad_sec2"]])
            return True
        except Exception:
            return False

class AuboBackend:
    KNOWN_PROCESSES = ("arachne_hardware", "teach_panel", "grasp_task_server",
                       "aubo_sdk_velocity_bridge", "aubo_move_joint_action_server")
    OWNER_FILE = Path("/tmp/arachne_aubo_control_owner")
    TEACH_FILE = Path("/tmp/arachne_aubo_teach_mode")

    def __init__(self, config: dict[str, Any]) -> None:
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
        self.rpc.close()
        self.connected = False

    def poll(self) -> dict[str, Any]:
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
        method = self.config["teach"]["method"]
        suffix = {"freedrive": "RobotManage.isFreedriveEnabled",
                  "backdrive": "RobotManage.isBackdriveEnabled",
                  "handguide": "RobotManage.getHandguideStatus"}[method]
        try:
            return self.rpc.robot_call(suffix)
        except Exception as exc:
            return f"unknown: {exc}"

    def execute(self, method: str, params: list[Any] | None = None) -> Any:
        if not self.connected:
            raise ConnectionError("Aubo RPC is disconnected")
        return self.rpc.robot_call(method, params)

    def read_optional(self, suffix: str) -> Any:
        try:
            return self.rpc.robot_call(suffix)
        except Exception:
            return None

    def stop(self) -> bool:
        try:
            result = self.execute("MotionControl.stopJoint", [self.config["motion"]["stop_acceleration_rad_sec2"]])
            return result in (0, None)
        except Exception as exc:
            self.last_error = str(exc)
            return False


def _six(values: Any) -> list[float]:
    if not isinstance(values, (list, tuple)) or len(values) != 6:
        raise ValueError("Aubo returned an invalid six-joint vector")
    result = [float(value) for value in values]
    if not all(math.isfinite(value) for value in result):
        raise ValueError("Aubo returned a non-finite joint value")
    return result


def create_backend(config: dict[str, Any]):
    if config["backend"] == "mock":
        return MockBackend(config)
    if config["backend"] == "aubo_jsonrpc":
        return AuboBackend(config)
    if config["backend"] == "ros2_sim":
        from importlib.metadata import entry_points

        providers = entry_points().select(group="embodied_nexus_arm.backends", name="ros2_sim")
        provider = next(iter(providers), None)
        if provider is None:
            raise RuntimeError("backend=ros2_sim requires the embodied_nexus_sim ROS package")
        return provider.load()(config)
    raise ValueError(f"unsupported arm backend {config['backend']!r}")
