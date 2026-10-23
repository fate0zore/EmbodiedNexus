from __future__ import annotations

import copy
import math
import socket
import threading
import time
import uuid
from typing import Any

from .config import public_config, validate_patch
from .geometry import CloudFrame, ScanFrame, filter_cloud, make_scan
from .protocol import build_motor_packet, build_speed_packet, inspect_difop


class StreamHub:
    """One latest cloud and scan per device; web clients never accumulate queues."""

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._changed = threading.Condition(self._lock)
        self._frames: dict[str, tuple[int, Any, float]] = {}
        self._seq: dict[str, int] = {}
        self._dropped = 0

    def publish(self, channel: str, frame: Any) -> int:
        with self._changed:
            seq = self._seq.get(channel, 0) + 1
            self._seq[channel] = seq
            if channel in self._frames:
                self._dropped += 1
            self._frames[channel] = (seq, frame, time.time())
            self._changed.notify_all()
            return seq

    def latest(self, channel: str):
        with self._lock:
            return self._frames.get(channel)

    def wait_after(self, sequences: dict[str, int], timeout: float):
        with self._changed:
            self._changed.wait_for(
                lambda: any(seq > sequences.get(channel, 0) for channel, (seq, _, _) in self._frames.items()),
                timeout=max(timeout, 0.01),
            )
            ready = [(channel, *entry) for channel, entry in self._frames.items()
                     if entry[0] > sequences.get(channel, 0)]
            for channel, seq, _, _ in ready:
                previous = sequences.get(channel, seq - 1)
                self._dropped += max(seq - previous - 1, 0)
            return ready

    def dropped(self) -> int:
        with self._lock:
            return self._dropped


class LidarRuntime:
    def __init__(self, config: dict[str, Any]) -> None:
        self.config = copy.deepcopy(config)
        self.hub = StreamHub()
        self._lock = threading.RLock()
        # Filtering and LaserScan projection can process hundreds of thousands
        # of points per frame. Keep that work outside the state/control lock so
        # DIFOP freshness, leases and status services remain responsive.
        self._cloud_processing_lock = threading.Lock()
        self._cloud: CloudFrame | None = None
        self._cloud_received = 0.0
        self._scan: ScanFrame | None = None
        self._difop = b""
        self._difop_received = 0.0
        self._sequence = 0
        self._previous_cloud_at = 0.0
        self._fps = 0.0
        self._capturing = True
        self._config_version = 1
        self._lease_id = ""
        self._lease_owner = ""
        self._lease_deadline = 0.0
        self._lease_ttl = 0.0
        self._motor_command_sent = False
        self._motor_state = "unknown"
        self._motor_speed_hz = 10
        self._requested_motor_speed_hz = 10
        self._last_error = ""
        self._mock_started = time.monotonic()
        self._shutdown = False

    def ingest_cloud(self, frame: CloudFrame) -> bool:
        # Serialize input frames, but never hold the shared control lock while
        # walking the PointCloud2 buffer. Config is replaced atomically by
        # apply_config; retry if a new runtime config lands mid-processing.
        with self._cloud_processing_lock:
            while True:
                with self._lock:
                    if not self._capturing:
                        return False
                    cfg = self.config
                    previous_at = self._previous_cloud_at
                now = time.monotonic()
                filtered = filter_cloud(frame, cfg["filter"])
                scan_time = now - previous_at if previous_at else 0.0
                scan = make_scan(filtered, cfg, scan_time)
                with self._lock:
                    if not self._capturing:
                        return False
                    if self.config is not cfg:
                        continue
                    if scan_time > 0:
                        instant = 1.0 / scan_time
                        self._fps = instant if self._fps == 0 else 0.15 * instant + 0.85 * self._fps
                    self._previous_cloud_at = now
                    self._sequence += 1
                    self._cloud = filtered
                    self._cloud_received = now
                    self._scan = scan
                    self._last_error = ""
                    self.hub.publish("points", filtered)
                    self.hub.publish("scan", scan)
                    return True

    def latest_scan(self) -> ScanFrame | None:
        with self._lock:
            return self._scan

    def latest_cloud(self) -> CloudFrame | None:
        with self._lock:
            return self._cloud

    def set_error(self, message: str) -> None:
        with self._lock:
            self._last_error = str(message)

    def ingest_difop(self, packet: bytes | bytearray | list[int]) -> None:
        raw = bytes(packet)
        status = inspect_difop(raw)
        del status
        with self._lock:
            self._difop = raw[:1206]
            self._difop_received = time.monotonic()
            self._last_error = ""

    def set_capture(self, enabled: bool) -> bool:
        with self._lock:
            self._capturing = bool(enabled)
            if enabled:
                self._last_error = ""
            return self._capturing

    def validate_config(self, patch: dict[str, Any]) -> dict[str, Any]:
        with self._lock:
            return validate_patch(self.config, patch)

    def apply_config(self, patch: dict[str, Any]) -> dict[str, Any]:
        with self._lock:
            self.config = validate_patch(self.config, patch)
            self._config_version += 1
            if self._cloud is not None:
                self._scan = make_scan(self._cloud, self.config)
            return self.configuration()

    def configuration(self) -> dict[str, Any]:
        with self._lock:
            result = public_config(self.config)
            result["config_version"] = self._config_version
            return result

    def descriptor(self) -> dict[str, Any]:
        backend = self.config["backend"]
        capabilities = ["read_pointcloud", "read_laserscan", "start_capture", "stop_capture", "configure_filters"]
        if backend in {"c16", "mock"}:
            capabilities.extend(["motor_start", "motor_stop", "set_motor_speed"])
        return {
            "device_id": self.config["device_id"], "model": self.config["model"],
            "driver": "embodied_nexus_lidar", "driver_version": "0.1.0",
            "interface_version": "0.1.0", "backend": backend,
            "capabilities": capabilities,
            "motor_speeds_hz": [5, 10, 20] if backend in {"c16", "mock"} else [],
            "pointcloud_fields": ["x", "y", "z", "intensity", "ring", "time"],
            "units": {"coordinates": "m", "angles": "rad", "time": "s", "intensity": "device-defined"},
        }

    def _fresh_difop_locked(self) -> bytes:
        age = time.monotonic() - self._difop_received if self._difop_received else math.inf
        if not self._difop or age > float(self.config["control"]["difop_stale_sec"]):
            raise RuntimeError(f"fresh DIFOP data unavailable (age={age:.3f}s)")
        inspect_difop(self._difop)
        return self._difop

    @staticmethod
    def _mock_difop() -> bytes:
        packet = bytearray(1206)
        packet[:4] = b"\xA5\xFF\x00\x5A"
        packet[8:10] = b"\x02\x58"
        packet[1198] = 0x04
        return bytes(packet)

    def _require_lease_locked(self, lease_id: str) -> None:
        self._expire_lease_locked()
        if not self._lease_id or not lease_id or lease_id != self._lease_id:
            raise PermissionError("a valid lidar control lease is required")

    def _send_packet_locked(self, packet: bytes) -> None:
        if self.config["backend"] == "mock":
            if self.config["mock"]["fail_command"]:
                raise OSError("mock injected lidar command failure")
            return
        if self.config["backend"] != "c16":
            raise RuntimeError("motor control is unsupported by backend=ros2_topics")
        driver = self.config["driver"]
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sent = sock.sendto(packet, (driver["device_ip"], int(driver["command_port"])))
        if sent != len(packet):
            raise OSError(f"short lidar UDP send: {sent}/{len(packet)} bytes")

    def set_motor(self, lease_id: str, enabled: bool) -> dict[str, Any]:
        with self._lock:
            if self.config["backend"] not in {"c16", "mock"}:
                raise RuntimeError("motor control is unsupported by backend=ros2_topics")
            if enabled:
                self._require_lease_locked(lease_id)
            packet_source = self._mock_difop() if self.config["backend"] == "mock" else self._fresh_difop_locked()
            packet = build_motor_packet(
                packet_source,
                enabled=bool(enabled),
                speed_hz=self._requested_motor_speed_hz if enabled else None,
            )
            self._send_packet_locked(packet)
            self._motor_command_sent = True
            self._motor_state = ("running" if enabled else "stopped") if self.config["backend"] == "mock" else "unknown"
            return {"success": True, "command_sent": True,
                    "hardware_state_confirmed": self.config["backend"] == "mock",
                    "motor_state": self._motor_state, "error": ""}

    def set_motor_speed(self, lease_id: str, speed_hz: int) -> dict[str, Any]:
        with self._lock:
            if self.config["backend"] not in {"c16", "mock"}:
                raise RuntimeError("motor control is unsupported by backend=ros2_topics")
            self._require_lease_locked(lease_id)
            packet_source = self._mock_difop() if self.config["backend"] == "mock" else self._fresh_difop_locked()
            packet = build_speed_packet(packet_source, speed_hz)
            self._send_packet_locked(packet)
            self._motor_command_sent = True
            self._requested_motor_speed_hz = int(speed_hz)
            if self.config["backend"] == "mock":
                self._motor_speed_hz = int(speed_hz)
            actual = self.actual_motor_speed()
            simulated = self.config["backend"] == "mock"
            return {"success": True, "command_sent": True, "hardware_state_confirmed": simulated,
                    "requested_speed_hz": int(speed_hz), "actual_speed_known": actual["known"],
                    "actual_speed_hz": actual["speed_hz"] or 0, "error": ""}

    def acquire(self, owner: str, ttl_sec: float | None = None) -> tuple[str, float]:
        owner = str(owner).strip()
        if not owner:
            raise ValueError("control owner must not be empty")
        with self._lock:
            self._expire_lease_locked()
            if self._lease_id:
                raise RuntimeError(f"lidar control is already held by {self._lease_owner}")
            if self.config["backend"] == "ros2_topics":
                raise RuntimeError("backend=ros2_topics does not own the external lidar motor")
            ttl = float(self.config["control"]["lease_ttl_sec"] if ttl_sec is None or ttl_sec <= 0 else ttl_sec)
            if not math.isfinite(ttl) or not 0 < ttl <= float(self.config["control"]["max_lease_ttl_sec"]):
                raise ValueError("ttl_sec is outside the configured lease range")
            self._lease_id = str(uuid.uuid4())
            self._lease_owner = owner
            self._lease_ttl = ttl
            self._lease_deadline = time.monotonic() + ttl
            return self._lease_id, ttl

    def renew(self, lease_id: str) -> float:
        with self._lock:
            self._require_lease_locked(lease_id)
            self._lease_deadline = time.monotonic() + self._lease_ttl
            return self._lease_ttl

    def _stop_locked(self) -> bool:
        if self.config["backend"] == "ros2_topics":
            return False
        if self.config["backend"] == "mock":
            if self.config["mock"]["fail_stop"]:
                self._last_error = "mock injected lidar stop failure"
                return False
            self._motor_command_sent = True
            self._motor_state = "stopped"
            return True
        try:
            packet = build_motor_packet(self._fresh_difop_locked(), enabled=False)
            self._send_packet_locked(packet)
            self._motor_command_sent = True
            self._motor_state = "unknown"
            return True
        except Exception as exc:
            self._last_error = str(exc)
            return False

    def release(self, lease_id: str) -> bool:
        with self._lock:
            self._require_lease_locked(lease_id)
            sent = self._stop_locked()
            self._clear_lease_locked()
            return sent

    def stop(self) -> bool:
        with self._lock:
            return self._stop_locked()

    def _clear_lease_locked(self) -> None:
        self._lease_id = self._lease_owner = ""
        self._lease_deadline = self._lease_ttl = 0.0

    def _expire_lease_locked(self) -> None:
        if self._lease_id and time.monotonic() >= self._lease_deadline:
            self._stop_locked()
            self._clear_lease_locked()

    def tick(self) -> None:
        with self._lock:
            self._expire_lease_locked()
            if self.config["backend"] == "mock":
                if self.config["mock"]["disconnect"]:
                    self._last_error = "mock injected lidar disconnect"
                elif self._cloud is not None:
                    self._last_error = ""

    def actual_motor_speed(self) -> dict[str, Any]:
        with self._lock:
            if self.config["backend"] == "mock":
                return {"known": True, "speed_hz": self._motor_speed_hz}
            try:
                self._expire_lease_locked()
                status = inspect_difop(self._fresh_difop_locked())
                return {"known": status.speed_confirmed, "speed_hz": status.speed_hz}
            except Exception:
                return {"known": False, "speed_hz": None}

    def state(self) -> dict[str, Any]:
        with self._lock:
            self._expire_lease_locked()
            now = time.monotonic()
            cloud_age = now - self._cloud_received if self._cloud_received else None
            difop_age = now - self._difop_received if self._difop_received else None
            stale = float(self.config["streams"]["stale_sec"])
            backend = self.config["backend"]
            if backend == "mock":
                connected = not self.config["mock"]["disconnect"]
            elif backend == "c16":
                connected = difop_age is not None and difop_age <= float(self.config["control"]["difop_stale_sec"])
            else:
                connected = cloud_age is not None and cloud_age <= stale
            data_fresh = bool(self._cloud is not None and cloud_age is not None and cloud_age <= stale)
            actual = self.actual_motor_speed()
            if self._last_error:
                lifecycle = "faulted"
            elif not connected:
                lifecycle = "connecting"
            elif data_fresh and self._capturing:
                lifecycle = "streaming"
            else:
                lifecycle = "ready"
            return {
                "device_id": self.config["device_id"], "model": self.config["model"],
                "backend": backend, "lifecycle_state": lifecycle, "connected": connected,
                "capturing": self._capturing, "data_fresh": data_fresh,
                "point_sequence": self._sequence, "pointcloud_fps": round(self._fps, 2),
                "pointcloud_age_sec": cloud_age,
                "frame_id": self._cloud.frame_id if self._cloud else str(self.config["driver"]["frame_id"]),
                "difop_age_sec": difop_age, "motor_state": self._motor_state,
                "motor_command_sent": self._motor_command_sent,
                "hardware_state_confirmed": backend == "mock" and self._motor_state in {"running", "stopped"},
                "motor_speed_known": actual["known"], "motor_speed_hz": actual["speed_hz"],
                "lease_owner": self._lease_owner if self._lease_id else "",
                "config_version": self._config_version, "last_error": self._last_error,
                "dropped_frames": self.hub.dropped(),
            }

    def shutdown(self) -> bool:
        with self._lock:
            if self._shutdown:
                return True
            sent = self._stop_locked()
            self._clear_lease_locked()
            self._shutdown = True
            return sent


def mock_cloud(stamp_sec: float, frame_id: str = "lidar_link") -> CloudFrame:
    import struct
    point_step = 32
    fields = [
        {"name": "x", "offset": 0, "datatype": 7, "count": 1},
        {"name": "y", "offset": 4, "datatype": 7, "count": 1},
        {"name": "z", "offset": 8, "datatype": 7, "count": 1},
        {"name": "intensity", "offset": 16, "datatype": 7, "count": 1},
        {"name": "ring", "offset": 20, "datatype": 4, "count": 1},
        {"name": "time", "offset": 24, "datatype": 7, "count": 1},
    ]
    data = bytearray()
    for index in range(360):
        angle = -math.pi + index * math.pi / 180.0
        distance = 2.0 if abs(angle) < 0.2 else 6.0
        data.extend(struct.pack("<fff4xfH2xf4x", distance * math.cos(angle), distance * math.sin(angle),
                                0.0, 100.0, index % 16, index * 1e-4))
    return CloudFrame(stamp_sec, frame_id, 360, 1, fields, False, point_step, 360 * point_step,
                      bytes(data), True)
