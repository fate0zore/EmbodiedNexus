from __future__ import annotations

import json
import urllib.error
import urllib.request
from typing import Any
from urllib.parse import quote

from .arm import ArmClient
from .base import BaseClient
from .camera import CameraClient
from .gripper import GripperClient
from .lidar import LidarClient


class CoreClient:
    """Authenticated manager and unified gateway for an EmbodiedNexus robot."""

    def __init__(self, base_url: str, token: str, timeout: float = 10.0, *,
                 management_timeout: float = 40.0):
        self.base_url = base_url.rstrip("/")
        self.token = token
        self.timeout = float(timeout)
        self.management_timeout = float(management_timeout)

    def _request(self, method: str, path: str, body: dict[str, Any] | None = None,
                 *, timeout: float | None = None) -> Any:
        data = json.dumps(body, allow_nan=False).encode("utf-8") if body is not None else None
        request = urllib.request.Request(
            self.base_url + path, data=data, method=method,
            headers={"Authorization": f"Bearer {self.token}", "Content-Type": "application/json"},
        )
        try:
            with urllib.request.urlopen(request, timeout=self.timeout if timeout is None else timeout) as response:
                payload = response.read()
                return json.loads(payload.decode("utf-8")) if payload else {}
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode("utf-8", errors="replace")
            raise RuntimeError(f"core API returned HTTP {exc.code}: {detail}") from exc

    def status(self) -> dict[str, Any]:
        return self._request("GET", "/api/v1/core/status")

    def devices(self) -> list[dict[str, Any]]:
        return self._request("GET", "/api/v1/core/devices")["devices"]

    def device(self, device_id: str) -> dict[str, Any]:
        path = f"/api/v1/core/devices/{quote(device_id, safe='')}"
        return self._request("GET", path)

    def manage_device(self, device_id: str, action: str) -> dict[str, Any]:
        if action not in {"start", "shutdown", "restart"}:
            raise ValueError("action must be start, shutdown or restart")
        path = f"/api/v1/core/devices/{quote(device_id, safe='')}"
        return self._request("POST", path, {"action": action}, timeout=self.management_timeout)

    def start(self, device_id: str) -> dict[str, Any]:
        return self.manage_device(device_id, "start")

    def shutdown(self, device_id: str) -> dict[str, Any]:
        return self.manage_device(device_id, "shutdown")

    def restart(self, device_id: str) -> dict[str, Any]:
        return self.manage_device(device_id, "restart")

    def _manage_all(self, action: str) -> dict[str, Any]:
        return self._request("POST", "/api/v1/core/devices/manage_all", {"action": action},
                             timeout=self.management_timeout)

    def start_all(self) -> dict[str, Any]:
        return self._manage_all("start_all")

    def shutdown_all(self) -> dict[str, Any]:
        return self._manage_all("shutdown_all")

    def stop_all(self) -> dict[str, Any]:
        return self._manage_all("stop_all")

    def shutdown_core(self) -> dict[str, Any]:
        return self._request("POST", "/api/v1/core/shutdown", {}, timeout=self.management_timeout)

    def configuration(self) -> dict[str, Any]:
        return self._request("GET", "/api/v1/core/config")

    def validate_configuration(self, config_file: str | None = None) -> dict[str, Any]:
        return self._request("POST", "/api/v1/core/config/validate", {"config_file": config_file})

    def _websocket_path(self, kind: str, device_id: str) -> str:
        plural = {"base": "bases", "arm": "arms", "gripper": "grippers",
                  "camera": "cameras", "lidar": "lidars"}[kind]
        return f"/api/v1/{plural}/{quote(device_id, safe='')}/ws"

    def base(self, device_id: str = "base_main") -> BaseClient:
        return BaseClient(self.base_url, self.token, device_id,
                          websocket_path=self._websocket_path("base", device_id))

    def arm(self, device_id: str = "arm_main") -> ArmClient:
        return ArmClient(self.base_url, self.token, device_id,
                         websocket_path=self._websocket_path("arm", device_id))

    def gripper(self, device_id: str = "gripper_main") -> GripperClient:
        return GripperClient(self.base_url, self.token, device_id)

    def camera(self, device_id: str = "camera_front") -> CameraClient:
        return CameraClient(self.base_url, self.token, device_id,
                            websocket_path=self._websocket_path("camera", device_id))

    def lidar(self, device_id: str = "lidar_main") -> LidarClient:
        return LidarClient(self.base_url, self.token, device_id,
                           websocket_path=self._websocket_path("lidar", device_id))
