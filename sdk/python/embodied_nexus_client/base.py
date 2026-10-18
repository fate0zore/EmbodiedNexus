from __future__ import annotations

import json
import threading
import time
import urllib.error
import urllib.request
from dataclasses import dataclass
from typing import Any, Iterator


class BaseClient:
    """Authenticated HTTP/WebSocket client for an EmbodiedNexus base service."""

    def __init__(self, base_url: str, token: str, device_id: str = "base_main", timeout: float = 5.0):
        self.base_url = base_url.rstrip("/")
        self.token = token
        self.device_id = device_id
        self.timeout = float(timeout)
        self._prefix = f"/api/v1/bases/{device_id}"

    def _request(self, method: str, suffix: str = "", body: dict[str, Any] | None = None) -> Any:
        data = json.dumps(body, allow_nan=False).encode("utf-8") if body is not None else None
        headers = {"Authorization": f"Bearer {self.token}"}
        if data is not None:
            headers["Content-Type"] = "application/json"
        request = urllib.request.Request(f"{self.base_url}{self._prefix}/{suffix.lstrip('/')}",
                                         data=data, method=method, headers=headers)
        try:
            with urllib.request.urlopen(request, timeout=self.timeout) as response:
                return json.loads(response.read().decode("utf-8"))
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode("utf-8", errors="replace")
            raise RuntimeError(f"base API returned HTTP {exc.code}: {detail}") from exc

    def info(self) -> dict[str, Any]:
        return self._request("GET", "info")

    def status(self) -> dict[str, Any]:
        return self._request("GET", "status")

    def state(self) -> dict[str, Any]:
        return self.status()

    def config(self) -> dict[str, Any]:
        return self._request("GET", "config")

    def validate_config(self, patch: dict[str, Any]) -> dict[str, Any]:
        return self._request("POST", "config/validate", {"patch": patch})

    def apply_config(self, lease_id: str, patch: dict[str, Any]) -> dict[str, Any]:
        return self._request("PUT", "config", {"lease_id": lease_id, "patch": patch})

    def acquire_control(self, owner: str = "python-client", ttl_sec: float | None = None) -> BaseLease:
        body: dict[str, Any] = {"owner": owner}
        if ttl_sec is not None:
            body["ttl_sec"] = float(ttl_sec)
        response = self._request("POST", "control/lease", body)
        return BaseLease(self, response["lease_id"], float(response["expires_in_sec"]))

    def stop(self, reason: str = "Python client stop requested") -> dict[str, Any]:
        return self._request("POST", "stop", {"reason": reason})

    def operation(self, operation_id: str) -> BaseOperationHandle:
        return BaseOperationHandle(self, operation_id)

    def subscribe_status(self) -> Iterator[dict[str, Any]]:
        try:
            from websockets.sync.client import connect
        except ImportError as exc:
            raise RuntimeError("status subscription requires websockets>=12") from exc
        url = self.base_url.replace("https://", "wss://").replace("http://", "ws://") + "/ws"
        with connect(f"{url}?streams=status", subprotocols=["en-base-v1", f"bearer.{self.token}"],
                    open_timeout=self.timeout) as websocket:
            while True:
                frame = websocket.recv()
                if isinstance(frame, str):
                    yield json.loads(frame)


class BaseLease:
    """Renewing control lease. Prefer ``with client.acquire_control() as lease``."""

    def __init__(self, client: BaseClient, lease_id: str, ttl_sec: float) -> None:
        self.client = client
        self.lease_id = lease_id
        self.ttl_sec = ttl_sec
        self._stop_event = threading.Event()
        self._renew_error = ""
        self._thread = threading.Thread(target=self._renew_loop, name="base-lease-renewal", daemon=True)
        self._thread.start()

    @property
    def renew_error(self) -> str:
        return self._renew_error

    def _renew_loop(self) -> None:
        interval = max(min(self.ttl_sec / 3.0, 0.5), 0.05)
        while not self._stop_event.wait(interval):
            try:
                self.client._request("POST", "control/lease/renew", {"lease_id": self.lease_id})
                self._renew_error = ""
            except Exception as exc:
                self._renew_error = str(exc)

    def enable_motion(self, enabled: bool = True) -> dict[str, Any]:
        return self.client._request("PUT", "motion/enabled", {
            "lease_id": self.lease_id, "enabled": bool(enabled)})

    def set_velocity(self, linear_mps: float, angular_rps: float) -> dict[str, Any]:
        return self.client._request("PUT", "velocity", {
            "lease_id": self.lease_id, "linear_mps": float(linear_mps),
            "angular_rps": float(angular_rps)})

    def move_relative(self, distance_m: float, speed_mps: float = 0.0) -> BaseOperationHandle:
        result = self.client._request("POST", "operations", {
            "lease_id": self.lease_id, "type": "distance",
            "distance_m": float(distance_m), "speed_mps": float(speed_mps)})
        return self.client.operation(result["operation_id"])

    def turn_relative(self, angle_rad: float, speed_rps: float = 0.0) -> BaseOperationHandle:
        result = self.client._request("POST", "operations", {
            "lease_id": self.lease_id, "type": "turn",
            "angle_rad": float(angle_rad), "speed_rps": float(speed_rps)})
        return self.client.operation(result["operation_id"])

    def release(self) -> None:
        if self._stop_event.is_set():
            return
        self._stop_event.set()
        self._thread.join(timeout=max(self.client.timeout, 1.0))
        self.client._request("POST", "control/lease/release", {"lease_id": self.lease_id})

    def __enter__(self) -> BaseLease:
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        self.release()


@dataclass
class BaseOperationHandle:
    client: BaseClient
    operation_id: str

    def status(self) -> dict[str, Any]:
        return self.client._request("GET", f"operations/{self.operation_id}")

    def cancel(self) -> dict[str, Any]:
        return self.client._request("DELETE", f"operations/{self.operation_id}")

    def wait(self, timeout: float | None = None, poll_interval: float = 0.1) -> dict[str, Any]:
        deadline = None if timeout is None else time.monotonic() + float(timeout)
        while True:
            state = self.status()
            if state.get("state") in {"succeeded", "failed", "cancelled"}:
                return state
            if deadline is not None and time.monotonic() >= deadline:
                raise TimeoutError(f"base operation {self.operation_id} did not finish before timeout")
            time.sleep(max(float(poll_interval), 0.01))
