from __future__ import annotations

import json
import threading
import time
import urllib.error
import urllib.request
from dataclasses import dataclass
from typing import Any


class GripperClient:
    """Authenticated HTTP client for an EmbodiedNexus MS42DC service."""

    def __init__(self, base_url: str, token: str, device_id: str = "gripper_main", timeout: float = 5.0):
        self.base_url = base_url.rstrip("/")
        self.token = token
        self.device_id = device_id
        self.timeout = float(timeout)
        self._prefix = f"/api/v1/grippers/{device_id}"

    def _request(self, method: str, suffix: str = "", body: dict[str, Any] | None = None) -> Any:
        data = json.dumps(body).encode("utf-8") if body is not None else None
        headers = {"Authorization": f"Bearer {self.token}"}
        if data is not None:
            headers["Content-Type"] = "application/json"
        request = urllib.request.Request(
            f"{self.base_url}{self._prefix}/{suffix.lstrip('/')}",
            data=data, method=method, headers=headers,
        )
        try:
            with urllib.request.urlopen(request, timeout=self.timeout) as response:
                return json.loads(response.read().decode("utf-8"))
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode("utf-8", errors="replace")
            raise RuntimeError(f"gripper API returned HTTP {exc.code}: {detail}") from exc

    def info(self) -> dict[str, Any]:
        return self._request("GET", "info")

    def status(self) -> dict[str, Any]:
        return self._request("GET", "status")

    def config(self) -> dict[str, Any]:
        return self._request("GET", "config")

    def validate_config(self, patch: dict[str, Any]) -> dict[str, Any]:
        return self._request("POST", "config/validate", patch)

    def apply_config(self, lease_id: str, patch: dict[str, Any]) -> dict[str, Any]:
        return self._request("PUT", "config", {"lease_id": lease_id, "patch": patch})

    def acquire_control(self, owner: str = "python-client", ttl_sec: float | None = None) -> GripperLease:
        body: dict[str, Any] = {"owner": owner}
        if ttl_sec is not None:
            body["ttl_sec"] = float(ttl_sec)
        response = self._request("POST", "control/lease", body)
        return GripperLease(self, response["lease_id"], float(response["expires_in_sec"]))

    def stop(self, reason: str = "Python client stop requested") -> dict[str, Any]:
        return self._request("POST", "stop", {"reason": reason})

    def operation(self, operation_id: str) -> GripperOperationHandle:
        return GripperOperationHandle(self, operation_id)


class GripperLease:
    """Renewing control lease. Use with ``with client.acquire_control() as lease``."""

    def __init__(self, client: GripperClient, lease_id: str, ttl_sec: float) -> None:
        self.client = client
        self.lease_id = lease_id
        self.ttl_sec = ttl_sec
        self._stop_event = threading.Event()
        self._renew_error = ""
        self._thread = threading.Thread(target=self._renew_loop, name="gripper-lease-renewal", daemon=True)
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
        return self.client._request("PUT", "motion/enabled", {"lease_id": self.lease_id, "enabled": enabled})

    def move_open(self) -> GripperOperationHandle:
        return self._move("open")

    def move_close(self) -> GripperOperationHandle:
        return self._move("close")

    def move_relative(self, angle_degrees: float) -> GripperOperationHandle:
        return self._move("relative", angle_degrees)

    def _move(self, command: str, angle_degrees: float = 0.0) -> GripperOperationHandle:
        response = self.client._request("POST", "operations", {
            "lease_id": self.lease_id, "command": command, "angle_degrees": angle_degrees,
        })
        return GripperOperationHandle(self.client, response["operation_id"])

    def release(self) -> None:
        if self._stop_event.is_set():
            return
        self._stop_event.set()
        self._thread.join(timeout=max(self.client.timeout, 1.0))
        self.client._request("POST", "control/lease/release", {"lease_id": self.lease_id})

    def __enter__(self) -> GripperLease:
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        self.release()


@dataclass
class GripperOperationHandle:
    client: GripperClient
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
                raise TimeoutError(f"gripper operation {self.operation_id} did not finish before timeout")
            time.sleep(max(float(poll_interval), 0.01))

\n