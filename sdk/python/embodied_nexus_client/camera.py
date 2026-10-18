from __future__ import annotations

import json
import urllib.error
import urllib.request
from dataclasses import dataclass
from typing import Any, Iterator
from urllib.parse import urlencode


@dataclass
class CameraFrame:
    metadata: dict[str, Any]
    data: bytes


class CameraClient:
    """HTTP/WebSocket client; does not import or require ROS 2."""

    def __init__(self, base_url: str, token: str, camera_id: str = "camera_front", timeout: float = 5.0):
        self.base_url = base_url.rstrip("/")
        self.token = token
        self.camera_id = camera_id
        self.timeout = float(timeout)

    def _request(self, method: str, suffix: str, body: dict[str, Any] | None = None) -> Any:
        data = json.dumps(body).encode("utf-8") if body is not None else None
        request = urllib.request.Request(
            f"{self.base_url}/api/v1/cameras/{self.camera_id}/{suffix.lstrip('/')}",
            data=data,
            method=method,
            headers={"Authorization": f"Bearer {self.token}", "Content-Type": "application/json"},
        )
        try:
            with urllib.request.urlopen(request, timeout=self.timeout) as response:
                content = response.read()
                if response.headers.get_content_type() == "application/json":
                    return json.loads(content.decode("utf-8"))
                return content
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode("utf-8", errors="replace")
            raise RuntimeError(f"camera API returned HTTP {exc.code}: {detail}") from exc

    def info(self) -> dict[str, Any]:
        return self._request("GET", "info")

    def status(self) -> dict[str, Any]:
        return self._request("GET", "status")

    def config(self) -> dict[str, Any]:
        return self._request("GET", "config")

    def snapshot(self) -> bytes:
        return self._request("GET", "snapshot.jpg")

    def patch_config(self, patch: dict[str, Any]) -> dict[str, Any]:
        return self._request("PUT", "config", patch)

    def create_calibration_session(self, **parameters: Any) -> dict[str, Any]:
        return self._request("POST", "calibration/sessions", parameters)

    def capture_calibration_sample(self, session_id: str) -> dict[str, Any]:
        return self._request("POST", f"calibration/sessions/{session_id}/capture", {})

    def delete_calibration_sample(self, session_id: str, filename: str) -> dict[str, Any]:
        request = urllib.request.Request(
            f"{self.base_url}/api/v1/cameras/{self.camera_id}/calibration/sessions/{session_id}/samples/{filename}",
            method="DELETE", headers={"Authorization": f"Bearer {self.token}"},
        )
        try:
            with urllib.request.urlopen(request, timeout=self.timeout) as response:
                return json.loads(response.read().decode("utf-8"))
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode("utf-8", errors="replace")
            raise RuntimeError(f"camera API returned HTTP {exc.code}: {detail}") from exc

    def solve_calibration(self, session_id: str) -> dict[str, Any]:
        return self._request("POST", f"calibration/sessions/{session_id}/solve", {})

    def calibration_result(self, result_id: str) -> dict[str, Any]:
        return self._request("GET", f"calibration/results/{result_id}")

    def calibration(self) -> dict[str, Any]:
        return self._request("GET", "calibration")

    def calibration_history(self) -> list[dict[str, Any]]:
        return self._request("GET", "calibration/history")["history"]

    def apply_calibration(self, result_id: str) -> dict[str, Any]:
        return self._request("POST", f"calibration/results/{result_id}/apply", {})

    def restore_calibration(self, calibration_id: str) -> dict[str, Any]:
        return self._request("POST", f"calibration/history/{calibration_id}/restore", {})

    def subscribe(
        self, streams: tuple[str, ...] = ("color", "depth", "points"), *,
        color_fps: float = 15.0, depth_fps: float = 5.0, points_fps: float = 5.0,
        color_codec: str = "jpeg",
    ) -> Iterator[CameraFrame]:
        """Yield metadata/payload pairs; each binary payload follows its JSON header."""
        try:
            from websockets.sync.client import connect
        except ImportError as exc:
            raise RuntimeError("install the client extra with: pip install websockets>=12") from exc
        query = urlencode({
            "streams": ",".join(streams), "color_fps": color_fps,
            "depth_fps": depth_fps, "points_fps": points_fps,
            "color_codec": color_codec,
        })
        url = self.base_url.replace("https://", "wss://", 1).replace("http://", "ws://", 1) + f"/ws?{query}"
        with connect(
            url,
            subprotocols=["en-camera-v1", f"bearer.{self.token}"],
            open_timeout=self.timeout,
            close_timeout=2.0,
            max_size=None,
            compression=None,
        ) as websocket:
            while True:
                metadata = websocket.recv()
                if isinstance(metadata, bytes):
                    raise RuntimeError("server sent a binary payload without metadata")
                header = json.loads(metadata)
                if header.get("type") == "status":
                    yield CameraFrame(header, b"")
                    continue
                payload = websocket.recv()
                if not isinstance(payload, bytes):
                    raise RuntimeError("server did not follow frame metadata with binary payload")
                yield CameraFrame(header, payload)
