from __future__ import annotations

import json
import urllib.error
import urllib.request
from dataclasses import dataclass
from typing import Any, Iterator
from urllib.parse import quote, urlencode


@dataclass
class CameraFrame:
    metadata: dict[str, Any]
    data: bytes


class CameraClient:
    """HTTP/WebSocket client; does not import or require ROS 2."""

    def __init__(self, base_url: str, token: str, camera_id: str = "camera_front", timeout: float = 5.0,
                 *, websocket_path: str = "/ws"):
        """初始化对象并保存构造参数或依赖。

        Args:
            base_url: 设备服务或 Core 服务的基础 HTTP URL。 类型为 str。
            token: 用于身份验证的访问令牌。 类型为 str。
            camera_id: 相机标识符，作为初始化对象并保存构造参数或依赖所需的输入。 类型为 str。
            timeout: 本次网络或服务调用的最长等待时长，通常以秒为单位。 类型为 float。
            websocket_path: websocket路径，作为初始化对象并保存构造参数或依赖所需的输入。 类型为 str。

        Returns:
            无返回值。
        """
        self.base_url = base_url.rstrip("/")
        self.token = token
        self.camera_id = camera_id
        self.timeout = float(timeout)
        self.websocket_path = websocket_path

    def _request(self, method: str, suffix: str, body: dict[str, Any] | None = None) -> Any:
        """向服务端发送 HTTP 请求并解析响应。

        Args:
            method: HTTP 请求方法，例如 GET 或 POST。 类型为 str。
            suffix: suffix，作为向服务端发送 HTTP 请求并解析响应所需的输入。 类型为 str。
            body: 随请求发送的 JSON 数据。 类型为 dict[str, Any] | None。

        Returns:
            本次处理产生的结果值。 返回类型为 Any。
        """
        data = json.dumps(body).encode("utf-8") if body is not None else None
        request = urllib.request.Request(
            f"{self.base_url}/api/v1/cameras/{quote(self.camera_id, safe='')}/{suffix.lstrip('/')}",
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
        """获取设备能力与静态信息。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any]。
        """
        return self._request("GET", "info")

    def status(self) -> dict[str, Any]:
        """获取服务当前运行状态。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any]。
        """
        return self._request("GET", "status")

    def config(self) -> dict[str, Any]:
        """获取当前配置。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any]。
        """
        return self._request("GET", "config")

    def snapshot(self) -> bytes:
        """获取相机当前图像快照。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。 返回类型为 bytes。
        """
        return self._request("GET", "snapshot.jpg")

    def patch_config(self, patch: dict[str, Any]) -> dict[str, Any]:
        """向相机服务提交运行时配置补丁。

        Args:
            patch: patch，作为执行 patch_config 对应的处理流程所需的输入。 类型为 dict[str, Any]。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any]。
        """
        return self._request("PUT", "config", patch)

    def create_calibration_session(self, **parameters: Any) -> dict[str, Any]:
        """创建calibration会话。

        Args:
            **parameters: 参数，作为创建calibration会话所需的输入。 类型为 Any。

        Returns:
            构造或创建完成的对象或资源。 返回类型为 dict[str, Any]。
        """
        return self._request("POST", "calibration/sessions", parameters)

    def capture_calibration_sample(self, session_id: str) -> dict[str, Any]:
        """采集calibration采集样本。

        Args:
            session_id: 会话标识符，作为采集calibration采集样本所需的输入。 类型为 str。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any]。
        """
        return self._request("POST", f"calibration/sessions/{session_id}/capture", {})

    def delete_calibration_sample(self, session_id: str, filename: str) -> dict[str, Any]:
        """删除calibration采集样本。

        Args:
            session_id: 会话标识符，作为删除calibration采集样本所需的输入。 类型为 str。
            filename: filename，作为删除calibration采集样本所需的输入。 类型为 str。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any]。
        """
        request = urllib.request.Request(
            f"{self.base_url}/api/v1/cameras/{quote(self.camera_id, safe='')}/calibration/sessions/{quote(session_id, safe='')}/samples/{quote(filename, safe='')}",
            method="DELETE", headers={"Authorization": f"Bearer {self.token}"},
        )
        try:
            with urllib.request.urlopen(request, timeout=self.timeout) as response:
                return json.loads(response.read().decode("utf-8"))
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode("utf-8", errors="replace")
            raise RuntimeError(f"camera API returned HTTP {exc.code}: {detail}") from exc

    def solve_calibration(self, session_id: str) -> dict[str, Any]:
        """启动相机标定求解并返回任务结果。

        Args:
            session_id: 会话标识符，作为执行 solve_calibration 对应的处理流程所需的输入。 类型为 str。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any]。
        """
        return self._request("POST", f"calibration/sessions/{session_id}/solve", {})

    def calibration_result(self, result_id: str) -> dict[str, Any]:
        """读取指定相机标定任务的求解结果。

        Args:
            result_id: 结果标识符，作为执行 calibration_result 对应的处理流程所需的输入。 类型为 str。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any]。
        """
        return self._request("GET", f"calibration/results/{result_id}")

    def calibration(self) -> dict[str, Any]:
        """获取当前相机标定信息。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any]。
        """
        return self._request("GET", "calibration")

    def calibration_history(self) -> list[dict[str, Any]]:
        """列出已保存的相机标定历史记录。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。 返回类型为 list[dict[str, Any]]。
        """
        return self._request("GET", "calibration/history")["history"]

    def apply_calibration(self, result_id: str) -> dict[str, Any]:
        """应用calibration。

        Args:
            result_id: 结果标识符，作为应用calibration所需的输入。 类型为 str。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any]。
        """
        return self._request("POST", f"calibration/results/{result_id}/apply", {})

    def restore_calibration(self, calibration_id: str) -> dict[str, Any]:
        """恢复指定历史相机标定。

        Args:
            calibration_id: calibration标识符，作为执行 restore_calibration 对应的处理流程所需的输入。 类型为 str。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any]。
        """
        return self._request("POST", f"calibration/history/{calibration_id}/restore", {})

    def subscribe(
        self, streams: tuple[str, ...] = ("color", "depth", "points"), *,
        color_fps: float = 15.0, depth_fps: float = 5.0, points_fps: float = 5.0,
        color_codec: str = "jpeg",
    ) -> Iterator[CameraFrame]:
        """Yield metadata/payload pairs; each binary payload follows its JSON header.

        Args:
            streams: streams，作为订阅相机数据流并依次产出元数据和二进制图像载荷所需的输入。 类型为 tuple[str, ...]。
            color_fps: colorfps，作为订阅相机数据流并依次产出元数据和二进制图像载荷所需的输入。 类型为 float。
            depth_fps: depthfps，作为订阅相机数据流并依次产出元数据和二进制图像载荷所需的输入。 类型为 float。
            points_fps: pointsfps，作为订阅相机数据流并依次产出元数据和二进制图像载荷所需的输入。 类型为 float。
            color_codec: colorcodec，作为订阅相机数据流并依次产出元数据和二进制图像载荷所需的输入。 类型为 str。

        Returns:
            本次处理产生的结果值。 返回类型为 Iterator[CameraFrame]。
        """
        try:
            from websockets.sync.client import connect
        except ImportError as exc:
            raise RuntimeError("install the client extra with: pip install websockets>=12") from exc
        query = urlencode({
            "streams": ",".join(streams), "color_fps": color_fps,
            "depth_fps": depth_fps, "points_fps": points_fps,
            "color_codec": color_codec,
        })
        url = self.base_url.replace("https://", "wss://", 1).replace("http://", "ws://", 1) + f"{self.websocket_path}?{query}"
        with connect(
            url,
            subprotocols=["en-camera-v1", f"bearer.{self.token}"],
            open_timeout=self.timeout,
            close_timeout=2.0,
            max_size=None,
            compression=None,
        ) as websocket:
            while True:
                metadata = websocket.recv(timeout=self.timeout)
                if isinstance(metadata, bytes):
                    raise RuntimeError("server sent a binary payload without metadata")
                header = json.loads(metadata)
                if header.get("type") == "status":
                    yield CameraFrame(header, b"")
                    continue
                payload = websocket.recv(timeout=self.timeout)
                if not isinstance(payload, bytes):
                    raise RuntimeError("server did not follow frame metadata with binary payload")
                yield CameraFrame(header, payload)
