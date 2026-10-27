from __future__ import annotations

import json
import struct
import threading
import time
import urllib.error
import urllib.request
from dataclasses import dataclass
from typing import Any, Iterator
from urllib.parse import quote, urlencode


@dataclass
class LidarFrame:
    metadata: dict[str, Any]
    data: bytes

    def points(self, limit: int | None = None) -> Iterator[dict[str, float | int]]:
        """获取雷达点云数据。

        Args:
            limit: limit，作为执行 points 对应的处理流程所需的输入。 类型为 int | None。

        Returns:
            本次处理产生的结果值。 返回类型为 Iterator[dict[str, float | int]]。
        """
        if self.metadata.get("channel") != "points":
            raise ValueError("points() is available for PointCloud2 frames")
        if limit is not None and (isinstance(limit, bool) or not isinstance(limit, int) or limit < 0):
            raise ValueError("limit must be a nonnegative integer or None")
        fields = self.metadata["fields"]
        endian = ">" if self.metadata.get("is_bigendian") else "<"
        point_step = int(self.metadata["point_step"])
        width, height = int(self.metadata["width"]), int(self.metadata["height"])
        row_step = int(self.metadata["row_step"])
        count = min(width * height, int(limit)) if limit is not None else width * height
        supported = {1: ("b", 1), 2: ("B", 1), 3: ("h", 2), 4: ("H", 2),
                     5: ("i", 4), 6: ("I", 4), 7: ("f", 4), 8: ("d", 8)}
        layouts = []
        for field in fields:
            kind = supported.get(int(field["datatype"]))
            if kind is None or int(field["count"]) != 1:
                continue
            offset = int(field["offset"])
            if offset < 0 or offset + kind[1] > point_step:
                raise ValueError(f"PointCloud2 field {field['name']} exceeds point_step")
            layouts.append((field["name"], endian + kind[0], offset))
        if not all(name in {item[0] for item in layouts} for name in ("x", "y", "z")):
            raise ValueError("PointCloud2 payload is missing x, y or z")
        if len(self.data) < row_step * height:
            raise ValueError("PointCloud2 payload is shorter than its declared layout")
        emitted = 0
        for row in range(height):
            for col in range(width):
                if emitted >= count:
                    return
                base = row * row_step + col * point_step
                yield {name: struct.unpack_from(fmt, self.data, base + offset)[0]
                       for name, fmt, offset in layouts}
                emitted += 1

    def scan(self) -> dict[str, Any]:
        """扫描当前设备或操作。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any]。
        """
        if self.metadata.get("channel") != "scan":
            raise ValueError("scan() is available for LaserScan frames")
        n = int(self.metadata["count"])
        if len(self.data) != n * 8:
            raise ValueError("LaserScan payload size does not match its range metadata")
        values = struct.unpack("<" + "f" * (2 * n), self.data)
        return {"ranges": list(values[:n]), "intensities": list(values[n:])}


class LidarClient:
    """Authenticated remote client for an EmbodiedNexus lidar device."""

    def __init__(self, base_url: str, token: str, device_id: str = "lidar_main", timeout: float = 5.0,
                 *, websocket_path: str = "/ws"):
        """初始化对象并保存构造参数或依赖。

        Args:
            base_url: 设备服务或 Core 服务的基础 HTTP URL。 类型为 str。
            token: 用于身份验证的访问令牌。 类型为 str。
            device_id: 设备标识符，作为初始化对象并保存构造参数或依赖所需的输入。 类型为 str。
            timeout: 本次网络或服务调用的最长等待时长，通常以秒为单位。 类型为 float。
            websocket_path: websocket路径，作为初始化对象并保存构造参数或依赖所需的输入。 类型为 str。

        Returns:
            无返回值。
        """
        self.base_url = base_url.rstrip("/")
        self.token = token
        self.device_id = device_id
        self.timeout = float(timeout)
        self.websocket_path = websocket_path
        self._prefix = f"/api/v1/lidars/{quote(device_id, safe='')}"

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
            f"{self.base_url}{self._prefix}/{suffix.lstrip('/')}",
            data=data, method=method,
            headers={"Authorization": f"Bearer {self.token}", "Content-Type": "application/json"},
        )
        try:
            with urllib.request.urlopen(request, timeout=self.timeout) as response:
                payload = response.read()
                if response.headers.get_content_type() == "application/json":
                    return json.loads(payload.decode("utf-8"))
                return payload
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode("utf-8", errors="replace")
            raise RuntimeError(f"lidar API returned HTTP {exc.code}: {detail}") from exc

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

    def validate_config(self, patch: dict[str, Any]) -> dict[str, Any]:
        """验证配置。

        Args:
            patch: patch，作为验证配置所需的输入。 类型为 dict[str, Any]。

        Returns:
            布尔值，表示验证配置是否成功或成立。 返回类型为 dict[str, Any]。
        """
        return self._request("POST", "config/validate", patch)

    def apply_config(self, patch: dict[str, Any]) -> dict[str, Any]:
        """应用配置。

        Args:
            patch: patch，作为应用配置所需的输入。 类型为 dict[str, Any]。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any]。
        """
        return self._request("PUT", "config", patch)

    def set_capture(self, capturing: bool) -> dict[str, Any]:
        """设置采集。

        Args:
            capturing: capturing，作为设置采集所需的输入。 类型为 bool。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any]。
        """
        return self._request("PUT", "capture", {"capturing": bool(capturing)})

    def acquire_control(self, owner: str = "python-client", ttl_sec: float | None = None) -> LidarLease:
        """获取控制权。

        Args:
            owner: owner，作为获取控制权所需的输入。 类型为 str。
            ttl_sec: ttlsec，作为获取控制权所需的输入。 类型为 float | None。

        Returns:
            本次处理产生的结果值。 返回类型为 LidarLease。
        """
        body: dict[str, Any] = {"owner": owner}
        if ttl_sec is not None:
            body["ttl_sec"] = float(ttl_sec)
        result = self._request("POST", "control/lease", body)
        return LidarLease(self, result["lease_id"], float(result["expires_in_sec"]))

    def stop_motor(self) -> dict[str, Any]:
        """停止控制电机。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any]。
        """
        return self._request("PUT", "motor", {"enabled": False})

    def subscribe(
        self,
        streams: tuple[str, ...] = ("points", "scan", "status"),
        *,
        points_fps: float = 5.0,
        scan_fps: float = 10.0,
        point_stride: int = 1,
    ) -> Iterator[LidarFrame]:
        """订阅相机数据流并依次产出元数据和二进制图像载荷。

        Args:
            streams: streams，作为订阅相机数据流并依次产出元数据和二进制图像载荷所需的输入。 类型为 tuple[str, ...]。
            points_fps: pointsfps，作为订阅相机数据流并依次产出元数据和二进制图像载荷所需的输入。 类型为 float。
            scan_fps: 扫描fps，作为订阅相机数据流并依次产出元数据和二进制图像载荷所需的输入。 类型为 float。
            point_stride: 点stride，作为订阅相机数据流并依次产出元数据和二进制图像载荷所需的输入。 类型为 int。

        Returns:
            本次处理产生的结果值。 返回类型为 Iterator[LidarFrame]。
        """
        try:
            from websockets.sync.client import connect
        except ImportError as exc:
            raise RuntimeError("install the client extra with: pip install websockets>=12") from exc
        query = urlencode({"streams": ",".join(streams), "points_fps": points_fps,
                           "scan_fps": scan_fps, "point_stride": max(1, int(point_stride))})
        url = self.base_url.replace("https://", "wss://", 1).replace("http://", "ws://", 1) + f"{self.websocket_path}?{query}"
        with connect(url, subprotocols=["en-lidar-v1", f"bearer.{self.token}"],
                     open_timeout=self.timeout, close_timeout=2.0, max_size=None, compression=None) as ws:
            while True:
                message = ws.recv()
                if isinstance(message, bytes):
                    raise RuntimeError("server sent a binary payload without metadata")
                metadata = json.loads(message)
                if metadata.get("type") != "frame":
                    yield LidarFrame(metadata, b"")
                    continue
                payload = ws.recv()
                if not isinstance(payload, bytes):
                    raise RuntimeError("server did not follow frame metadata with a binary payload")
                if len(payload) != int(metadata.get("payload_size", -1)):
                    raise RuntimeError("lidar frame payload size does not match metadata")
                yield LidarFrame(metadata, payload)


class LidarLease:
    """Auto-renewing motor-control lease."""

    def __init__(self, client: LidarClient, lease_id: str, ttl_sec: float):
        """初始化对象并保存构造参数或依赖。

        Args:
            client: 用于发送请求或查询状态的客户端实例。 类型为 LidarClient。
            lease_id: 租约标识符，作为初始化对象并保存构造参数或依赖所需的输入。 类型为 str。
            ttl_sec: ttlsec，作为初始化对象并保存构造参数或依赖所需的输入。 类型为 float。

        Returns:
            无返回值。
        """
        self.client, self.lease_id, self.ttl_sec = client, lease_id, float(ttl_sec)
        self._stop_event = threading.Event()
        self._renew_error = ""
        self._thread = threading.Thread(target=self._renew_loop, name="lidar-lease-renewal", daemon=True)
        self._thread.start()

    @property
    def renew_error(self) -> str:
        """续租error。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。 返回类型为 str。
        """
        return self._renew_error

    def _renew_loop(self) -> None:
        """在后台周期性续租控制权。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        interval = max(min(self.ttl_sec / 3.0, 0.5), 0.05)
        while not self._stop_event.wait(interval):
            try:
                self.client._request("POST", "control/lease/renew", {"lease_id": self.lease_id})
                self._renew_error = ""
            except Exception as exc:
                self._renew_error = str(exc)

    def start_motor(self) -> dict[str, Any]:
        """启动控制电机。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any]。
        """
        return self.client._request("PUT", "motor", {"lease_id": self.lease_id, "enabled": True})

    def set_speed(self, speed_hz: int) -> dict[str, Any]:
        """设置速度。

        Args:
            speed_hz: 目标雷达转速，单位为 Hz。 类型为 int。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any]。
        """
        if speed_hz not in (5, 10, 20):
            raise ValueError("speed_hz must be one of 5, 10, or 20")
        return self.client._request("PUT", "motor/speed", {"lease_id": self.lease_id, "speed_hz": speed_hz})

    def stop_motor(self) -> dict[str, Any]:
        """停止控制电机。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any]。
        """
        return self.client.stop_motor()

    def release(self) -> dict[str, Any]:
        """释放当前设备控制租约。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any]。
        """
        if self._stop_event.is_set():
            return {"released": True}
        self._stop_event.set()
        self._thread.join(timeout=max(self.client.timeout, 1.0))
        result = self.client._request("POST", "control/lease/release", {"lease_id": self.lease_id})
        return result

    def __enter__(self) -> LidarLease:
        """进入上下文管理器并返回当前资源。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。 返回类型为 LidarLease。
        """
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        """退出上下文管理器并释放上下文资源。

        Args:
            exc_type: exc类型，作为退出上下文管理器并释放上下文资源所需的输入。
            exc: 捕获到的异常对象。
            traceback: traceback，作为退出上下文管理器并释放上下文资源所需的输入。

        Returns:
            无返回值。
        """
        self.release()
