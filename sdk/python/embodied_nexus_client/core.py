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
        """初始化对象并保存构造参数或依赖。

        Args:
            base_url: 设备服务或 Core 服务的基础 HTTP URL。 类型为 str。
            token: 用于身份验证的访问令牌。 类型为 str。
            timeout: 本次网络或服务调用的最长等待时长，通常以秒为单位。 类型为 float。
            management_timeout: 设备启动、停止等管理请求的最长等待时长，单位为秒。 类型为 float。

        Returns:
            无返回值。
        """
        self.base_url = base_url.rstrip("/")
        self.token = token
        self.timeout = float(timeout)
        self.management_timeout = float(management_timeout)

    def _request(self, method: str, path: str, body: dict[str, Any] | None = None,
                 *, timeout: float | None = None) -> Any:
        """向服务端发送 HTTP 请求并解析响应。

        Args:
            method: HTTP 请求方法，例如 GET 或 POST。 类型为 str。
            path: 目标文件、资源或接口的路径。 类型为 str。
            body: 随请求发送的 JSON 数据。 类型为 dict[str, Any] | None。
            timeout: 本次网络或服务调用的最长等待时长，通常以秒为单位。 类型为 float | None。

        Returns:
            本次处理产生的结果值。 返回类型为 Any。
        """
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
        """获取服务当前运行状态。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any]。
        """
        return self._request("GET", "/api/v1/core/status")

    def devices(self) -> list[dict[str, Any]]:
        """设备列表当前设备或操作。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。 返回类型为 list[dict[str, Any]]。
        """
        return self._request("GET", "/api/v1/core/devices")["devices"]

    def device(self, device_id: str) -> dict[str, Any]:
        """设备当前设备或操作。

        Args:
            device_id: 设备标识符，作为设备当前设备或操作所需的输入。 类型为 str。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any]。
        """
        path = f"/api/v1/core/devices/{quote(device_id, safe='')}"
        return self._request("GET", path)

    def manage_device(self, device_id: str, action: str) -> dict[str, Any]:
        """管理设备。

        Args:
            device_id: 设备标识符，作为管理设备所需的输入。 类型为 str。
            action: 动作，作为管理设备所需的输入。 类型为 str。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any]。
        """
        if action not in {"start", "shutdown", "restart"}:
            raise ValueError("action must be start, shutdown or restart")
        path = f"/api/v1/core/devices/{quote(device_id, safe='')}"
        return self._request("POST", path, {"action": action}, timeout=self.management_timeout)

    def start(self, device_id: str) -> dict[str, Any]:
        """启动设备或后台任务。

        Args:
            device_id: 设备标识符，作为启动设备或后台任务所需的输入。 类型为 str。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any]。
        """
        return self.manage_device(device_id, "start")

    def shutdown(self, device_id: str) -> dict[str, Any]:
        """关闭当前设备或操作。

        Args:
            device_id: 设备标识符，作为关闭当前设备或操作所需的输入。 类型为 str。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any]。
        """
        return self.manage_device(device_id, "shutdown")

    def restart(self, device_id: str) -> dict[str, Any]:
        """重启当前设备或操作。

        Args:
            device_id: 设备标识符，作为重启当前设备或操作所需的输入。 类型为 str。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any]。
        """
        return self.manage_device(device_id, "restart")

    def _manage_all(self, action: str) -> dict[str, Any]:
        """管理all。

        Args:
            action: 动作，作为管理all所需的输入。 类型为 str。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any]。
        """
        return self._request("POST", "/api/v1/core/devices/manage_all", {"action": action},
                             timeout=self.management_timeout)

    def start_all(self) -> dict[str, Any]:
        """启动all。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any]。
        """
        return self._manage_all("start_all")

    def shutdown_all(self) -> dict[str, Any]:
        """关闭all。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any]。
        """
        return self._manage_all("shutdown_all")

    def stop_all(self) -> dict[str, Any]:
        """停止all。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any]。
        """
        return self._manage_all("stop_all")

    def shutdown_core(self) -> dict[str, Any]:
        """关闭core。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any]。
        """
        return self._request("POST", "/api/v1/core/shutdown", {}, timeout=self.management_timeout)

    def configuration(self) -> dict[str, Any]:
        """获取当前设备配置。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any]。
        """
        return self._request("GET", "/api/v1/core/config")

    def validate_configuration(self, config_file: str | None = None) -> dict[str, Any]:
        """验证配置。

        Args:
            config_file: 配置文件，作为验证配置所需的输入。 类型为 str | None。

        Returns:
            布尔值，表示验证配置是否成功或成立。 返回类型为 dict[str, Any]。
        """
        return self._request("POST", "/api/v1/core/config/validate", {"config_file": config_file})

    def _websocket_path(self, kind: str, device_id: str) -> str:
        """生成设备数据流对应的 WebSocket 路径。

        Args:
            kind: kind，作为执行 _websocket_path 对应的处理流程所需的输入。 类型为 str。
            device_id: 设备标识符，作为执行 _websocket_path 对应的处理流程所需的输入。 类型为 str。

        Returns:
            本次处理产生的结果值。 返回类型为 str。
        """
        plural = {"base": "bases", "arm": "arms", "gripper": "grippers",
                  "camera": "cameras", "lidar": "lidars"}[kind]
        return f"/api/v1/{plural}/{quote(device_id, safe='')}/ws"

    def base(self, device_id: str = "base_main") -> BaseClient:
        """底盘当前设备或操作。

        Args:
            device_id: 设备标识符，作为底盘当前设备或操作所需的输入。 类型为 str。

        Returns:
            本次处理产生的结果值。 返回类型为 BaseClient。
        """
        return BaseClient(self.base_url, self.token, device_id,
                          websocket_path=self._websocket_path("base", device_id))

    def arm(self, device_id: str = "arm_main") -> ArmClient:
        """机械臂当前设备或操作。

        Args:
            device_id: 设备标识符，作为机械臂当前设备或操作所需的输入。 类型为 str。

        Returns:
            本次处理产生的结果值。 返回类型为 ArmClient。
        """
        return ArmClient(self.base_url, self.token, device_id,
                         websocket_path=self._websocket_path("arm", device_id))

    def gripper(self, device_id: str = "gripper_main") -> GripperClient:
        """夹爪当前设备或操作。

        Args:
            device_id: 设备标识符，作为夹爪当前设备或操作所需的输入。 类型为 str。

        Returns:
            本次处理产生的结果值。 返回类型为 GripperClient。
        """
        return GripperClient(self.base_url, self.token, device_id)

    def camera(self, device_id: str = "camera_front") -> CameraClient:
        """相机当前设备或操作。

        Args:
            device_id: 设备标识符，作为相机当前设备或操作所需的输入。 类型为 str。

        Returns:
            本次处理产生的结果值。 返回类型为 CameraClient。
        """
        return CameraClient(self.base_url, self.token, device_id,
                            websocket_path=self._websocket_path("camera", device_id))

    def lidar(self, device_id: str = "lidar_main") -> LidarClient:
        """雷达当前设备或操作。

        Args:
            device_id: 设备标识符，作为雷达当前设备或操作所需的输入。 类型为 str。

        Returns:
            本次处理产生的结果值。 返回类型为 LidarClient。
        """
        return LidarClient(self.base_url, self.token, device_id,
                           websocket_path=self._websocket_path("lidar", device_id))
