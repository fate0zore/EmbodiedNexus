from __future__ import annotations

import json
import threading
import time
import urllib.error
import urllib.request
from urllib.parse import quote
from dataclasses import dataclass
from typing import Any, Iterator


class ArmClient:
    """Authenticated HTTP/WebSocket client for an EmbodiedNexus arm service."""

    def __init__(self, base_url: str, token: str, device_id: str = "arm_main", timeout: float = 60.0,
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
        self._prefix = f"/api/v1/arms/{device_id}"

    def _request(self, method: str, suffix: str = "", body: dict[str, Any] | None = None) -> Any:
        """向服务端发送 HTTP 请求并解析响应。

        Args:
            method: HTTP 请求方法，例如 GET 或 POST。 类型为 str。
            suffix: suffix，作为向服务端发送 HTTP 请求并解析响应所需的输入。 类型为 str。
            body: 随请求发送的 JSON 数据。 类型为 dict[str, Any] | None。

        Returns:
            本次处理产生的结果值。 返回类型为 Any。
        """
        data = json.dumps(body, allow_nan=False).encode("utf-8") if body is not None else None
        headers = {"Authorization": f"Bearer {self.token}"}
        if data is not None:
            headers["Content-Type"] = "application/json"
        req = urllib.request.Request(f"{self.base_url}{self._prefix}/{suffix.lstrip('/')}",
                                     data=data, method=method, headers=headers)
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as response:
                return json.loads(response.read().decode("utf-8"))
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode("utf-8", errors="replace")
            raise RuntimeError(f"arm API returned HTTP {exc.code}: {detail}") from exc

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

    def state(self) -> dict[str, Any]:
        """获取设备当前状态。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any]。
        """
        return self.status()

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
        return self._request("POST", "config/validate", {"patch": patch})

    def apply_config(self, lease_id: str, patch: dict[str, Any]) -> dict[str, Any]:
        """应用配置。

        Args:
            lease_id: 租约标识符，作为应用配置所需的输入。 类型为 str。
            patch: patch，作为应用配置所需的输入。 类型为 dict[str, Any]。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any]。
        """
        return self._request("PUT", "config", {"lease_id": lease_id, "patch": patch})

    def acquire_control(self, owner: str = "python-client", ttl_sec: float | None = None) -> ArmLease:
        """获取控制权。

        Args:
            owner: owner，作为获取控制权所需的输入。 类型为 str。
            ttl_sec: ttlsec，作为获取控制权所需的输入。 类型为 float | None。

        Returns:
            本次处理产生的结果值。 返回类型为 ArmLease。
        """
        body: dict[str, Any] = {"owner": owner}
        if ttl_sec is not None:
            body["ttl_sec"] = float(ttl_sec)
        result = self._request("POST", "control/lease", body)
        return ArmLease(self, result["lease_id"], float(result["expires_in_sec"]))

    def stop(self, reason: str = "Python client stop requested") -> dict[str, Any]:
        """停止设备当前动作。

        Args:
            reason: 原因，作为停止设备当前动作所需的输入。 类型为 str。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any]。
        """
        return self._request("POST", "stop", {"reason": reason})

    def operation(self, operation_id: str) -> ArmOperationHandle:
        """操作当前设备或操作。

        Args:
            operation_id: 操作标识符，作为操作当前设备或操作所需的输入。 类型为 str。

        Returns:
            本次处理产生的结果值。 返回类型为 ArmOperationHandle。
        """
        return ArmOperationHandle(self, operation_id)

    def subscribe_status(self) -> Iterator[dict[str, Any]]:
        """订阅状态。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。 返回类型为 Iterator[dict[str, Any]]。
        """
        try:
            from websockets.sync.client import connect
        except ImportError as exc:
            raise RuntimeError("status subscription requires websockets>=12") from exc
        url = self.base_url.replace("https://", "wss://").replace("http://", "ws://") + self.websocket_path
        with connect(url, subprotocols=["en-arm-v1", f"bearer.{self.token}"],
                     open_timeout=self.timeout) as websocket:
            while True:
                frame = websocket.recv()
                if isinstance(frame, str):
                    yield json.loads(frame)


class ArmLease:
    """Renewing arm control lease. Prefer ``with client.acquire_control()``."""

    def __init__(self, client: ArmClient, lease_id: str, ttl_sec: float) -> None:
        """初始化对象并保存构造参数或依赖。

        Args:
            client: 用于发送请求或查询状态的客户端实例。 类型为 ArmClient。
            lease_id: 租约标识符，作为初始化对象并保存构造参数或依赖所需的输入。 类型为 str。
            ttl_sec: ttlsec，作为初始化对象并保存构造参数或依赖所需的输入。 类型为 float。

        Returns:
            无返回值。
        """
        self.client = client
        self.lease_id = lease_id
        self.ttl_sec = ttl_sec
        self._stop_event = threading.Event()
        self._renew_error = ""
        self._thread = threading.Thread(target=self._renew_loop, name="arm-lease-renewal", daemon=True)
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

    def enable_motion(self, enabled: bool = True) -> dict[str, Any]:
        """启用motion。

        Args:
            enabled: 表示对应功能是否启用的布尔值。 类型为 bool。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any]。
        """
        return self.client._request("PUT", "motion/enabled", {
            "lease_id": self.lease_id, "enabled": bool(enabled)})

    def move_joints(self, positions_rad: list[float], speed_rad_sec: float = 0.0,
                    acceleration_rad_sec2: float = 0.0, timeout_sec: float = 0.0) -> ArmOperationHandle:
        """运动joints。

        Args:
            positions_rad: positionsrad，作为运动joints所需的输入。 类型为 list[float]。
            speed_rad_sec: 速度radsec，作为运动joints所需的输入。 类型为 float。
            acceleration_rad_sec2: accelerationradsec2，作为运动joints所需的输入。 类型为 float。
            timeout_sec: 本次调用的最长等待时长，单位为秒。 类型为 float。

        Returns:
            本次处理产生的结果值。 返回类型为 ArmOperationHandle。
        """
        result = self.client._request("POST", "operations", {
            "type": "move_joints", "lease_id": self.lease_id,
            "positions_rad": [float(v) for v in positions_rad],
            "speed_rad_sec": float(speed_rad_sec),
            "acceleration_rad_sec2": float(acceleration_rad_sec2),
            "timeout_sec": float(timeout_sec)})
        return self.client.operation(result["operation_id"])

    def set_joint_velocity(self, velocity_rad_sec: list[float]) -> dict[str, Any]:
        """设置关节速度。

        Args:
            velocity_rad_sec: 速度radsec，作为设置关节速度所需的输入。 类型为 list[float]。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any]。
        """
        return self.client._request("PUT", "joint_velocity", {
            "lease_id": self.lease_id,
            "velocity_rad_sec": [float(v) for v in velocity_rad_sec]})

    def power_on(self) -> dict[str, Any]:
        """电源on。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any]。
        """
        return self._lifecycle("power_on")

    def startup(self) -> dict[str, Any]:
        """等待设备服务启动并确认接口就绪。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any]。
        """
        return self._lifecycle("startup")

    def power_off(self) -> dict[str, Any]:
        """电源off。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any]。
        """
        return self._lifecycle("power_off")

    def _lifecycle(self, command: str) -> dict[str, Any]:
        """设置设备生命周期状态。

        Args:
            command: command，作为设置设备生命周期状态所需的输入。 类型为 str。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any]。
        """
        return self.client._request("POST", "lifecycle", {
            "lease_id": self.lease_id, "command": command})

    def set_teach_mode(self, enabled: bool = True) -> dict[str, Any]:
        """设置示教mode。

        Args:
            enabled: 表示对应功能是否启用的布尔值。 类型为 bool。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any]。
        """
        return self.client._request("PUT", "teach_mode", {
            "lease_id": self.lease_id, "enabled": bool(enabled)})

    def release(self) -> None:
        """释放当前设备控制租约。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        if self._stop_event.is_set():
            return
        self._stop_event.set()
        self._thread.join(timeout=max(self.client.timeout, 1.0))
        self.client._request("POST", "control/lease/release", {"lease_id": self.lease_id})

    def __enter__(self) -> ArmLease:
        """进入上下文管理器并返回当前资源。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。 返回类型为 ArmLease。
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


@dataclass
class ArmOperationHandle:
    client: ArmClient
    operation_id: str

    def status(self) -> dict[str, Any]:
        """获取服务当前运行状态。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any]。
        """
        return self.client._request("GET", f"operations/{self.operation_id}")

    def cancel(self) -> dict[str, Any]:
        """取消当前设备或操作。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any]。
        """
        return self.client._request("DELETE", f"operations/{self.operation_id}")

    def wait(self, timeout: float | None = None, poll_interval: float = 0.1) -> dict[str, Any]:
        """等待当前设备或操作。

        Args:
            timeout: 本次网络或服务调用的最长等待时长，通常以秒为单位。 类型为 float | None。
            poll_interval: 轮询interval，作为等待当前设备或操作所需的输入。 类型为 float。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any]。
        """
        deadline = None if timeout is None else time.monotonic() + float(timeout)
        while True:
            state = self.status()
            if state.get("state") in {"succeeded", "failed", "cancelled"}:
                return state
            if deadline is not None and time.monotonic() >= deadline:
                raise TimeoutError(f"arm operation {self.operation_id} did not finish before timeout")
            time.sleep(max(float(poll_interval), 0.01))
