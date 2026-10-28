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


class AuboJsonRpc:
    """Synchronous Aubo JSON-RPC client that frames fragmented TCP responses."""

    def __init__(self, host: str, port: int, timeout: float) -> None:
        """初始化对象并保存构造参数或依赖。

        Args:
            host: 目标主机名称或 IP 地址。 类型为 str。
            port: 网络服务使用的端口号。 类型为 int。
            timeout: 本次网络或服务调用的最长等待时长，通常以秒为单位。 类型为 float。

        Returns:
            无返回值。
        """
        self.host, self.port, self.timeout = host, int(port), float(timeout)
        self.robot_name = "rob1"
        self._socket: socket.socket | None = None
        self._buffer = ""
        self._decoder = JSONDecoder()
        self._text_decoder = codecs.getincrementaldecoder("utf-8")()
        self._request_id = 0
        self._lock = threading.RLock()

    def connect(self) -> None:
        """连接当前设备或操作。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        self.close()
        self._socket = socket.create_connection((self.host, self.port), timeout=self.timeout)
        self._socket.settimeout(self.timeout)
        names = self.call("getRobotNames")
        if isinstance(names, list) and names:
            self.robot_name = str(names[0])

    def close(self) -> None:
        """关闭连接并释放相关资源。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        sock, self._socket = self._socket, None
        self._buffer = ""
        self._text_decoder.reset()
        if sock is not None:
            sock.close()

    def call(self, method: str, params: list[Any] | None = None) -> Any:
        """调用当前设备或操作。

        Args:
            method: HTTP 请求方法，例如 GET 或 POST。 类型为 str。
            params: params，作为调用当前设备或操作所需的输入。 类型为 list[Any] | None。

        Returns:
            本次处理产生的结果值。 返回类型为 Any。
        """
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
        """通过机器人 RPC 接口调用指定方法并返回响应。

        Args:
            suffix: suffix，作为执行 robot_call 对应的处理流程所需的输入。 类型为 str。
            params: params，作为执行 robot_call 对应的处理流程所需的输入。 类型为 list[Any] | None。

        Returns:
            本次处理产生的结果值。 返回类型为 Any。
        """
        return self.call(f"{self.robot_name}.{suffix}", params)

    def _read_response(self, request_id: int) -> dict[str, Any]:
        """读取响应。

        Args:
            request_id: 请求标识符，作为读取响应所需的输入。 类型为 int。

        Returns:
            查询或读取到的数据；未找到时的处理方式由实现决定。 返回类型为 dict[str, Any]。
        """
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
