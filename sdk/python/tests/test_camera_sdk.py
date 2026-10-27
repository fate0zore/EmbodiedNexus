from __future__ import annotations

import json
import sys
import types
import unittest
from unittest.mock import patch

from embodied_nexus_client.camera import CameraClient


class FakeWebSocket:
    def __init__(self):
        """初始化对象并保存构造参数或依赖。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        self.frames = [
            json.dumps({"type": "frame", "channel": "color", "stamp_sec": 1.0}),
            b"jpeg-bytes",
        ]
        self.timeouts = []

    def __enter__(self):
        """进入上下文管理器并返回当前资源。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。
        """
        return self

    def __exit__(self, *_args):
        """退出上下文管理器并释放上下文资源。

        Args:
            *_args: args，作为退出上下文管理器并释放上下文资源所需的输入。

        Returns:
            本次处理产生的结果值。
        """
        return False

    def recv(self, timeout=None):
        """接收并返回下一条 WebSocket 消息。

        Args:
            timeout: 本次网络或服务调用的最长等待时长，通常以秒为单位。

        Returns:
            本次处理产生的结果值。
        """
        self.timeouts.append(timeout)
        if not self.frames:
            raise AssertionError("unexpected receive")
        return self.frames.pop(0)


class CameraSdkTests(unittest.TestCase):
    def test_subscribe_bounds_metadata_and_payload_receives(self):
        """验证订阅boundsmetadata和payloadreceives。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        socket = FakeWebSocket()
        websocket_root = types.ModuleType("websockets")
        sync = types.ModuleType("websockets.sync")
        client = types.ModuleType("websockets.sync.client")
        client.connect = lambda *_args, **_kwargs: socket
        sync.client = client
        websocket_root.sync = sync
        with patch.dict(sys.modules, {
            "websockets": websocket_root,
            "websockets.sync": sync,
            "websockets.sync.client": client,
        }):
            frame = next(CameraClient("http://camera", "token", timeout=0.7).subscribe(("color",)))
        self.assertEqual(frame.data, b"jpeg-bytes")
        self.assertEqual(socket.timeouts, [0.7, 0.7])


if __name__ == "__main__":
    unittest.main()
