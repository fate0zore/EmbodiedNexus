from __future__ import annotations

import json
import sys
import types
import unittest
from unittest.mock import patch

from embodied_nexus_client.camera import CameraClient


class FakeWebSocket:
    def __init__(self):
        self.frames = [
            json.dumps({"type": "frame", "channel": "color", "stamp_sec": 1.0}),
            b"jpeg-bytes",
        ]
        self.timeouts = []

    def __enter__(self):
        return self

    def __exit__(self, *_args):
        return False

    def recv(self, timeout=None):
        self.timeouts.append(timeout)
        if not self.frames:
            raise AssertionError("unexpected receive")
        return self.frames.pop(0)


class CameraSdkTests(unittest.TestCase):
    def test_subscribe_bounds_metadata_and_payload_receives(self):
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
