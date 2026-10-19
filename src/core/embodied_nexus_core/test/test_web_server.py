from __future__ import annotations

import json
import socket
import threading
import time
import urllib.error
import urllib.request
import unittest

import uvicorn
import websockets.sync.client
from starlette.applications import Starlette
from starlette.responses import Response
from starlette.routing import Route, WebSocketRoute
from starlette.websockets import WebSocket

from embodied_nexus_core.manager import OperationInProgress
from embodied_nexus_core.web_server import create_app


def _port() -> int:
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


class _Server:
    def __init__(self, app):
        self.port = _port()
        self.server = uvicorn.Server(uvicorn.Config(
            app, host="127.0.0.1", port=self.port, log_level="critical",
            lifespan="off", ws_max_queue=1,
        ))
        self.thread = threading.Thread(target=self.server.run, daemon=True)
        self.thread.start()
        deadline = time.monotonic() + 3.0
        while not self.server.started and self.thread.is_alive() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not self.server.started:
            raise RuntimeError("test ASGI server failed to start")

    def close(self):
        self.server.should_exit = True
        self.thread.join(timeout=3.0)


class _Process:
    pid = 12345

    @staticmethod
    def poll():
        return None


class _Manager:
    def __init__(self, upstream_port):
        self.platform = {"config_file": "platform.yaml", "core": {"port": 8780}, "devices": []}
        self._items = {"base_one": {
            "id": "base_one", "type": "base", "enabled": True,
            "process": _Process(), "port": upstream_port, "token": "internal-secret",
        }}

    def status(self):
        return {"state": "running"}

    def list_devices(self):
        return [{"id": "base_one"}]

    def get_device(self, device_id):
        if device_id not in self._items:
            raise KeyError(device_id)
        return {"id": device_id}

    def start(self, device_id):
        return {"success": True, "device": {"id": device_id}}

    def shutdown(self, device_id):
        return {"success": True, "device": {"id": device_id}}

    def restart(self, device_id):
        return {"success": True, "device": {"id": device_id}}

    def stop_all(self):
        return [{"device_id": "base_one", "success": True}]

    def request_core_shutdown(self):
        return {"success": True, "results": []}


class CoreGatewayTests(unittest.TestCase):
    def setUp(self):
        self.upstream_observed = {}

        async def info(request):
            self.upstream_observed["http_query"] = request.url.query
            self.upstream_observed["http_authorization"] = request.headers.get("authorization")
            self.upstream_observed["http_custom_header"] = request.headers.get("x-correlation-id")
            return Response(b"original-body", media_type="text/plain; charset=utf-8",
                            headers={"Cache-Control": "no-store", "X-Upstream": "preserved"})

        async def ws(socket: WebSocket):
            offered = socket.scope["subprotocols"]
            self.upstream_observed["protocols"] = offered
            self.upstream_observed["ws_query"] = socket.scope["query_string"].decode("ascii")
            if "bearer.internal-secret" not in offered:
                await socket.close(code=4401)
                return
            await socket.accept(subprotocol="en-base-v1")
            if "slow=1" in self.upstream_observed["ws_query"]:
                try:
                    while True:
                        await socket.send_bytes(b"x" * 65536)
                except Exception:
                    return
            await socket.send_text('{"frame":1}')
            await socket.send_bytes(b"\x00\x01payload")
            message = await socket.receive_text()
            await socket.send_text(f"echo:{message}")
            await socket.close()

        self.upstream = _Server(Starlette(routes=[
            Route("/api/v1/bases/base_one/info", info),
            WebSocketRoute("/ws", ws),
        ]))
        self.manager = _Manager(self.upstream.port)
        self.core = _Server(create_app(self.manager, "core-secret"))
        self.base_url = f"http://127.0.0.1:{self.core.port}"

    def tearDown(self):
        self.core.close()
        self.upstream.close()

    def _request(self, path, *, method="GET", token="core-secret", body=None, extra_headers=None):
        data = json.dumps(body).encode() if body is not None else None
        headers = {"Authorization": f"Bearer {token}"} if token else {}
        headers.update(extra_headers or {})
        if data is not None:
            headers["Content-Type"] = "application/json"
        request = urllib.request.Request(self.base_url + path, data=data, method=method, headers=headers)
        try:
            return urllib.request.urlopen(request, timeout=3.0)
        except urllib.error.HTTPError as exc:
            return exc

    def test_core_auth_and_http_proxy_preserve_device_body_and_query(self):
        denied = self._request("/api/v1/core/status", token="")
        self.assertEqual(denied.code, 401)

        response = self._request("/api/v1/bases/base_one/info?mode=raw&name=a%2Fb",
                                 extra_headers={"X-Correlation-ID": "request-42"})
        self.assertEqual(response.status, 200)
        self.assertEqual(response.read(), b"original-body")
        self.assertEqual(response.headers.get_content_type(), "text/plain")
        self.assertEqual(response.headers.get_content_charset(), "utf-8")
        self.assertEqual(response.headers.get("Cache-Control"), "no-store")
        self.assertEqual(response.headers.get("X-Upstream"), "preserved")
        self.assertEqual(self.upstream_observed["http_query"], "mode=raw&name=a%2Fb")
        self.assertEqual(self.upstream_observed["http_authorization"], "Bearer internal-secret")
        self.assertEqual(self.upstream_observed["http_custom_header"], "request-42")

    def test_conflicting_management_action_returns_http_409(self):
        def busy(_device_id):
            raise OperationInProgress("device is busy")

        self.manager.shutdown = busy
        response = self._request("/api/v1/core/devices/base_one", method="POST",
                                 body={"action": "shutdown"})
        self.assertEqual(response.code, 409)
        self.assertIn("device is busy", response.read().decode())

    def test_websocket_proxy_preserves_query_protocol_and_message_order(self):
        url = f"ws://127.0.0.1:{self.core.port}/api/v1/bases/base_one/ws?streams=status&filter=x%2Fy"
        with websockets.sync.client.connect(
            url, subprotocols=["en-base-v1", "bearer.core-secret"], open_timeout=3.0,
        ) as ws:
            self.assertEqual(ws.subprotocol, "en-base-v1")
            self.assertEqual(ws.recv(), '{"frame":1}')
            self.assertEqual(ws.recv(), b"\x00\x01payload")
            ws.send("control")
            self.assertEqual(ws.recv(), "echo:control")
        self.assertEqual(self.upstream_observed["ws_query"], "streams=status&filter=x%2Fy")
        self.assertIn("bearer.internal-secret", self.upstream_observed["protocols"])
        self.assertNotIn("bearer.core-secret", self.upstream_observed["protocols"])

    def test_stop_management_remains_available_with_unread_data_stream(self):
        ready = threading.Event()
        finished = threading.Event()

        def unread_client():
            url = f"ws://127.0.0.1:{self.core.port}/api/v1/bases/base_one/ws?slow=1"
            try:
                with websockets.sync.client.connect(
                    url, subprotocols=["en-base-v1", "bearer.core-secret"], open_timeout=3.0,
                ):
                    ready.set()
                    finished.wait(3.0)
            finally:
                ready.set()

        thread = threading.Thread(target=unread_client, daemon=True)
        thread.start()
        self.assertTrue(ready.wait(3.0))
        started = time.monotonic()
        response = self._request("/api/v1/core/devices/manage_all", method="POST",
                                 body={"action": "stop_all"})
        elapsed = time.monotonic() - started
        finished.set()
        thread.join(timeout=2.0)
        self.assertEqual(response.status, 200)
        self.assertLess(elapsed, 2.0)


if __name__ == "__main__":
    unittest.main()
