from __future__ import annotations

import json
import inspect
import socket
import sys
import tempfile
import threading
import time
import types
import urllib.error
import urllib.request
import unittest
from unittest.mock import patch
from pathlib import Path

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
        options = {
            "host": "127.0.0.1", "port": self.port, "log_level": "critical",
            "lifespan": "off",
        }
        if "ws_max_queue" in inspect.signature(uvicorn.Config).parameters:
            options["ws_max_queue"] = 1
        self.server = uvicorn.Server(uvicorn.Config(app, **options))
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

    def test_core_web_dashboard_and_assets_are_public(self):
        for path, content_type in (("/", "text/html"),
                                   ("/rviz/", "text/html"),
                                   ("/app.css", "text/css"),
                                   ("/app.js", "javascript"),
                                   ("/commands.json", "application/json")):
            response = self._request(path, token="")
            self.assertEqual(response.status, 200, path)
            self.assertIn(content_type, response.headers.get_content_type(), path)
            body = response.read()
            self.assertTrue(body, path)
            if path == "/":
                self.assertIn("Core 运维面板".encode("utf-8"), body)
                self.assertIn(b"/rviz/", body)
            elif path == "/rviz/":
                self.assertIn("Web RViz".encode("utf-8"), body)
            elif path == "/app.js":
                self.assertIn(b"localStorage", body)
                self.assertIn(b"clipboard", body)
            elif path == "/commands.json":
                catalog = json.loads(body)
                self.assertEqual({item["type"] for item in catalog["devices"]},
                                 {"base", "arm", "gripper", "camera", "lidar"})
                self.assertNotIn(b"core-secret", body)
                self.assertNotIn(b"internal-secret", body)

    def test_visualization_status_is_authenticated_and_reports_supported_topics(self):
        class Visualization:
            enabled = True
            port = 9090
            resource_packages = ("description",)

            @staticmethod
            def info():
                return {"enabled": True, "available": True, "port": 9090,
                        "resource_packages": ["description"], "topics": [
                            {"name": "/tf", "type": "tf2_msgs/msg/TFMessage"}], "presets": []}

            @staticmethod
            def topic_types():
                return {"/tf": "tf2_msgs/msg/TFMessage"}

            @staticmethod
            def ready():
                return True

        server = _Server(create_app(self.manager, "core-secret", Visualization()))
        self.addCleanup(server.close)
        url = f"http://127.0.0.1:{server.port}/api/v1/core/visualization"
        request = urllib.request.Request(url)
        with self.assertRaises(urllib.error.HTTPError) as denied_error:
            urllib.request.urlopen(request)
        self.assertEqual(denied_error.exception.code, 401)
        response = urllib.request.urlopen(urllib.request.Request(url, headers={"Authorization": "Bearer core-secret"}))
        self.assertEqual(json.loads(response.read())["topics"][0]["name"], "/tf")

    def test_visualization_resource_route_requires_auth_and_confines_paths(self):
        class Visualization:
            resource_packages = ("description",)

        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        root = Path(temp.name)
        (root / "meshes").mkdir()
        (root / "meshes" / "robot.dae").write_text("model", encoding="utf-8")
        ament = types.ModuleType("ament_index_python")
        packages = types.ModuleType("ament_index_python.packages")
        packages.get_package_share_directory = lambda _name: str(root)
        ament.packages = packages
        server = _Server(create_app(self.manager, "core-secret", Visualization()))
        self.addCleanup(server.close)
        resource_url = f"http://127.0.0.1:{server.port}/api/v1/core/visualization/resources/description/meshes/robot.dae"
        with patch.dict(sys.modules, {"ament_index_python": ament, "ament_index_python.packages": packages}):
            with self.assertRaises(urllib.error.HTTPError) as denied_error:
                urllib.request.urlopen(resource_url)
            self.assertEqual(denied_error.exception.code, 401)
            request = urllib.request.Request(resource_url, headers={"Authorization": "Bearer core-secret"})
            self.assertEqual(urllib.request.urlopen(request).read(), b"model")
            traversal = urllib.request.Request(
                f"http://127.0.0.1:{server.port}/api/v1/core/visualization/resources/description/%2e%2e/robot.dae",
                headers={"Authorization": "Bearer core-secret"})
            with self.assertRaises(urllib.error.HTTPError) as traversal_error:
                urllib.request.urlopen(traversal)
            self.assertEqual(traversal_error.exception.code, 404)
            unallowed = urllib.request.Request(
                f"http://127.0.0.1:{server.port}/api/v1/core/visualization/resources/unlisted/meshes/robot.dae",
                headers={"Authorization": "Bearer core-secret"})
            with self.assertRaises(urllib.error.HTTPError) as unallowed_error:
                urllib.request.urlopen(unallowed)
            self.assertEqual(unallowed_error.exception.code, 404)

    def test_visualization_websocket_forwards_only_subscriptions_and_binary_data(self):
        observed = {}
        topic_graph = {"/points": "sensor_msgs/msg/PointCloud2"}

        async def bridge(socket: WebSocket):
            await socket.accept()
            observed["subscribe"] = json.loads(await socket.receive_text())
            await socket.send_bytes(b"\x81\x01")
            observed["unsubscribe"] = json.loads(await socket.receive_text())
            await socket.send_text("unsubscribed")
            try:
                await socket.receive_text()
            except Exception:
                return

        bridge_server = _Server(Starlette(routes=[WebSocketRoute("/", bridge)]))

        class Visualization:
            enabled = True
            port = bridge_server.port
            resource_packages = ()
            explicit_qos = True

            @staticmethod
            def topic_types():
                return dict(topic_graph)

            @staticmethod
            def ready():
                return True

            @staticmethod
            def info():
                return {"enabled": True, "available": True, "port": bridge_server.port,
                        "resource_packages": [], "topics": [], "presets": []}

        core = _Server(create_app(self.manager, "core-secret", Visualization()))
        self.addCleanup(core.close)
        self.addCleanup(bridge_server.close)
        url = f"ws://127.0.0.1:{core.port}/api/v1/core/visualization/ws"
        with websockets.sync.client.connect(
            url, subprotocols=["en-core-viz-v1", "bearer.core-secret"], open_timeout=3.0,
        ) as client:
            client.send(json.dumps({"op": "subscribe", "topic": "/points", "type": "sensor_msgs/msg/PointCloud2",
                                   "id": "viewer-1", "throttle_rate": 200, "queue_length": 1,
                                   "qos": {"history": "keep_last", "depth": 1,
                                           "reliability": "best_effort", "durability": "volatile"}}))
            self.assertEqual(client.recv(), b"\x81\x01")
            topic_graph.clear()
            client.send(json.dumps({"op": "unsubscribe", "topic": "/points", "id": "viewer-1"}))
            self.assertEqual(client.recv(), "unsubscribed")
            client.send(json.dumps({"op": "publish", "topic": "/cmd_vel", "msg": {}}))
            with self.assertRaises(websockets.exceptions.ConnectionClosedError) as close_error:
                client.recv()
            self.assertEqual(close_error.exception.rcvd.code, 4403)
        self.assertEqual(observed["subscribe"]["compression"], "cbor")
        self.assertEqual(observed["subscribe"]["topic"], "/points")
        self.assertEqual(observed["subscribe"]["qos"]["reliability"], "best_effort")
        self.assertEqual(observed["unsubscribe"]["op"], "unsubscribe")

    def test_command_catalog_matches_mock_and_sim_launch_assets(self):
        assets = Path(__file__).resolve().parents[1] / "web"
        catalog = json.loads((assets / "commands.json").read_text(encoding="utf-8"))
        repo_root = Path(__file__).resolve().parents[4]
        packages = {
            "embodied_nexus_base": "src/base/embodied_nexus_base",
            "embodied_nexus_arm": "src/arm/embodied_nexus_arm",
            "embodied_nexus_gripper": "src/gripper/embodied_nexus_gripper",
            "embodied_nexus_camera": "src/camera/embodied_nexus_camera",
            "embodied_nexus_lidar": "src/lidar/embodied_nexus_lidar",
        }
        for device in catalog["devices"]:
            package_root = repo_root / packages[device["package"]]
            self.assertTrue((package_root / "launch" / device["launch"]).is_file())
            self.assertEqual({profile["id"] for profile in device["profiles"]}, {"mock", "simulation"})
            for profile in device["profiles"]:
                self.assertTrue((repo_root / profile["config"]).is_file(), profile["config"])
                self.assertTrue(profile["config"].endswith(("_mock.yaml", "_sim.yaml")))

        operation_ids = {item["id"] for item in catalog["core"]["operations"]}
        self.assertEqual(operation_ids, {
            "status", "device_start", "device_restart", "device_shutdown",
            "start_all", "stop_all", "shutdown_all", "shutdown_core",
        })
        core_node = (repo_root / "src/core/embodied_nexus_core/embodied_nexus_core/core_node.py").read_text(
            encoding="utf-8"
        )
        operation_commands = {item["id"]: item["command"] for item in catalog["core"]["operations"]}
        self.assertIn("/embodied_nexus/core/get_status", core_node)
        self.assertIn("/embodied_nexus/core/manage_device", core_node)
        self.assertIn("/embodied_nexus/core/manage_all", core_node)
        self.assertIn("embodied_nexus_core_interfaces/srv/GetCoreStatus", operation_commands["status"])
        for operation_id in ("device_start", "device_restart", "device_shutdown"):
            self.assertIn("embodied_nexus_core_interfaces/srv/ManageCoreDevice", operation_commands[operation_id])
        for operation_id in ("start_all", "stop_all", "shutdown_all", "shutdown_core"):
            self.assertIn("embodied_nexus_core_interfaces/srv/ManageAllCoreDevices", operation_commands[operation_id])

    def test_core_management_api_remains_authenticated_with_web_ui(self):
        denied = self._request("/api/v1/core/status", token="")
        self.assertEqual(denied.code, 401)

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
