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
    """端口当前设备或操作。

    Args:
        无参数。

    Returns:
        本次处理产生的结果值。 返回类型为 int。
    """
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


class _Server:
    def __init__(self, app):
        """初始化对象并保存构造参数或依赖。

        Args:
            app: app，作为初始化对象并保存构造参数或依赖所需的输入。

        Returns:
            无返回值。
        """
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
        """关闭连接并释放相关资源。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        self.server.should_exit = True
        self.thread.join(timeout=3.0)


class _Process:
    pid = 12345

    @staticmethod
    def poll():
        """轮询当前设备或操作。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        return None


class _Manager:
    def __init__(self, upstream_port):
        """初始化对象并保存构造参数或依赖。

        Args:
            upstream_port: upstream端口，作为初始化对象并保存构造参数或依赖所需的输入。

        Returns:
            无返回值。
        """
        self.platform = {"config_file": "platform.yaml", "core": {"port": 8780}, "devices": []}
        self._items = {"base_one": {
            "id": "base_one", "type": "base", "enabled": True,
            "process": _Process(), "port": upstream_port, "token": "internal-secret",
        }}

    def status(self):
        """获取服务当前运行状态。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。
        """
        return {"state": "running"}

    def list_devices(self):
        """列出设备列表。

        Args:
            无参数。

        Returns:
            查询或读取到的数据；未找到时的处理方式由实现决定。
        """
        return [{"id": "base_one"}]

    def get_device(self, device_id):
        """获取设备。

        Args:
            device_id: 设备标识符，作为获取设备所需的输入。

        Returns:
            查询或读取到的数据；未找到时的处理方式由实现决定。
        """
        if device_id not in self._items:
            raise KeyError(device_id)
        return {"id": device_id}

    def start(self, device_id):
        """启动设备或后台任务。

        Args:
            device_id: 设备标识符，作为启动设备或后台任务所需的输入。

        Returns:
            本次处理产生的结果值。
        """
        return {"success": True, "device": {"id": device_id}}

    def shutdown(self, device_id):
        """关闭当前设备或操作。

        Args:
            device_id: 设备标识符，作为关闭当前设备或操作所需的输入。

        Returns:
            本次处理产生的结果值。
        """
        return {"success": True, "device": {"id": device_id}}

    def restart(self, device_id):
        """重启当前设备或操作。

        Args:
            device_id: 设备标识符，作为重启当前设备或操作所需的输入。

        Returns:
            本次处理产生的结果值。
        """
        return {"success": True, "device": {"id": device_id}}

    def stop_all(self):
        """停止all。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。
        """
        return [{"device_id": "base_one", "success": True}]

    def request_core_shutdown(self):
        """请求core关闭。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。
        """
        return {"success": True, "results": []}


class CoreGatewayTests(unittest.TestCase):
    def setUp(self):
        """为当前测试准备所需对象和资源。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。
        """
        self.upstream_observed = {}

        async def info(request):
            """获取设备能力与静态信息。

            Args:
                request: 封装调用输入字段的请求对象。

            Returns:
                本次处理产生的结果值。
            """
            self.upstream_observed["http_query"] = request.url.query
            self.upstream_observed["http_authorization"] = request.headers.get("authorization")
            self.upstream_observed["http_custom_header"] = request.headers.get("x-correlation-id")
            return Response(b"original-body", media_type="text/plain; charset=utf-8",
                            headers={"Cache-Control": "no-store", "X-Upstream": "preserved"})

        async def ws(socket: WebSocket):
            """执行 ws 对应的处理流程。

            Args:
                socket: 套接字，作为执行 ws 对应的处理流程所需的输入。 类型为 WebSocket。

            Returns:
                无返回值。
            """
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
        """清理当前测试创建的对象和资源。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        self.core.close()
        self.upstream.close()

    def _request(self, path, *, method="GET", token="core-secret", body=None, extra_headers=None):
        """向服务端发送 HTTP 请求并解析响应。

        Args:
            path: 目标文件、资源或接口的路径。
            method: HTTP 请求方法，例如 GET 或 POST。
            token: 用于身份验证的访问令牌。
            body: 随请求发送的 JSON 数据。
            extra_headers: extraheaders，作为向服务端发送 HTTP 请求并解析响应所需的输入。

        Returns:
            本次处理产生的结果值。
        """
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

    def test_scene_editor_routes_require_auth_and_support_scene_and_asset_operations(self):
        """Protect the scene editor API and forward CRUD and mesh operations.

        Returns:
            None.
        """
        class SceneEditor:
            """Small scene provider fake for Core route tests."""

            def __init__(self):
                """Initialize in-memory scene and asset records.

                Args:
                    None.

                Returns:
                    None.
                """
                self.scenes = {"default": {"name": "Default", "objects": []}}
                self.assets = {}

            def list_scenes(self):
                """Return saved scene summaries.

                Args:
                    None.

                Returns:
                    Scene identifier and display name pairs.
                """
                return [{"id": key, "name": value["name"]} for key, value in self.scenes.items()]

            def get_scene(self, scene_id):
                """Return one scene mapping.

                Args:
                    scene_id: Stable scene identifier.

                Returns:
                    A copied scene mapping.
                """
                if scene_id not in self.scenes:
                    raise FileNotFoundError(scene_id)
                return dict(self.scenes[scene_id])

            def validate_scene(self, scene_id, scene):
                """Reject invalid candidates and return compilation metadata.

                Args:
                    scene_id: Stable scene identifier.
                    scene: Candidate scene mapping.

                Returns:
                    A compilation summary.
                """
                if scene.get("name") == "Invalid":
                    raise ValueError("invalid scene")
                return {"valid": True, "body_count": 3, "geom_count": 4}

            def save_scene(self, scene_id, scene):
                """Validate and retain a scene mapping.

                Args:
                    scene_id: Stable scene identifier.
                    scene: Candidate scene mapping.

                Returns:
                    The saved scene summary.
                """
                self.validate_scene(scene_id, scene)
                self.scenes[scene_id] = dict(scene)
                return {"id": scene_id, "name": scene["name"]}

            def delete_scene(self, scene_id):
                """Delete one saved scene.

                Args:
                    scene_id: Stable scene identifier.

                Returns:
                    None.
                """
                if scene_id == "default":
                    raise ValueError("default scene cannot be deleted")
                if scene_id not in self.scenes:
                    raise FileNotFoundError(scene_id)
                del self.scenes[scene_id]

            def upload_asset(self, scene_id, filename, contents):
                """Retain raw uploaded mesh bytes.

                Args:
                    scene_id: Stable scene identifier.
                    filename: Asset basename.
                    contents: Raw asset bytes.

                Returns:
                    A scene-relative asset path.
                """
                self.assets[(scene_id, filename)] = contents
                return {"path": f"assets/{scene_id}/{filename}"}

            def read_asset(self, scene_id, filename):
                """Return stored mesh bytes and format.

                Args:
                    scene_id: Stable scene identifier.
                    filename: Asset basename.

                Returns:
                    Raw bytes and lowercase extension.
                """
                return self.assets[(scene_id, filename)], ".obj"

            def robot_model(self):
                """Return a small URDF document for preview route testing.

                Args:
                    None.

                Returns:
                    URDF XML string.
                """
                return "<robot name='test'/>"

        provider = SceneEditor()
        server = _Server(create_app(self.manager, "core-secret", scene_editor=provider))
        self.addCleanup(server.close)
        base = f"http://127.0.0.1:{server.port}/api/v1/core/scenes"
        with self.assertRaises(urllib.error.HTTPError) as unauthorized:
            urllib.request.urlopen(base)
        self.assertEqual(unauthorized.exception.code, 401)

        editor_list_request = urllib.request.Request(base, headers={"Authorization": "Bearer core-secret"})
        self.assertEqual(json.loads(urllib.request.urlopen(editor_list_request).read())["scenes"][0]["id"], "default")
        status_request = urllib.request.Request(f"http://127.0.0.1:{server.port}/api/v1/core/status",
                                                headers={"Authorization": "Bearer core-secret"})
        self.assertTrue(json.loads(urllib.request.urlopen(status_request).read())["scene_editor"]["enabled"])

        editor_request = urllib.request.Request(f"{base}/room-a", data=json.dumps({"name": "Room A", "objects": []}).encode(),
                                                method="PUT", headers={"Authorization": "Bearer core-secret", "Content-Type": "application/json"})
        self.assertEqual(json.loads(urllib.request.urlopen(editor_request).read())["id"], "room-a")
        validate = urllib.request.Request(f"{base}/room-a/validate", data=json.dumps({"name": "Room A", "objects": []}).encode(),
                                          method="POST", headers={"Authorization": "Bearer core-secret", "Content-Type": "application/json"})
        self.assertTrue(json.loads(urllib.request.urlopen(validate).read())["valid"])
        invalid = urllib.request.Request(f"{base}/room-a/validate", data=json.dumps({"name": "Invalid"}).encode(),
                                         method="POST", headers={"Authorization": "Bearer core-secret", "Content-Type": "application/json"})
        with self.assertRaises(urllib.error.HTTPError) as invalid_response:
            urllib.request.urlopen(invalid)
        self.assertEqual(invalid_response.exception.code, 400)

        upload = urllib.request.Request(f"{base}/room-a/assets/table.obj", data=b"v 0 0 0\n", method="PUT",
                                        headers={"Authorization": "Bearer core-secret"})
        self.assertEqual(json.loads(urllib.request.urlopen(upload).read())["path"], "assets/room-a/table.obj")
        download = urllib.request.Request(f"{base}/room-a/assets/table.obj", headers={"Authorization": "Bearer core-secret"})
        self.assertEqual(urllib.request.urlopen(download).read(), b"v 0 0 0\n")
        delete = urllib.request.Request(f"{base}/room-a", method="DELETE", headers={"Authorization": "Bearer core-secret"})
        self.assertEqual(urllib.request.urlopen(delete).status, 204)

    def test_core_auth_and_http_proxy_preserve_device_body_and_query(self):
        """验证coreauth和httpproxypreserve设备请求体和查询。

        Args:
            无参数。

        Returns:
            无返回值。
        """
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
        """验证corewebdashboard和assetsare公开。

        Args:
            无参数。

        Returns:
            无返回值。
        """
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
        """验证visualization状态isauthenticated和reportssupportedtopics。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。
        """
        class Visualization:
            enabled = True
            port = 9090
            resource_packages = ("description",)

            @staticmethod
            def info():
                """获取设备能力与静态信息。

                Args:
                    无参数。

                Returns:
                    本次处理产生的结果值。
                """
                return {"enabled": True, "available": True, "port": 9090,
                        "resource_packages": ["description"], "topics": [
                            {"name": "/tf", "type": "tf2_msgs/msg/TFMessage"}], "presets": []}

            @staticmethod
            def topic_types():
                """话题类型。

                Args:
                    无参数。

                Returns:
                    本次处理产生的结果值。
                """
                return {"/tf": "tf2_msgs/msg/TFMessage"}

            @staticmethod
            def ready():
                """就绪当前设备或操作。

                Args:
                    无参数。

                Returns:
                    本次处理产生的结果值。
                """
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
        """验证visualizationresourcerouterequiresauth和confinespaths。

        Args:
            无参数。

        Returns:
            无返回值。
        """
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

    def test_visualization_resource_route_supports_colcon_symlink_install(self):
        """验证visualizationresourceroutesupportscolconsymlinkinstall。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        class Visualization:
            resource_packages = ("description",)

        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        root = Path(temp.name)
        source = root / "source" / "description"
        source_meshes = source / "meshes"
        share = root / "install" / "share" / "description"
        source_meshes.mkdir(parents=True)
        (source / "package.xml").write_text(
            "<package><name>description</name></package>", encoding="utf-8")
        (source_meshes / "robot.dae").write_text("model", encoding="utf-8")
        (root / "outside.dae").write_text("outside", encoding="utf-8")
        (share / "meshes").mkdir(parents=True)
        (share / "package.xml").symlink_to(source / "package.xml")
        (share / "meshes" / "robot.dae").symlink_to(source_meshes / "robot.dae")
        (share / "meshes" / "escape.dae").symlink_to(root / "outside.dae")

        ament = types.ModuleType("ament_index_python")
        packages = types.ModuleType("ament_index_python.packages")
        packages.get_package_share_directory = lambda _name: str(share)
        ament.packages = packages
        server = _Server(create_app(self.manager, "core-secret", Visualization()))
        self.addCleanup(server.close)
        resource_base = f"http://127.0.0.1:{server.port}/api/v1/core/visualization/resources/description/meshes"
        with patch.dict(sys.modules, {"ament_index_python": ament, "ament_index_python.packages": packages}):
            request = urllib.request.Request(f"{resource_base}/robot.dae",
                                             headers={"Authorization": "Bearer core-secret"})
            self.assertEqual(urllib.request.urlopen(request).read(), b"model")
            escape = urllib.request.Request(f"{resource_base}/escape.dae",
                                            headers={"Authorization": "Bearer core-secret"})
            with self.assertRaises(urllib.error.HTTPError) as escape_error:
                urllib.request.urlopen(escape)
            self.assertEqual(escape_error.exception.code, 404)

    def test_visualization_websocket_forwards_only_subscriptions_and_binary_data(self):
        """验证visualizationwebsocketforwardsonlysubscriptions和binary数据。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。
        """
        observed = {}
        topic_graph = {"/points": "sensor_msgs/msg/PointCloud2"}

        async def bridge(socket: WebSocket):
            """执行 bridge 对应的处理流程。

            Args:
                socket: 套接字，作为执行 bridge 对应的处理流程所需的输入。 类型为 WebSocket。

            Returns:
                无返回值。
            """
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
                """话题类型。

                Args:
                    无参数。

                Returns:
                    本次处理产生的结果值。
                """
                return dict(topic_graph)

            @staticmethod
            def ready():
                """就绪当前设备或操作。

                Args:
                    无参数。

                Returns:
                    本次处理产生的结果值。
                """
                return True

            @staticmethod
            def info():
                """获取设备能力与静态信息。

                Args:
                    无参数。

                Returns:
                    本次处理产生的结果值。
                """
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
        """验证commandcatalogmatchesmock和sim启动assets。

        Args:
            无参数。

        Returns:
            无返回值。
        """
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
        """验证coremanagementapiremainsauthenticated并web界面。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        denied = self._request("/api/v1/core/status", token="")
        self.assertEqual(denied.code, 401)

    def test_conflicting_management_action_returns_http_409(self):
        """验证conflictingmanagement动作returnshttp409。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        def busy(_device_id):
            """构造控制器繁忙状态的模拟响应。

            Args:
                _device_id: 设备标识符，作为执行 busy 对应的处理流程所需的输入。

            Returns:
                无返回值。
            """
            raise OperationInProgress("device is busy")

        self.manager.shutdown = busy
        response = self._request("/api/v1/core/devices/base_one", method="POST",
                                 body={"action": "shutdown"})
        self.assertEqual(response.code, 409)
        self.assertIn("device is busy", response.read().decode())

    def test_websocket_proxy_preserves_query_protocol_and_message_order(self):
        """验证websocketproxypreserves查询协议和消息order。

        Args:
            无参数。

        Returns:
            无返回值。
        """
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
        """验证停止managementremainsavailable并unread数据数据流。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        ready = threading.Event()
        finished = threading.Event()

        def unread_client():
            """建立连接但不读取响应数据的测试客户端。

            Args:
                无参数。

            Returns:
                无返回值。
            """
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
