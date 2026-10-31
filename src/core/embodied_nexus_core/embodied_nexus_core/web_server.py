from __future__ import annotations

import asyncio
import hmac
import json
import mimetypes
import re
import threading
import xml.etree.ElementTree as ET
from pathlib import Path
from pathlib import PurePosixPath
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.parse import quote
from urllib.request import Request, urlopen

import websockets
from starlette.applications import Starlette
from starlette.requests import Request as StarletteRequest
from starlette.responses import JSONResponse, Response
from starlette.routing import Mount, Route, WebSocketRoute
from starlette.staticfiles import StaticFiles
from starlette.websockets import WebSocket, WebSocketDisconnect

from .config import DEVICE_TYPES, public_platform
from .manager import OperationInProgress


def _authorized(request: StarletteRequest, token: str) -> bool:
    """授权当前设备或操作。

    Args:
        request: 封装调用输入字段的请求对象。 类型为 StarletteRequest。
        token: 用于身份验证的访问令牌。 类型为 str。

    Returns:
        布尔值，表示授权当前设备或操作是否成功或成立。
    """
    return hmac.compare_digest(request.headers.get("authorization", ""), f"Bearer {token}")


def create_app(manager: Any, token: str, visualization: Any = None,
               scene_editor: Any = None) -> Starlette:
    """创建app。

    Args:
        manager: manager，作为创建app所需的输入。 类型为 Any。
        token: 用于身份验证的访问令牌。 类型为 str。
        visualization: visualization，作为创建app所需的输入。 类型为 Any。
        scene_editor: Optional scene library provider for the authenticated scene editor.

    Returns:
        构造或创建完成的对象或资源。 返回类型为 Starlette。
    """
    if not token:
        raise ValueError("core gateway requires a token from the configured environment variable")

    async def auth(request: StarletteRequest) -> JSONResponse | None:
        """执行 auth 对应的处理流程。

        Args:
            request: 封装调用输入字段的请求对象。 类型为 StarletteRequest。

        Returns:
            本次处理产生的结果值。 返回类型为 JSONResponse | None。
        """
        return None if _authorized(request, token) else JSONResponse({"error": "unauthorized"}, status_code=401)

    async def status(request: StarletteRequest):
        """获取服务当前运行状态。

        Args:
            request: 封装调用输入字段的请求对象。 类型为 StarletteRequest。

        Returns:
            本次处理产生的结果值。
        """
        if denied := await auth(request):
            return denied
        payload = await asyncio.to_thread(manager.status)
        payload["scene_editor"] = {"enabled": scene_editor is not None}
        return JSONResponse(payload)

    async def devices(request: StarletteRequest):
        """设备列表当前设备或操作。

        Args:
            request: 封装调用输入字段的请求对象。 类型为 StarletteRequest。

        Returns:
            本次处理产生的结果值。
        """
        if denied := await auth(request):
            return denied
        return JSONResponse({"devices": await asyncio.to_thread(manager.list_devices)})

    async def device(request: StarletteRequest):
        """设备当前设备或操作。

        Args:
            request: 封装调用输入字段的请求对象。 类型为 StarletteRequest。

        Returns:
            本次处理产生的结果值。
        """
        if denied := await auth(request):
            return denied
        try:
            state = await asyncio.to_thread(manager.get_device, request.path_params["device_id"])
            return JSONResponse(state)
        except KeyError as exc:
            return JSONResponse({"error": str(exc)}, status_code=404)

    async def config(request: StarletteRequest):
        """获取当前配置。

        Args:
            request: 封装调用输入字段的请求对象。 类型为 StarletteRequest。

        Returns:
            本次处理产生的结果值。
        """
        if denied := await auth(request):
            return denied
        return JSONResponse(public_platform(manager.platform))

    async def visualization_info(request: StarletteRequest):
        """执行 visualization_info 对应的处理流程。

        Args:
            request: 封装调用输入字段的请求对象。 类型为 StarletteRequest。

        Returns:
            本次处理产生的结果值。
        """
        if denied := await auth(request):
            return denied
        if visualization is None:
            return JSONResponse({"enabled": False, "available": False, "port": 0,
                                 "error": "Web RViz 未配置", "topics": [], "presets": []})
        return JSONResponse(await asyncio.to_thread(visualization.info))

    async def visualization_resource(request: StarletteRequest):
        """执行 visualization_resource 对应的处理流程。

        Args:
            request: 封装调用输入字段的请求对象。 类型为 StarletteRequest。

        Returns:
            本次处理产生的结果值。
        """
        if denied := await auth(request):
            return denied
        package = request.path_params["package"]
        relative = request.path_params["path"]
        if visualization is None or package not in visualization.resource_packages:
            return JSONResponse({"error": "model resource package is not allowed"}, status_code=404)
        parts = PurePosixPath(relative)
        if (parts.is_absolute() or "\\" in relative or ".." in parts.parts or
                Path(relative).suffix.lower() not in {".dae", ".stl", ".obj", ".mtl", ".urdf", ".xacro", ".png", ".jpg", ".jpeg"}):
            return JSONResponse({"error": "invalid model resource path"}, status_code=404)
        try:
            from ament_index_python.packages import get_package_share_directory
            root = Path(get_package_share_directory(package)).resolve()
            allowed_roots = [root]
            manifest = root / "package.xml"
            if manifest.is_symlink():
                source_manifest = manifest.resolve(strict=True)
                source_package = ET.parse(source_manifest).getroot().findtext("name", "")
                source_root = source_manifest.parent.resolve()
                if source_package == package and source_root != root:
                    allowed_roots.append(source_root)
            target = root.joinpath(*parts.parts).resolve(strict=True)
            if not any(target.is_relative_to(allowed_root) for allowed_root in allowed_roots):
                raise ValueError("model resource escapes its installed package")
            if not target.is_file():
                raise FileNotFoundError(relative)
            return Response(target.read_bytes(), media_type=mimetypes.guess_type(target.name)[0] or "application/octet-stream",
                            headers={"Cache-Control": "private, max-age=3600", "X-Content-Type-Options": "nosniff"})
        except (ImportError, FileNotFoundError, ValueError, OSError):
            return JSONResponse({"error": "model resource not found"}, status_code=404)

    async def scene_list(request: StarletteRequest):
        """Return saved scene summaries to an authenticated browser client.

        Args:
            request: Incoming Starlette request.

        Returns:
            JSON scene summaries, or an authorization or unavailable response.
        """
        if denied := await auth(request):
            return denied
        if scene_editor is None:
            return JSONResponse({"error": "scene editor is not enabled"}, status_code=404)
        return JSONResponse({"scenes": await asyncio.to_thread(scene_editor.list_scenes)})

    async def scene_robot_description(request: StarletteRequest):
        """Return the robot URDF used by the MuJoCo preview.

        Args:
            request: Incoming Starlette request.

        Returns:
            The calibrated URDF XML, or an authorization or unavailable response.
        """
        if denied := await auth(request):
            return denied
        if scene_editor is None:
            return JSONResponse({"error": "scene editor is not enabled"}, status_code=404)
        return Response(await asyncio.to_thread(scene_editor.robot_model), media_type="application/xml",
                        headers={"Cache-Control": "no-store", "X-Content-Type-Options": "nosniff"})

    async def scene_get(request: StarletteRequest):
        """Load one named scene for the browser editor.

        Args:
            request: Incoming Starlette request.

        Returns:
            The requested scene mapping or a JSON error response.
        """
        if denied := await auth(request):
            return denied
        if scene_editor is None:
            return JSONResponse({"error": "scene editor is not enabled"}, status_code=404)
        try:
            scene = await asyncio.to_thread(scene_editor.get_scene, request.path_params["scene_id"])
            return JSONResponse(scene)
        except FileNotFoundError as exc:
            return JSONResponse({"error": str(exc)}, status_code=404)
        except ValueError as exc:
            return JSONResponse({"error": str(exc)}, status_code=400)

    async def scene_validate(request: StarletteRequest):
        """Validate scene YAML structure and compile the corresponding MJCF.

        Args:
            request: Incoming Starlette request containing a scene JSON mapping.

        Returns:
            A validation result, or a JSON error response.
        """
        if denied := await auth(request):
            return denied
        if scene_editor is None:
            return JSONResponse({"error": "scene editor is not enabled"}, status_code=404)
        try:
            body = await request.json()
            result = await asyncio.to_thread(
                scene_editor.validate_scene, request.path_params["scene_id"], body,
            )
            return JSONResponse(result)
        except Exception as exc:
            return JSONResponse({"valid": False, "error": str(exc)}, status_code=400)

    async def scene_save(request: StarletteRequest):
        """Validate and atomically save one named scene.

        Args:
            request: Incoming Starlette request containing a scene JSON mapping.

        Returns:
            The saved scene summary, or a JSON error response.
        """
        if denied := await auth(request):
            return denied
        if scene_editor is None:
            return JSONResponse({"error": "scene editor is not enabled"}, status_code=404)
        try:
            body = await request.json()
            result = await asyncio.to_thread(
                scene_editor.save_scene, request.path_params["scene_id"], body,
            )
            return JSONResponse(result)
        except Exception as exc:
            return JSONResponse({"error": str(exc)}, status_code=400)

    async def scene_apply(request: StarletteRequest):
        """Validate and save a scene draft, then apply it in the running simulator.

        Args:
            request: Authenticated request containing the current scene draft.

        Returns:
            Saved and applied scene status, or a JSON error response.
        """
        if denied := await auth(request):
            return denied
        if scene_editor is None:
            return JSONResponse({"error": "scene editor is not enabled"}, status_code=404)
        try:
            body = await request.json()
            result = await asyncio.to_thread(
                scene_editor.save_and_apply_scene, request.path_params["scene_id"], body,
            )
            return JSONResponse(result, status_code=200 if result["applied"] else 503)
        except Exception as exc:
            return JSONResponse(
                {"saved": False, "applied": False, "error": str(exc)}, status_code=400)

    async def scene_delete(request: StarletteRequest):
        """Delete a named scene through the scene library provider.

        Args:
            request: Incoming Starlette request naming the scene to delete.

        Returns:
            An empty success response or a JSON error response.
        """
        if denied := await auth(request):
            return denied
        if scene_editor is None:
            return JSONResponse({"error": "scene editor is not enabled"}, status_code=404)
        try:
            await asyncio.to_thread(scene_editor.delete_scene, request.path_params["scene_id"])
            return Response(status_code=204)
        except FileNotFoundError as exc:
            return JSONResponse({"error": str(exc)}, status_code=404)
        except ValueError as exc:
            return JSONResponse({"error": str(exc)}, status_code=400)

    async def scene_asset(request: StarletteRequest):
        """Upload or read a scene-owned STL or OBJ asset.

        Args:
            request: Incoming Starlette request containing an asset name and optional bytes.

        Returns:
            Asset metadata or bytes, or a JSON error response.
        """
        if denied := await auth(request):
            return denied
        if scene_editor is None:
            return JSONResponse({"error": "scene editor is not enabled"}, status_code=404)
        scene_id, filename = request.path_params["scene_id"], request.path_params["filename"]
        try:
            if request.method == "PUT":
                contents = await request.body()
                if len(contents) > 100 * 1024 * 1024:
                    return JSONResponse({"error": "mesh upload exceeds 100 MiB"}, status_code=413)
                result = await asyncio.to_thread(scene_editor.upload_asset, scene_id, filename, contents)
                return JSONResponse(result)
            contents, extension = await asyncio.to_thread(scene_editor.read_asset, scene_id, filename)
            mime = "model/stl" if extension == ".stl" else "text/plain; charset=utf-8"
            return Response(contents, media_type=mime, headers={"Cache-Control": "no-store",
                                                                 "X-Content-Type-Options": "nosniff"})
        except FileNotFoundError as exc:
            return JSONResponse({"error": str(exc)}, status_code=404)
        except (ValueError, OSError) as exc:
            return JSONResponse({"error": str(exc)}, status_code=400)

    async def visualization_ws(websocket: WebSocket):
        """执行 visualization_ws 对应的处理流程。

        Args:
            websocket: websocket，作为执行 visualization_ws 对应的处理流程所需的输入。 类型为 WebSocket。

        Returns:
            无返回值。
        """
        offered = [part.strip() for part in websocket.headers.get("sec-websocket-protocol", "").split(",")]
        supplied = next((part[7:] for part in offered if part.startswith("bearer.")), "")
        expected = "en-core-viz-v1"
        if expected not in offered or not hmac.compare_digest(supplied, token):
            await websocket.close(code=4401)
            return
        if visualization is None or not visualization.ready():
            await websocket.close(code=1013)
            return
        try:
            async with websockets.connect(
                f"ws://127.0.0.1:{visualization.port}", open_timeout=2.0, close_timeout=1.0,
                max_queue=2, max_size=32 * 1024 * 1024, compression=None,
            ) as upstream:
                await websocket.accept(subprotocol=expected)
                client_subscriptions: dict[str, str] = {}
                async def client_to_rosbridge():
                    """客户端torosbridge。

                    Args:
                        无参数。

                    Returns:
                        无返回值。
                    """
                    while True:
                        incoming = await websocket.receive()
                        if incoming["type"] == "websocket.disconnect":
                            return
                        payload = incoming.get("text")
                        if payload is None:
                            await websocket.close(code=4403, reason="JSON subscribe messages only")
                            return
                        try:
                            message = json.loads(payload)
                        except (TypeError, ValueError):
                            await websocket.close(code=4403, reason="invalid JSON")
                            return
                        if not isinstance(message, dict) or message.get("op") not in {"subscribe", "unsubscribe"}:
                            await websocket.close(code=4403, reason="read-only topic subscriptions only")
                            return
                        topic = message.get("topic")
                        if not isinstance(topic, str):
                            await websocket.send_text(json.dumps({"op": "core_error", "error": "invalid topic"}))
                            continue
                        if message["op"] == "unsubscribe":
                            subscription_id = message.get("id")
                            if topic not in client_subscriptions or subscription_id != client_subscriptions[topic]:
                                await websocket.send_text(json.dumps({"op": "core_error", "error": "topic is not subscribed"}))
                                continue
                            forwarded = {"op": "unsubscribe", "topic": topic, "id": subscription_id}
                            client_subscriptions.pop(topic, None)
                            await upstream.send(json.dumps(forwarded, separators=(",", ":")))
                            continue
                        topic_types = await asyncio.to_thread(visualization.topic_types)
                        if topic not in topic_types:
                            await websocket.send_text(json.dumps({"op": "core_error", "error": "unsupported or unavailable topic"}))
                            continue
                        if len(client_subscriptions) >= 16 or topic in client_subscriptions:
                            await websocket.send_text(json.dumps({"op": "core_error", "error": "subscription limit reached or topic already subscribed"}))
                            continue
                        subscription_id = message.get("id")
                        if not isinstance(subscription_id, str) or len(subscription_id) > 128 or not re.fullmatch(r"[A-Za-z0-9._:-]+", subscription_id):
                            await websocket.send_text(json.dumps({"op": "core_error", "error": "invalid subscription id"}))
                            continue
                        msg_type = topic_types[topic]
                        if message.get("type", msg_type) != msg_type:
                            await websocket.send_text(json.dumps({"op": "core_error", "error": "topic type does not match ROS graph"}))
                            continue
                        qos = message.get("qos")
                        if qos is not None and (
                            not isinstance(qos, dict) or set(qos) - {"history", "depth", "reliability", "durability"} or
                            qos.get("history", "keep_last") not in {"keep_last", "keep_all"} or
                            qos.get("reliability", "best_effort") not in {"reliable", "best_effort", "best_available"} or
                            qos.get("durability", "volatile") not in {"transient_local", "volatile", "best_available"} or
                            not isinstance(qos.get("depth", 10), int) or isinstance(qos.get("depth", 10), bool) or
                            not 1 <= qos.get("depth", 10) <= 100
                        ):
                            await websocket.send_text(json.dumps({"op": "core_error", "error": "invalid subscription QoS"}))
                            continue
                        throttle = message.get("throttle_rate", 0)
                        queue = message.get("queue_length", 1)
                        if (not isinstance(throttle, int) or isinstance(throttle, bool) or not 0 <= throttle <= 60000 or
                                not isinstance(queue, int) or isinstance(queue, bool) or not 1 <= queue <= 10):
                            await websocket.send_text(json.dumps({"op": "core_error", "error": "invalid subscription limits"}))
                            continue
                        forwarded = {"op": "subscribe", "topic": topic, "type": msg_type,
                                     "compression": "cbor", "throttle_rate": throttle,
                                     "queue_length": queue}
                        forwarded["id"] = subscription_id
                        if "qos" in message and getattr(visualization, "explicit_qos", False):
                            forwarded["qos"] = message["qos"]
                        client_subscriptions[topic] = subscription_id
                        await upstream.send(json.dumps(forwarded, separators=(",", ":")))

                async def rosbridge_to_client():
                    """执行 rosbridge_to_client 对应的处理流程。

                    Args:
                        无参数。

                    Returns:
                        无返回值。
                    """
                    async for message in upstream:
                        if isinstance(message, bytes):
                            await websocket.send_bytes(message)
                        else:
                            await websocket.send_text(message)

                client_task = asyncio.create_task(client_to_rosbridge())
                bridge_task = asyncio.create_task(rosbridge_to_client())
                tasks = [client_task, bridge_task]
                try:
                    done, pending = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
                    for task in pending:
                        task.cancel()
                    if (bridge_task in done and websocket.client_state.name == "CONNECTED" and
                            websocket.application_state.name == "CONNECTED"):
                        await websocket.close(code=1013, reason="rosbridge disconnected")
                finally:
                    for task in tasks:
                        if not task.done():
                            task.cancel()
                    await asyncio.gather(*tasks, return_exceptions=True)
        except WebSocketDisconnect:
            return
        except Exception:
            try:
                await websocket.close(code=1013)
            except Exception:
                pass

    async def validate(request: StarletteRequest):
        """验证当前设备或操作。

        Args:
            request: 封装调用输入字段的请求对象。 类型为 StarletteRequest。

        Returns:
            本次处理产生的结果值。
        """
        if denied := await auth(request):
            return denied
        from .config import load_platform
        try:
            body = await request.json()
            checked = await asyncio.to_thread(
                load_platform, body.get("config_file") or manager.platform["config_file"]
            )
            return JSONResponse({"valid": True, "device_count": len(checked["devices"])})
        except Exception as exc:
            return JSONResponse({"valid": False, "error": str(exc)}, status_code=400)

    async def manage_device(request: StarletteRequest):
        """管理设备。

        Args:
            request: 封装调用输入字段的请求对象。 类型为 StarletteRequest。

        Returns:
            本次处理产生的结果值。
        """
        if denied := await auth(request):
            return denied
        device_id = request.path_params["device_id"]
        try:
            body = await request.json()
            action = str(body.get("action", ""))
            if action == "start":
                result = await asyncio.to_thread(manager.start, device_id)
            elif action == "shutdown":
                result = await asyncio.to_thread(manager.shutdown, device_id)
            elif action == "restart":
                result = await asyncio.to_thread(manager.restart, device_id)
            else:
                return JSONResponse({"error": "action must be start, shutdown or restart"}, status_code=400)
            return JSONResponse(result, status_code=200 if result.get("success") else 409)
        except KeyError as exc:
            return JSONResponse({"error": str(exc)}, status_code=404)
        except Exception as exc:
            if isinstance(exc, OperationInProgress):
                return JSONResponse({"success": False, "error": str(exc)}, status_code=409)
            return JSONResponse({"success": False, "error": str(exc)}, status_code=500)

    async def manage_all(request: StarletteRequest):
        """管理all。

        Args:
            request: 封装调用输入字段的请求对象。 类型为 StarletteRequest。

        Returns:
            本次处理产生的结果值。
        """
        if denied := await auth(request):
            return denied
        try:
            body = await request.json()
            action = str(body.get("action", ""))
            if action == "start_all":
                results = await asyncio.to_thread(manager.start_all)
            elif action == "shutdown_all":
                results = await asyncio.to_thread(manager.shutdown_all)
            elif action == "stop_all":
                results = await asyncio.to_thread(manager.stop_all)
            else:
                return JSONResponse({"error": "action must be start_all, shutdown_all or stop_all"}, status_code=400)
            success = all(result.get("success", False) for result in results)
            return JSONResponse({"success": success, "results": results}, status_code=200 if success else 409)
        except Exception as exc:
            return JSONResponse({"success": False, "error": str(exc)}, status_code=500)

    async def shutdown_core(request: StarletteRequest):
        """关闭core。

        Args:
            request: 封装调用输入字段的请求对象。 类型为 StarletteRequest。

        Returns:
            本次处理产生的结果值。
        """
        if denied := await auth(request):
            return denied
        result = await asyncio.to_thread(manager.request_core_shutdown)
        if result["success"]:
            request.app.state.shutdown_requested.set()
            return JSONResponse(result)
        return JSONResponse(result, status_code=409)

    async def proxy_http(request: StarletteRequest):
        """执行 proxy_http 对应的处理流程。

        Args:
            request: 封装调用输入字段的请求对象。 类型为 StarletteRequest。

        Returns:
            本次处理产生的结果值。
        """
        if denied := await auth(request):
            return denied
        item, target = _device_target(manager, {**request.path_params, "query_string": request.url.query})
        if item is None:
            return JSONResponse({"error": "device not found"}, status_code=404)
        if item["process"] is None or item["process"].poll() is not None:
            return JSONResponse({"error": "device process is not running"}, status_code=503)
        body = await request.body()
        try:
            code, payload, response_headers = await asyncio.to_thread(
                _proxy_request, item, target, request.method, body, dict(request.headers),
            )
            return Response(payload, status_code=code, headers=response_headers)
        except Exception as exc:
            return JSONResponse({"error": f"device gateway unavailable: {exc}"}, status_code=503)

    async def proxy_websocket(websocket: WebSocket):
        """执行 proxy_websocket 对应的处理流程。

        Args:
            websocket: websocket，作为执行 proxy_websocket 对应的处理流程所需的输入。 类型为 WebSocket。

        Returns:
            无返回值。
        """
        category = websocket.path_params["category"]
        device_id = websocket.path_params["device_id"]
        kind = next((key for key, value in DEVICE_TYPES.items() if value[3] == category), None)
        item = manager._items.get(device_id)
        if kind is None or item is None or item["type"] != kind:
            await websocket.close(code=4404)
            return
        expected = f"en-{kind}-v1"
        offered = [part.strip() for part in websocket.headers.get("sec-websocket-protocol", "").split(",")]
        supplied = next((part[7:] for part in offered if part.startswith("bearer.")), "")
        if expected not in offered or not hmac.compare_digest(supplied, token):
            await websocket.close(code=4401)
            return
        if item["process"] is None or item["process"].poll() is not None:
            await websocket.close(code=1013)
            return
        query = websocket.scope.get("query_string", b"").decode("ascii", errors="ignore")
        uri = f"ws://127.0.0.1:{item['port']}/ws" + (f"?{query}" if query else "")
        try:
            async with websockets.connect(
                uri, subprotocols=[expected, f"bearer.{item['token']}"],
                open_timeout=3.0, close_timeout=1.0, max_queue=2, max_size=None,
                compression=None,
            ) as upstream:
                await websocket.accept(subprotocol=expected)

                async def client_to_device():
                    """客户端to设备。

                    Args:
                        无参数。

                    Returns:
                        无返回值。
                    """
                    while True:
                        message = await websocket.receive()
                        if message["type"] == "websocket.disconnect":
                            return
                        if message.get("text") is not None:
                            await upstream.send(message["text"])
                        elif message.get("bytes") is not None:
                            await upstream.send(message["bytes"])

                async def device_to_client():
                    """设备to客户端。

                    Args:
                        无参数。

                    Returns:
                        无返回值。
                    """
                    async for message in upstream:
                        if isinstance(message, bytes):
                            await websocket.send_bytes(message)
                        else:
                            await websocket.send_text(message)

                tasks = [asyncio.create_task(client_to_device()), asyncio.create_task(device_to_client())]
                try:
                    _, pending = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
                    for task in pending:
                        task.cancel()
                finally:
                    for task in tasks:
                        if not task.done():
                            task.cancel()
                    await asyncio.gather(*tasks, return_exceptions=True)
        except WebSocketDisconnect:
            return
        except Exception:
            try:
                await websocket.close(code=1013)
            except Exception:
                pass

    routes = [
        Route("/api/v1/core/scenes", scene_list, methods=["GET"]),
        Route("/api/v1/core/scenes/robot-description", scene_robot_description, methods=["GET"]),
        Route("/api/v1/core/scenes/{scene_id:str}/assets/{filename:str}", scene_asset,
              methods=["GET", "PUT"]),
        Route("/api/v1/core/scenes/{scene_id:str}/validate", scene_validate, methods=["POST"]),
        Route("/api/v1/core/scenes/{scene_id:str}/apply", scene_apply, methods=["POST"]),
        Route("/api/v1/core/scenes/{scene_id:str}", scene_get, methods=["GET"]),
        Route("/api/v1/core/scenes/{scene_id:str}", scene_save, methods=["PUT"]),
        Route("/api/v1/core/scenes/{scene_id:str}", scene_delete, methods=["DELETE"]),
        Route("/api/v1/core/visualization", visualization_info, methods=["GET"]),
        Route("/api/v1/core/visualization/resources/{package:str}/{path:path}", visualization_resource,
              methods=["GET"]),
        WebSocketRoute("/api/v1/core/visualization/ws", visualization_ws),
        Route("/api/v1/core/status", status, methods=["GET"]),
        Route("/api/v1/core/devices", devices, methods=["GET"]),
        Route("/api/v1/core/devices/manage_all", manage_all, methods=["POST"]),
        Route("/api/v1/core/devices/{device_id:str}", device, methods=["GET"]),
        Route("/api/v1/core/devices/{device_id:str}", manage_device, methods=["POST"]),
        Route("/api/v1/core/config", config, methods=["GET"]),
        Route("/api/v1/core/config/validate", validate, methods=["POST"]),
        Route("/api/v1/core/shutdown", shutdown_core, methods=["POST"]),
        WebSocketRoute("/api/v1/{category:str}/{device_id:str}/ws", proxy_websocket),
        Route("/api/v1/{category:str}/{device_id:str}/{path:path}", proxy_http,
              methods=["GET", "POST", "PUT", "PATCH", "DELETE", "OPTIONS", "HEAD"]),
        Mount("/rviz", app=StaticFiles(directory=str(_rviz_assets_dir()), html=True), name="core-rviz"),
        Mount("/", app=StaticFiles(directory=str(_web_assets_dir()), html=True), name="core-web"),
    ]
    app = Starlette(routes=routes)
    app.state.shutdown_requested = threading_event()
    return app


def _web_assets_dir() -> Path:
    """定位已安装的 Core Web 静态资源目录。

    Args:
        无参数。

    Returns:
        本次处理产生的结果值。 返回类型为 Path。
    """
    source_roots = (Path(__file__).resolve().parents[1] / "web",
                    Path(__file__).resolve().parent / "web")
    for candidate in source_roots:
        if (candidate / "index.html").is_file():
            return candidate
    try:
        from ament_index_python.packages import get_package_share_directory
        installed = Path(get_package_share_directory("embodied_nexus_core")) / "web"
    except Exception as exc:
        raise FileNotFoundError("Core web assets are not installed") from exc
    if not (installed / "index.html").is_file():
        raise FileNotFoundError(f"Core web assets are missing from {installed}")
    return installed


def _rviz_assets_dir() -> Path:
    """定位已安装的 Web RViz 静态资源目录。

    Args:
        无参数。

    Returns:
        本次处理产生的结果值。 返回类型为 Path。
    """
    source = Path(__file__).resolve().parents[1] / "web_rviz" / "dist"
    if (source / "index.html").is_file():
        return source
    try:
        from ament_index_python.packages import get_package_share_directory
        installed = Path(get_package_share_directory("embodied_nexus_core")) / "web" / "rviz"
    except Exception as exc:
        raise FileNotFoundError("Web RViz assets are not built or installed; run core.sh build core") from exc
    if not (installed / "index.html").is_file():
        raise FileNotFoundError("Web RViz assets are missing; run core.sh build core")
    return installed


def threading_event():
    """创建供后台线程协作使用的同步事件。

    Args:
        无参数。

    Returns:
        本次处理产生的结果值。
    """
    return threading.Event()


def _device_target(manager: Any, params: dict[str, Any]) -> tuple[dict[str, Any] | None, str]:
    """设备目标。

    Args:
        manager: manager，作为设备目标所需的输入。 类型为 Any。
        params: params，作为设备目标所需的输入。 类型为 dict[str, Any]。

    Returns:
        本次处理产生的结果值。 返回类型为 tuple[dict[str, Any] | None, str]。
    """
    category = params["category"]
    device_id = params["device_id"]
    kind = next((key for key, value in DEVICE_TYPES.items() if value[3] == category), None)
    item = manager._items.get(device_id)
    if kind is None or item is None or item["type"] != kind:
        return None, ""
    suffix = params.get("path", "")
    query = params.get("query_string", "")
    return item, f"/api/v1/{category}/{quote(device_id, safe='')}/{suffix}" + (f"?{query}" if query else "")


_HOP_BY_HOP_HEADERS = {
    "connection", "keep-alive", "proxy-authenticate", "proxy-authorization",
    "te", "trailer", "transfer-encoding", "upgrade", "host", "content-length",
}


def _forward_headers(headers: dict[str, str]) -> dict[str, str]:
    """复制允许转发到目标服务的请求头。

    Args:
        headers: 随请求发送的 HTTP 请求头集合。 类型为 dict[str, str]。

    Returns:
        本次处理产生的结果值。 返回类型为 dict[str, str]。
    """
    excluded = set(_HOP_BY_HOP_HEADERS)
    excluded.update(value.strip().lower() for value in headers.get("connection", "").split(","))
    return {key: value for key, value in headers.items()
            if key.lower() not in excluded and key.lower() != "authorization"}


def _response_headers(headers: Any) -> dict[str, str]:
    """响应headers。

    Args:
        headers: 随请求发送的 HTTP 请求头集合。 类型为 Any。

    Returns:
        本次处理产生的结果值。 返回类型为 dict[str, str]。
    """
    result = {key: value for key, value in headers.items()
              if key.lower() not in _HOP_BY_HOP_HEADERS}
    if "content-type" not in {key.lower() for key in result}:
        result["content-type"] = "application/octet-stream"
    return result


def _proxy_request(item: dict[str, Any], target: str, method: str, body: bytes,
                   request_headers: dict[str, str]) -> tuple[int, bytes, dict[str, str]]:
    """将浏览器请求代理到 ROS Bridge 并转发响应。

    Args:
        item: 条目，作为执行 _proxy_request 对应的处理流程所需的输入。 类型为 dict[str, Any]。
        target: 目标，作为执行 _proxy_request 对应的处理流程所需的输入。 类型为 str。
        method: HTTP 请求方法，例如 GET 或 POST。 类型为 str。
        body: 随请求发送的 JSON 数据。 类型为 bytes。
        request_headers: 请求headers，作为执行 _proxy_request 对应的处理流程所需的输入。 类型为 dict[str, str]。

    Returns:
        本次处理产生的结果值。 返回类型为 tuple[int, bytes, dict[str, str]]。
    """
    headers = _forward_headers(request_headers)
    headers["Authorization"] = f"Bearer {item['token']}"
    headers.setdefault("Content-Type", "application/json")
    request = Request(f"http://127.0.0.1:{item['port']}{target}", data=body or None,
                      headers=headers, method=method)
    try:
        with urlopen(request, timeout=70.0) as response:
            return response.status, response.read(), _response_headers(response.headers)
    except HTTPError as exc:
        return exc.code, exc.read(), _response_headers(exc.headers)
    except URLError as exc:
        raise RuntimeError(str(exc.reason)) from exc
