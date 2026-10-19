from __future__ import annotations

import asyncio
import hmac
import json
import threading
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.parse import quote
from urllib.request import Request, urlopen

import websockets
from starlette.applications import Starlette
from starlette.requests import Request as StarletteRequest
from starlette.responses import JSONResponse, Response
from starlette.routing import Route, WebSocketRoute
from starlette.websockets import WebSocket, WebSocketDisconnect

from .config import DEVICE_TYPES, public_platform
from .manager import OperationInProgress


def _authorized(request: StarletteRequest, token: str) -> bool:
    return hmac.compare_digest(request.headers.get("authorization", ""), f"Bearer {token}")


def create_app(manager: Any, token: str) -> Starlette:
    if not token:
        raise ValueError("core gateway requires a token from the configured environment variable")

    async def auth(request: StarletteRequest) -> JSONResponse | None:
        return None if _authorized(request, token) else JSONResponse({"error": "unauthorized"}, status_code=401)

    async def status(request: StarletteRequest):
        if denied := await auth(request):
            return denied
        return JSONResponse(await asyncio.to_thread(manager.status))

    async def devices(request: StarletteRequest):
        if denied := await auth(request):
            return denied
        return JSONResponse({"devices": await asyncio.to_thread(manager.list_devices)})

    async def device(request: StarletteRequest):
        if denied := await auth(request):
            return denied
        try:
            state = await asyncio.to_thread(manager.get_device, request.path_params["device_id"])
            return JSONResponse(state)
        except KeyError as exc:
            return JSONResponse({"error": str(exc)}, status_code=404)

    async def config(request: StarletteRequest):
        if denied := await auth(request):
            return denied
        return JSONResponse(public_platform(manager.platform))

    async def validate(request: StarletteRequest):
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
        if denied := await auth(request):
            return denied
        result = await asyncio.to_thread(manager.request_core_shutdown)
        if result["success"]:
            request.app.state.shutdown_requested.set()
            return JSONResponse(result)
        return JSONResponse(result, status_code=409)

    async def proxy_http(request: StarletteRequest):
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
                    while True:
                        message = await websocket.receive()
                        if message["type"] == "websocket.disconnect":
                            return
                        if message.get("text") is not None:
                            await upstream.send(message["text"])
                        elif message.get("bytes") is not None:
                            await upstream.send(message["bytes"])

                async def device_to_client():
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
    ]
    app = Starlette(routes=routes)
    app.state.shutdown_requested = threading_event()
    return app


def threading_event():
    return threading.Event()


def _device_target(manager: Any, params: dict[str, Any]) -> tuple[dict[str, Any] | None, str]:
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
    excluded = set(_HOP_BY_HOP_HEADERS)
    excluded.update(value.strip().lower() for value in headers.get("connection", "").split(","))
    return {key: value for key, value in headers.items()
            if key.lower() not in excluded and key.lower() != "authorization"}


def _response_headers(headers: Any) -> dict[str, str]:
    result = {key: value for key, value in headers.items()
              if key.lower() not in _HOP_BY_HOP_HEADERS}
    if "content-type" not in {key.lower() for key in result}:
        result["content-type"] = "application/octet-stream"
    return result


def _proxy_request(item: dict[str, Any], target: str, method: str, body: bytes,
                   request_headers: dict[str, str]) -> tuple[int, bytes, dict[str, str]]:
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
