from __future__ import annotations

import asyncio
import hmac
import json
import re
import threading
from typing import Any

import uvicorn
from starlette.applications import Starlette
from starlette.endpoints import WebSocketEndpoint
from starlette.requests import Request
from starlette.responses import JSONResponse
from starlette.routing import Route, WebSocketRoute
from starlette.websockets import WebSocket, WebSocketDisconnect


def _error(exc: Exception) -> JSONResponse:
    if isinstance(exc, KeyError):
        status = 404
    elif isinstance(exc, PermissionError):
        status = 403
    elif isinstance(exc, TimeoutError):
        status = 504
    elif isinstance(exc, ValueError):
        status = 400
    elif isinstance(exc, RuntimeError):
        status = 409
    else:
        status = 500
    return JSONResponse({"error": str(exc)}, status_code=status, headers={"Cache-Control": "no-store"})


def create_app(gateway: Any, token: str) -> Starlette:
    if not token or not re.fullmatch(r"[A-Za-z0-9._~-]+", token):
        raise RuntimeError("Web mode requires a URL-safe token in the configured environment variable")
    base = f"/api/v1/bases/{gateway.device_id}"

    def authorized(request: Request) -> bool:
        return hmac.compare_digest(request.headers.get("authorization", ""), f"Bearer {token}")

    def unauthorized():
        return JSONResponse({"error": "unauthorized"}, status_code=401,
                            headers={"WWW-Authenticate": "Bearer", "Cache-Control": "no-store"})

    async def body_object(request: Request) -> dict[str, Any]:
        if int(request.headers.get("content-length", "0") or 0) > 1024 * 1024:
            raise ValueError("request body exceeds 1 MiB")
        value = await request.json()
        if not isinstance(value, dict):
            raise ValueError("request body must be a JSON object")
        return value

    async def handle(request: Request, fn, *args):
        if not authorized(request):
            return unauthorized()
        try:
            from starlette.concurrency import run_in_threadpool
            result = await run_in_threadpool(fn, *args)
            return JSONResponse(result, headers={"Cache-Control": "no-store"})
        except Exception as exc:
            return _error(exc)

    async def body_call(request: Request, fn, *args):
        if not authorized(request):
            return unauthorized()
        try:
            body = await body_object(request)
            from starlette.concurrency import run_in_threadpool
            return JSONResponse(await run_in_threadpool(fn, body, *args),
                                headers={"Cache-Control": "no-store"})
        except Exception as exc:
            return _error(exc)

    async def info(request: Request):
        return await handle(request, gateway.info)

    async def status(request: Request):
        return await handle(request, gateway.status)

    async def config(request: Request):
        return await handle(request, gateway.get_config)

    async def validate(request: Request):
        return await body_call(request, lambda body: gateway.validate_config(body.get("patch", body)))

    async def apply(request: Request):
        return await body_call(request, lambda body: gateway.apply_config(
            body.get("lease_id", ""), body.get("patch", {})))

    async def acquire(request: Request):
        return await body_call(request, lambda body: gateway.acquire(
            body.get("owner", "python-client"), body.get("ttl_sec")))

    async def renew(request: Request):
        return await body_call(request, lambda body: gateway.renew(body.get("lease_id", "")))

    async def release(request: Request):
        return await body_call(request, lambda body: gateway.release(body.get("lease_id", "")))

    async def enable(request: Request):
        def action(body):
            if not isinstance(body.get("enabled"), bool):
                raise ValueError("enabled must be a boolean")
            return gateway.enable(body.get("lease_id", ""), body["enabled"])
        return await body_call(request, action)

    async def velocity(request: Request):
        def action(body):
            return gateway.set_velocity(body.get("lease_id", ""),
                                        body.get("linear_mps", 0), body.get("angular_rps", 0))
        return await body_call(request, action)

    async def stop(request: Request):
        return await body_call(request, lambda body: gateway.stop(body.get("reason", "HTTP stop requested")))

    async def operation(request: Request):
        def action(body):
            if body.get("type") not in {"distance", "turn"}:
                raise ValueError("type must be distance or turn")
            return gateway.create_operation(body)
        return await body_call(request, action)

    async def get_operation(request: Request):
        return await handle(request, gateway.get_operation, request.path_params["operation_id"])

    async def cancel_operation(request: Request):
        if not authorized(request):
            return unauthorized()
        return await handle(request, gateway.cancel_operation, request.path_params["operation_id"])

    class BaseStatusSocket(WebSocketEndpoint):
        encoding = "text"

        async def on_connect(self, websocket: WebSocket) -> None:
            offered = [part.strip() for part in websocket.headers.get("sec-websocket-protocol", "").split(",")]
            valid = ("en-base-v1" in offered and
                     any(hmac.compare_digest(part, f"bearer.{token}") for part in offered))
            if not valid:
                await websocket.close(code=4401)
                return
            await websocket.accept(subprotocol="en-base-v1")

        async def on_receive(self, websocket: WebSocket, data: str) -> None:
            await websocket.close(code=1003)

        async def dispatch(self) -> None:
            websocket = WebSocket(self.scope, receive=self.receive, send=self.send)
            await self.on_connect(websocket)
            if websocket.application_state.name != "CONNECTED":
                return
            try:
                while websocket.application_state.name == "CONNECTED":
                    try:
                        state = await asyncio.to_thread(gateway.status)
                        await websocket.send_text(json.dumps(
                            {"type": "status", **state}, ensure_ascii=False,
                            allow_nan=False, separators=(",", ":")))
                    except Exception as exc:
                        await websocket.send_text(json.dumps({"type": "error", "error": str(exc)}))
                    await asyncio.sleep(0.1)
            except (WebSocketDisconnect, RuntimeError, OSError):
                return

    routes = [
        Route(base + "/info", info, methods=["GET"]),
        Route(base + "/status", status, methods=["GET"]),
        Route(base + "/config", config, methods=["GET"]),
        Route(base + "/config/validate", validate, methods=["POST"]),
        Route(base + "/config", apply, methods=["PUT"]),
        Route(base + "/control/lease", acquire, methods=["POST"]),
        Route(base + "/control/lease/renew", renew, methods=["POST"]),
        Route(base + "/control/lease/release", release, methods=["POST"]),
        Route(base + "/motion/enabled", enable, methods=["PUT"]),
        Route(base + "/velocity", velocity, methods=["PUT"]),
        Route(base + "/stop", stop, methods=["POST"]),
        Route(base + "/operations", operation, methods=["POST"]),
        Route(base + "/operations/{operation_id:str}", get_operation, methods=["GET"]),
        Route(base + "/operations/{operation_id:str}", cancel_operation, methods=["DELETE"]),
        WebSocketRoute("/ws", BaseStatusSocket),
    ]
    return Starlette(routes=routes)


class BaseWebServer:
    def __init__(self, gateway: Any, host: str, port: int, token: str) -> None:
        self.app = create_app(gateway, token)
        self.server = uvicorn.Server(uvicorn.Config(self.app, host=host, port=port,
                                                     log_level="warning", access_log=False))
        self.thread = threading.Thread(target=self.server.run, name="embodied-nexus-base-web", daemon=True)

    def start(self) -> None:
        self.thread.start()

    def close(self) -> None:
        self.server.should_exit = True
        if self.thread.is_alive():
            self.thread.join(timeout=5.0)
