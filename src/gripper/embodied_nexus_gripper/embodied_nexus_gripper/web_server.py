from __future__ import annotations

import hmac
import json
from starlette.applications import Starlette
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.middleware import Middleware
from starlette.requests import Request
from starlette.responses import JSONResponse
from starlette.routing import Route
from starlette.concurrency import run_in_threadpool


class BearerAuthMiddleware(BaseHTTPMiddleware):
    def __init__(self, app, token: str) -> None:
        super().__init__(app)
        self.token = token.encode("utf-8")

    async def dispatch(self, request: Request, call_next):
        value = request.headers.get("authorization", "")
        scheme, _, token = value.partition(" ")
        if scheme.lower() != "bearer" or not token or not hmac.compare_digest(token.encode("utf-8"), self.token):
            return JSONResponse({"error": "valid bearer token required"}, status_code=401,
                                headers={"WWW-Authenticate": "Bearer"})
        return await call_next(request)


def _error(exc: Exception) -> JSONResponse:
    if isinstance(exc, KeyError):
        status = 404
        message = f"unknown gripper operation: {exc.args[0]}"
    elif isinstance(exc, PermissionError):
        status, message = 403, str(exc)
    elif isinstance(exc, TimeoutError):
        status, message = 504, str(exc)
    elif isinstance(exc, ValueError):
        status, message = 400, str(exc)
    elif isinstance(exc, RuntimeError):
        status, message = 409, str(exc)
    else:
        status, message = 500, str(exc)
    return JSONResponse({"error": message}, status_code=status)


def create_app(gateway, token: str) -> Starlette:
    if not token:
        raise RuntimeError("HTTP gateway requires a token from the configured auth_token_env")
    device_id = gateway.device_id
    prefix = f"/api/v1/grippers/{device_id}"

    async def invoke(request: Request, fn, *args):
        try:
            return JSONResponse(await run_in_threadpool(fn, *args))
        except Exception as exc:
            return _error(exc)

    async def json_object(request: Request) -> dict:
        body = await request.json()
        if not isinstance(body, dict):
            raise ValueError("request body must be a JSON object")
        return body

    async def info(request: Request):
        return await invoke(request, gateway.info)

    async def state(request: Request):
        return await invoke(request, gateway.state)

    async def get_config(request: Request):
        return await invoke(request, gateway.config)

    async def validate_config(request: Request):
        try:
            patch = await json_object(request)
            return JSONResponse({"valid": True, "config": await run_in_threadpool(gateway.validate_config, patch)})
        except Exception as exc:
            return _error(exc)

    async def apply_config(request: Request):
        try:
            body = await json_object(request)
            return JSONResponse(await run_in_threadpool(gateway.apply_config, body.get("lease_id", ""), body.get("patch", {})))
        except Exception as exc:
            return _error(exc)

    async def acquire(request: Request):
        try:
            body = await json_object(request)
            result = await run_in_threadpool(gateway.acquire, body.get("owner", "python-client"), body.get("ttl_sec"))
            return JSONResponse(result, status_code=201)
        except Exception as exc:
            return _error(exc)

    async def renew(request: Request):
        try:
            body = await json_object(request)
            return JSONResponse(await run_in_threadpool(gateway.renew, body.get("lease_id", "")))
        except Exception as exc:
            return _error(exc)

    async def release(request: Request):
        try:
            body = await json_object(request)
            return JSONResponse(await run_in_threadpool(gateway.release, body.get("lease_id", "")))
        except Exception as exc:
            return _error(exc)

    async def enable(request: Request):
        try:
            body = await json_object(request)
            return JSONResponse(await run_in_threadpool(
                gateway.enable, body.get("lease_id", ""), body.get("enabled", False)
            ))
        except Exception as exc:
            return _error(exc)

    async def stop(request: Request):
        try:
            body = await json_object(request)
            return JSONResponse(await run_in_threadpool(gateway.stop, body.get("reason", "HTTP stop requested")))
        except Exception as exc:
            return _error(exc)

    async def create_operation(request: Request):
        try:
            body = await json_object(request)
            result = await run_in_threadpool(
                gateway.start_operation, body.get("lease_id", ""), body.get("command", ""),
                body.get("angle_degrees", 0.0),
            )
            return JSONResponse(result, status_code=202)
        except Exception as exc:
            return _error(exc)

    async def get_operation(request: Request):
        return await invoke(request, gateway.operation, request.path_params["operation_id"])

    async def cancel_operation(request: Request):
        return await invoke(request, gateway.cancel_operation, request.path_params["operation_id"])

    routes = [
        Route(prefix + "/info", info, methods=["GET"]),
        Route(prefix + "/status", state, methods=["GET"]),
        Route(prefix + "/config", get_config, methods=["GET"]),
        Route(prefix + "/config/validate", validate_config, methods=["POST"]),
        Route(prefix + "/config", apply_config, methods=["PUT"]),
        Route(prefix + "/control/lease", acquire, methods=["POST"]),
        Route(prefix + "/control/lease/renew", renew, methods=["POST"]),
        Route(prefix + "/control/lease/release", release, methods=["POST"]),
        Route(prefix + "/motion/enabled", enable, methods=["PUT"]),
        Route(prefix + "/stop", stop, methods=["POST"]),
        Route(prefix + "/operations", create_operation, methods=["POST"]),
        Route(prefix + "/operations/{operation_id:str}", get_operation, methods=["GET"]),
        Route(prefix + "/operations/{operation_id:str}", cancel_operation, methods=["DELETE"]),
    ]
    return Starlette(routes=routes, middleware=[Middleware(BearerAuthMiddleware, token=token)])
