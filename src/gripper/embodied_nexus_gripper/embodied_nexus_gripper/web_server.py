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
        """初始化对象并保存构造参数或依赖。

        Args:
            app: app，作为初始化对象并保存构造参数或依赖所需的输入。
            token: 用于身份验证的访问令牌。 类型为 str。

        Returns:
            无返回值。
        """
        super().__init__(app)
        self.token = token.encode("utf-8")

    async def dispatch(self, request: Request, call_next):
        """分发当前设备或操作。

        Args:
            request: 封装调用输入字段的请求对象。 类型为 Request。
            call_next: 调用next，作为分发当前设备或操作所需的输入。

        Returns:
            本次处理产生的结果值。
        """
        value = request.headers.get("authorization", "")
        scheme, _, token = value.partition(" ")
        if scheme.lower() != "bearer" or not token or not hmac.compare_digest(token.encode("utf-8"), self.token):
            return JSONResponse({"error": "valid bearer token required"}, status_code=401,
                                headers={"WWW-Authenticate": "Bearer"})
        return await call_next(request)


def _error(exc: Exception) -> JSONResponse:
    """将异常转换为统一的 HTTP 错误响应。

    Args:
        exc: 捕获到的异常对象。 类型为 Exception。

    Returns:
        本次处理产生的结果值。 返回类型为 JSONResponse。
    """
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
    """创建app。

    Args:
        gateway: gateway，作为创建app所需的输入。
        token: 用于身份验证的访问令牌。 类型为 str。

    Returns:
        构造或创建完成的对象或资源。 返回类型为 Starlette。
    """
    if not token:
        raise RuntimeError("HTTP gateway requires a token from the configured auth_token_env")
    device_id = gateway.device_id
    prefix = f"/api/v1/grippers/{device_id}"

    async def invoke(request: Request, fn, *args):
        """执行 invoke 对应的处理流程。

        Args:
            request: 封装调用输入字段的请求对象。 类型为 Request。
            fn: fn，作为执行 invoke 对应的处理流程所需的输入。
            *args: 额外的位置参数集合。

        Returns:
            本次处理产生的结果值。
        """
        try:
            return JSONResponse(await run_in_threadpool(fn, *args))
        except Exception as exc:
            return _error(exc)

    async def json_object(request: Request) -> dict:
        """JSON对象。

        Args:
            request: 封装调用输入字段的请求对象。 类型为 Request。

        Returns:
            本次处理产生的结果值。 返回类型为 dict。
        """
        body = await request.json()
        if not isinstance(body, dict):
            raise ValueError("request body must be a JSON object")
        return body

    async def info(request: Request):
        """获取设备能力与静态信息。

        Args:
            request: 封装调用输入字段的请求对象。 类型为 Request。

        Returns:
            本次处理产生的结果值。
        """
        return await invoke(request, gateway.info)

    async def state(request: Request):
        """获取设备当前状态。

        Args:
            request: 封装调用输入字段的请求对象。 类型为 Request。

        Returns:
            本次处理产生的结果值。
        """
        return await invoke(request, gateway.state)

    async def get_config(request: Request):
        """获取配置。

        Args:
            request: 封装调用输入字段的请求对象。 类型为 Request。

        Returns:
            查询或读取到的数据；未找到时的处理方式由实现决定。
        """
        return await invoke(request, gateway.config)

    async def validate_config(request: Request):
        """验证配置。

        Args:
            request: 封装调用输入字段的请求对象。 类型为 Request。

        Returns:
            布尔值，表示验证配置是否成功或成立。
        """
        try:
            patch = await json_object(request)
            return JSONResponse({"valid": True, "config": await run_in_threadpool(gateway.validate_config, patch)})
        except Exception as exc:
            return _error(exc)

    async def apply_config(request: Request):
        """应用配置。

        Args:
            request: 封装调用输入字段的请求对象。 类型为 Request。

        Returns:
            本次处理产生的结果值。
        """
        try:
            body = await json_object(request)
            return JSONResponse(await run_in_threadpool(gateway.apply_config, body.get("lease_id", ""), body.get("patch", {})))
        except Exception as exc:
            return _error(exc)

    async def acquire(request: Request):
        """申请设备控制权。

        Args:
            request: 封装调用输入字段的请求对象。 类型为 Request。

        Returns:
            本次处理产生的结果值。
        """
        try:
            body = await json_object(request)
            result = await run_in_threadpool(gateway.acquire, body.get("owner", "python-client"), body.get("ttl_sec"))
            return JSONResponse(result, status_code=201)
        except Exception as exc:
            return _error(exc)

    async def renew(request: Request):
        """续租当前设备控制权。

        Args:
            request: 封装调用输入字段的请求对象。 类型为 Request。

        Returns:
            本次处理产生的结果值。
        """
        try:
            body = await json_object(request)
            return JSONResponse(await run_in_threadpool(gateway.renew, body.get("lease_id", "")))
        except Exception as exc:
            return _error(exc)

    async def release(request: Request):
        """释放当前设备控制租约。

        Args:
            request: 封装调用输入字段的请求对象。 类型为 Request。

        Returns:
            本次处理产生的结果值。
        """
        try:
            body = await json_object(request)
            return JSONResponse(await run_in_threadpool(gateway.release, body.get("lease_id", "")))
        except Exception as exc:
            return _error(exc)

    async def enable(request: Request):
        """更新设备运动使能状态。

        Args:
            request: 封装调用输入字段的请求对象。 类型为 Request。

        Returns:
            本次处理产生的结果值。
        """
        try:
            body = await json_object(request)
            return JSONResponse(await run_in_threadpool(
                gateway.enable, body.get("lease_id", ""), body.get("enabled", False)
            ))
        except Exception as exc:
            return _error(exc)

    async def stop(request: Request):
        """停止设备当前动作。

        Args:
            request: 封装调用输入字段的请求对象。 类型为 Request。

        Returns:
            本次处理产生的结果值。
        """
        try:
            body = await json_object(request)
            return JSONResponse(await run_in_threadpool(gateway.stop, body.get("reason", "HTTP stop requested")))
        except Exception as exc:
            return _error(exc)

    async def create_operation(request: Request):
        """创建操作。

        Args:
            request: 封装调用输入字段的请求对象。 类型为 Request。

        Returns:
            构造或创建完成的对象或资源。
        """
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
        """获取操作。

        Args:
            request: 封装调用输入字段的请求对象。 类型为 Request。

        Returns:
            查询或读取到的数据；未找到时的处理方式由实现决定。
        """
        return await invoke(request, gateway.operation, request.path_params["operation_id"])

    async def cancel_operation(request: Request):
        """取消操作。

        Args:
            request: 封装调用输入字段的请求对象。 类型为 Request。

        Returns:
            本次处理产生的结果值。
        """
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
