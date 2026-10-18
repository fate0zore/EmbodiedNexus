from __future__ import annotations

import asyncio
import hmac
import json
import math
import re
import struct
import threading
import time
from typing import Any

from starlette.applications import Starlette
from starlette.endpoints import WebSocketEndpoint
from starlette.requests import Request
from starlette.responses import JSONResponse
from starlette.routing import Route, WebSocketRoute
from starlette.websockets import WebSocket, WebSocketDisconnect
import uvicorn

from .geometry import ScanFrame, downsample_cloud


def _json(value: Any) -> str:
    return json.dumps(value, ensure_ascii=False, allow_nan=False, separators=(",", ":"))


def _scan_payload(scan: ScanFrame) -> bytes:
    return struct.pack("<" + "f" * (len(scan.ranges) + len(scan.intensities)), *(scan.ranges + scan.intensities))


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
        raise RuntimeError("Web mode requires a URL-safe token from the configured environment variable")
    base = f"/api/v1/lidars/{gateway.device_id}"

    def authorized(request: Request) -> bool:
        return hmac.compare_digest(request.headers.get("authorization", ""), f"Bearer {token}")

    async def body_object(request: Request) -> dict[str, Any]:
        if int(request.headers.get("content-length", "0") or 0) > 1024 * 1024:
            raise ValueError("request body exceeds 1 MiB")
        value = await request.json()
        if not isinstance(value, dict):
            raise ValueError("request body must be a JSON object")
        return value

    async def handle(request: Request, fn, *args):
        if not authorized(request):
            return JSONResponse({"error": "unauthorized"}, status_code=401,
                                headers={"WWW-Authenticate": "Bearer", "Cache-Control": "no-store"})
        try:
            from starlette.concurrency import run_in_threadpool
            result = await run_in_threadpool(fn, *args)
            return JSONResponse(result, headers={"Cache-Control": "no-store"})
        except Exception as exc:
            return _error(exc)

    async def info(request: Request):
        return await handle(request, gateway.info)

    async def status(request: Request):
        return await handle(request, gateway.status)

    async def config(request: Request):
        return await handle(request, gateway.get_config)

    async def validate_config(request: Request):
        try:
            return await handle(request, gateway.validate_config, await body_object(request))
        except Exception as exc:
            return _error(exc)

    async def apply_config(request: Request):
        try:
            return await handle(request, gateway.apply_config, await body_object(request))
        except Exception as exc:
            return _error(exc)

    async def capture(request: Request):
        try:
            body = await body_object(request)
            capturing = body.get("capturing")
            if not isinstance(capturing, bool):
                raise ValueError("capturing must be a boolean")
            return await handle(request, gateway.set_capture, capturing)
        except Exception as exc:
            return _error(exc)

    async def acquire(request: Request):
        try:
            body = await body_object(request)
            return await handle(request, gateway.acquire, body.get("owner", "python-client"), body.get("ttl_sec"))
        except Exception as exc:
            return _error(exc)

    async def renew(request: Request):
        try:
            body = await body_object(request)
            return await handle(request, gateway.renew, body.get("lease_id", ""))
        except Exception as exc:
            return _error(exc)

    async def release(request: Request):
        try:
            body = await body_object(request)
            return await handle(request, gateway.release, body.get("lease_id", ""))
        except Exception as exc:
            return _error(exc)

    async def motor(request: Request):
        try:
            body = await body_object(request)
            enabled = body.get("enabled")
            if not isinstance(enabled, bool):
                raise ValueError("enabled must be a boolean")
            return await handle(request, gateway.set_motor, body.get("lease_id", ""), enabled)
        except Exception as exc:
            return _error(exc)

    async def speed(request: Request):
        try:
            body = await body_object(request)
            speed_hz = body.get("speed_hz")
            if isinstance(speed_hz, bool) or not isinstance(speed_hz, int):
                raise ValueError("speed_hz must be an integer")
            return await handle(request, gateway.set_speed, body.get("lease_id", ""), speed_hz)
        except Exception as exc:
            return _error(exc)

    class LidarWebSocket(WebSocketEndpoint):
        encoding = "text"

        async def on_connect(self, websocket: WebSocket) -> None:
            offered = [part.strip() for part in websocket.headers.get("sec-websocket-protocol", "").split(",")]
            valid = ("en-lidar-v1" in offered and
                     any(hmac.compare_digest(part, f"bearer.{token}") for part in offered))
            if not valid:
                await websocket.close(code=4401)
                return
            if not gateway.acquire_client():
                await websocket.close(code=1013)
                return
            self._registered = True
            await websocket.accept(subprotocol="en-lidar-v1")

        async def on_disconnect(self, websocket: WebSocket, close_code: int) -> None:
            if getattr(self, "_registered", False):
                gateway.release_client()
                self._registered = False

        async def on_receive(self, websocket: WebSocket, data: str) -> None:
            await websocket.close(code=1003)

        async def dispatch(self) -> None:
            websocket = WebSocket(self.scope, receive=self.receive, send=self.send)
            await self.on_connect(websocket)
            if websocket.application_state.name != "CONNECTED":
                return
            query = self.scope.get("query_string", b"").decode("ascii", errors="ignore")
            from urllib.parse import parse_qs
            params = parse_qs(query)
            requested = set(params.get("streams", ["points,scan,status"])[0].split(","))
            requested &= {"points", "scan", "status"}
            stride = max(1, min(int(params.get("point_stride", ["1"])[0]), 1000))
            rates = {
                "points": float(params.get("points_fps", [gateway.config["streams"]["web_points_fps"]])[0]),
                "scan": float(params.get("scan_fps", [gateway.config["streams"]["web_scan_fps"]])[0]),
                "status": 1.0,
            }
            rates = {key: max(0.1, min(value, 60.0)) for key, value in rates.items()}
            sequences: dict[str, int] = {}
            sent_at: dict[str, float] = {}
            client_dropped = 0
            try:
                while websocket.application_state.name == "CONNECTED":
                    ready = await asyncio.to_thread(gateway.hub.wait_after, sequences, 0.05)
                    now = time.monotonic()
                    for channel, seq, frame, received in ready:
                        previous = sequences.get(channel, seq - 1)
                        sequences[channel] = seq
                        if channel not in requested:
                            continue
                        client_dropped += max(seq - previous - 1, 0)
                        if now - sent_at.get(channel, 0.0) < 1.0 / rates[channel]:
                            client_dropped += 1
                            continue
                        if time.time() - received > float(gateway.config["streams"]["stale_sec"]):
                            client_dropped += 1
                            continue
                        if channel == "points":
                            payload_frame = downsample_cloud(frame, stride)
                            payload = payload_frame.data
                            meta = {
                                "type": "frame", "device_id": gateway.device_id,
                                "channel": channel, "sequence": seq, "stamp_sec": frame.stamp_sec,
                                "timestamp_source": "driver_header", "frame_id": frame.frame_id,
                                "encoding": "PointCloud2", "width": payload_frame.width,
                                "height": payload_frame.height,
                                "fields": payload_frame.fields, "is_bigendian": payload_frame.is_bigendian,
                                "point_step": payload_frame.point_step, "row_step": payload_frame.row_step,
                                "is_dense": payload_frame.is_dense, "unit": "m",
                                "point_count": payload_frame.point_count,
                                "payload_size": len(payload), "point_stride": stride,
                                "client_dropped_frames": client_dropped,
                            }
                        else:
                            payload = _scan_payload(frame)
                            n = len(frame.ranges)
                            meta = {
                                "type": "frame", "device_id": gateway.device_id,
                                "channel": "scan", "sequence": seq, "stamp_sec": frame.stamp_sec,
                                "timestamp_source": "driver_header", "frame_id": frame.frame_id,
                                "encoding": "LaserScanFloat32", "dtype": "<f4",
                                "angle_min_rad": frame.angle_min, "angle_max_rad": frame.angle_max,
                                "angle_increment_rad": frame.angle_increment,
                                "time_increment_sec": frame.time_increment, "scan_time_sec": frame.scan_time,
                                "range_min_m": frame.range_min, "range_max_m": frame.range_max,
                                "count": n, "ranges_offset": 0, "intensities_offset": n * 4,
                                "payload_size": len(payload),
                                "client_dropped_frames": client_dropped,
                            }
                        await websocket.send_text(_json(meta))
                        await websocket.send_bytes(payload)
                        sent_at[channel] = time.monotonic()
                    if "status" in requested and time.monotonic() - sent_at.get("status", 0.0) >= 1.0:
                        await websocket.send_text(_json({"type": "status", **gateway.status(),
                                                         "client_dropped_frames": client_dropped}))
                        sent_at["status"] = time.monotonic()
            except (WebSocketDisconnect, RuntimeError, OSError, ValueError):
                return
            finally:
                if getattr(self, "_registered", False):
                    gateway.release_client()
                    self._registered = False

    routes = [
        Route(base + "/info", info, methods=["GET"]),
        Route(base + "/status", status, methods=["GET"]),
        Route(base + "/config", config, methods=["GET"]),
        Route(base + "/config/validate", validate_config, methods=["POST"]),
        Route(base + "/config", apply_config, methods=["PUT"]),
        Route(base + "/capture", capture, methods=["PUT"]),
        Route(base + "/control/lease", acquire, methods=["POST"]),
        Route(base + "/control/lease/renew", renew, methods=["POST"]),
        Route(base + "/control/lease/release", release, methods=["POST"]),
        Route(base + "/motor", motor, methods=["PUT"]),
        Route(base + "/motor/speed", speed, methods=["PUT"]),
        WebSocketRoute("/ws", LidarWebSocket),
    ]
    return Starlette(routes=routes)


class LidarWebServer:
    def __init__(self, gateway: Any, host: str, port: int, token: str) -> None:
        self.app = create_app(gateway, token)
        self.server = uvicorn.Server(uvicorn.Config(self.app, host=host, port=port,
                                                     log_level="warning", access_log=False))
        self.thread = threading.Thread(target=self.server.run, name="embodied-nexus-lidar-web", daemon=True)

    def start(self) -> None:
        self.thread.start()

    def close(self) -> None:
        self.server.should_exit = True
        if self.thread.is_alive():
            self.thread.join(timeout=5.0)
