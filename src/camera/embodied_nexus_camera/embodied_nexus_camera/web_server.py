from __future__ import annotations

import base64
import hashlib
import hmac
import json
import os
import re
import socket
import struct
import threading
import time
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np


@dataclass
class Frame:
    channel: str
    sequence: int
    stamp_sec: float
    frame_id: str
    encoding: str
    width: int
    height: int
    payload: bytes | np.ndarray
    extra: dict[str, Any]
    received_at: float


class CameraHub:
    """Thread-safe latest-frame cache; slow WebSocket readers never build a queue."""

    def __init__(self, camera_id: str, stale_sec: float = 1.0) -> None:
        """初始化对象并保存构造参数或依赖。

        Args:
            camera_id: 相机标识符，作为初始化对象并保存构造参数或依赖所需的输入。 类型为 str。
            stale_sec: stalesec，作为初始化对象并保存构造参数或依赖所需的输入。 类型为 float。

        Returns:
            无返回值。
        """
        self.camera_id = camera_id
        self.stale_sec = stale_sec
        self._frames: dict[str, Frame] = {}
        self._sequences: dict[str, int] = {}
        self._encoded: dict[tuple[str, int, str, int], bytes] = {}
        self._lock = threading.RLock()
        self._changed = threading.Condition(self._lock)
        self.started_at = time.time()
        self.last_error = ""
        self._errors: dict[str, str] = {}
        self.dropped_frames = 0
        self._last_publish: dict[str, float] = {}
        self._rates: dict[str, float] = {}

    def publish(
        self, channel: str, payload: bytes | np.ndarray, *, stamp_sec: float,
        frame_id: str = "", encoding: str = "", width: int = 0, height: int = 0,
        extra: dict[str, Any] | None = None,
    ) -> Frame:
        """发布当前设备或操作。

        Args:
            channel: channel，作为发布当前设备或操作所需的输入。 类型为 str。
            payload: 消息载荷或接口响应内容。 类型为 bytes | np.ndarray。
            stamp_sec: 时间戳sec，作为发布当前设备或操作所需的输入。 类型为 float。
            frame_id: 坐标系标识符，作为发布当前设备或操作所需的输入。 类型为 str。
            encoding: encoding，作为发布当前设备或操作所需的输入。 类型为 str。
            width: width，作为发布当前设备或操作所需的输入。 类型为 int。
            height: height，作为发布当前设备或操作所需的输入。 类型为 int。
            extra: extra，作为发布当前设备或操作所需的输入。 类型为 dict[str, Any] | None。

        Returns:
            本次处理产生的结果值。 返回类型为 Frame。
        """
        with self._changed:
            seq = self._sequences.get(channel, 0) + 1
            self._sequences[channel] = seq
            frame = Frame(channel, seq, float(stamp_sec), frame_id, encoding, int(width), int(height), payload, extra or {}, now := time.time())
            self._frames[channel] = frame
            now = time.monotonic()
            previous = self._last_publish.get(channel)
            if previous is not None and now > previous:
                old = self._rates.get(channel, 0.0)
                instant = 1.0 / (now - previous)
                self._rates[channel] = instant if old == 0 else 0.15 * instant + 0.85 * old
            self._last_publish[channel] = now
            self._errors.pop(channel, None)
            self.last_error = "; ".join(f"{key}: {value}" for key, value in sorted(self._errors.items()))
            self._changed.notify_all()
            return frame

    def error(self, message: str, channel: str = "camera") -> None:
        """构造包含错误代码和说明的统一响应。

        Args:
            message: 待处理的消息对象。 类型为 str。
            channel: channel，作为执行 error 对应的处理流程所需的输入。 类型为 str。

        Returns:
            无返回值。
        """
        with self._changed:
            self._errors[str(channel)] = str(message)
            self.last_error = "; ".join(f"{key}: {value}" for key, value in sorted(self._errors.items()))
            self._changed.notify_all()

    def record_drops(self, count: int) -> None:
        """记录新发生的数据丢帧数量。

        Args:
            count: count，作为执行 record_drops 对应的处理流程所需的输入。 类型为 int。

        Returns:
            无返回值。
        """
        if count <= 0:
            return
        with self._lock:
            self.dropped_frames += int(count)

    def latest(self, channel: str) -> Frame | None:
        """返回指定数据通道的最新样本。

        Args:
            channel: channel，作为执行 latest 对应的处理流程所需的输入。 类型为 str。

        Returns:
            本次处理产生的结果值。 返回类型为 Frame | None。
        """
        with self._lock:
            return self._frames.get(channel)

    def wait_for_change(self, sequences: dict[str, int], timeout: float) -> list[Frame]:
        """等待的change。

        Args:
            sequences: sequences，作为等待的change所需的输入。 类型为 dict[str, int]。
            timeout: 本次网络或服务调用的最长等待时长，通常以秒为单位。 类型为 float。

        Returns:
            本次处理产生的结果值。 返回类型为 list[Frame]。
        """
        with self._changed:
            self._changed.wait_for(
                lambda: any(self._sequences.get(key, 0) > sequences.get(key, 0) for key in self._frames),
                timeout=max(timeout, 0.01),
            )
            return [frame for key, frame in self._frames.items() if frame.sequence > sequences.get(key, 0)]

    def status(self, config: dict[str, Any], calibration_version: str = "uncalibrated") -> dict[str, Any]:
        """获取服务当前运行状态。

        Args:
            config: 设备或平台配置对象，提供本次操作所需参数。 类型为 dict[str, Any]。
            calibration_version: calibrationversion，作为获取服务当前运行状态所需的输入。 类型为 str。

        Returns:
            本次处理产生的结果值。 返回类型为 dict[str, Any]。
        """
        with self._lock:
            now = time.time()
            streams = {}
            for channel in ("color", "depth", "points"):
                frame = self._frames.get(channel)
                age = max(now - frame.received_at, 0.0) if frame else None
                streams[channel] = {
                    "available": bool(frame and age is not None and age <= self.stale_sec),
                    "sequence": frame.sequence if frame else 0,
                    "stamp_sec": frame.stamp_sec if frame else None,
                    "frame_age_sec": age,
                    "fps": round(self._rates.get(channel, 0.0), 2),
                    "width": frame.width if frame else None,
                    "height": frame.height if frame else None,
                    "encoding": frame.encoding if frame else None,
                    "frame_id": frame.frame_id if frame else None,
                }
            online = streams["color"]["available"] or streams["depth"]["available"]
            return {
                "camera_id": self.camera_id,
                "online": online,
                "state": "streaming" if online else "offline",
                "streams": streams,
                "dropped_frames": self.dropped_frames,
                "last_error": self.last_error,
                "calibration_version": calibration_version,
                "config_version": config.get("config_version", 1),
                "uptime_sec": round(max(now - self.started_at, 0.0), 3),
            }

    def encode(self, frame: Frame, codec: str, quality: int) -> bytes:
        """编码当前设备或操作。

        Args:
            frame: 坐标系，作为编码当前设备或操作所需的输入。 类型为 Frame。
            codec: codec，作为编码当前设备或操作所需的输入。 类型为 str。
            quality: quality，作为编码当前设备或操作所需的输入。 类型为 int。

        Returns:
            本次处理产生的结果值。 返回类型为 bytes。
        """
        if isinstance(frame.payload, bytes):
            return frame.payload
        key = (frame.channel, frame.sequence, codec, quality)
        with self._lock:
            cached = self._encoded.get(key)
            if cached is not None:
                return cached
        if frame.channel == "color":
            ext = ".jpg" if codec == "jpeg" else ".png"
            params = [cv2.IMWRITE_JPEG_QUALITY, quality] if ext == ".jpg" else [cv2.IMWRITE_PNG_COMPRESSION, 1]
            ok, encoded = cv2.imencode(ext, frame.payload, params)
        elif frame.channel == "depth":
            ok, encoded = cv2.imencode(".png", frame.payload, [cv2.IMWRITE_PNG_COMPRESSION, 1])
        else:
            ok, encoded = True, np.frombuffer(frame.payload, dtype=np.uint8)
        if not ok:
            raise RuntimeError(f"failed to encode {frame.channel} frame")
        result = encoded.tobytes()
        with self._lock:
            self._encoded[key] = result
            # Bounded cache: retain only the newest two sequences per channel.
            for old_key in list(self._encoded):
                if old_key[0] == frame.channel and old_key[1] < frame.sequence - 2:
                    self._encoded.pop(old_key, None)
        return result


def _json_bytes(value: Any) -> bytes:
    """JSONbytes。

    Args:
        value: 要应用、比较或转换的输入值。 类型为 Any。

    Returns:
        本次处理产生的结果值。 返回类型为 bytes。
    """
    return json.dumps(value, ensure_ascii=False, separators=(",", ":")).encode("utf-8")


def _ws_frame(opcode: int, payload: bytes) -> bytes:
    """将通道元数据和二进制数据组装为 WebSocket 帧。

    Args:
        opcode: opcode，作为执行 _ws_frame 对应的处理流程所需的输入。 类型为 int。
        payload: 消息载荷或接口响应内容。 类型为 bytes。

    Returns:
        本次处理产生的结果值。 返回类型为 bytes。
    """
    size = len(payload)
    if size < 126:
        head = bytes([0x80 | opcode, size])
    elif size <= 65535:
        head = bytes([0x80 | opcode, 126]) + struct.pack("!H", size)
    else:
        head = bytes([0x80 | opcode, 127]) + struct.pack("!Q", size)
    return head + payload


def _read_exact(sock: socket.socket, count: int) -> bytes:
    """读取exact。

    Args:
        sock: sock，作为读取exact所需的输入。 类型为 socket.socket。
        count: count，作为读取exact所需的输入。 类型为 int。

    Returns:
        查询或读取到的数据；未找到时的处理方式由实现决定。 返回类型为 bytes。
    """
    chunks = bytearray()
    while len(chunks) < count:
        part = sock.recv(count - len(chunks))
        if not part:
            raise ConnectionError("WebSocket peer disconnected")
        chunks.extend(part)
    return bytes(chunks)


def _read_ws_message(sock: socket.socket) -> tuple[int, bytes]:
    """读取ws消息。

    Args:
        sock: sock，作为读取ws消息所需的输入。 类型为 socket.socket。

    Returns:
        查询或读取到的数据；未找到时的处理方式由实现决定。 返回类型为 tuple[int, bytes]。
    """
    first, second = _read_exact(sock, 2)
    opcode = first & 0x0F
    masked = bool(second & 0x80)
    length = second & 0x7F
    if length == 126:
        length = struct.unpack("!H", _read_exact(sock, 2))[0]
    elif length == 127:
        length = struct.unpack("!Q", _read_exact(sock, 8))[0]
    if length > 65536:
        raise ValueError("client WebSocket message is too large")
    mask = _read_exact(sock, 4) if masked else b""
    payload = _read_exact(sock, length)
    if masked:
        payload = bytes(value ^ mask[index % 4] for index, value in enumerate(payload))
    return opcode, payload


class CameraWebServer:
    def __init__(self, runtime: Any, bind_host: str, port: int, token: str) -> None:
        """初始化对象并保存构造参数或依赖。

        Args:
            runtime: runtime，作为初始化对象并保存构造参数或依赖所需的输入。 类型为 Any。
            bind_host: bind主机，作为初始化对象并保存构造参数或依赖所需的输入。 类型为 str。
            port: 网络服务使用的端口号。 类型为 int。
            token: 用于身份验证的访问令牌。 类型为 str。

        Returns:
            无返回值。
        """
        if not token:
            raise ValueError("Web mode requires a token from the configured environment variable")
        if not re.fullmatch(r"[A-Za-z0-9._~-]+", token):
            raise ValueError("Web token must contain only URL-safe token characters for WebSocket authentication")
        self.runtime = runtime
        self.hub: CameraHub = runtime.hub
        self.token = token
        self._clients = 0
        self._clients_lock = threading.Lock()
        owner = self

        class Handler(BaseHTTPRequestHandler):
            protocol_version = "HTTP/1.1"
            server_version = "EmbodiedNexusCamera/0.1"

            def log_message(self, fmt: str, *args: Any) -> None:
                """将设备日志消息按级别写入 ROS 日志。

                Args:
                    fmt: fmt，作为执行 log_message 对应的处理流程所需的输入。 类型为 str。
                    *args: 额外的位置参数集合。 类型为 Any。

                Returns:
                    无返回值。
                """
                owner.runtime.log("web", fmt % args)

            def _auth(self, ws: bool = False) -> bool:
                """检查 HTTP 请求是否携带有效的授权令牌。

                Args:
                    ws: ws，作为执行 _auth 对应的处理流程所需的输入。 类型为 bool。

                Returns:
                    布尔值，表示执行 _auth 对应的处理流程是否成功或成立。
                """
                if ws:
                    raw = self.headers.get("Sec-WebSocket-Protocol", "")
                    supplied = [part.strip() for part in raw.split(",")]
                    return any(hmac.compare_digest(part, f"bearer.{owner.token}") for part in supplied)
                return hmac.compare_digest(self.headers.get("Authorization", ""), f"Bearer {owner.token}")

            def _send_json(self, code: int, value: Any) -> None:
                """发送JSON。

                Args:
                    code: code，作为发送JSON所需的输入。 类型为 int。
                    value: 要应用、比较或转换的输入值。 类型为 Any。

                Returns:
                    无返回值。
                """
                body = _json_bytes(value)
                self.send_response(code)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)

            def _body(self) -> dict[str, Any]:
                """请求体当前设备或操作。

                Args:
                    无参数。

                Returns:
                    本次处理产生的结果值。 返回类型为 dict[str, Any]。
                """
                length = int(self.headers.get("Content-Length", "0"))
                if length > 1024 * 1024:
                    raise ValueError("request body exceeds 1 MiB")
                raw = self.rfile.read(length) if length else b"{}"
                parsed = json.loads(raw.decode("utf-8"))
                if not isinstance(parsed, dict):
                    raise ValueError("JSON request body must be an object")
                return parsed

            def do_GET(self) -> None:
                """处理 HTTP GET 请求并返回对应资源。

                Args:
                    无参数。

                Returns:
                    无返回值。
                """
                parsed = urlparse(self.path)
                path = parsed.path.rstrip("/")
                base = f"/api/v1/cameras/{owner.hub.camera_id}"
                if path == f"{base}/snapshot.jpg":
                    if not self._auth():
                        self._send_json(401, {"error": "unauthorized"}); return
                    frame = owner.hub.latest("color")
                    status = owner.hub.status(owner.runtime.config, owner.runtime.calibration.version)
                    if frame is None or status["streams"]["color"]["frame_age_sec"] is None or status["streams"]["color"]["frame_age_sec"] > owner.hub.stale_sec:
                        self._send_json(503, {"error": "color frame unavailable"}); return
                    body = owner.hub.encode(frame, "jpeg", int(owner.runtime.config["streams"]["jpeg_quality"]))
                    self.send_response(200); self.send_header("Content-Type", "image/jpeg"); self.send_header("Content-Length", str(len(body))); self.send_header("Cache-Control", "no-store"); self.end_headers(); self.wfile.write(body); return
                if path == "/ws":
                    self._websocket(parsed); return
                if not self._auth():
                    self._send_json(401, {"error": "unauthorized"}); return
                if path == f"{base}/info":
                    self._send_json(200, {"camera_id": owner.hub.camera_id, "interfaces": owner.runtime.config["interface_mode"], "websocket": "/ws", "streams": ["color", "depth", "points"]}); return
                if path == f"{base}/status":
                    self._send_json(200, owner.hub.status(owner.runtime.config, owner.runtime.calibration.version)); return
                if path == f"{base}/config":
                    self._send_json(200, owner.runtime.public_config()); return
                if path == f"{base}/calibration":
                    self._send_json(200, owner.runtime.calibration_summary()); return
                if path == f"{base}/calibration/history":
                    self._send_json(200, {"history": owner.runtime.calibration_history()}); return
                match = re.fullmatch(re.escape(base) + r"/calibration/results/([A-Za-z0-9-]+)", path)
                if match:
                    result = owner.runtime.get_calibration_result(match.group(1))
                    self._send_json(200 if result else 404, result or {"error": "result not found"}); return
                self._send_json(404, {"error": "not found"})

            def do_PUT(self) -> None:
                """处理 HTTP PUT 请求并更新目标资源。

                Args:
                    无参数。

                Returns:
                    无返回值。
                """
                if not self._auth():
                    self._send_json(401, {"error": "unauthorized"}); return
                try:
                    body = self._body()
                    if self.path.rstrip("/") == f"/api/v1/cameras/{owner.hub.camera_id}/config":
                        self._send_json(200, owner.runtime.apply_config(body)); return
                    self._send_json(404, {"error": "not found"})
                except (ValueError, KeyError, TypeError) as exc:
                    self._send_json(400, {"error": str(exc)})

            def do_POST(self) -> None:
                """处理 HTTP POST 请求并执行目标操作。

                Args:
                    无参数。

                Returns:
                    无返回值。
                """
                if not self._auth():
                    self._send_json(401, {"error": "unauthorized"}); return
                try:
                    body = self._body()
                    prefix = f"/api/v1/cameras/{owner.hub.camera_id}/calibration"
                    if self.path.rstrip("/") == f"{prefix}/sessions":
                        self._send_json(201, owner.runtime.start_calibration(body)); return
                    match = re.fullmatch(re.escape(prefix) + r"/sessions/([A-Za-z0-9-]+)/(capture|solve)", self.path.rstrip("/"))
                    if match and match.group(2) == "capture":
                        self._send_json(201, owner.runtime.capture_calibration(match.group(1))); return
                    if match and match.group(2) == "solve":
                        self._send_json(202, owner.runtime.solve_calibration(match.group(1))); return
                    apply_match = re.fullmatch(re.escape(prefix) + r"/results/([A-Za-z0-9-]+)/apply", self.path.rstrip("/"))
                    if apply_match:
                        self._send_json(200, owner.runtime.apply_calibration(apply_match.group(1))); return
                    restore_match = re.fullmatch(re.escape(prefix) + r"/history/([A-Za-z0-9-]+)/restore", self.path.rstrip("/"))
                    if restore_match:
                        self._send_json(200, owner.runtime.apply_calibration(restore_match.group(1))); return
                    self._send_json(404, {"error": "not found"})
                except (ValueError, KeyError, TypeError, FileNotFoundError) as exc:
                    self._send_json(400, {"error": str(exc)})
                except RuntimeError as exc:
                    self._send_json(409, {"error": str(exc)})

            def do_DELETE(self) -> None:
                """处理 HTTP DELETE 请求并移除目标资源。

                Args:
                    无参数。

                Returns:
                    无返回值。
                """
                if not self._auth():
                    self._send_json(401, {"error": "unauthorized"}); return
                prefix = f"/api/v1/cameras/{owner.hub.camera_id}/calibration/sessions/"
                match = re.fullmatch(re.escape(prefix) + r"([A-Za-z0-9-]+)/samples/(frame_\d{4,}\.png)", self.path.rstrip("/"))
                if not match:
                    self._send_json(404, {"error": "not found"}); return
                try:
                    self._send_json(200, owner.runtime.delete_calibration_sample(match.group(1), match.group(2)))
                except (ValueError, OSError) as exc:
                    self._send_json(400, {"error": str(exc)})

            def _websocket(self, parsed: Any) -> None:
                """处理 WebSocket 连接、订阅和消息转发。

                Args:
                    parsed: parsed，作为执行 _websocket 对应的处理流程所需的输入。 类型为 Any。

                Returns:
                    无返回值。
                """
                if parsed.path != "/ws":
                    self._send_json(404, {"error": "not found"}); return
                if not self._auth(ws=True):
                    self._send_json(401, {"error": "unauthorized"}); return
                with owner._clients_lock:
                    if owner._clients >= int(owner.runtime.config["streams"]["max_clients"]):
                        self._send_json(503, {"error": "client limit reached"}); return
                    owner._clients += 1
                try:
                    key = self.headers.get("Sec-WebSocket-Key", "")
                    if not key or self.headers.get("Upgrade", "").lower() != "websocket":
                        self._send_json(400, {"error": "invalid websocket upgrade"}); return
                    accept = base64.b64encode(hashlib.sha1((key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode()).digest()).decode()
                    self.send_response(101, "Switching Protocols")
                    self.send_header("Upgrade", "websocket"); self.send_header("Connection", "Upgrade")
                    self.send_header("Sec-WebSocket-Accept", accept); self.send_header("Sec-WebSocket-Protocol", "en-camera-v1")
                    self.end_headers(); self.wfile.flush()
                    self.connection.settimeout(0.25)
                    query = parse_qs(parsed.query)
                    requested = set(query.get("streams", ["color,depth,points"])[0].split(","))
                    requested &= {"color", "depth", "points", "status"}
                    rates = {"color": float(query.get("color_fps", [owner.runtime.config["streams"]["web_color_fps"]])[0]), "depth": float(query.get("depth_fps", [owner.runtime.config["streams"]["web_depth_fps"]])[0]), "points": float(query.get("points_fps", [5.0])[0]), "status": 1.0}
                    codec = query.get("color_codec", ["jpeg"])[0]
                    if codec not in {"jpeg", "png"}: codec = "jpeg"
                    last_seq: dict[str, int] = {}
                    last_sent: dict[str, float] = {}
                    while True:
                        now = time.monotonic()
                        for frame in owner.hub.wait_for_change(last_seq, 0.05):
                            channel = frame.channel
                            if channel not in requested: continue
                            previous = last_seq.get(channel, frame.sequence - 1)
                            owner.hub.record_drops(frame.sequence - previous - 1)
                            last_seq[channel] = frame.sequence
                            if time.time() - frame.received_at > owner.hub.stale_sec: continue
                            max_fps = max(0.1, min(rates.get(channel, 5.0), 60.0))
                            if now - last_sent.get(channel, 0.0) < 1.0 / max_fps: continue
                            payload = owner.hub.encode(frame, codec if channel == "color" else "png", int(owner.runtime.config["streams"]["jpeg_quality"]))
                            meta = {"type": "frame", "camera_id": owner.hub.camera_id, "channel": channel, "sequence": frame.sequence, "stamp_sec": frame.stamp_sec, "timestamp_source": frame.extra.get("timestamp_source", "host_receive"), "frame_id": frame.frame_id, "encoding": frame.encoding, "codec": codec if channel == "color" else ("png16" if channel == "depth" else "binary"), "width": frame.width, "height": frame.height, "calibration_version": owner.runtime.calibration.version, "config_version": owner.runtime.config.get("config_version", 1), **frame.extra}
                            self.connection.sendall(_ws_frame(1, _json_bytes(meta)) + _ws_frame(2, payload))
                            last_sent[channel] = time.monotonic()
                        if "status" in requested and time.monotonic() - last_sent.get("status", 0.0) >= 1.0:
                            self.connection.sendall(_ws_frame(1, _json_bytes({"type": "status", **owner.hub.status(owner.runtime.config, owner.runtime.calibration.version)})))
                            last_sent["status"] = time.monotonic()
                        try:
                            opcode, payload = _read_ws_message(self.connection)
                            if opcode == 8: self.connection.sendall(_ws_frame(8, payload)); return
                            if opcode == 9: self.connection.sendall(_ws_frame(10, payload))
                        except socket.timeout:
                            continue
                except (ConnectionError, OSError, ValueError):
                    return
                finally:
                    with owner._clients_lock: owner._clients = max(owner._clients - 1, 0)

        self.server = ThreadingHTTPServer((bind_host, int(port)), Handler)
        self.server.daemon_threads = True
        self.thread = threading.Thread(target=self.server.serve_forever, name="camera-web", daemon=True)

    def start(self) -> None:
        """启动设备或后台任务。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        self.thread.start()

    def close(self) -> None:
        """关闭连接并释放相关资源。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        self.server.shutdown()
        self.server.server_close()


def main() -> None:
    """解析启动参数并运行程序入口。

    Args:
        无参数。

    Returns:
        无返回值。
    """
    raise SystemExit("camera_web is embedded in camera_node; start it with camera.launch.py interface_mode:=web or both")
