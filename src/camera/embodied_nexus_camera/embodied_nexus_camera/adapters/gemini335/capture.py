"""Gemini335 V4L2 capture adapter."""

from __future__ import annotations

import os
import select
import subprocess
import threading
import time
from typing import Any

import cv2
import numpy as np


def _read_pipe_exact(stream, size: int, stop: threading.Event, timeout_sec: float) -> bytearray | None:
    """读取pipeexact。

    Args:
        stream: 数据流，作为读取pipeexact所需的输入。
        size: size，作为读取pipeexact所需的输入。 类型为 int。
        stop: 停止，作为读取pipeexact所需的输入。 类型为 threading.Event。
        timeout_sec: 本次调用的最长等待时长，单位为秒。 类型为 float。

    Returns:
        查询或读取到的数据；未找到时的处理方式由实现决定。 返回类型为 bytearray | None。
    """
    deadline = time.monotonic() + timeout_sec
    raw = bytearray()
    while len(raw) < size and not stop.is_set():
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError(f"depth stream timed out after {timeout_sec:g}s")
        ready, _, _ = select.select([stream], [], [], min(0.2, remaining))
        if not ready:
            continue
        chunk = os.read(stream.fileno(), size - len(raw))
        if not chunk:
            return raw
        raw.extend(chunk)
    return None if stop.is_set() else raw


class Gemini335Capture:
    """Capture Gemini335 color and depth frames and pass them to the device service."""

    def __init__(self, config, stop, hub, accept_color, accept_depth, logger):
        """Save capture dependencies.

        Args:
            config: Camera configuration.
            stop: Event that requests capture shutdown.
            hub: Device stream hub receiving capture errors.
            accept_color: Callback accepting decoded color frames.
            accept_depth: Callback accepting decoded depth frames.
            logger: Logger for capture status.

        Returns:
            None.
        """
        self.config = config
        self._stop = stop
        self.hub = hub
        self._accept_color = accept_color
        self._accept_depth = accept_depth
        self.logger = logger

    @staticmethod
    def _apply_v4l2_controls(device: str, controls: dict[str, Any]) -> None:
        """应用v4l2controls。

        Args:
            device: 设备，作为应用v4l2controls所需的输入。 类型为 str。
            controls: controls，作为应用v4l2controls所需的输入。 类型为 dict[str, Any]。

        Returns:
            无返回值。
        """
        if not controls:
            return
        command = ["v4l2-ctl", "-d", device]
        command.extend(f"--set-ctrl={name}={value}" for name, value in controls.items())
        result = subprocess.run(command, capture_output=True, text=True, check=False)
        if result.returncode:
            detail = result.stderr.strip() or result.stdout.strip() or f"exit code {result.returncode}"
            raise RuntimeError(f"failed to apply V4L2 controls on {device}: {detail}")

    def capture_color(self) -> None:
        """循环读取 V4L2 彩色相机帧并提交新图像。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        cfg = self.config["capture"]
        source = self.config["source"]
        backoff = 0.2
        while not self._stop.is_set():
            cap = None
            try:
                self._apply_v4l2_controls(source["color_device"], cfg["color_controls"])
                cap = cv2.VideoCapture(source["color_device"], cv2.CAP_V4L2)
                if not cap.isOpened():
                    raise RuntimeError(f"cannot open RGB device {source['color_device']}")
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*str(cfg["color_fourcc"])[:4]))
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(cfg["color_width"]))
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(cfg["color_height"]))
                cap.set(cv2.CAP_PROP_FPS, float(cfg["color_fps"]))
                actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                actual_fps = float(cap.get(cv2.CAP_PROP_FPS))
                self.logger.info(f"RGB opened {source['color_device']} at {actual_w}x{actual_h}, negotiated {actual_fps:g} FPS")
                while not self._stop.is_set():
                    ok, frame = cap.read()
                    if not ok or frame is None:
                        raise RuntimeError("RGB capture returned no frame")
                    stamp = time.time()
                    self._accept_color(frame, stamp, cfg["color_frame_id"], "host_receive")
                    backoff = 0.2
                    # V4L2 read blocks at the negotiated device rate; do not add a second frame delay.
            except Exception as exc:
                if not self._stop.is_set():
                    self.hub.error(str(exc), "color")
                    self.logger.warning(f"RGB capture reconnecting: {exc}")
                    self._stop.wait(backoff)
                    backoff = min(backoff * 2.0, float(cfg["reconnect_backoff_sec"]))
            finally:
                if cap is not None:
                    cap.release()

    def capture_depth(self) -> None:
        """循环读取 V4L2 深度相机帧并提交新图像。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        cfg = self.config["capture"]
        device = self.config["source"]["depth_device"]
        width, height = int(cfg["depth_width"]), int(cfg["depth_height"])
        frame_pixels = width * height
        frame_bytes = frame_pixels * 2
        backoff = 0.2
        command = ["v4l2-ctl", "-d", device,
                   f"--set-fmt-video=width={width},height={height},pixelformat={str(cfg['depth_fourcc'])[:4]}",
                   f"--set-parm={max(float(cfg['depth_fps']), 1.0):g}", "--stream-mmap=4",
                   "--stream-count=10000000", "--stream-to=/dev/stdout"]
        while not self._stop.is_set():
            process = None
            try:
                self._apply_v4l2_controls(device, cfg["depth_controls"])
                process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=0)
                while not self._stop.is_set():
                    raw = _read_pipe_exact(process.stdout, frame_bytes, self._stop, float(cfg["capture_timeout_sec"]))
                    if raw is None:
                        break
                    if len(raw) != frame_bytes:
                        raise RuntimeError(f"depth stream ended after {len(raw)} of {frame_bytes} frame bytes")
                    depth = np.frombuffer(raw, dtype=np.uint16).reshape(height, width)
                    self._accept_depth(depth, time.time(), cfg["depth_frame_id"], "host_receive")
                    backoff = 0.2
            except Exception as exc:
                if not self._stop.is_set():
                    self.hub.error(str(exc), "depth")
                    self.logger.warning(f"depth capture reconnecting: {exc}")
                    self._stop.wait(backoff)
                    backoff = min(backoff * 2.0, float(cfg["reconnect_backoff_sec"]))
            finally:
                if process is not None:
                    if process.poll() is None:
                        process.terminate()
                        try:
                            process.wait(timeout=1.0)
                        except subprocess.TimeoutExpired:
                            process.kill()
                    if process.stdout:
                        process.stdout.close()
