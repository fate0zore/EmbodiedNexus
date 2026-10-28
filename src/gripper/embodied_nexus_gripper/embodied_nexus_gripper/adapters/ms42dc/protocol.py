"""MS42DC serial protocol helpers, extracted from Arachne's direct driver."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable
from ...errors import FeedbackError


class ProtocolError(FeedbackError):
    """A received MS42DC feedback frame is incomplete or invalid."""


@dataclass(frozen=True)
class MS42DCFeedback:
    """Raw MS42DC status values decoded from one device frame."""

    device_id: int
    reached: int
    speed_raw: int
    angle_raw: int


def xor_checksum(values: Iterable[int]) -> int:
    """计算协议数据的异或校验值。

    Args:
        values: 要应用、比较或转换的一组输入值。 类型为 Iterable[int]。

    Returns:
        本次处理产生的结果值。 返回类型为 int。
    """
    result = 0
    for value in values:
        result ^= int(value) & 0xFF
    return result


def build_motion_frame(
    *, device_id: int, mode: int, direction: int, sub_divide: int,
    angle_tenths: int, speed_tenths: int,
) -> bytes:
    """构建motion坐标系。

    Args:
        device_id: 设备标识符，作为构建motion坐标系所需的输入。 类型为 int。
        mode: mode，作为构建motion坐标系所需的输入。 类型为 int。
        direction: direction，作为构建motion坐标系所需的输入。 类型为 int。
        sub_divide: subdivide，作为构建motion坐标系所需的输入。 类型为 int。
        angle_tenths: 角度tenths，作为构建motion坐标系所需的输入。 类型为 int。
        speed_tenths: 速度tenths，作为构建motion坐标系所需的输入。 类型为 int。

    Returns:
        构造或创建完成的对象或资源。 返回类型为 bytes。
    """
    values = (device_id, mode, direction, sub_divide)
    if any(not 0 <= int(value) <= 255 for value in values):
        raise ValueError("device_id, mode, direction and sub_divide must fit in one byte")
    if direction not in (0, 1):
        raise ValueError("direction must be 0 (open) or 1 (close)")
    if not 0 <= int(angle_tenths) <= 65535 or not 0 <= int(speed_tenths) <= 65535:
        raise ValueError("angle_tenths and speed_tenths must be in [0, 65535]")
    payload = [0x7B, int(device_id), int(mode), int(direction), int(sub_divide),
               int(angle_tenths) >> 8, int(angle_tenths) & 0xFF,
               int(speed_tenths) >> 8, int(speed_tenths) & 0xFF]
    return bytes(payload + [xor_checksum(payload), 0x7D])


def build_state_request_frame(device_id: int) -> bytes:
    """构建状态请求坐标系。

    Args:
        device_id: 设备标识符，作为构建状态请求坐标系所需的输入。 类型为 int。

    Returns:
        构造或创建完成的对象或资源。 返回类型为 bytes。
    """
    if not 0 <= int(device_id) <= 255:
        raise ValueError("device_id must fit in one byte")
    payload = [0x7B, int(device_id), 0, 0, 0, 0, 0, 0, 0]
    return bytes(payload + [xor_checksum(payload), 0x7D])


def parse_feedback_frame(frame: bytes, expected_device_id: int) -> MS42DCFeedback:
    """解析反馈坐标系。

    Args:
        frame: 坐标系，作为解析反馈坐标系所需的输入。 类型为 bytes。
        expected_device_id: expected设备标识符，作为解析反馈坐标系所需的输入。 类型为 int。

    Returns:
        解析或解码后的结构化数据。 返回类型为 MS42DCFeedback。
    """
    if len(frame) != 9:
        raise ProtocolError(f"feedback must contain 9 bytes; received {len(frame)}")
    actual_checksum = xor_checksum(frame[:8])
    if actual_checksum != frame[8]:
        raise ProtocolError(
            f"feedback checksum mismatch: expected {actual_checksum:02X}, got {frame[8]:02X}"
        )
    if frame[0] != expected_device_id:
        raise ProtocolError(
            f"feedback device id mismatch: expected {expected_device_id}, got {frame[0]}"
        )
    return MS42DCFeedback(
        device_id=frame[0], reached=frame[1],
        speed_raw=(frame[2] << 8) | frame[3],
        angle_raw=(frame[4] << 24) | (frame[5] << 16) | (frame[6] << 8) | frame[7],
    )


def build_feedback_frame(feedback: MS42DCFeedback) -> bytes:
    """Build a response frame for tests and the deterministic mock backend.

    Args:
        feedback: 反馈，作为构造供测试与 mock 后端使用的夹爪反馈帧所需的输入。 类型为 MS42DCFeedback。

    Returns:
        构造或创建完成的对象或资源。 返回类型为 bytes。
    """
    if not 0 <= feedback.angle_raw <= 0xFFFFFFFF:
        raise ValueError("angle_raw must fit in 32 bits")
    payload = [feedback.device_id, feedback.reached,
               feedback.speed_raw >> 8, feedback.speed_raw & 0xFF,
               (feedback.angle_raw >> 24) & 0xFF, (feedback.angle_raw >> 16) & 0xFF,
               (feedback.angle_raw >> 8) & 0xFF, feedback.angle_raw & 0xFF]
    return bytes(payload + [xor_checksum(payload)])
