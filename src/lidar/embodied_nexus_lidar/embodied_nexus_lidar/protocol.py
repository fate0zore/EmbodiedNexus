from __future__ import annotations

from dataclasses import dataclass

_SPEED_BYTES = {5: (0x01, 0x2C), 10: (0x02, 0x58), 20: (0x04, 0xB0)}
_SPEED_VALUES = {value: key for key, value in _SPEED_BYTES.items()}


@dataclass(frozen=True)
class DifopStatus:
    model_version: int
    speed_hz: int | None
    speed_confirmed: bool


def inspect_difop(packet: bytes | bytearray | list[int]) -> DifopStatus:
    """解析雷达 DIFOP 配置数据包字段。

    Args:
        packet: 数据包，作为执行 inspect_difop 对应的处理流程所需的输入。 类型为 bytes | bytearray | list[int]。

    Returns:
        本次处理产生的结果值。 返回类型为 DifopStatus。
    """
    raw = bytes(packet)
    if len(raw) < 1206 or raw[:4] != b"\xA5\xFF\x00\x5A":
        raise ValueError("DIFOP packet is incomplete or has an invalid header")
    is_v3 = raw[1196] == 0x03 or (raw[1196] == 0x02 and raw[1197] // 16 == 8)
    is_v4 = raw[1198] == 0x07 or (raw[1198] & 0x0F) == 4
    if is_v3:
        version = 3
    elif is_v4:
        version = 4
    else:
        raise ValueError("DIFOP packet does not identify a supported C16 protocol revision")
    speed = _SPEED_VALUES.get((raw[8], raw[9]))
    return DifopStatus(version, speed, speed is not None)


def build_motor_packet(
    packet: bytes | bytearray | list[int], *, enabled: bool, speed_hz: int | None = None,
) -> bytes:
    """构建控制电机数据包。

    Args:
        packet: 数据包，作为构建控制电机数据包所需的输入。 类型为 bytes | bytearray | list[int]。
        enabled: 表示对应功能是否启用的布尔值。 类型为 bool。
        speed_hz: 目标雷达转速，单位为 Hz。 类型为 int | None。

    Returns:
        构造或创建完成的对象或资源。 返回类型为 bytes。
    """
    status = inspect_difop(packet)
    command = bytearray(bytes(packet[:1206]))
    command[:8] = b"\xAA\x00\xFF\x11\x22\x22\xAA\xAA"
    if enabled:
        if speed_hz is not None:
            if isinstance(speed_hz, bool) or speed_hz not in _SPEED_BYTES:
                raise ValueError("speed_hz must be one of 5, 10, or 20")
            command[8], command[9] = _SPEED_BYTES[speed_hz]
        elif status.model_version == 3:
            command[8], command[9] = _SPEED_BYTES[10]
        command[41] = 0
    else:
        command[41] = 1
    return bytes(command)


def build_speed_packet(packet: bytes | bytearray | list[int], speed_hz: int) -> bytes:
    """构建速度数据包。

    Args:
        packet: 数据包，作为构建速度数据包所需的输入。 类型为 bytes | bytearray | list[int]。
        speed_hz: 目标雷达转速，单位为 Hz。 类型为 int。

    Returns:
        构造或创建完成的对象或资源。 返回类型为 bytes。
    """
    inspect_difop(packet)
    if isinstance(speed_hz, bool) or speed_hz not in _SPEED_BYTES:
        raise ValueError("speed_hz must be one of 5, 10, or 20")
    command = bytearray(bytes(packet[:1206]))
    command[:8] = b"\xAA\x00\xFF\x11\x22\x22\xAA\xAA"
    command[8], command[9] = _SPEED_BYTES[speed_hz]
    return bytes(command)
