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


def build_motor_packet(packet: bytes | bytearray | list[int], *, enabled: bool) -> bytes:
    status = inspect_difop(packet)
    command = bytearray(bytes(packet[:1206]))
    command[:8] = b"\xAA\x00\xFF\x11\x22\x22\xAA\xAA"
    if enabled:
        if status.model_version == 3:
            command[8], command[9] = _SPEED_BYTES[10]
        command[41] = 0
    else:
        command[41] = 1
    return bytes(command)


def build_speed_packet(packet: bytes | bytearray | list[int], speed_hz: int) -> bytes:
    inspect_difop(packet)
    if isinstance(speed_hz, bool) or speed_hz not in _SPEED_BYTES:
        raise ValueError("speed_hz must be one of 5, 10, or 20")
    command = bytearray(bytes(packet[:1206]))
    command[:8] = b"\xAA\x00\xFF\x11\x22\x22\xAA\xAA"
    command[8], command[9] = _SPEED_BYTES[speed_hz]
    return bytes(command)
