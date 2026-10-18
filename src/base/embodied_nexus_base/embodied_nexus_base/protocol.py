from __future__ import annotations

from dataclasses import dataclass


CAN_BITRATE_CODES = {1000000: 0x01, 800000: 0x02, 500000: 0x03, 400000: 0x04,
                     250000: 0x05, 200000: 0x06, 125000: 0x07, 100000: 0x08,
                     50000: 0x09, 20000: 0x0A, 10000: 0x0B, 5000: 0x0C}
FRAME_TYPE_CODES = {"standard": 0x01, "extended": 0x02}
MODE_CODES = {"normal": 0x00, "silent": 0x01, "loopback": 0x02, "silent_loopback": 0x03}
PROTOCOL_CONFIG_CODES = {"fixed": 0x02, "variable": 0x12}

SCOUT_CONTROL_MODE_ID = 0x421
SCOUT_MOTION_COMMAND_ID = 0x111
SCOUT_SYSTEM_STATE_ID = 0x211
SCOUT_MOTION_STATE_ID = 0x221


@dataclass(frozen=True)
class CanFrame:
    can_id: int
    data: bytes
    frame_type: str


@dataclass(frozen=True)
class ScoutFeedback:
    system: dict | None = None
    motion: dict | None = None


def encode_config(can_bitrate: int, frame_type: str = "standard", protocol: str = "variable",
                  mode: str = "normal") -> bytes:
    try:
        values = (PROTOCOL_CONFIG_CODES[protocol], CAN_BITRATE_CODES[can_bitrate],
                  FRAME_TYPE_CODES[frame_type], MODE_CODES[mode])
    except KeyError as exc:
        raise ValueError(f"unsupported Scout adapter setting: {exc.args[0]}") from exc
    payload = bytearray([0xAA, 0x55, values[0], values[1], values[2], 0, 0, 0, 0,
                         0, 0, 0, 0, values[3], 0, 0, 0, 0, 0])
    payload.append(sum(payload[2:19]) & 0xFF)
    return bytes(payload)


def encode_variable_frame(can_id: int, data: bytes, frame_type: str = "standard") -> bytes:
    if frame_type not in FRAME_TYPE_CODES:
        raise ValueError(f"unsupported CAN frame type: {frame_type}")
    if len(data) > 8:
        raise ValueError("CAN2.0 data length must be <= 8 bytes")
    if can_id < 0 or can_id > (0x7FF if frame_type == "standard" else 0x1FFFFFFF):
        raise ValueError("CAN identifier is outside the selected frame type range")
    type_byte = 0xC0 | (0x20 if frame_type == "extended" else 0) | len(data)
    id_length = 2 if frame_type == "standard" else 4
    return b"\xaa" + bytes([type_byte]) + can_id.to_bytes(id_length, "little") + data + b"\x55"


def decode_variable_frames(stream: bytes) -> tuple[list[CanFrame], bytes]:
    """Decode complete Waveshare USB-CAN-A variable frames and keep a partial tail."""
    frames: list[CanFrame] = []
    index = 0
    while index < len(stream):
        try:
            start = stream.index(0xAA, index)
        except ValueError:
            return frames, b""
        if start + 3 >= len(stream):
            return frames, stream[start:]
        type_byte = stream[start + 1]
        if type_byte & 0xC0 != 0xC0:
            index = start + 1
            continue
        frame_type = "extended" if type_byte & 0x20 else "standard"
        remote = bool(type_byte & 0x10)
        dlc = type_byte & 0x0F
        if dlc > 8:
            index = start + 1
            continue
        id_length = 4 if frame_type == "extended" else 2
        data_length = 0 if remote else dlc
        end = start + 2 + id_length + data_length
        if end >= len(stream):
            return frames, stream[start:]
        if stream[end] != 0x55:
            index = start + 1
            continue
        can_id = int.from_bytes(stream[start + 2:start + 2 + id_length], "little")
        data_start = start + 2 + id_length
        frames.append(CanFrame(can_id, stream[data_start:data_start + data_length], frame_type))
        index = end + 1
    return frames, b""


def int16_to_bytes(value: int) -> bytes:
    value = max(-32768, min(32767, int(value)))
    return value.to_bytes(2, "big", signed=True)


def int16_from_bytes(data: bytes) -> int:
    return int.from_bytes(data, "big", signed=True)


def decode_scout_frames(frames: list[CanFrame]) -> ScoutFeedback:
    system = motion = None
    for frame in frames:
        if frame.can_id == SCOUT_SYSTEM_STATE_ID and len(frame.data) >= 8:
            system = {"vehicle_state": frame.data[0], "control_mode": frame.data[1],
                      "battery_voltage": int16_from_bytes(frame.data[2:4]) / 10.0,
                      "error_code": int16_from_bytes(frame.data[4:6]) & 0xFFFF}
        elif frame.can_id == SCOUT_MOTION_STATE_ID and len(frame.data) >= 8:
            motion = {"linear_mps": int16_from_bytes(frame.data[0:2]) / 1000.0,
                      "angular_rps": int16_from_bytes(frame.data[2:4]) / 1000.0,
                      "lateral_mps": int16_from_bytes(frame.data[4:6]) / 1000.0}
    return ScoutFeedback(system=system, motion=motion)
