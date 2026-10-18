from __future__ import annotations

import copy
import math
import struct
from dataclasses import dataclass
from typing import Any, Iterator


_ROS_TYPES = {1: ("b", 1), 2: ("B", 1), 3: ("h", 2), 4: ("H", 2),
              5: ("i", 4), 6: ("I", 4), 7: ("f", 4), 8: ("d", 8)}


@dataclass
class CloudFrame:
    stamp_sec: float
    frame_id: str
    width: int
    height: int
    fields: list[dict[str, Any]]
    is_bigendian: bool
    point_step: int
    row_step: int
    data: bytes
    is_dense: bool

    @property
    def point_count(self) -> int:
        return self.width * self.height


@dataclass
class ScanFrame:
    stamp_sec: float
    frame_id: str
    angle_min: float
    angle_max: float
    angle_increment: float
    time_increment: float
    scan_time: float
    range_min: float
    range_max: float
    ranges: list[float]
    intensities: list[float]


def cloud_from_ros(message: Any) -> CloudFrame:
    return CloudFrame(
        stamp_sec=float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1e-9,
        frame_id=str(message.header.frame_id), width=int(message.width), height=int(message.height),
        fields=[{"name": str(f.name), "offset": int(f.offset), "datatype": int(f.datatype), "count": int(f.count)}
                for f in message.fields],
        is_bigendian=bool(message.is_bigendian), point_step=int(message.point_step),
        row_step=int(message.row_step), data=bytes(message.data), is_dense=bool(message.is_dense),
    )


def cloud_to_ros(frame: CloudFrame, message: Any) -> Any:
    stamp_sec = math.floor(frame.stamp_sec)
    stamp_nanosec = round((frame.stamp_sec - stamp_sec) * 1e9)
    if stamp_nanosec >= 1_000_000_000:
        stamp_sec += 1
        stamp_nanosec -= 1_000_000_000
    message.header.stamp.sec = stamp_sec
    message.header.stamp.nanosec = stamp_nanosec
    message.header.frame_id = frame.frame_id
    message.width, message.height = frame.width, frame.height
    message.fields = []
    from sensor_msgs.msg import PointField
    for item in frame.fields:
        field = PointField()
        field.name, field.offset, field.datatype, field.count = (
            item["name"], item["offset"], item["datatype"], item["count"]
        )
        message.fields.append(field)
    message.is_bigendian = frame.is_bigendian
    message.point_step, message.row_step = frame.point_step, frame.row_step
    message.data = array_bytes(frame.data)
    message.is_dense = frame.is_dense
    return message


def array_bytes(data: bytes):
    from array import array
    result = array("B")
    result.frombytes(data)
    return result


def _xyz_layout(frame: CloudFrame) -> tuple[dict[str, dict[str, int]], str]:
    if frame.point_step <= 0 or frame.width < 0 or frame.height < 0 or frame.row_step < frame.width * frame.point_step:
        raise ValueError("PointCloud2 dimensions and strides are invalid")
    if len(frame.data) < frame.row_step * frame.height:
        raise ValueError("PointCloud2 data is shorter than its declared layout")
    endian = ">" if frame.is_bigendian else "<"
    result = {}
    for field in frame.fields:
        if field["name"] in {"x", "y", "z", "intensity", "time", "ring"}:
            kind = _ROS_TYPES.get(int(field["datatype"]))
            if kind is None or int(field["count"]) != 1:
                raise ValueError(f"PointCloud2 field {field['name']} has an invalid datatype")
            if int(field["offset"]) < 0 or int(field["offset"]) + kind[1] > frame.point_step:
                raise ValueError(f"PointCloud2 field {field['name']} exceeds point_step")
            result[field["name"]] = {"offset": int(field["offset"]), "datatype": int(field["datatype"])}
    if not all(name in result and result[name]["datatype"] in (7, 8) for name in ("x", "y", "z")):
        raise ValueError("PointCloud2 must contain scalar FLOAT32/FLOAT64 x, y and z fields")
    return result, endian


def iter_points(frame: CloudFrame) -> Iterator[dict[str, float]]:
    fields, endian = _xyz_layout(frame)
    for row in range(frame.height):
        base = row * frame.row_step
        for col in range(frame.width):
            point_base = base + col * frame.point_step
            values: dict[str, float] = {}
            for name, spec in fields.items():
                fmt = _ROS_TYPES[spec["datatype"]][0]
                values[name] = float(struct.unpack_from(endian + fmt, frame.data, point_base + spec["offset"])[0])
            yield values


def make_scan(frame: CloudFrame, config: dict[str, Any], scan_time: float = 0.0) -> ScanFrame:
    fields, endian = _xyz_layout(frame)
    del fields, endian
    cfg = config["scan"]
    n = int(math.ceil((cfg["angle_max_rad"] - cfg["angle_min_rad"]) / cfg["angle_increment_rad"]))
    if n <= 0:
        raise ValueError("LaserScan must contain at least one angular bin")
    ranges = [math.inf] * n
    intensities = [0.0] * n
    xyz, endian = _xyz_layout(frame)
    for row in range(frame.height):
        for col in range(frame.width):
            base = row * frame.row_step + col * frame.point_step
            point = {}
            for name in ("x", "y", "z"):
                point[name] = struct.unpack_from(endian + _ROS_TYPES[xyz[name]["datatype"]][0], frame.data, base + xyz[name]["offset"])[0]
            x, y, z = (float(point[k]) for k in ("x", "y", "z"))
            distance = math.hypot(x, y)
            if (not all(math.isfinite(v) for v in (x, y, z, distance)) or
                    z < cfg["min_height_m"] or z > cfg["max_height_m"] or
                    distance < cfg["range_min_m"] or distance > cfg["range_max_m"]):
                continue
            angle = math.atan2(y, x)
            full_circle = math.isclose(
                cfg["angle_max_rad"] - cfg["angle_min_rad"], 2.0 * math.pi,
                rel_tol=0.0, abs_tol=1e-9,
            )
            if full_circle and math.isclose(angle, cfg["angle_max_rad"], rel_tol=0.0, abs_tol=1e-9):
                angle = cfg["angle_min_rad"]
            index = int(math.floor((angle - cfg["angle_min_rad"]) / cfg["angle_increment_rad"]))
            if not 0 <= index < n:
                continue
            if distance < ranges[index]:
                ranges[index] = distance
                if "intensity" in xyz:
                    rangespec = xyz["intensity"]
                    intensity = struct.unpack_from(endian + _ROS_TYPES[rangespec["datatype"]][0], frame.data, base + rangespec["offset"])[0]
                    intensities[index] = float(intensity) if math.isfinite(float(intensity)) else 0.0
    if not cfg["use_inf"]:
        ranges = [cfg["range_max_m"] + 1.0 if math.isinf(v) else v for v in ranges]
    actual_max = cfg["angle_min_rad"] + (n - 1) * cfg["angle_increment_rad"]
    return ScanFrame(frame.stamp_sec, frame.frame_id, cfg["angle_min_rad"], actual_max,
                     cfg["angle_increment_rad"], 0.0, max(float(scan_time), 0.0),
                     cfg["range_min_m"], cfg["range_max_m"], ranges, intensities)


def scan_payload(scan: ScanFrame) -> bytes:
    return struct.pack("<" + "f" * len(scan.ranges), *scan.ranges)


def downsample_cloud(frame: CloudFrame, stride: int) -> CloudFrame:
    if stride <= 1 or frame.point_count == 0:
        return frame
    _xyz_layout(frame)
    records = []
    index = 0
    for row in range(frame.height):
        base = row * frame.row_step
        for col in range(frame.width):
            if index % stride == 0:
                offset = base + col * frame.point_step
                records.append(frame.data[offset:offset + frame.point_step])
            index += 1
    result = copy.copy(frame)
    result.width, result.height = len(records), 1
    result.row_step = result.width * result.point_step
    result.data = b"".join(records)
    return result


def filter_cloud(frame: CloudFrame, config: dict[str, Any]) -> CloudFrame:
    fields, endian = _xyz_layout(frame)
    selected = []
    for row in range(frame.height):
        for col in range(frame.width):
            base = row * frame.row_step + col * frame.point_step
            values = [float(struct.unpack_from(endian + _ROS_TYPES[fields[name]["datatype"]][0],
                                               frame.data, base + fields[name]["offset"])[0])
                      for name in ("x", "y", "z")]
            x, y, z = values
            distance = math.sqrt(x*x + y*y + z*z)
            if (not all(math.isfinite(v) for v in (x, y, z, distance)) or
                    distance < config["min_range_m"] or distance > config["max_range_m"] or
                    z < config["min_height_m"] or z > config["max_height_m"]):
                continue
            selected.append(frame.data[base:base + frame.point_step])
    result = copy.copy(frame)
    result.width, result.height = len(selected), 1
    result.row_step = result.width * result.point_step
    result.data = b"".join(selected)
    result.is_dense = True
    return result


def transform_cloud(frame: CloudFrame, translation: tuple[float, float, float], quaternion: tuple[float, float, float, float], target_frame: str) -> CloudFrame:
    result = copy.copy(frame)
    result.data = bytes(frame.data)
    result.frame_id = target_frame
    fields, endian = _xyz_layout(result)
    x, y, z, w = quaternion
    norm = x*x + y*y + z*z + w*w
    if norm < 1e-12:
        raise ValueError("target transform has an invalid quaternion")
    s = 2.0 / norm
    matrix = ((1-s*(y*y+z*z), s*(x*y-z*w), s*(x*z+y*w)),
              (s*(x*y+z*w), 1-s*(x*x+z*z), s*(y*z-x*w)),
              (s*(x*z-y*w), s*(y*z+x*w), 1-s*(x*x+y*y)))
    mutable = bytearray(result.data)
    for row in range(frame.height):
        for col in range(frame.width):
            base = row * frame.row_step + col * frame.point_step
            p = [float(struct.unpack_from(endian + _ROS_TYPES[fields[n]["datatype"]][0], frame.data, base + fields[n]["offset"])[0]) for n in ("x", "y", "z")]
            if not all(math.isfinite(v) for v in p):
                continue
            q = tuple(sum(matrix[i][j] * p[j] for j in range(3)) + translation[i] for i in range(3))
            for i, name in enumerate(("x", "y", "z")):
                struct.pack_into(endian + _ROS_TYPES[fields[name]["datatype"]][0], mutable,
                                 base + fields[name]["offset"], q[i])
    result.data = bytes(mutable)
    return result
