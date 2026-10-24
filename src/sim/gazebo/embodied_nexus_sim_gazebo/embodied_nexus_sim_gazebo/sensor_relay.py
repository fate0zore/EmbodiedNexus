from __future__ import annotations

import math
import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2, PointField


def _transform_lidar_cloud(message: PointCloud2, yaw_rad: float, z_offset_m: float) -> PointCloud2:
    offsets = {field.name: field.offset for field in message.fields}
    if not {"x", "y", "z"}.issubset(offsets):
        raise ValueError("simulated lidar cloud must include x, y and z fields")
    field_types = {field.name: field.datatype for field in message.fields}
    if any(field_types[name] != PointField.FLOAT32 for name in ("x", "y", "z")):
        raise ValueError("simulated lidar x/y/z fields must be float32")

    endian = ">" if message.is_bigendian else "<"
    data = bytearray(message.data)
    cosine, sine = math.cos(yaw_rad), math.sin(yaw_rad)
    for row in range(message.height):
        row_start = row * message.row_step
        for column in range(message.width):
            point_start = row_start + column * message.point_step
            x = struct.unpack_from(endian + "f", data, point_start + offsets["x"])[0]
            y = struct.unpack_from(endian + "f", data, point_start + offsets["y"])[0]
            z = struct.unpack_from(endian + "f", data, point_start + offsets["z"])[0]
            if not all(math.isfinite(value) for value in (x, y, z)):
                continue
            struct.pack_into(endian + "f", data, point_start + offsets["x"], cosine * x - sine * y)
            struct.pack_into(endian + "f", data, point_start + offsets["y"], sine * x + cosine * y)
            struct.pack_into(endian + "f", data, point_start + offsets["z"], z + z_offset_m)
    message.data = bytes(data)
    message.header.frame_id = "lidar_link"
    return message


def _merge_lidar_clouds(front: PointCloud2, rear: PointCloud2) -> PointCloud2:
    if (front.height != rear.height or front.point_step != rear.point_step or
            front.fields != rear.fields or front.is_bigendian != rear.is_bigendian):
        raise ValueError("front and rear simulated lidar cloud layouts must match")
    cloud = PointCloud2()
    cloud.header = front.header
    if (rear.header.stamp.sec, rear.header.stamp.nanosec) > (
            front.header.stamp.sec, front.header.stamp.nanosec):
        cloud.header.stamp = rear.header.stamp
    cloud.header.frame_id = "lidar_link"
    cloud.height = front.height
    cloud.width = front.width + rear.width
    cloud.fields = front.fields
    cloud.is_bigendian = front.is_bigendian
    cloud.point_step = front.point_step
    row_bytes = cloud.width * cloud.point_step
    merged_rows = []
    for row in range(cloud.height):
        front_start, rear_start = row * front.row_step, row * rear.row_step
        merged_rows.append(
            bytes(front.data[front_start:front_start + front.width * front.point_step]) +
            bytes(rear.data[rear_start:rear_start + rear.width * rear.point_step])
        )
    cloud.row_step = row_bytes
    cloud.data = b"".join(merged_rows)
    cloud.is_dense = front.is_dense and rear.is_dense
    return cloud


class SimSensorRelay(Node):
    """Normalize simulated sensor frame IDs at the Gazebo-specific boundary."""

    def __init__(self, namespace: str = "/sim/arachne") -> None:
        super().__init__("sim_sensor_relay", parameter_overrides=[
            rclpy.parameter.Parameter("use_sim_time", value=True),
        ])
        root = namespace.rstrip("/")
        self._cloud_pub = self.create_publisher(PointCloud2, f"{root}/lidar/points", qos_profile_sensor_data)
        self._lidar_clouds: dict[str, PointCloud2 | None] = {"front": None, "rear": None}
        self._lidar_received = {"front": 0.0, "rear": 0.0}
        self._lidar_subscriptions = [
            self.create_subscription(
                PointCloud2, f"{root}/lidar_{half}/points",
                lambda message, half=half, yaw=yaw: self._cloud(half, yaw, message),
                qos_profile_sensor_data,
            )
            for half, yaw in (("front", 0.0), ("rear", math.pi))
        ]
        self._color_pub = self.create_publisher(Image, f"{root}/camera/image_raw", qos_profile_sensor_data)
        self._color_sub = self.create_subscription(
            Image, f"{root}/camera/image", self._color, qos_profile_sensor_data
        )
        self._depth_pub = self.create_publisher(Image, f"{root}/camera/depth_image_raw", qos_profile_sensor_data)
        self._depth_sub = self.create_subscription(
            Image, f"{root}/camera/depth_image", self._depth, qos_profile_sensor_data
        )

    def _cloud(self, half: str, yaw_rad: float, message: PointCloud2) -> None:
        try:
            self._lidar_clouds[half] = _transform_lidar_cloud(message, yaw_rad, -0.2)
            self._lidar_received[half] = time.monotonic()
            front, rear = self._lidar_clouds["front"], self._lidar_clouds["rear"]
            if front is None or rear is None:
                return
            if abs(self._lidar_received["front"] - self._lidar_received["rear"]) > 0.75:
                return
            front_stamp = front.header.stamp.sec * 1_000_000_000 + front.header.stamp.nanosec
            rear_stamp = rear.header.stamp.sec * 1_000_000_000 + rear.header.stamp.nanosec
            if abs(front_stamp - rear_stamp) > 750_000_000:
                return
            self._cloud_pub.publish(_merge_lidar_clouds(front, rear))
        except Exception as exc:
            self.get_logger().warning(f"simulated lidar cloud relay failed: {exc}")

    def _color(self, message: Image) -> None:
        message.header.frame_id = "camera_color_optical_frame"
        self._color_pub.publish(message)

    def _depth(self, message: Image) -> None:
        message.header.frame_id = "camera_depth_optical_frame"
        self._depth_pub.publish(message)


def main(argv=None) -> int:
    rclpy.init(args=argv)
    node = SimSensorRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0
