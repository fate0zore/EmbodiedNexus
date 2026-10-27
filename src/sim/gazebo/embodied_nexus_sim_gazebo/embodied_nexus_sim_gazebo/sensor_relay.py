from __future__ import annotations

import math
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2, PointField


def _transform_lidar_cloud(message: PointCloud2, yaw_rad: float) -> PointCloud2:
    """转换雷达点云。

    Args:
        message: 待处理的消息对象。 类型为 PointCloud2。
        yaw_rad: yawrad，作为转换雷达点云所需的输入。 类型为 float。

    Returns:
        转换、格式化或编码后的数据。 返回类型为 PointCloud2。
    """
    offsets = {field.name: field.offset for field in message.fields}
    if not {"x", "y", "z"}.issubset(offsets):
        raise ValueError("simulated lidar cloud must include x, y and z fields")
    field_types = {field.name: field.datatype for field in message.fields}
    if any(field_types[name] != PointField.FLOAT32 for name in ("x", "y", "z")):
        raise ValueError("simulated lidar x/y/z fields must be float32")

    endian = ">" if message.is_bigendian else "<"
    data = bytearray(message.data)
    cosine, sine = math.cos(yaw_rad), math.sin(yaw_rad)
    layout = np.dtype({"names": ["x", "y", "z"], "formats": [endian + "f4"] * 3,
                       "offsets": [offsets[name] for name in ("x", "y", "z")],
                       "itemsize": message.point_step})
    points = np.ndarray((message.height, message.width), dtype=layout, buffer=data,
                        strides=(message.row_step, message.point_step))
    valid = np.isfinite(points["x"]) & np.isfinite(points["y"]) & np.isfinite(points["z"])
    x, y = points["x"][valid], points["y"][valid]
    points["x"][valid] = cosine * x - sine * y
    points["y"][valid] = sine * x + cosine * y
    message.data = bytes(data)
    message.header.frame_id = "lidar_link"
    return message


def _merge_lidar_clouds(front: PointCloud2, rear: PointCloud2) -> PointCloud2:
    """合并雷达clouds。

    Args:
        front: front，作为合并雷达clouds所需的输入。 类型为 PointCloud2。
        rear: rear，作为合并雷达clouds所需的输入。 类型为 PointCloud2。

    Returns:
        本次处理产生的结果值。 返回类型为 PointCloud2。
    """
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
        """初始化对象并保存构造参数或依赖。

        Args:
            namespace: namespace，作为初始化对象并保存构造参数或依赖所需的输入。 类型为 str。

        Returns:
            无返回值。
        """
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
        """点云当前设备或操作。

        Args:
            half: half，作为点云当前设备或操作所需的输入。 类型为 str。
            yaw_rad: yawrad，作为点云当前设备或操作所需的输入。 类型为 float。
            message: 待处理的消息对象。 类型为 PointCloud2。

        Returns:
            无返回值。
        """
        try:
            self._lidar_clouds[half] = _transform_lidar_cloud(message, yaw_rad)
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
        """将彩色图像消息转发到对应 ROS 话题。

        Args:
            message: 待处理的消息对象。 类型为 Image。

        Returns:
            无返回值。
        """
        message.header.frame_id = "camera_color_optical_frame"
        self._color_pub.publish(message)

    def _depth(self, message: Image) -> None:
        """将深度图像消息转发到对应 ROS 话题。

        Args:
            message: 待处理的消息对象。 类型为 Image。

        Returns:
            无返回值。
        """
        message.header.frame_id = "camera_depth_optical_frame"
        self._depth_pub.publish(message)


def main(argv=None) -> int:
    """解析启动参数并运行程序入口。

    Args:
        argv: argv，作为解析启动参数并运行程序入口所需的输入。

    Returns:
        本次处理产生的结果值。 返回类型为 int。
    """
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
