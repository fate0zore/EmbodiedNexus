from __future__ import annotations

import argparse

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo
from .camera_geometry import camera_geometry


class SimCameraInfo(Node):
    def __init__(self, channel: str, namespace: str, width: int = 640, height: int = 480, rate: float = 10.0) -> None:
        super().__init__(f"sim_camera_info_{channel}", parameter_overrides=[
            Parameter("use_sim_time", value=True),
        ])
        self.channel = channel
        self.namespace = namespace.rstrip("/")
        self.width, self.height = width, height
        self.intrinsic = camera_geometry(width, height)[channel]["intrinsic"]
        self.publisher = self.create_publisher(
            CameraInfo, f"{self.namespace}/camera/{channel}/camera_info", 10
        )
        self.timer = self.create_timer(1.0 / rate, self.publish_info)

    def publish_info(self) -> None:
        width, height = self.width, self.height
        fx, fy, cx, cy = (self.intrinsic[key] for key in ("fx", "fy", "cx", "cy"))
        frame = "camera_color_optical_frame" if self.channel == "color" else "camera_depth_optical_frame"
        info = CameraInfo()
        info.header.stamp = self.get_clock().now().to_msg()
        info.header.frame_id = frame
        info.width, info.height = width, height
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5
        info.k = [fx, 0.0, cx,
                  0.0, fy, cy,
                  0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0,
                  0.0, fy, cy, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        self.publisher.publish(info)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--channel", choices=("color", "depth"), required=True)
    parser.add_argument("--namespace", default="/sim/arachne")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--rate", type=float, default=10.0)
    args, ros_args = parser.parse_known_args(argv)
    rclpy.init(args=ros_args)
    node = SimCameraInfo(args.channel, args.namespace, args.width, args.height, args.rate)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
