from __future__ import annotations

import argparse
import math

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo


class SimCameraInfo(Node):
    def __init__(self, channel: str, namespace: str) -> None:
        super().__init__(f"sim_camera_info_{channel}", parameter_overrides=[
            Parameter("use_sim_time", value=True),
        ])
        self.channel = channel
        self.namespace = namespace.rstrip("/")
        self.publisher = self.create_publisher(
            CameraInfo, f"{self.namespace}/camera/{channel}/camera_info", 10
        )
        self.timer = self.create_timer(0.1, self.publish_info)

    def publish_info(self) -> None:
        width, height, fov = 640, 480, 1.0471975512
        focal = width / (2.0 * math.tan(fov / 2.0))
        frame = "camera_color_optical_frame" if self.channel == "color" else "camera_depth_optical_frame"
        info = CameraInfo()
        info.header.stamp = self.get_clock().now().to_msg()
        info.header.frame_id = frame
        info.width, info.height = width, height
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5
        info.k = [focal, 0.0, (width - 1) / 2.0,
                  0.0, focal, (height - 1) / 2.0,
                  0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [focal, 0.0, (width - 1) / 2.0, 0.0,
                  0.0, focal, (height - 1) / 2.0, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        self.publisher.publish(info)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--channel", choices=("color", "depth"), required=True)
    parser.add_argument("--namespace", default="/sim/arachne")
    args, ros_args = parser.parse_known_args(argv)
    rclpy.init(args=ros_args)
    node = SimCameraInfo(args.channel, args.namespace)
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
