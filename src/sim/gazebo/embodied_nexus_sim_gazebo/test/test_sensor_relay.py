from __future__ import annotations

import math
import struct
import unittest

from sensor_msgs.msg import PointCloud2, PointField

from embodied_nexus_sim_gazebo.sensor_relay import _merge_lidar_clouds, _transform_lidar_cloud


def cloud(points: list[tuple[float, float, float]]) -> PointCloud2:
    message = PointCloud2()
    message.header.frame_id = "sensor_frame"
    message.height = 1
    message.width = len(points)
    message.fields = [
        PointField(name=name, offset=index * 4, datatype=PointField.FLOAT32, count=1)
        for index, name in enumerate(("x", "y", "z"))
    ]
    message.is_bigendian = False
    message.point_step = 12
    message.row_step = message.width * message.point_step
    message.data = b"".join(struct.pack("<fff", *point) for point in points)
    message.is_dense = True
    return message


class SensorRelayTests(unittest.TestCase):
    def test_front_and_rear_clouds_transform_into_common_lidar_frame(self):
        front = _transform_lidar_cloud(cloud([(1.0, 0.0, 0.2)]), math.pi / 2, -0.2)
        rear = _transform_lidar_cloud(cloud([(2.0, 0.0, 0.5)]), -math.pi / 2, -0.2)

        merged = _merge_lidar_clouds(front, rear)
        points = list(struct.iter_unpack("<fff", merged.data))

        self.assertEqual(merged.header.frame_id, "lidar_link")
        self.assertEqual((merged.width, merged.height, merged.point_step, merged.row_step), (2, 1, 12, 24))
        self.assertAlmostEqual(points[0][0], 0.0, places=6)
        self.assertAlmostEqual(points[0][1], 1.0, places=6)
        self.assertAlmostEqual(points[0][2], 0.0, places=6)
        self.assertAlmostEqual(points[1][0], 0.0, places=6)
        self.assertAlmostEqual(points[1][1], -2.0, places=6)
        self.assertAlmostEqual(points[1][2], 0.3, places=6)


if __name__ == "__main__":
    unittest.main()
