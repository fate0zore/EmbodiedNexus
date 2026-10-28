import math
import struct
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import patch

from embodied_nexus_lidar.config import load_config, validate_config, validate_patch
from embodied_nexus_lidar.geometry import (
    CloudFrame, downsample_cloud, filter_cloud, iter_points, make_scan, transform_cloud,
)
from embodied_nexus_lidar.adapters.c16.protocol import build_motor_packet, build_speed_packet, inspect_difop
from embodied_nexus_lidar.runtime import LidarRuntime, StreamHub, mock_cloud
import embodied_nexus_lidar.runtime as lidar_runtime_module


PACKAGE = Path(__file__).resolve().parents[1]


def make_cloud(points, *, endian="<", point_step=20, row_padding=0, stamp=123.456):
    """生成点云。

    Args:
        points: points，作为生成点云所需的输入。
        endian: endian，作为生成点云所需的输入。
        point_step: 点step，作为生成点云所需的输入。
        row_padding: rowpadding，作为生成点云所需的输入。
        stamp: 时间戳，作为生成点云所需的输入。

    Returns:
        构造或创建完成的对象或资源。
    """
    fields = [
        {"name": "x", "offset": 0, "datatype": 7, "count": 1},
        {"name": "y", "offset": 4, "datatype": 7, "count": 1},
        {"name": "z", "offset": 8, "datatype": 7, "count": 1},
        {"name": "intensity", "offset": 12, "datatype": 7, "count": 1},
        {"name": "ring", "offset": 16, "datatype": 4, "count": 1},
    ]
    data = bytearray()
    for x, y, z, intensity, ring in points:
        record = bytearray(point_step)
        struct.pack_into(endian + "ffffH", record, 0, x, y, z, intensity, ring)
        data.extend(record)
    return CloudFrame(stamp, "lidar_link", len(points), 1, fields, endian == ">", point_step,
                      len(points) * point_step + row_padding, bytes(data), False)


def difop(speed=(0x02, 0x58)):
    """设备配置包当前设备或操作。

    Args:
        speed: 速度，作为设备配置包当前设备或操作所需的输入。

    Returns:
        本次处理产生的结果值。
    """
    packet = bytearray(1206)
    packet[:4] = b"\xA5\xFF\x00\x5A"
    packet[8:10] = bytes(speed)
    packet[1198] = 0x04
    return bytes(packet)


class ConfigTests(unittest.TestCase):
    def test_all_backends_have_checked_configs(self):
        """验证allbackendshavecheckedconfigs。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        for name, backend in (("c16.yaml", "c16"), ("c16_topics.yaml", "ros2_topics"),
                              ("c16_mock.yaml", "mock"), ("c16_sim.yaml", "ros2_topics")):
            cfg = load_config(PACKAGE / "config" / name)
            self.assertEqual(cfg["backend"], backend)
            self.assertEqual(cfg["device_id"], "lidar_main")
        self.assertEqual(load_config(PACKAGE / "config" / "c16.yaml")["web"]["port"], 8777)

    def test_unknown_fields_ranges_addresses_and_topic_loops_are_rejected(self):
        """验证未知项fieldsrangesaddresses和话题loopsarerejected。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        cfg = load_config(PACKAGE / "config" / "c16_mock.yaml")
        for update in (
            {"extra": True},
            {"driver": {"device_ip": "not-an-ip"}},
            {"driver": {"group_ip": "192.168.1.1"}},
            {"source": {"pointcloud_topic": "points"}},
        ):
            candidate = {**cfg}
            from copy import deepcopy
            candidate = deepcopy(cfg)
            for section, values in update.items():
                if isinstance(values, dict):
                    candidate[section].update(values)
                else:
                    candidate[section] = values
            with self.subTest(update=update), self.assertRaises(ValueError):
                validate_config(candidate)

    def test_config_apply_is_atomic_and_runtime_only(self):
        """验证配置应用isatomic和runtimeonly。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        cfg = load_config(PACKAGE / "config" / "c16_mock.yaml")
        candidate = validate_patch(cfg, {"scan": {"range_max_m": 12.0}})
        self.assertEqual(candidate["scan"]["range_max_m"], 12.0)
        with self.assertRaises(ValueError):
            validate_patch(cfg, {"scan": {"range_max_m": 12.0}, "driver": {"device_ip": "10.0.0.2"}})
        self.assertEqual(cfg["scan"]["range_max_m"], 18.0)


class ProtocolTests(unittest.TestCase):
    def test_difop_validation_and_motor_packets(self):
        """验证设备配置包validation和控制电机packets。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        source = difop((0x01, 0x2C))
        status = inspect_difop(source)
        self.assertEqual(status.model_version, 4)
        self.assertEqual(status.speed_hz, 5)
        started = build_motor_packet(source, enabled=True)
        stopped = build_motor_packet(source, enabled=False)
        self.assertEqual(started[:8], b"\xAA\x00\xFF\x11\x22\x22\xAA\xAA")
        self.assertEqual(started[41], 0)
        self.assertEqual(stopped[41], 1)
        self.assertEqual(build_speed_packet(source, 20)[8:10], b"\x04\xB0")
        unknown_speed = difop((0x00, 0x00))
        started_with_speed = build_motor_packet(unknown_speed, enabled=True, speed_hz=10)
        self.assertEqual(started_with_speed[8:10], b"\x02\x58")
        with self.assertRaises(ValueError):
            build_motor_packet(unknown_speed, enabled=True, speed_hz=7)
        with self.assertRaises(ValueError):
            build_speed_packet(source, 7)
        with self.assertRaises(ValueError):
            inspect_difop(bytes(1206))
        unsupported = bytearray(source)
        unsupported[1198] = 0
        with self.assertRaises(ValueError):
            inspect_difop(unsupported)


class GeometryTests(unittest.TestCase):
    def test_big_endian_point_layout_and_stride_bytes_are_preserved(self):
        """验证bigendian点layout和stridebytesarepreserved。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        cloud = make_cloud([(1, 2, 3, 17, 4), (4, 5, 6, 23, 7)], endian=">")
        decoded = list(iter_points(cloud))
        self.assertEqual(decoded[0]["x"], 1.0)
        self.assertEqual(decoded[1]["ring"], 7.0)
        sampled = downsample_cloud(cloud, 2)
        self.assertEqual((sampled.width, sampled.height, sampled.point_step, sampled.row_step), (1, 1, 20, 20))
        self.assertEqual(sampled.data, cloud.data[:20])
        self.assertEqual(sampled.is_bigendian, True)

    def test_organized_rows_keep_declared_padding(self):
        """验证organizedrowskeepdeclaredpadding。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        cloud = make_cloud([(1, 0, 0, 10, 1), (2, 0, 0, 20, 2)])
        padded = CloudFrame(cloud.stamp_sec, cloud.frame_id, 1, 2, cloud.fields, False,
                            cloud.point_step, cloud.point_step + 4,
                            cloud.data[:20] + b"PAD!" + cloud.data[20:] + b"TAIL", cloud.is_dense)
        self.assertEqual([point["x"] for point in iter_points(padded)], [1.0, 2.0])

    def test_filter_compacts_rows_and_rejects_nonfinite_and_out_of_height(self):
        """验证筛选compactsrows和rejectsnonfinite和out的height。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        cloud = make_cloud([(1, 0, 0, 9, 1), (0, 2, 0.5, 8, 2), (-2, 0, 0, 7, 3),
                            (math.nan, 0, 0, 6, 4)])
        cfg = load_config(PACKAGE / "config" / "c16_mock.yaml")
        cfg["filter"].update(min_height_m=-0.35, max_height_m=0.35)
        result = filter_cloud(cloud, cfg["filter"])
        self.assertEqual((result.width, result.height, result.row_step), (2, 1, 40))
        self.assertEqual([p["intensity"] for p in iter_points(result)], [9.0, 7.0])

    def test_scan_full_circle_edges_empty_bins_and_timestamp(self):
        """验证扫描fullcircleedgesemptybins和timestamp。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        points = [(2, 0, 0, 1, 0), (0, -2, 0, 2, 1), (-2, 0, 0, 3, 2),
                  (0, 2, 0, 4, 3)]
        cloud = make_cloud(points, stamp=77.25)
        cfg = load_config(PACKAGE / "config" / "c16_mock.yaml")
        cfg["scan"]["angle_increment_rad"] = math.pi / 2
        scan = make_scan(cloud, cfg)
        self.assertEqual(len(scan.ranges), 4)
        self.assertAlmostEqual(scan.angle_max, math.pi / 2)
        self.assertTrue(all(abs(value - 2.0) < 1e-6 for value in scan.ranges))
        self.assertEqual(scan.stamp_sec, 77.25)
        self.assertEqual(scan.time_increment, 0.0)
        self.assertEqual(scan.scan_time, 0.0)
        empty = make_scan(make_cloud([(2, 0, 0, 1, 0)]), cfg)
        self.assertTrue(math.isinf(empty.ranges[0]))

    def test_tf_translation_rotation_and_timestamp_are_real(self):
        """验证tftranslationrotation和timestamparereal。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        cloud = make_cloud([(1, 0, 0, 12, 3)], endian=">")
        qz = math.sqrt(0.5)
        transformed = transform_cloud(cloud, (1, 2, 3), (0, 0, qz, qz), "base_link")
        point = next(iter(iter_points(transformed)))
        self.assertAlmostEqual(point["x"], 1.0, places=5)
        self.assertAlmostEqual(point["y"], 3.0, places=5)
        self.assertAlmostEqual(point["z"], 3.0, places=5)
        self.assertEqual(transformed.frame_id, "base_link")
        self.assertEqual(transformed.stamp_sec, cloud.stamp_sec)
        with self.assertRaises(ValueError):
            transform_cloud(cloud, (0, 0, 0), (0, 0, 0, 0), "base_link")


class RuntimeTests(unittest.TestCase):
    def test_external_topic_backend_rejects_motor_control_capability(self):
        """Reject C16 motor commands when the ROS topic source owns no motor.

        Returns:
            None.
        """
        runtime = LidarRuntime(load_config(PACKAGE / "config" / "c16_topics.yaml"))
        with self.assertRaisesRegex(RuntimeError, "motor control is unsupported"):
            runtime.set_motor("", False)

    def test_ros_topic_sensor_interruption_expires_lidar_data(self):
        """Report a disconnected sensor after point-cloud input stops.

        Args:
            None.

        Returns:
            None.
        """
        config = load_config(PACKAGE / "config" / "c16_topics.yaml")
        config["streams"]["stale_sec"] = 0.01
        runtime = LidarRuntime(config)
        runtime.ingest_cloud(mock_cloud(time.time()))
        self.assertTrue(runtime.state()["connected"])

        time.sleep(0.02)
        interrupted = runtime.state()
        self.assertFalse(interrupted["connected"])
        self.assertFalse(interrupted["data_fresh"])
        self.assertGreater(interrupted["pointcloud_age_sec"], config["streams"]["stale_sec"])

    def test_difop_and_status_remain_responsive_during_cloud_projection(self):
        """验证设备配置包和状态remainresponsiveduring点云projection。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。
        """
        config = load_config(PACKAGE / "config" / "c16_mock.yaml")
        runtime = LidarRuntime(config)
        projection_entered = threading.Event()
        continue_projection = threading.Event()
        difop_ingested = threading.Event()
        original_filter = lidar_runtime_module.filter_cloud

        def slow_filter(frame, filter_config):
            """在测试中模拟耗时的点云过滤操作。

            Args:
                frame: 坐标系，作为执行 slow_filter 对应的处理流程所需的输入。
                filter_config: 筛选配置，作为执行 slow_filter 对应的处理流程所需的输入。

            Returns:
                本次处理产生的结果值。
            """
            projection_entered.set()
            continue_projection.wait(2.0)
            return original_filter(frame, filter_config)

        with patch.object(lidar_runtime_module, "filter_cloud", side_effect=slow_filter):
            cloud_thread = threading.Thread(
                target=runtime.ingest_cloud,
                args=(mock_cloud(time.time()),),
            )
            cloud_thread.start()
            self.assertTrue(projection_entered.wait(1.0))
            difop_thread = threading.Thread(
                target=lambda: (runtime.ingest_device_status(difop()), difop_ingested.set()),
            )
            difop_thread.start()
            try:
                self.assertTrue(difop_ingested.wait(0.5), "DIFOP ingestion waited behind point-cloud work")
                self.assertLess(runtime.state()["device_feedback_age_sec"], 0.5)
                self.assertNotIn("difop_age_sec", runtime.state())
            finally:
                continue_projection.set()
                cloud_thread.join(2.0)
                difop_thread.join(2.0)
            self.assertFalse(cloud_thread.is_alive())
            self.assertFalse(difop_thread.is_alive())

    def test_mock_control_lease_conflict_expiry_and_stop_without_lease(self):
        """验证mock控制权租约conflictexpiry和停止without租约。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        runtime = LidarRuntime(load_config(PACKAGE / "config" / "c16_mock.yaml"))
        self.assertTrue(runtime.set_motor("", False)["command_sent"])
        lease, _ = runtime.acquire("first", 0.05)
        with self.assertRaises(RuntimeError):
            runtime.acquire("second")
        self.assertTrue(runtime.set_motor(lease, True)["hardware_state_confirmed"])
        time.sleep(0.07)
        state = runtime.state()
        self.assertEqual(state["lease_owner"], "")
        self.assertEqual(state["motor_state"], "stopped")
        self.assertTrue(state["hardware_state_confirmed"])

    def test_c16_requires_fresh_checked_difop_and_reports_send_failures(self):
        """验证c16requiresfreshchecked设备配置包和reports发送failures。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        cfg = load_config(PACKAGE / "config" / "c16.yaml")
        runtime = LidarRuntime(cfg)
        sent = []
        runtime._send_packet_locked = lambda packet: sent.append(packet)
        lease, _ = runtime.acquire("test")
        with self.assertRaises(RuntimeError):
            runtime.set_motor(lease, True)
        self.assertEqual(sent, [])
        runtime.ingest_device_status(difop((0x01, 0x2C)))
        result = runtime.set_motor(lease, True)
        self.assertTrue(result["command_sent"])
        speed = runtime.actual_motor_speed()
        self.assertEqual(speed, {"known": True, "speed_hz": 5})
        runtime.config["control"]["difop_stale_sec"] = 0.01
        time.sleep(0.05)
        with self.assertRaises(RuntimeError):
            runtime.set_motor_speed(lease, 10)
        self.assertEqual(len(sent), 1)

    def test_c16_motor_start_preserves_requested_speed_when_difop_is_unknown(self):
        """验证c16控制电机启动preservesrequested速度时设备配置包is未知项。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        runtime = LidarRuntime(load_config(PACKAGE / "config" / "c16.yaml"))
        sent = []
        runtime._send_packet_locked = lambda packet: sent.append(packet)
        lease, _ = runtime.acquire("test")
        runtime.ingest_device_status(difop((0x00, 0x00)))

        speed = runtime.set_motor_speed(lease, 10)
        self.assertTrue(speed["command_sent"])
        self.assertFalse(speed["actual_speed_known"])
        start = runtime.set_motor(lease, True)

        self.assertTrue(start["command_sent"])
        self.assertEqual(sent[0][8:10], b"\x02\x58")
        self.assertEqual(sent[1][8:10], b"\x02\x58")
        self.assertEqual(sent[1][41], 0)
        state = runtime.state()
        self.assertFalse(state["motor_speed_known"])
        self.assertFalse(state["data_fresh"])

    def test_release_failure_is_reported_and_shutdown_attempts_stop(self):
        """验证释放failureisreported和关闭attempts停止。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        runtime = LidarRuntime(load_config(PACKAGE / "config" / "c16_mock.yaml"))
        runtime.config["mock"]["fail_stop"] = True
        lease, _ = runtime.acquire("test")
        self.assertFalse(runtime.release(lease))
        self.assertFalse(runtime.shutdown())

    def test_stream_cache_keeps_only_latest_and_counts_overwrite(self):
        """验证数据流cachekeepsonlylatest和countsoverwrite。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        hub = StreamHub()
        hub.publish("points", 1)
        hub.publish("points", 2)
        self.assertEqual(hub.latest("points")[1], 2)
        self.assertEqual(hub.dropped(), 1)
        self.assertEqual(hub.wait_after({}, 0.01)[0][1], 2)

    def test_mock_cloud_is_deterministic_xyzirt(self):
        """验证mock点云isdeterministicxyzirt。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        cloud = mock_cloud(12.5)
        self.assertEqual((cloud.width, cloud.height, cloud.point_step, cloud.row_step), (360, 1, 32, 11520))
        first = next(iter(iter_points(cloud)))
        self.assertAlmostEqual(first["x"], -6.0, places=5)
        self.assertEqual(first["ring"], 0.0)
        self.assertEqual(cloud.stamp_sec, 12.5)


if __name__ == "__main__":
    unittest.main()
