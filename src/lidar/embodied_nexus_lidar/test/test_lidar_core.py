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
from embodied_nexus_lidar.protocol import build_motor_packet, build_speed_packet, inspect_difop
from embodied_nexus_lidar.runtime import LidarRuntime, StreamHub, mock_cloud
import embodied_nexus_lidar.runtime as lidar_runtime_module


PACKAGE = Path(__file__).resolve().parents[1]


def make_cloud(points, *, endian="<", point_step=20, row_padding=0, stamp=123.456):
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
    packet = bytearray(1206)
    packet[:4] = b"\xA5\xFF\x00\x5A"
    packet[8:10] = bytes(speed)
    packet[1198] = 0x04
    return bytes(packet)


class ConfigTests(unittest.TestCase):
    def test_all_backends_have_checked_configs(self):
        for name, backend in (("c16.yaml", "c16"), ("c16_topics.yaml", "ros2_topics"),
                              ("c16_mock.yaml", "mock")):
            cfg = load_config(PACKAGE / "config" / name)
            self.assertEqual(cfg["backend"], backend)
            self.assertEqual(cfg["device_id"], "lidar_main")
        self.assertEqual(load_config(PACKAGE / "config" / "c16.yaml")["web"]["port"], 8777)

    def test_unknown_fields_ranges_addresses_and_topic_loops_are_rejected(self):
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
        cfg = load_config(PACKAGE / "config" / "c16_mock.yaml")
        candidate = validate_patch(cfg, {"scan": {"range_max_m": 12.0}})
        self.assertEqual(candidate["scan"]["range_max_m"], 12.0)
        with self.assertRaises(ValueError):
            validate_patch(cfg, {"scan": {"range_max_m": 12.0}, "driver": {"device_ip": "10.0.0.2"}})
        self.assertEqual(cfg["scan"]["range_max_m"], 18.0)


class ProtocolTests(unittest.TestCase):
    def test_difop_validation_and_motor_packets(self):
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
        cloud = make_cloud([(1, 2, 3, 17, 4), (4, 5, 6, 23, 7)], endian=">")
        decoded = list(iter_points(cloud))
        self.assertEqual(decoded[0]["x"], 1.0)
        self.assertEqual(decoded[1]["ring"], 7.0)
        sampled = downsample_cloud(cloud, 2)
        self.assertEqual((sampled.width, sampled.height, sampled.point_step, sampled.row_step), (1, 1, 20, 20))
        self.assertEqual(sampled.data, cloud.data[:20])
        self.assertEqual(sampled.is_bigendian, True)

    def test_organized_rows_keep_declared_padding(self):
        cloud = make_cloud([(1, 0, 0, 10, 1), (2, 0, 0, 20, 2)])
        padded = CloudFrame(cloud.stamp_sec, cloud.frame_id, 1, 2, cloud.fields, False,
                            cloud.point_step, cloud.point_step + 4,
                            cloud.data[:20] + b"PAD!" + cloud.data[20:] + b"TAIL", cloud.is_dense)
        self.assertEqual([point["x"] for point in iter_points(padded)], [1.0, 2.0])

    def test_filter_compacts_rows_and_rejects_nonfinite_and_out_of_height(self):
        cloud = make_cloud([(1, 0, 0, 9, 1), (0, 2, 0.5, 8, 2), (-2, 0, 0, 7, 3),
                            (math.nan, 0, 0, 6, 4)])
        cfg = load_config(PACKAGE / "config" / "c16_mock.yaml")
        cfg["filter"].update(min_height_m=-0.35, max_height_m=0.35)
        result = filter_cloud(cloud, cfg["filter"])
        self.assertEqual((result.width, result.height, result.row_step), (2, 1, 40))
        self.assertEqual([p["intensity"] for p in iter_points(result)], [9.0, 7.0])

    def test_scan_full_circle_edges_empty_bins_and_timestamp(self):
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
    def test_difop_and_status_remain_responsive_during_cloud_projection(self):
        config = load_config(PACKAGE / "config" / "c16_mock.yaml")
        runtime = LidarRuntime(config)
        projection_entered = threading.Event()
        continue_projection = threading.Event()
        difop_ingested = threading.Event()
        original_filter = lidar_runtime_module.filter_cloud

        def slow_filter(frame, filter_config):
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
                target=lambda: (runtime.ingest_difop(difop()), difop_ingested.set()),
            )
            difop_thread.start()
            try:
                self.assertTrue(difop_ingested.wait(0.5), "DIFOP ingestion waited behind point-cloud work")
                self.assertLess(runtime.state()["difop_age_sec"], 0.5)
            finally:
                continue_projection.set()
                cloud_thread.join(2.0)
                difop_thread.join(2.0)
            self.assertFalse(cloud_thread.is_alive())
            self.assertFalse(difop_thread.is_alive())

    def test_mock_control_lease_conflict_expiry_and_stop_without_lease(self):
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
        cfg = load_config(PACKAGE / "config" / "c16.yaml")
        runtime = LidarRuntime(cfg)
        sent = []
        runtime._send_packet_locked = lambda packet: sent.append(packet)
        lease, _ = runtime.acquire("test")
        with self.assertRaises(RuntimeError):
            runtime.set_motor(lease, True)
        self.assertEqual(sent, [])
        runtime.ingest_difop(difop((0x01, 0x2C)))
        result = runtime.set_motor(lease, True)
        self.assertTrue(result["command_sent"])
        speed = runtime.actual_motor_speed()
        self.assertEqual(speed, {"known": True, "speed_hz": 5})
        runtime.config["control"]["difop_stale_sec"] = 0.01
        time.sleep(0.05)
        with self.assertRaises(RuntimeError):
            runtime.set_motor_speed(lease, 10)
        self.assertEqual(len(sent), 1)

    def test_release_failure_is_reported_and_shutdown_attempts_stop(self):
        runtime = LidarRuntime(load_config(PACKAGE / "config" / "c16_mock.yaml"))
        runtime.config["mock"]["fail_stop"] = True
        lease, _ = runtime.acquire("test")
        self.assertFalse(runtime.release(lease))
        self.assertFalse(runtime.shutdown())

    def test_stream_cache_keeps_only_latest_and_counts_overwrite(self):
        hub = StreamHub()
        hub.publish("points", 1)
        hub.publish("points", 2)
        self.assertEqual(hub.latest("points")[1], 2)
        self.assertEqual(hub.dropped(), 1)
        self.assertEqual(hub.wait_after({}, 0.01)[0][1], 2)

    def test_mock_cloud_is_deterministic_xyzirt(self):
        cloud = mock_cloud(12.5)
        self.assertEqual((cloud.width, cloud.height, cloud.point_step, cloud.row_step), (360, 1, 32, 11520))
        first = next(iter(iter_points(cloud)))
        self.assertAlmostEqual(first["x"], -6.0, places=5)
        self.assertEqual(first["ring"], 0.0)
        self.assertEqual(cloud.stamp_sec, 12.5)


if __name__ == "__main__":
    unittest.main()
