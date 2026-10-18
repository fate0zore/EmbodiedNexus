import copy
import json
import struct
import unittest

try:
    from starlette.testclient import TestClient
except ImportError:
    TestClient = None

from embodied_nexus_lidar.config import load_config
from embodied_nexus_lidar.geometry import make_scan
from embodied_nexus_lidar.runtime import StreamHub, mock_cloud


class FakeGateway:
    device_id = "lidar_main"

    def __init__(self):
        self.config = load_config()
        self.hub = StreamHub()
        self.capture_values = []

    def info(self):
        return {"device_id": self.device_id}

    def status(self):
        return {"connected": True, "pointcloud_fps": 10.0}

    def get_config(self):
        return copy.deepcopy(self.config)

    def validate_config(self, patch):
        return patch

    def apply_config(self, patch):
        return patch

    def set_capture(self, capturing):
        self.capture_values.append(capturing)
        return {"capturing": capturing}

    def acquire(self, owner, ttl):
        return {"lease_id": "test-lease", "expires_in_sec": ttl or 2.0}

    def renew(self, lease_id):
        return {"expires_in_sec": 2.0}

    def release(self, lease_id):
        return {"released": True, "stop_sent": True}

    def set_motor(self, lease_id, enabled):
        return {"command_sent": True, "motor_state": "running" if enabled else "stopped"}

    def set_speed(self, lease_id, speed_hz):
        return {"command_sent": True, "requested_speed_hz": speed_hz}

    def acquire_client(self):
        return True

    def release_client(self):
        pass


@unittest.skipIf(TestClient is None, "Starlette test client is not installed")
class WebTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from embodied_nexus_lidar.web_server import create_app
        cls.gateway = FakeGateway()
        cls.client = TestClient(create_app(cls.gateway, "test-token"))

    def test_http_bearer_auth_and_strict_control_types(self):
        endpoint = "/api/v1/lidars/lidar_main/status"
        self.assertEqual(self.client.get(endpoint).status_code, 401)
        auth = {"Authorization": "Bearer test-token"}
        self.assertEqual(self.client.get(endpoint, headers=auth).status_code, 200)
        bad = self.client.put("/api/v1/lidars/lidar_main/capture", headers=auth,
                              json={"capturing": "false"})
        self.assertEqual(bad.status_code, 400)
        self.assertEqual(self.gateway.capture_values, [])

    def test_websocket_binary_layout_roundtrip_and_multiple_clients(self):
        cfg = load_config()
        cloud = mock_cloud(42.25)
        scan = make_scan(cloud, cfg)
        self.gateway.hub.publish("points", cloud)
        self.gateway.hub.publish("scan", scan)

        def receive_frames(ws):
            meta = json.loads(ws.receive_text())
            cloud_bytes = ws.receive_bytes()
            self.assertEqual(meta["channel"], "points")
            self.assertEqual(len(cloud_bytes), meta["payload_size"])
            self.assertEqual(meta["point_step"], cloud.point_step)
            self.assertEqual(meta["point_count"], 90)
            self.assertEqual(len(cloud_bytes), meta["point_count"] * cloud.point_step)
            scan_meta = json.loads(ws.receive_text())
            scan_bytes = ws.receive_bytes()
            self.assertEqual(scan_meta["encoding"], "LaserScanFloat32")
            self.assertEqual(len(scan_bytes), scan_meta["count"] * 8)
            decoded_ranges = struct.unpack_from("<" + "f" * scan_meta["count"], scan_bytes)
            self.assertEqual(len(decoded_ranges), scan_meta["count"])
            self.assertEqual(meta["stamp_sec"], 42.25)

        with self.client.websocket_connect(
            "/ws?streams=points,scan&point_stride=4",
            subprotocols=["en-lidar-v1", "bearer.test-token"],
        ) as first, self.client.websocket_connect(
            "/ws?streams=points,scan&point_stride=4",
            subprotocols=["en-lidar-v1", "bearer.test-token"],
        ) as second:
            receive_frames(first)
            receive_frames(second)


if __name__ == "__main__":
    unittest.main()
