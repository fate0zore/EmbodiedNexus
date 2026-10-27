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
        """初始化对象并保存构造参数或依赖。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        self.config = load_config()
        self.hub = StreamHub()
        self.capture_values = []

    def info(self):
        """获取设备能力与静态信息。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。
        """
        return {"device_id": self.device_id}

    def status(self):
        """获取服务当前运行状态。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。
        """
        return {"connected": True, "pointcloud_fps": 10.0}

    def get_config(self):
        """获取配置。

        Args:
            无参数。

        Returns:
            查询或读取到的数据；未找到时的处理方式由实现决定。
        """
        return copy.deepcopy(self.config)

    def validate_config(self, patch):
        """验证配置。

        Args:
            patch: patch，作为验证配置所需的输入。

        Returns:
            布尔值，表示验证配置是否成功或成立。
        """
        return patch

    def apply_config(self, patch):
        """应用配置。

        Args:
            patch: patch，作为应用配置所需的输入。

        Returns:
            本次处理产生的结果值。
        """
        return patch

    def set_capture(self, capturing):
        """设置采集。

        Args:
            capturing: capturing，作为设置采集所需的输入。

        Returns:
            本次处理产生的结果值。
        """
        self.capture_values.append(capturing)
        return {"capturing": capturing}

    def acquire(self, owner, ttl):
        """申请设备控制权。

        Args:
            owner: owner，作为申请设备控制权所需的输入。
            ttl: ttl，作为申请设备控制权所需的输入。

        Returns:
            本次处理产生的结果值。
        """
        return {"lease_id": "test-lease", "expires_in_sec": ttl or 2.0}

    def renew(self, lease_id):
        """续租当前设备控制权。

        Args:
            lease_id: 租约标识符，作为续租当前设备控制权所需的输入。

        Returns:
            本次处理产生的结果值。
        """
        return {"expires_in_sec": 2.0}

    def release(self, lease_id):
        """释放当前设备控制租约。

        Args:
            lease_id: 租约标识符，作为释放当前设备控制租约所需的输入。

        Returns:
            本次处理产生的结果值。
        """
        return {"released": True, "stop_sent": True}

    def set_motor(self, lease_id, enabled):
        """设置控制电机。

        Args:
            lease_id: 租约标识符，作为设置控制电机所需的输入。
            enabled: 表示对应功能是否启用的布尔值。

        Returns:
            本次处理产生的结果值。
        """
        return {"command_sent": True, "motor_state": "running" if enabled else "stopped"}

    def set_speed(self, lease_id, speed_hz):
        """设置速度。

        Args:
            lease_id: 租约标识符，作为设置速度所需的输入。
            speed_hz: 目标雷达转速，单位为 Hz。

        Returns:
            本次处理产生的结果值。
        """
        return {"command_sent": True, "requested_speed_hz": speed_hz}

    def acquire_client(self):
        """获取客户端。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。
        """
        return True

    def release_client(self):
        """释放客户端。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        pass


@unittest.skipIf(TestClient is None, "Starlette test client is not installed")
class WebTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        """设置upclass。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        from embodied_nexus_lidar.web_server import create_app
        cls.gateway = FakeGateway()
        cls.client = TestClient(create_app(cls.gateway, "test-token"))

    def test_http_bearer_auth_and_strict_control_types(self):
        """验证httpbearerauth和strict控制权类型。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        endpoint = "/api/v1/lidars/lidar_main/status"
        self.assertEqual(self.client.get(endpoint).status_code, 401)
        auth = {"Authorization": "Bearer test-token"}
        self.assertEqual(self.client.get(endpoint, headers=auth).status_code, 200)
        bad = self.client.put("/api/v1/lidars/lidar_main/capture", headers=auth,
                              json={"capturing": "false"})
        self.assertEqual(bad.status_code, 400)
        self.assertEqual(self.gateway.capture_values, [])

    def test_websocket_binary_layout_roundtrip_and_multiple_clients(self):
        """验证websocketbinarylayoutroundtrip和multipleclients。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        cfg = load_config()
        cloud = mock_cloud(42.25)
        scan = make_scan(cloud, cfg)
        self.gateway.hub.publish("points", cloud)
        self.gateway.hub.publish("scan", scan)

        def receive_frames(ws):
            """接收frames。

            Args:
                ws: ws，作为接收frames所需的输入。

            Returns:
                无返回值。
            """
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
