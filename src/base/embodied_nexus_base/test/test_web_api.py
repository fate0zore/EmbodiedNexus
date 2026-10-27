from __future__ import annotations

import importlib.util
import unittest


@unittest.skipUnless(importlib.util.find_spec("starlette"), "Starlette is not installed in this environment")
class BaseWebApiTests(unittest.TestCase):
    def test_bearer_auth_and_status_endpoint(self):
        """验证bearerauth和状态端点。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。
        """
        from starlette.testclient import TestClient
        from embodied_nexus_base.web_server import create_app

        class Gateway:
            device_id = "base_main"

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
                return {"connected": True, "motion_enabled": False}

            def get_config(self):
                """获取配置。

                Args:
                    无参数。

                Returns:
                    查询或读取到的数据；未找到时的处理方式由实现决定。
                """
                return {"backend": "mock"}

            def stop(self, reason):
                """停止设备当前动作。

                Args:
                    reason: 原因，作为停止设备当前动作所需的输入。

                Returns:
                    本次处理产生的结果值。
                """
                return {"stop_requested": True, "zero_velocity_written": False,
                        "stationary_confirmed": False, "hardware_estop": False,
                        "error": "mock stop write failed"}

        client = TestClient(create_app(Gateway(), "test-token"))
        path = "/api/v1/bases/base_main/status"
        self.assertEqual(client.get(path).status_code, 401)
        response = client.get(path, headers={"Authorization": "Bearer test-token"})
        self.assertEqual(response.status_code, 200)
        self.assertFalse(response.json()["motion_enabled"])
        stop = client.post("/api/v1/bases/base_main/stop", json={"reason": "test"},
                           headers={"Authorization": "Bearer test-token"})
        self.assertEqual(stop.status_code, 200)
        self.assertTrue(stop.json()["stop_requested"])
        self.assertFalse(stop.json()["zero_velocity_written"])

    def test_websocket_requires_both_protocol_and_bearer_token(self):
        """验证websocketrequiresboth协议和bearer令牌。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。
        """
        from starlette.testclient import TestClient
        from embodied_nexus_base.web_server import create_app

        class Gateway:
            device_id = "base_main"

            def status(self):
                """获取服务当前运行状态。

                Args:
                    无参数。

                Returns:
                    本次处理产生的结果值。
                """
                return {"connected": True, "motion_enabled": False}

        client = TestClient(create_app(Gateway(), "test-token"))
        with self.assertRaises(Exception):
            with client.websocket_connect("/ws", subprotocols=["en-base-v1"]):
                pass


if __name__ == "__main__":
    unittest.main()
