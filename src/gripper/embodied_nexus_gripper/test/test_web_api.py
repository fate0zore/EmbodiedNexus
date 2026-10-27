from __future__ import annotations

import unittest

try:
    from starlette.testclient import TestClient
    from embodied_nexus_gripper.web_server import create_app
except ImportError:  # pragma: no cover - optional on the Windows development host.
    TestClient = None
    create_app = None


class FakeGateway:
    device_id = "gripper_main"

    def __init__(self):
        """初始化对象并保存构造参数或依赖。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        self.operation_record = {"operation_id": "op-1", "state": "succeeded", "success": True}

    def info(self):
        """获取设备能力与静态信息。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。
        """
        return {"model": "ms42dc"}
    def state(self):
        """获取设备当前状态。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。
        """
        return {"connected": True, "motion_enabled": False}
    def config(self):
        """获取当前配置。

        Args:
            无参数。

        Returns:
            本次处理产生的结果值。
        """
        return {"backend": "mock"}
    def validate_config(self, patch):
        """验证配置。

        Args:
            patch: patch，作为验证配置所需的输入。

        Returns:
            布尔值，表示验证配置是否成功或成立。
        """
        return {"backend": "mock", **patch}
    def apply_config(self, lease_id, patch):
        """应用配置。

        Args:
            lease_id: 租约标识符，作为应用配置所需的输入。
            patch: patch，作为应用配置所需的输入。

        Returns:
            本次处理产生的结果值。
        """
        return {"applied": bool(lease_id), "patch": patch}
    def acquire(self, owner, ttl_sec=None):
        """申请设备控制权。

        Args:
            owner: owner，作为申请设备控制权所需的输入。
            ttl_sec: ttlsec，作为申请设备控制权所需的输入。

        Returns:
            本次处理产生的结果值。
        """
        return {"lease_id": "lease-1", "owner": owner}
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
        return {"released": True}
    def enable(self, lease_id, enabled):
        """更新设备运动使能状态。

        Args:
            lease_id: 租约标识符，作为更新设备运动使能状态所需的输入。
            enabled: 表示对应功能是否启用的布尔值。

        Returns:
            本次处理产生的结果值。
        """
        return {"motion_enabled": enabled}
    def stop(self, reason):
        """停止设备当前动作。

        Args:
            reason: 原因，作为停止设备当前动作所需的输入。

        Returns:
            本次处理产生的结果值。
        """
        return {"stop_sent": True, "hardware_stop_confirmed": True}
    def start_operation(self, lease_id, command, angle_degrees):
        """启动操作。

        Args:
            lease_id: 租约标识符，作为启动操作所需的输入。
            command: command，作为启动操作所需的输入。
            angle_degrees: 角度degrees，作为启动操作所需的输入。

        Returns:
            本次处理产生的结果值。
        """
        return {"operation_id": "op-1", "state": "accepted"}
    def operation(self, operation_id):
        """操作当前设备或操作。

        Args:
            operation_id: 操作标识符，作为操作当前设备或操作所需的输入。

        Returns:
            本次处理产生的结果值。
        """
        if operation_id != "op-1": raise KeyError(operation_id)
        return self.operation_record
    def cancel_operation(self, operation_id):
        """取消操作。

        Args:
            operation_id: 操作标识符，作为取消操作所需的输入。

        Returns:
            本次处理产生的结果值。
        """
        return {"cancel_requested": True}


@unittest.skipIf(TestClient is None, "Starlette/httpx test dependencies are not installed")
class GripperHttpApiTests(unittest.TestCase):
    def setUp(self):
        """为当前测试准备所需对象和资源。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        self.client = TestClient(create_app(FakeGateway(), "test-token"))
        self.headers = {"Authorization": "Bearer test-token"}
        self.base = "/api/v1/grippers/gripper_main"

    def test_authentication_and_read_routes(self):
        """验证authentication和读取routes。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        self.assertEqual(self.client.get(self.base + "/status").status_code, 401)
        self.assertEqual(self.client.get(self.base + "/info", headers=self.headers).json()["model"], "ms42dc")
        self.assertTrue(self.client.get(self.base + "/status", headers=self.headers).json()["connected"])

    def test_control_operation_and_stop_routes(self):
        """验证控制权操作和停止routes。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        lease = self.client.post(self.base + "/control/lease", headers=self.headers,
                                 json={"owner": "test"}).json()
        self.assertEqual(lease["lease_id"], "lease-1")
        created = self.client.post(self.base + "/operations", headers=self.headers,
                                   json={"lease_id": "lease-1", "command": "close"})
        self.assertEqual(created.status_code, 202)
        self.assertTrue(self.client.get(self.base + "/operations/op-1", headers=self.headers).json()["success"])
        self.assertTrue(self.client.post(self.base + "/stop", headers=self.headers, json={}).json()["stop_sent"])


if __name__ == "__main__":
    unittest.main()
