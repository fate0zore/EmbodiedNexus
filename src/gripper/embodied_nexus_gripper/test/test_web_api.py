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
        self.operation_record = {"operation_id": "op-1", "state": "succeeded", "success": True}

    def info(self): return {"model": "ms42dc"}
    def state(self): return {"connected": True, "motion_enabled": False}
    def config(self): return {"backend": "mock"}
    def validate_config(self, patch): return {"backend": "mock", **patch}
    def apply_config(self, lease_id, patch): return {"applied": bool(lease_id), "patch": patch}
    def acquire(self, owner, ttl_sec=None): return {"lease_id": "lease-1", "owner": owner}
    def renew(self, lease_id): return {"expires_in_sec": 2.0}
    def release(self, lease_id): return {"released": True}
    def enable(self, lease_id, enabled): return {"motion_enabled": enabled}
    def stop(self, reason): return {"stop_sent": True, "hardware_stop_confirmed": True}
    def start_operation(self, lease_id, command, angle_degrees):
        return {"operation_id": "op-1", "state": "accepted"}
    def operation(self, operation_id):
        if operation_id != "op-1": raise KeyError(operation_id)
        return self.operation_record
    def cancel_operation(self, operation_id): return {"cancel_requested": True}


@unittest.skipIf(TestClient is None, "Starlette/httpx test dependencies are not installed")
class GripperHttpApiTests(unittest.TestCase):
    def setUp(self):
        self.client = TestClient(create_app(FakeGateway(), "test-token"))
        self.headers = {"Authorization": "Bearer test-token"}
        self.base = "/api/v1/grippers/gripper_main"

    def test_authentication_and_read_routes(self):
        self.assertEqual(self.client.get(self.base + "/status").status_code, 401)
        self.assertEqual(self.client.get(self.base + "/info", headers=self.headers).json()["model"], "ms42dc")
        self.assertTrue(self.client.get(self.base + "/status", headers=self.headers).json()["connected"])

    def test_control_operation_and_stop_routes(self):
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
