from __future__ import annotations

import time
import unittest

from embodied_nexus_client import BaseClient, BaseLease, BaseOperationHandle


class FakeBaseClient(BaseClient):
    def __init__(self):
        super().__init__("http://localhost", "token")
        self.calls = []
        self.states = [{"state": "accepted"}, {"state": "succeeded", "success": True}]

    def _request(self, method, suffix="", body=None):
        self.calls.append((method, suffix, body))
        if suffix == "control/lease":
            return {"lease_id": "base-lease", "expires_in_sec": 0.15}
        if suffix == "operations":
            return {"operation_id": "base-op", "state": "submitting"}
        if suffix == "operations/base-op" and method == "GET":
            return self.states.pop(0) if len(self.states) > 1 else self.states[0]
        return {"ok": True}


class BaseSdkTests(unittest.TestCase):
    def test_public_control_and_operation_handle(self):
        client = FakeBaseClient()
        lease = client.acquire_control("test")
        self.assertIsInstance(lease, BaseLease)
        lease.enable_motion()
        lease.set_velocity(0.16, 0.6)
        operation = lease.move_relative(0.2, 0.1)
        self.assertIsInstance(operation, BaseOperationHandle)
        self.assertTrue(operation.wait(timeout=1, poll_interval=0.01)["success"])
        time.sleep(0.2)
        lease.release()
        paths = [path for _method, path, _body in client.calls]
        self.assertIn("control/lease/renew", paths)
        self.assertIn("motion/enabled", paths)
        self.assertIn("velocity", paths)
        self.assertEqual(paths[-1], "control/lease/release")

    def test_http_prefix_includes_selected_device_id(self):
        client = BaseClient("http://localhost", "token", device_id="base_lab")
        self.assertEqual(client._prefix, "/api/v1/bases/base_lab")


if __name__ == "__main__":
    unittest.main()
