from __future__ import annotations

import time
import unittest

from embodied_nexus_client.gripper import GripperClient


class FakeGripperClient(GripperClient):
    def __init__(self):
        super().__init__("http://localhost", "token")
        self.calls = []
        self.states = [{"state": "accepted"}, {"state": "succeeded", "success": True}]

    def _request(self, method, suffix="", body=None):
        self.calls.append((method, suffix, body))
        if suffix == "control/lease":
            return {"lease_id": "lease-1", "expires_in_sec": 0.15}
        if suffix.startswith("operations/") and method == "GET":
            return self.states.pop(0) if len(self.states) > 1 else self.states[0]
        if suffix == "operations":
            return {"operation_id": "op-1", "state": "submitting"}
        return {"ok": True}


class GripperSdkTests(unittest.TestCase):
    def test_lease_renews_and_operation_handle_waits(self):
        client = FakeGripperClient()
        lease = client.acquire_control("test")
        lease.enable_motion()
        operation = lease.move_close()
        self.assertTrue(operation.wait(timeout=1, poll_interval=0.01)["success"])
        time.sleep(0.2)
        lease.release()
        self.assertTrue(any(path == "control/lease/renew" for _, path, _ in client.calls))
        self.assertEqual(client.calls[-1][1], "control/lease/release")


if __name__ == "__main__":
    unittest.main()
