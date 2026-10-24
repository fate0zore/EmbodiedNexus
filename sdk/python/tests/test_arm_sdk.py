from __future__ import annotations

import time
import unittest

from embodied_nexus_client import ArmClient, ArmLease, ArmOperationHandle


class FakeArmClient(ArmClient):
    def __init__(self):
        super().__init__("http://localhost", "token")
        self.calls = []
        self.states = [{"state": "accepted"}, {"state": "succeeded", "success": True}]

    def _request(self, method, suffix="", body=None):
        self.calls.append((method, suffix, body))
        if suffix == "control/lease":
            return {"lease_id": "arm-lease", "expires_in_sec": 0.15}
        if suffix == "operations":
            return {"operation_id": "arm-op", "state": "submitting"}
        if suffix == "operations/arm-op" and method == "GET":
            return self.states.pop(0) if len(self.states) > 1 else self.states[0]
        return {"ok": True}


class ArmSdkTests(unittest.TestCase):
    def test_lease_control_and_operation_handle(self):
        client = FakeArmClient()
        lease = client.acquire_control("test")
        self.assertIsInstance(lease, ArmLease)
        lease.enable_motion()
        lease.set_joint_velocity([0.0] * 6)
        operation = lease.move_joints([0.1, 0, 0, 0, 0, 0])
        self.assertIsInstance(operation, ArmOperationHandle)
        self.assertTrue(operation.wait(timeout=1, poll_interval=0.01)["success"])
        time.sleep(0.2)
        lease.release()
        paths = [path for _method, path, _body in client.calls]
        self.assertIn("control/lease/renew", paths)
        self.assertIn("motion/enabled", paths)
        self.assertIn("joint_velocity", paths)
        self.assertEqual(paths[-1], "control/lease/release")

    def test_http_prefix_includes_device_id(self):
        client = ArmClient("http://localhost", "token", device_id="arm_lab")
        self.assertEqual(client._prefix, "/api/v1/arms/arm_lab")

    def test_velocity_response_preserves_retry_and_command_feedback(self):
        client = FakeArmClient()
        client._request = lambda method, suffix="", body=None: {
            "controller_accepted": False, "retry_next_cycle": True,
            "commanded_positions_rad": [0.02, 0, 0, 0, 0, 0],
            "stationary_confirmed": False,
        }
        lease = ArmLease(client, "lease", 3.0)
        result = lease.set_joint_velocity([0.1, 0, 0, 0, 0, 0])
        self.assertTrue(result["retry_next_cycle"])
        self.assertFalse(result["controller_accepted"])
        self.assertEqual(result["commanded_positions_rad"][0], 0.02)
        self.assertFalse(result["stationary_confirmed"])
        lease._stop_event.set()


if __name__ == "__main__":
    unittest.main()
