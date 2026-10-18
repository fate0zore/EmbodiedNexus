from __future__ import annotations

import copy
import math
import time
import unittest

from embodied_nexus_base.config import DEFAULT_CONFIG, validate_config, validate_patch
from embodied_nexus_base.controller import (
    BaseController,
    relative_distance_progress,
    unwrapped_yaw_delta,
)


class BaseControllerTests(unittest.TestCase):
    def test_relative_distance_uses_start_heading_for_forward_and_reverse(self):
        start = (2.0, 3.0, math.pi / 2)
        self.assertAlmostEqual(relative_distance_progress(start, (2.0, 4.25)), 1.25)
        self.assertAlmostEqual(relative_distance_progress(start, (2.0, 1.5)), -1.5)

    def test_turn_delta_unwraps_across_positive_and_negative_pi(self):
        epsilon = math.radians(2)
        self.assertAlmostEqual(unwrapped_yaw_delta(-math.pi + epsilon, math.pi - epsilon), 2 * epsilon)
        self.assertAlmostEqual(unwrapped_yaw_delta(math.pi - epsilon, -math.pi + epsilon), -2 * epsilon)

    def setUp(self):
        self.config = copy.deepcopy(DEFAULT_CONFIG)
        self.controller = BaseController(self.config)
        self.controller.tick()

    def tearDown(self):
        self.controller.shutdown()

    def control(self):
        lease_id, _ = self.controller.acquire_control("test")
        return lease_id

    def test_motion_is_disabled_until_lease_and_explicit_enable(self):
        lease_id = self.control()
        with self.assertRaisesRegex(RuntimeError, "disabled"):
            self.controller.set_velocity(lease_id, 0.1, 0.0)
        enabled = self.controller.enable_motion(lease_id, True)
        self.assertTrue(enabled["control_mode_confirmed"])
        self.controller.set_velocity(lease_id, 0.1, 0.0)
        self.assertEqual(self.controller.target_linear, 0.1)

    def test_only_one_lease_and_expiry_stops_motion(self):
        lease_id = self.control()
        with self.assertRaisesRegex(RuntimeError, "already held"):
            self.controller.acquire_control("other")
        self.controller.enable_motion(lease_id, True)
        self.controller.set_velocity(lease_id, 0.2, 0.0)
        self.controller.lease_deadline = time.monotonic() - 0.01
        self.controller.tick()
        self.assertFalse(self.controller.motion_enabled)
        self.assertEqual(self.controller.backend.command, (0.0, 0.0))
        self.assertTrue(self.controller.zero_velocity_written)

    def test_velocity_watchdog_sends_zero(self):
        lease_id = self.control()
        self.controller.enable_motion(lease_id, True)
        self.controller.set_velocity(lease_id, 0.1, 0.2)
        self.controller.tick()
        self.assertEqual(self.controller.backend.command, (0.1, 0.2))
        self.controller.command_time = time.monotonic() - 1.0
        self.controller.tick()
        self.assertEqual(self.controller.backend.command, (0.0, 0.0))
        self.assertTrue(self.controller.motion_enabled)

    def test_disconnect_stops_and_requires_reenable(self):
        lease_id = self.control()
        self.controller.enable_motion(lease_id, True)
        self.controller.set_velocity(lease_id, 0.1, 0.0)
        self.config["mock"]["disconnect"] = True
        self.controller.tick()
        self.assertFalse(self.controller.motion_enabled)
        self.assertTrue(self.controller.state()["last_error"])

    def test_stop_failure_is_reported(self):
        self.config["mock"]["fail_stop"] = True
        result = self.controller.stop("test")
        self.assertTrue(result["stop_requested"])
        self.assertFalse(result["zero_velocity_written"])
        self.assertIn("failed", self.controller.state()["last_error"])

    def test_active_relative_operation_blocks_velocity_and_finishes_with_zero(self):
        lease_id = self.control()
        self.controller.enable_motion(lease_id, True)
        operation_id, _ = self.controller.begin_operation(lease_id)
        with self.assertRaisesRegex(RuntimeError, "relative base operation"):
            self.controller.set_velocity(lease_id, 0.1, 0.0)
        self.controller.set_operation_velocity(operation_id, 0.1, 0.0)
        result = self.controller.finish_operation_with_zero(operation_id, "succeeded")
        self.assertTrue(result["zero_velocity_written"])
        self.assertTrue(self.controller.motion_enabled)
        self.assertEqual(self.controller.backend.command, (0.0, 0.0))

    def test_live_config_rejects_unknown_fields_and_higher_speed_limit(self):
        with self.assertRaisesRegex(ValueError, "unknown configuration"):
            validate_config(self.config | {"unrecognized": True})
        with self.assertRaisesRegex(ValueError, "require restart"):
            validate_patch(self.config, {"motion": {"max_linear_mps": 2.0}})


if __name__ == "__main__":
    unittest.main()
