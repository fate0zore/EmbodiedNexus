from __future__ import annotations

import copy
import time
import unittest
from pathlib import Path

from embodied_nexus_arm.backend import MockBackend
from embodied_nexus_arm.config import load_config, validate_config
from embodied_nexus_arm.controller import ArmController


CONFIG = Path(__file__).resolve().parents[1] / "config" / "aubo_i5_mock.yaml"


class ArmControllerTests(unittest.TestCase):
    def setUp(self):
        self.config = load_config(CONFIG)
        self.backend = MockBackend(self.config)
        self.controller = ArmController(self.config, self.backend)
        self.controller.poll_once()

    def tearDown(self):
        self.controller.shutdown()

    def lease_and_enable(self):
        lease, _ = self.controller.acquire_control("test")
        self.controller.enable_motion(lease, True)
        return lease

    def test_config_checks_joint_mapping_and_unknown_fields(self):
        broken = copy.deepcopy(self.config)
        broken["joints"]["names"][1] = broken["joints"]["names"][0]
        with self.assertRaises(ValueError):
            validate_config(broken)
        broken = copy.deepcopy(self.config)
        broken["extra"] = True
        with self.assertRaises(ValueError):
            validate_config(broken)

    def test_move_uses_measured_feedback_and_reports_acceptance(self):
        lease = self.lease_and_enable()
        target = [0.08, 0, 0, 0, 0, 0]
        result = self.controller.move_joints(lease, target, speed=0.25, timeout=2.0)
        self.assertTrue(result["success"])
        self.assertTrue(result["controller_accepted"])
        self.assertTrue(result["stationary_confirmed"])
        self.assertLessEqual(result["max_error_rad"], self.config["motion"]["goal_tolerance_rad"])

    def test_cancel_requests_stop_and_confirms_measured_stationary_state(self):
        lease = self.lease_and_enable()
        result = self.controller.move_joints(
            lease, [0.5, 0, 0, 0, 0, 0], speed=0.25, timeout=2.0,
            cancel_requested=lambda: True)
        self.assertFalse(result["success"])
        self.assertTrue(result["controller_accepted"])
        self.assertTrue(result["stop_requested"])
        self.assertTrue(result["stationary_confirmed"])

    def test_state_reports_missing_or_moving_feedback_as_unknown_or_not_stationary(self):
        self.assertTrue(self.controller.state()["stationary_confirmed"])
        self.backend.velocities = [0.1, 0, 0, 0, 0, 0]
        self.controller.poll_once()
        self.assertFalse(self.controller.state()["stationary_confirmed"])
        self.backend.config["mock"]["disconnect"] = True
        self.controller.poll_once()
        state = self.controller.state()
        self.assertFalse(state["feedback_valid"])
        self.assertIsNone(state["positions_rad"])
        self.assertIsNone(state["hardware_estop"])
        self.assertIsNone(state["stationary_confirmed"])

    def test_unsafe_or_stale_state_cannot_enable(self):
        lease, _ = self.controller.acquire_control("test")
        self.backend.safety = "EmergencyStop"
        with self.assertRaises(RuntimeError):
            self.controller.enable_motion(lease, True)
        self.assertFalse(self.controller.motion_enabled)

    def test_velocity_watchdog_stops_and_disables_motion(self):
        lease = self.lease_and_enable()
        self.controller.config["motion"]["watchdog_sec"] = 0.04
        self.controller.set_joint_velocity(lease, [0.1, 0, 0, 0, 0, 0])
        time.sleep(0.06)
        self.controller.tick()
        self.assertFalse(self.controller.motion_enabled)
        self.assertTrue(self.controller.state()["stop_requested"])
        self.assertEqual(self.backend.velocities, [0.0] * 6)

    def test_lease_conflict_expiry_and_stop_failure(self):
        lease, _ = self.controller.acquire_control("first", ttl_sec=0.05)
        with self.assertRaises(RuntimeError):
            self.controller.acquire_control("second")
        time.sleep(0.06)
        self.controller.tick()
        self.assertFalse(self.controller.lease_id)
        self.backend.config["mock"]["fail_stop"] = True
        result = self.controller.stop("injected stop error")
        self.assertFalse(result["controller_accepted"])
        self.assertTrue(result["stationary_confirmed"])

    def test_teach_mode_excludes_joint_motion(self):
        lease = self.lease_and_enable()
        self.controller.set_teach_mode(lease, True)
        with self.assertRaises(RuntimeError):
            self.controller.move_joints(lease, [0.1] * 6)
        self.controller.set_teach_mode(lease, False)


if __name__ == "__main__":
    unittest.main()
