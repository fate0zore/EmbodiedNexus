from __future__ import annotations

import math
import threading
import time
import unittest
from pathlib import Path

from embodied_nexus_gripper.backend import MockBackend
from embodied_nexus_gripper.config import load_config
from embodied_nexus_gripper.config import validate_config
from embodied_nexus_gripper.controller import GripperController
from embodied_nexus_gripper.protocol import (
    Feedback, ProtocolError, build_feedback_frame, build_motion_frame,
    build_state_request_frame, parse_feedback_frame,
)


CONFIG = Path(__file__).resolve().parents[1] / "config" / "ms42dc_mock.yaml"


class MS42DCProtocolTests(unittest.TestCase):
    def test_motion_frame_direction_and_checksum(self):
        closing = build_motion_frame(device_id=1, mode=2, direction=1, sub_divide=32,
                                     angle_tenths=19656, speed_tenths=150)
        opening = build_motion_frame(device_id=1, mode=2, direction=0, sub_divide=32,
                                     angle_tenths=19656, speed_tenths=150)
        self.assertEqual(len(closing), 11)
        self.assertEqual(closing[3], 1)
        self.assertEqual(opening[3], 0)
        self.assertEqual(closing[9], __import__("functools").reduce(lambda a, b: a ^ b, closing[:9], 0))
        self.assertEqual(build_state_request_frame(1)[0], 0x7B)

    def test_feedback_keeps_unsigned_32_bit_angle(self):
        expected = Feedback(device_id=1, reached=1, speed_raw=12, angle_raw=0xF1234567)
        self.assertEqual(parse_feedback_frame(build_feedback_frame(expected), 1), expected)

    def test_short_bad_checksum_and_wrong_id_are_rejected(self):
        with self.assertRaises(ProtocolError):
            parse_feedback_frame(b"123", 1)
        valid = bytearray(build_feedback_frame(Feedback(1, 1, 0, 4)))
        valid[-1] ^= 0x01
        with self.assertRaisesRegex(ProtocolError, "checksum"):
            parse_feedback_frame(bytes(valid), 1)
        with self.assertRaisesRegex(ProtocolError, "device id"):
            parse_feedback_frame(build_feedback_frame(Feedback(2, 1, 0, 4)), 1)


class GripperControllerTests(unittest.TestCase):
    def make_controller(self, duration=0.06, timeout=1.0):
        config = load_config(CONFIG)
        config["mock"]["move_duration_sec"] = duration
        config["control"]["operation_timeout_sec"] = timeout
        backend = MockBackend(config)
        controller = GripperController(config, backend)
        controller.poll_once()
        return controller, backend

    def test_motion_requires_lease_enable_and_fresh_transition(self):
        controller, _ = self.make_controller()
        with self.assertRaises(PermissionError):
            controller.move("missing", "close")
        lease_id, _ = controller.acquire_control("test")
        controller.set_motion_enabled(lease_id, True)
        result = controller.move(lease_id, "relative", 20)
        self.assertTrue(result.success, result.error)
        state = controller.state()
        self.assertEqual(state["operation_state"], "succeeded")
        self.assertEqual(state["feedback_angle_raw"], 200)
        controller.shutdown()

    def test_action_goal_reservation_rejects_concurrent_motion(self):
        controller, _ = self.make_controller()
        lease_id, _ = controller.acquire_control("test")
        controller.set_motion_enabled(lease_id, True)
        controller.reserve_motion_goal(lease_id, "close", 0.0, "goal-1")
        with self.assertRaisesRegex(RuntimeError, "another gripper operation"):
            controller.reserve_motion_goal(lease_id, "open", 0.0, "goal-2")
        result = controller.move(lease_id, "close", operation_id="goal-1")
        self.assertTrue(result.success, result.error)
        controller.shutdown()

    def test_nonfinite_angle_is_rejected(self):
        controller, _ = self.make_controller()
        lease_id, _ = controller.acquire_control("test")
        controller.set_motion_enabled(lease_id, True)
        with self.assertRaisesRegex(ValueError, "finite"):
            controller.move(lease_id, "relative", math.nan)
        controller.shutdown()

    def test_configuration_updates_are_atomic_and_versioned(self):
        controller, backend = self.make_controller()
        lease_id, _ = controller.acquire_control("test")
        with self.assertRaisesRegex(ValueError, "requires restart"):
            controller.apply_configuration({
                "control": {"speed_tenths": 80},
                "serial": {"port": "/dev/other"},
            }, lease_id)
        self.assertEqual(controller.configuration()["control"]["speed_tenths"], 150)
        applied = controller.apply_configuration({"control": {"speed_tenths": 80}}, lease_id)
        self.assertEqual(applied["config_version"], 2)
        self.assertEqual(backend.config["control"]["speed_tenths"], 80)
        controller.shutdown()

    def test_unknown_configuration_fields_are_rejected(self):
        config = load_config(CONFIG)
        config["serial"]["silently_ignored"] = True
        with self.assertRaisesRegex(ValueError, "unknown configuration field"):
            validate_config(config)
        config = load_config(CONFIG)
        config["serial"]["timeout_sec"] = math.nan
        with self.assertRaisesRegex(ValueError, "finite"):
            validate_config(config)

    def test_control_conflict_and_lease_expiration_disable_motion(self):
        controller, _ = self.make_controller()
        lease_id, _ = controller.acquire_control("first", 0.12)
        with self.assertRaisesRegex(RuntimeError, "already held"):
            controller.acquire_control("second")
        controller.set_motion_enabled(lease_id, True)
        time.sleep(0.14)
        with self.assertRaises(PermissionError):
            controller.renew_control(lease_id)
        self.assertFalse(controller.state()["motion_enabled"])
        controller.shutdown()

    def test_disconnect_drops_lease_and_requires_new_control(self):
        controller, backend = self.make_controller()
        lease_id, _ = controller.acquire_control("test")
        controller.set_motion_enabled(lease_id, True)
        backend.config["mock"]["disconnect"] = True
        self.assertFalse(controller.poll_once())
        state = controller.state()
        self.assertFalse(state["connected"])
        self.assertFalse(state["motion_enabled"])
        with self.assertRaises(PermissionError):
            controller.renew_control(lease_id)
        controller.shutdown()

    def test_timeout_requests_stop_and_reports_confirmation(self):
        controller, backend = self.make_controller(duration=0.5, timeout=0.12)
        lease_id, _ = controller.acquire_control("test")
        controller.set_motion_enabled(lease_id, True)
        result = controller.move(lease_id, "close")
        self.assertFalse(result.success)
        self.assertIn("timed out", result.error)
        self.assertTrue(result.stop_sent)
        self.assertTrue(result.hardware_stop_confirmed)
        controller.shutdown()

    def test_feedback_loss_times_out_and_stop_failure_is_visible(self):
        controller, backend = self.make_controller(duration=0.5, timeout=0.12)
        lease_id, _ = controller.acquire_control("test")
        controller.set_motion_enabled(lease_id, True)
        backend.config["mock"]["fail_feedback"] = True
        backend.config["mock"]["fail_stop"] = True
        result = controller.move(lease_id, "close")
        self.assertFalse(result.success)
        self.assertFalse(result.stop_sent)
        self.assertFalse(result.hardware_stop_confirmed)
        controller.shutdown()

    def test_external_stop_cancels_active_operation(self):
        controller, _ = self.make_controller(duration=0.5, timeout=1.5)
        lease_id, _ = controller.acquire_control("test")
        controller.set_motion_enabled(lease_id, True)
        result_box = []
        worker = threading.Thread(target=lambda: result_box.append(controller.move(lease_id, "close")))
        worker.start()
        deadline = time.monotonic() + 1
        while controller.state()["operation_state"] != "running" and time.monotonic() < deadline:
            time.sleep(0.01)
        stopped = controller.stop("test stop")
        worker.join(timeout=2)
        self.assertTrue(stopped["stop_sent"])
        self.assertTrue(result_box and result_box[0].cancelled)
        controller.shutdown()


if __name__ == "__main__":
    unittest.main()
