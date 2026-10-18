from __future__ import annotations

import unittest

from embodied_nexus_base.protocol import (
    CanFrame, SCOUT_MOTION_STATE_ID, SCOUT_SYSTEM_STATE_ID,
    decode_scout_frames, decode_variable_frames, encode_variable_frame, int16_to_bytes,
)


class ScoutProtocolTests(unittest.TestCase):
    def test_waveshare_standard_frame_matches_vendor_example(self):
        frame = encode_variable_frame(0x123, bytes.fromhex("11 22 33 44 55 66 77 88"))
        self.assertEqual(frame, bytes.fromhex("AA C8 23 01 11 22 33 44 55 66 77 88 55"))

    def test_split_packet_noise_and_multiple_frames(self):
        first = encode_variable_frame(0x123, b"\x01")
        second = encode_variable_frame(0x124, b"\x02\x03")
        frames, remainder = decode_variable_frames(b"noise" + first[:3])
        self.assertEqual(frames, [])
        frames2, remainder2 = decode_variable_frames(remainder + first[3:] + second)
        self.assertEqual([item.can_id for item in frames2], [0x123, 0x124])
        self.assertEqual([item.data for item in frames2], [b"\x01", b"\x02\x03"])
        self.assertEqual(remainder2, b"")

    def test_invalid_frames_are_discarded_without_blocking_next_frame(self):
        valid = encode_variable_frame(0x222, b"\x00")
        frames, remainder = decode_variable_frames(b"\xaa\xffgarbage" + valid)
        self.assertEqual([item.can_id for item in frames], [0x222])
        self.assertEqual(remainder, b"")

    def test_feedback_fields_and_units(self):
        system = CanFrame(SCOUT_SYSTEM_STATE_ID,
                          bytes([0, 1]) + (240).to_bytes(2, "big", signed=True) +
                          (3).to_bytes(2, "big", signed=True) + b"\x00\x00", "standard")
        motion = CanFrame(SCOUT_MOTION_STATE_ID,
                          int16_to_bytes(150) + int16_to_bytes(-250) +
                          int16_to_bytes(0) + int16_to_bytes(0), "standard")
        feedback = decode_scout_frames([system, motion])
        self.assertEqual(feedback.system["vehicle_state"], 0)
        self.assertEqual(feedback.system["control_mode"], 1)
        self.assertEqual(feedback.system["battery_voltage"], 24.0)
        self.assertEqual(feedback.system["error_code"], 3)
        self.assertEqual(feedback.motion["linear_mps"], 0.15)
        self.assertEqual(feedback.motion["angular_rps"], -0.25)


if __name__ == "__main__":
    unittest.main()
