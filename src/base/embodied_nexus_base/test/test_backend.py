from __future__ import annotations

import copy
import unittest

from embodied_nexus_base.backend import ScoutWaveshareSerialBackend
from embodied_nexus_base.config import DEFAULT_CONFIG
from embodied_nexus_base.protocol import decode_variable_frames, encode_variable_frame


class FakeSerial:
    def __init__(self, port, **kwargs):
        self.port = port
        self.kwargs = kwargs
        self.writes: list[bytes] = []
        self.reads: list[bytes] = []
        self.closed = False

    def write(self, data):
        self.writes.append(data)
        return len(data)

    def read(self, _size):
        return self.reads.pop(0) if self.reads else b""

    def close(self):
        self.closed = True


class ScoutBackendTests(unittest.TestCase):
    def setUp(self):
        self.config = copy.deepcopy(DEFAULT_CONFIG)
        self.config["backend"] = "scout_waveshare_serial"
        self.serial = None

        def factory(port, **kwargs):
            self.serial = FakeSerial(port, **kwargs)
            return self.serial

        self.backend = ScoutWaveshareSerialBackend(self.config, serial_factory=factory)

    def test_open_only_configures_adapter_and_does_not_enable_scout(self):
        self.backend.open()
        self.assertEqual(len(self.serial.writes), 1)
        frames, remainder = decode_variable_frames(self.serial.writes[0])
        self.assertEqual(frames, [])
        self.assertNotIn(0x421, [frame.can_id for frame in frames])
        self.assertEqual(remainder, b"")
        self.assertFalse(self.backend.control_mode_requested)

    def test_command_and_feedback_use_serial_boundary(self):
        self.backend.open()
        self.backend.request_control_mode()
        self.backend.set_velocity(0.15, -0.25)
        mode_frames, _ = decode_variable_frames(self.serial.writes[1])
        velocity_frames, _ = decode_variable_frames(self.serial.writes[2])
        self.assertEqual(mode_frames[0].can_id, 0x421)
        self.assertEqual(velocity_frames[0].can_id, 0x111)
        self.serial.reads.append(encode_variable_frame(0x221, bytes.fromhex("00 96 FF 06 00 00 00 00")))
        _, motion, connected, error = self.backend.poll()
        self.assertTrue(connected)
        self.assertEqual(error, "")
        self.assertEqual(motion["linear_mps"], 0.15)
        self.assertEqual(motion["angular_rps"], -0.25)


if __name__ == "__main__":
    unittest.main()
