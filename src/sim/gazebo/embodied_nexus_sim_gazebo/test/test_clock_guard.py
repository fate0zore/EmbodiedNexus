from __future__ import annotations

import unittest
from importlib.util import find_spec

if find_spec("rclpy") is None:
    raise unittest.SkipTest("Gazebo clock guard tests require ROS 2 Humble")

from builtin_interfaces.msg import Time

from embodied_nexus_sim_gazebo.clock_guard import ClockRewindLatch


class ClockRewindLatchTests(unittest.TestCase):
    def test_backward_jump_latches_pause_across_subsequent_forward_time(self):
        guard = ClockRewindLatch()
        self.assertFalse(guard.observe(Time(sec=20)))
        self.assertFalse(guard.observe(Time(sec=20, nanosec=100_000_000)))
        self.assertTrue(guard.observe(Time(sec=0, nanosec=100_000_000)))
        self.assertFalse(guard.observe(Time(sec=0, nanosec=100_000_000)))
        self.assertTrue(guard.observe(Time(sec=0, nanosec=200_000_000)))
        self.assertFalse(guard.observe(Time(sec=0, nanosec=200_000_000)))


if __name__ == "__main__":
    unittest.main()
