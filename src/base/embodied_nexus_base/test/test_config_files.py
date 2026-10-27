from __future__ import annotations

import unittest
from pathlib import Path

from embodied_nexus_base.config import load_config


class BaseConfigTests(unittest.TestCase):
    def test_shipped_mock_and_scout_configs_validate(self):
        """验证shippedmock和scoutconfigs验证。

        Args:
            无参数。

        Returns:
            无返回值。
        """
        config_dir = Path(__file__).resolve().parents[1] / "config"
        mock = load_config(config_dir / "scout_mock.yaml")
        scout = load_config(config_dir / "scout_waveshare.yaml")
        sim = load_config(config_dir / "scout_sim.yaml")
        self.assertEqual(mock["backend"], "mock")
        self.assertEqual(scout["backend"], "scout_waveshare_serial")
        self.assertEqual(scout["serial"]["usb_baudrate"], 2_000_000)
        self.assertEqual(scout["serial"]["can_bitrate"], 500_000)
        self.assertEqual(sim["backend"], "ros2_sim")
        self.assertEqual(sim["backend_options"]["odom_topic"], "/sim/arachne/base_controller/odom")


if __name__ == "__main__":
    unittest.main()
