"""Tests for the root service manager's selection and safety rules."""

import importlib.util
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location("services", Path(__file__).parents[1] / "services.py")
services = importlib.util.module_from_spec(spec)
spec.loader.exec_module(services)


class ServiceManagerTests(unittest.TestCase):
    """Exercise argument parsing, ownership and safety behavior."""

    def test_invalid_arguments(self):
        """Reject missing modes, unknown simulators and conflicting selections.

        Args:
            None.

        Returns:
            None.
        """
        for argv in (["start", "-all"], ["start", "--mode", "sim", "--simulator", "mujoco", "-all"],
                     ["start", "--mode", "mock", "-all", "base"],
                     ["start", "--mode", "real", "base"]):
            with self.subTest(argv=argv), self.assertRaises(SystemExit):
                services.arguments(argv)

    def test_aliases_and_ambiguous_types(self):
        """Resolve IDs and reject ambiguous or duplicate aliases.

        Args:
            None.

        Returns:
            None.
        """
        data = {"devices": [{"id": "base_1", "type": "base"}, {"id": "base_2", "type": "base"}]}
        args = services.arguments(["start", "--mode", "mock", "base"])
        with self.assertRaises(ValueError):
            services.select_devices(args, data)
        args = services.arguments(["start", "--mode", "mock", "base_1"])
        self.assertEqual(services.select_devices(args, data)[0]["id"], "base_1")

    def test_unconfirmed_stop_keeps_process(self):
        """Never terminate a moving device without confirmed feedback.

        Args:
            None.

        Returns:
            None.
        """
        item = {"id": "base_main", "type": "base"}
        result = subprocess.CompletedProcess([], 0, "stationary_confirmed=False", "")
        with patch.object(services, "alive", return_value=True), patch.object(services, "ros", return_value=result), patch.object(services, "terminate") as terminate:
            with self.assertRaises(RuntimeError):
                services.stop_standalone(item, {"pid": 1, "start": "1"})
            terminate.assert_not_called()

    def test_stale_pid_is_not_owned(self):
        """Reject a reused PID before signalling it.

        Args:
            None.

        Returns:
            None.
        """
        with patch.object(services, "identity", return_value="different"), patch.object(services.os, "killpg") as kill:
            services.terminate({"pid": 42, "start": "original"})
            kill.assert_not_called()

    def test_camera_readiness_service(self):
        """Use the camera's actual configuration service for readiness.

        Args:
            None.

        Returns:
            None.
        """
        self.assertEqual(services.readiness_service({"id": "camera_front", "type": "camera"}),
                         "/embodied_nexus/cameras/camera_front/config/get")

    def test_mock_rejects_real_backend(self):
        """Reject a real backend even when a custom platform is labeled mock.

        Args:
            None.

        Returns:
            None.
        """
        with tempfile.TemporaryDirectory() as directory:
            Path(directory, "base.yaml").write_text("device_id: base_main\nbackend: scout_waveshare_serial\n")
            item = {"id": "base_main", "type": "base", "config": "base.yaml"}
            with self.assertRaises(ValueError):
                services.validate_device_config(item, str(Path(directory, "platform.yaml")), "mock")

    def test_failed_core_start_cleans_owned_process(self):
        """Terminate the newly owned process if Core never becomes ready.

        Args:
            None.

        Returns:
            None.
        """
        args = services.arguments(["start", "--mode", "mock", "base"])
        item = {"id": "base_main", "type": "base", "config": "base.yaml"}
        record = {"pid": 42, "start": "1", "log": "test.log"}
        state = {}
        with patch.dict(services.os.environ, {"EMBODIED_NEXUS_CORE_TOKEN": "test"}), \
             patch.object(services, "platform", return_value=("platform.yaml", {"devices": [item]})), \
             patch.object(services, "validate_device_config", return_value="base.yaml"), \
             patch.object(services, "port_busy", return_value=False), \
             patch.object(services, "service_exists", return_value=False), \
             patch.object(services, "save_state"), patch.object(services, "launch", return_value=record), \
             patch.object(services, "wait_for", side_effect=RuntimeError("timeout")), \
             patch.object(services, "terminate") as terminate:
            with self.assertRaisesRegex(RuntimeError, "timeout"):
                services.run(args, state)
            terminate.assert_called_once_with(record)
            self.assertEqual(state, {})


if __name__ == "__main__":
    unittest.main()
