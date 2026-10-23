from __future__ import annotations

import json
import sys
import tempfile
import time
import types
import unittest
from pathlib import Path
from unittest.mock import patch

import yaml

from embodied_nexus_core.config import load_platform
from embodied_nexus_core.manager import DeviceManager, OperationInProgress


class FakeProcess:
    next_pid = 1000

    def __init__(self):
        self.pid = FakeProcess.next_pid
        FakeProcess.next_pid += 1
        self.returncode = None
        self.signals = []

    def poll(self):
        return self.returncode

    def send_signal(self, value):
        self.signals.append(value)

    def wait(self, timeout=None):
        self.returncode = 0
        return self.returncode


class CoreManagerTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.addCleanup(self.temp.cleanup)
        self.processes = []
        self.launch_arguments = []

    def make_platform(self, ids=("base_one", "base_two"), *, stop_timeout=0.05):
        devices = []
        for device_id in ids:
            config_path = self.root / f"{device_id}.yaml"
            config_path.write_text(yaml.safe_dump({"device_id": device_id}), encoding="utf-8")
            devices.append({"id": device_id, "type": "base", "config": config_path.name, "enabled": True})
        platform_path = self.root / "platform.yaml"
        platform_path.write_text(yaml.safe_dump({
            "core": {"start_timeout_sec": 0.2, "shutdown_timeout_sec": stop_timeout,
                     "poll_interval_sec": 0.02, "gateway_unavailable_sec": 0.05},
            "devices": devices,
        }), encoding="utf-8")
        return platform_path

    def make_manager(self, *, ids=("base_one", "base_two"), fail_start=(),
                     stop_confirmed=True, stop_timeout=0.05):
        def popen(arguments, **_options):
            self.launch_arguments.append(list(arguments))
            if any(device_id in arg for arg in arguments for device_id in fail_start):
                raise OSError("mock launch failed")
            process = FakeProcess()
            self.processes.append(process)
            return process

        manager = DeviceManager(str(self.make_platform(ids=ids, stop_timeout=stop_timeout)),
                                popen=popen, check_packages=False)
        self.addCleanup(manager.close_monitor)
        def http(item, suffix, *, method="GET", body=None, timeout=0.5):
            if method == "POST":
                return 200, json.dumps({"success": True, "stop_sent": True,
                                        "zero_velocity_written": True,
                                        "stationary_confirmed": stop_confirmed}).encode(), "application/json"
            return 200, json.dumps({"connected": True, "stationary_confirmed": stop_confirmed,
                                    "motion_enabled": False}).encode(), "application/json"
        manager._http = http
        return manager

    def test_relative_paths_and_duplicate_ids_are_validated(self):
        platform_path = self.make_platform(ids=("base_one",))
        loaded = load_platform(platform_path, check_packages=False)
        self.assertEqual(Path(loaded["devices"][0]["config"]).name, "base_one.yaml")
        data = yaml.safe_load(platform_path.read_text(encoding="utf-8"))
        data["devices"].append(dict(data["devices"][0]))
        platform_path.write_text(yaml.safe_dump(data), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "duplicate device id"):
            load_platform(platform_path, check_packages=False)

    def test_bundled_mock_and_real_profiles_resolve_all_installed_package_resources(self):
        repo = Path(__file__).resolve().parents[4]
        packages = {
            f"embodied_nexus_{kind}": repo / "src" / kind / f"embodied_nexus_{kind}"
            for kind in ("base", "arm", "camera", "gripper", "lidar")
        }
        ament_index = types.ModuleType("ament_index_python")
        package_index = types.ModuleType("ament_index_python.packages")
        package_index.get_package_share_directory = lambda name: str(packages[name])
        ament_index.packages = package_index
        with patch.dict(sys.modules, {
            "ament_index_python": ament_index,
            "ament_index_python.packages": package_index,
        }):
            for profile in ("platform_mock.yaml", "platform_real.yaml"):
                loaded = load_platform(repo / "src/core/embodied_nexus_core/config" / profile)
                self.assertEqual(len(loaded["devices"]), 5)
                self.assertEqual({item["type"] for item in loaded["devices"]},
                                 {"base", "arm", "camera", "gripper", "lidar"})

    def test_package_config_reference_supports_symlink_install_resources(self):
        source_config = self.root / "source" / "config.yaml"
        source_config.parent.mkdir()
        source_config.write_text(yaml.safe_dump({"device_id": "base_one"}), encoding="utf-8")
        package_share = self.root / "install" / "share" / "embodied_nexus_base"
        installed_config = package_share / "config" / "base_one.yaml"
        installed_config.parent.mkdir(parents=True)
        try:
            installed_config.symlink_to(source_config)
        except (NotImplementedError, OSError) as exc:
            self.skipTest(f"symlinks are unavailable: {exc}")

        platform_path = self.root / "platform.yaml"
        platform_path.write_text(yaml.safe_dump({"core": {}, "devices": [{
            "id": "base_one", "type": "base",
            "config": "package://embodied_nexus_base/config/base_one.yaml",
        }]}), encoding="utf-8")
        ament_index = types.ModuleType("ament_index_python")
        package_index = types.ModuleType("ament_index_python.packages")
        package_index.get_package_share_directory = lambda _name: str(package_share)
        ament_index.packages = package_index
        with patch.dict(sys.modules, {
            "ament_index_python": ament_index,
            "ament_index_python.packages": package_index,
        }):
            loaded = load_platform(platform_path, check_packages=False)

        loaded_config = Path(loaded["devices"][0]["config"])
        self.assertTrue(loaded_config.is_symlink())
        self.assertEqual(loaded_config.parent.resolve(), installed_config.parent.resolve())
        self.assertTrue(loaded_config.is_file())

    def test_camera_launch_uses_its_single_node_without_web_node_override(self):
        config_path = self.root / "camera.yaml"
        config_path.write_text(yaml.safe_dump({"camera_id": "camera_one"}), encoding="utf-8")
        platform_path = self.root / "platform.yaml"
        platform_path.write_text(yaml.safe_dump({"core": {}, "devices": [{
            "id": "camera_one", "type": "camera", "config": config_path.name,
        }]}), encoding="utf-8")
        arguments_seen = []

        def popen(arguments, **_options):
            arguments_seen.extend(arguments)
            process = FakeProcess()
            self.processes.append(process)
            return process

        manager = DeviceManager(str(platform_path), popen=popen, check_packages=False)
        self.addCleanup(manager.close_monitor)
        manager._http = lambda *_args, **_kwargs: (
            200, b'{"connected":true}', "application/json",
        )
        result = manager.start("camera_one")
        self.assertTrue(result["success"])
        self.assertTrue(any(argument.startswith("node_name:=en_camera_camera_one_")
                            for argument in arguments_seen))
        self.assertFalse(any(argument.startswith("node_name_web:=") for argument in arguments_seen))

    def test_sanitized_instance_ids_get_distinct_ros_node_names(self):
        manager = self.make_manager(ids=("base-1", "base_1"))
        results = manager.start_all()
        self.assertTrue(all(result["success"] for result in results))
        names = [next(argument.split(":=", 1)[1] for argument in launch
                      if argument.startswith("node_name:=")
                      ) for launch in self.launch_arguments]
        self.assertEqual(len(names), 2)
        self.assertNotEqual(names[0], names[1])

    def test_same_type_instances_start_and_shutdown_independently(self):
        manager = self.make_manager()
        results = manager.start_all()
        self.assertEqual([entry["device_id"] for entry in results], ["base_one", "base_two"])
        self.assertTrue(all(entry["success"] for entry in results))
        self.assertNotEqual(manager._items["base_one"]["port"], manager._items["base_two"]["port"])
        result = manager.shutdown("base_one")
        self.assertTrue(result["success"])
        self.assertEqual(manager.get_device("base_two")["process_state"], "running")

    def test_unconfirmed_stop_retains_device_process(self):
        manager = self.make_manager(stop_confirmed=False)
        manager.start("base_one")
        process = manager._items["base_one"]["process"]
        result = manager.shutdown("base_one")
        self.assertFalse(result["success"])
        self.assertEqual(manager._items["base_one"]["process_state"], "stop_blocked")
        self.assertIs(manager._items["base_one"]["process"], process)
        self.assertIsNone(process.returncode)

    def test_conflicting_operation_is_rejected_without_waiting(self):
        manager = self.make_manager()
        lock = manager._operation_locks["base_one"]
        lock.acquire()
        self.addCleanup(lock.release)
        with self.assertRaises(OperationInProgress):
            manager.start("base_one")
        self.assertEqual(self.processes, [])

    def test_repeated_start_and_shutdown_are_idempotent(self):
        manager = self.make_manager()
        first = manager.start("base_one")
        second = manager.start("base_one")
        self.assertTrue(first["success"])
        self.assertTrue(second["success"])
        self.assertTrue(second["already_running"])
        stopped = manager.shutdown("base_one")
        repeated = manager.shutdown("base_one")
        self.assertTrue(stopped["success"])
        self.assertTrue(repeated["success"])
        self.assertTrue(repeated["already_stopped"])

    def test_restart_does_not_start_after_stop_confirmation_failure(self):
        manager = self.make_manager(stop_confirmed=False)
        manager.start("base_one")
        process = manager._items["base_one"]["process"]
        result = manager.restart("base_one")
        self.assertFalse(result["success"])
        self.assertEqual(len(self.processes), 1)
        self.assertIs(manager._items["base_one"]["process"], process)
        self.assertIsNone(process.returncode)

    def test_stop_feedback_deadline_includes_the_stop_request(self):
        manager = self.make_manager(stop_confirmed=False, stop_timeout=0.6)
        manager.start("base_one")

        def slow_http(item, suffix, *, method="GET", body=None, timeout=0.5):
            time.sleep(0.35)
            if method == "POST":
                payload = {"stop_sent": True, "zero_velocity_written": True,
                           "stationary_confirmed": False}
            else:
                payload = {"stationary_confirmed": False}
            return 200, json.dumps(payload).encode(), "application/json"

        manager._http = slow_http
        started = time.monotonic()
        result = manager.shutdown("base_one")
        elapsed = time.monotonic() - started
        self.assertFalse(result["success"])
        self.assertLess(elapsed, 0.9)

    def test_process_exit_reason_remains_visible_after_last_successful_probe(self):
        manager = self.make_manager()
        manager.start("base_one")
        process = manager._items["base_one"]["process"]
        process.returncode = 7
        manager._probe("base_one")
        snapshot = manager.get_device("base_one")
        self.assertEqual(snapshot["process_state"], "failed")
        self.assertIn("exited with code 7", snapshot["last_error"])

    def test_batch_start_reports_partial_failure_without_rollback(self):
        manager = self.make_manager(fail_start=("base_two",))
        results = manager.start_all()
        by_id = {entry["device_id"]: entry for entry in results}
        self.assertTrue(by_id["base_one"]["success"])
        self.assertFalse(by_id["base_two"]["success"])
        self.assertEqual(manager._items["base_one"]["process_state"], "running")


if __name__ == "__main__":
    unittest.main()
