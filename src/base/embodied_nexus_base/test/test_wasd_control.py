import importlib.util
import queue
import sys
import threading
import types
import unittest
from pathlib import Path
from types import SimpleNamespace


try:
    import embodied_nexus_client  # noqa: F401
except ImportError:
    sdk_stub = types.ModuleType("embodied_nexus_client")
    sdk_stub.BaseClient = object
    sys.modules["embodied_nexus_client"] = sdk_stub

EXAMPLE = Path(__file__).resolve().parents[2] / "examples" / "wasd_control.py"
SPEC = importlib.util.spec_from_file_location("base_wasd_control_example", EXAMPLE)
WASD = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(WASD)


class WasdControlTests(unittest.TestCase):
    def make_window(self):
        window = WASD.WasdWindow.__new__(WASD.WasdWindow)
        window.lock = threading.RLock()
        window.keys = set()
        window.motion_enabled = True
        window.connected = True
        window.closing = False
        window._linear_speed = 0.16
        window._angular_speed = 0.60
        window.ui_events = queue.Queue()
        window.status_text = object()
        window.state_text = object()
        window.stop_requested = threading.Event()
        window.client = None
        window.lease = None
        return window

    def test_combined_and_opposing_keys(self):
        self.assertEqual(WASD.velocity_for_keys({"w", "a"}, 0.16, 0.60), (0.16, 0.60))
        self.assertEqual(WASD.velocity_for_keys({"s", "d"}, 0.16, 0.60), (-0.16, -0.60))
        self.assertEqual(WASD.velocity_for_keys({"w", "s", "a", "d"}, 0.16, 0.60), (0.0, 0.0))

    def test_key_release_updates_remaining_axis(self):
        window = self.make_window()
        window.keys.update({"w", "a"})
        window._key_release(SimpleNamespace(keysym="w"))
        self.assertEqual(window.keys, {"a"})
        self.assertEqual(WASD.velocity_for_keys(window.keys, 0.16, 0.60), (0.0, 0.60))

    def test_space_and_focus_loss_clear_keys_and_request_stop(self):
        window = self.make_window()
        window.keys.update({"w", "d"})
        window._key_press(SimpleNamespace(widget=object(), keysym="space"))
        self.assertEqual(window.keys, set())
        self.assertTrue(window.stop_requested.is_set())
        window.stop_requested.clear()
        window.keys.add("s")
        window.root = SimpleNamespace(focus_displayof=lambda: None)
        window._focus_lost(None)
        self.assertEqual(window.keys, set())
        self.assertTrue(window.stop_requested.is_set())

    def test_reconnect_stops_and_releases_old_session(self):
        window = self.make_window()

        class OldClient:
            def __init__(self):
                self.stop_calls = []

            def stop(self, reason):
                self.stop_calls.append(reason)
                return {"zero_velocity_written": True}

        class OldLease:
            def __init__(self):
                self.released = False

            def release(self):
                self.released = True

        old_client, old_lease = OldClient(), OldLease()
        window.client, window.lease = old_client, old_lease
        window.keys.add("w")
        window._prepare_reconnect()
        self.assertEqual(old_client.stop_calls, ["重新连接"])
        self.assertTrue(old_lease.released)
        self.assertIsNone(window.client)
        self.assertIsNone(window.lease)
        self.assertFalse(window.motion_enabled)
        self.assertFalse(window.connected)
        self.assertEqual(window.keys, set())


if __name__ == "__main__":
    unittest.main()
