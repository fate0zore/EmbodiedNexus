"""Mock the dynamic demo orchestration without starting robot devices."""

import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

try:
    from embodied_nexus_dynamic_grasp.demo import Demo
except ImportError:
    Demo = None


@unittest.skipIf(Demo is None, "ROS 2 dynamic grasp interfaces unavailable")
class DemoTest(unittest.TestCase):
    """Exercise fixture triggering and failure cleanup."""

    def setUp(self):
        """Prevent ROS spinning in mock tests; returns None."""
        spin = patch('embodied_nexus_dynamic_grasp.demo.rclpy.spin_once')
        spin.start()
        self.addCleanup(spin.stop)

    def fake_demo(self):
        """Build an isolated mock demo.

        Returns:
            Mock with the demo's required attributes.
        """
        demo = MagicMock()
        demo.object_id = 'cylinder-1'
        demo.events = []
        demo.started = False
        demo.prepare.get_subscription_count.return_value = 1
        demo.start.get_subscription_count.return_value = 1
        demo.release.get_subscription_count.return_value = 1
        demo.count_publishers.return_value = 1
        demo.client.wait_for_server.return_value = True
        return demo

    def test_feedback_triggers_once(self):
        """Only tracking feedback starts fixture movement; returns None."""
        demo = self.fake_demo()
        Demo.on_feedback(demo, SimpleNamespace(feedback=SimpleNamespace(phase='plan', detail='planning')))
        demo.start.publish.assert_not_called()
        feedback = SimpleNamespace(feedback=SimpleNamespace(phase='track', detail='tracking'))
        Demo.on_feedback(demo, feedback)
        Demo.on_feedback(demo, feedback)
        self.assertEqual(demo.start.publish.call_count, 1)

    def test_preparation_timeout_releases(self):
        """A preparation timeout releases the fixture and fails; returns None."""
        demo = self.fake_demo()
        demo.wait.side_effect = [True, False]
        with self.assertRaisesRegex(RuntimeError, 'preparation timed out'):
            Demo.run(demo, 'green cylinder')
        demo.release.publish.assert_called_once()
        demo.client.send_goal_async.assert_not_called()

    def test_rejected_action_releases(self):
        """A rejected goal fails and releases the fixture; returns None."""
        demo = self.fake_demo()
        demo.events = ['ready:cylinder-1']
        demo.wait.return_value = True
        demo.client.send_goal_async.return_value.result.return_value = SimpleNamespace(accepted=False)
        with self.assertRaisesRegex(RuntimeError, 'goal rejected'):
            Demo.run(demo, 'green cylinder')
        demo.release.publish.assert_called_once()

    def test_action_timeout_cancels(self):
        """An accepted timed-out goal is cancelled; returns None."""
        demo = self.fake_demo()
        demo.events = ['ready:cylinder-1']
        demo.wait.side_effect = [True, True, True, False, True, True]
        handle = demo.client.send_goal_async.return_value.result.return_value
        handle.accepted = True
        handle.get_result_async.return_value.done.return_value = False
        with self.assertRaisesRegex(RuntimeError, 'action timed out'):
            Demo.run(demo, 'green cylinder')
        handle.cancel_goal_async.assert_called_once()
        demo.release.publish.assert_called_once()


if __name__ == '__main__':
    unittest.main()
