"""Tests for action cancellation and guarded target observation."""

import threading
import time
from types import SimpleNamespace
import unittest

try:
    from embodied_nexus_dynamic_grasp.grasp_node import Cancelled, DynamicGraspNode, TargetMoved
    from embodied_nexus_dynamic_grasp_interfaces.msg import DynamicTarget
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


@unittest.skipUnless(ROS_AVAILABLE, "ROS 2 Humble interfaces are unavailable")
class ActionSafetyTest(unittest.TestCase):
    """Exercise safety decisions without moving a simulated robot."""

    def setUp(self):
        """Build a fresh target and a minimal guarded action context.

        Returns:
            None.
        """
        self.target = DynamicTarget()
        self.target.observation_valid = True
        self.target.stationary = True
        self.target.pose.pose.orientation.w = 1.0
        self.target.pose.header.stamp.sec = 10
        self.context = SimpleNamespace(
            lost=threading.Event(), lock=threading.Lock(), target=(self.target, time.monotonic()),
            guard=(self.target, True), target_class="box",
            config={"target_freshness_sec": 1.0, "maximum_target_shift_m": 0.01,
                    "targets": {"box": {"shape": "box"}}},
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_000_000_000)))
        self.goal = SimpleNamespace(is_cancel_requested=False)

    def test_rejects_loss_motion_cancel_and_lease_failure(self):
        """Stop guarded work on observation loss, renewed motion, cancellation or lost lease.

        Returns:
            None.
        """
        DynamicGraspNode._check(self.context, self.goal)
        self.target.stationary = False
        with self.assertRaises(TargetMoved):
            DynamicGraspNode._check(self.context, self.goal)
        self.target.stationary = True
        self.context.target = (self.target, time.monotonic() - 2.0)
        with self.assertRaises(TargetMoved):
            DynamicGraspNode._check(self.context, self.goal)
        self.goal.is_cancel_requested = True
        with self.assertRaises(Cancelled):
            DynamicGraspNode._check(self.context, self.goal)
        self.goal.is_cancel_requested = False
        self.context.lost.set()
        with self.assertRaisesRegex(RuntimeError, "lease"):
            DynamicGraspNode._check(self.context, self.goal)

    def test_guard_failure_cancels_child_action(self):
        """Wait for child cancellation before propagating a target-motion failure.

        Returns:
            None.
        """
        calls = []
        child = SimpleNamespace(accepted=True,
                                get_result_async=lambda: "result",
                                cancel_goal_async=lambda: calls.append("cancel") or "cancelled")
        client = SimpleNamespace(wait_for_server=lambda timeout_sec: True,
                                 send_goal_async=lambda request: "goal")

        def wait(future, timeout, goal=None):
            """Return deterministic fake action results.

            Args:
                future: Identifier of the fake operation.
                timeout: Supplied timeout in seconds.
                goal: Optional parent goal.

            Returns:
                Accepted child for the goal operation, or a sentinel result.
            """
            del timeout, goal
            calls.append("wait_" + future)
            return child if future == "goal" else object()

        def check(goal):
            """Simulate target motion during a guarded child action.

            Args:
                goal: Parent goal.

            Returns:
                None; always raises TargetMoved.
            """
            del goal
            raise TargetMoved("moved")

        context = SimpleNamespace(_wait=wait, _check=check)
        with self.assertRaises(TargetMoved):
            DynamicGraspNode._run_action(context, client, object(), self.goal, 5.0)
        self.assertEqual(calls, ["wait_goal", "cancel", "wait_cancelled", "wait_result"])
