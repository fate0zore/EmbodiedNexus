"""Checks for trajectory progress and cancellation logging."""

import threading
import unittest
from types import SimpleNamespace

from moveit_msgs.msg import RobotTrajectory
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetCartesianPath
from geometry_msgs.msg import Pose
from builtin_interfaces.msg import Time
from trajectory_msgs.msg import JointTrajectoryPoint

from embodied_nexus_operator.grasp_node import GraspCancelled, GraspOperator


class GraspTraceTests(unittest.TestCase):
    """Check progress snapshots remain available if execution is cancelled."""

    def test_progress_snapshot_survives_cancel_and_late_feedback_is_ignored(self):
        """Persist the last feedback and simulator data before a cancellation.

        Returns:
            None.
        """
        events = []
        feedback_callbacks = []
        snapshot = {"simulation_time_sec": 7.0, "arm_trajectory_active": True}
        fake = SimpleNamespace(
            config={"arm_speed_rad_sec": 0.2, "arm_acceleration_rad_sec2": 0.3,
                    "arm_timeout_sec": 45.0},
            _mujoco_diagnostics_lock=threading.Lock(), _diagnostic_endpoint={},
            _latest_endpoint_diagnostics=None, arm_trajectory_client=object(),
            _record_trajectory=lambda *_: None, _publish_trajectory=lambda *_: None,
            _trajectory_profile=lambda _: {"point_count": 1, "duration_sec": 1.0},
            _publish_phase=lambda *_: None,
            _current_mujoco_diagnostics=lambda **_: snapshot,
            _log_run_event=lambda event, **fields: events.append((event, fields)),
            get_logger=lambda: SimpleNamespace(info=lambda *_: None, error=lambda *_: None),
        )

        def run_action(_client, _goal, _timeout, _cancel_check, feedback_callback):
            """Emit one feedback sample, then emulate action cancellation.

            Args:
                _client: Action client placeholder.
                _goal: Submitted action goal.
                _timeout: Action timeout.
                _cancel_check: Cancellation callback.
                feedback_callback: Callback receiving action feedback.

            Returns:
                None; raises GraspCancelled.
            """
            feedback_callbacks.append(feedback_callback)
            feedback_callback(SimpleNamespace(feedback=SimpleNamespace(
                state="moving", point_index=2, max_error_rad=0.9)))
            raise GraspCancelled("cancelled")

        fake._run_action = run_action
        trajectory = RobotTrajectory()
        trajectory.joint_trajectory.joint_names = ["joint"]
        point = JointTrajectoryPoint()
        point.positions = [0.5]
        point.time_from_start.sec = 1
        trajectory.joint_trajectory.points = [point]
        with self.assertRaises(GraspCancelled):
            GraspOperator._execute_joint_trajectory(fake, trajectory, "lease", object(),
                                                     threading.Event(), "observe")
        progress = [fields for event, fields in events if event == "arm_trajectory_progress"]
        self.assertEqual(progress[0]["tracked_waypoint_error_rad"], 0.9)
        self.assertEqual(progress[0]["mujoco_diagnostics"], snapshot)
        self.assertEqual(events[-1][0], "arm_trajectory_failed")
        count = len(events)
        feedback_callbacks[0](SimpleNamespace(feedback=SimpleNamespace(
            state="moving", point_index=3, max_error_rad=1.0)))
        self.assertEqual(len(events), count)

    def test_partial_approach_is_recorded_and_rejected(self):
        """Reject a 99 percent approach while recording its collision-checked plan.

        Returns:
            None.
        """
        response = GetCartesianPath.Response()
        response.error_code.val = MoveItErrorCodes.SUCCESS
        response.fraction = 0.99
        response.solution.joint_trajectory.points = [JointTrajectoryPoint()]
        events, requests, recorded = [], [], []

        def call_service(_client, request, **_kwargs):
            """Return a partial plan and retain the requested collision setting.

            Args:
                _client: Planning client placeholder.
                request: Cartesian planning request.
                **_kwargs: Timeout and cancellation settings.

            Returns:
                A partial Cartesian planning response.
            """
            requests.append(request)
            return response

        fake = SimpleNamespace(
            base_frame="base_link", tip_frame="grasp_frame", cartesian_client=object(),
            config={"moveit_velocity_scaling": 0.25, "moveit_acceleration_scaling": 0.25,
                    "cartesian_min_fraction": 0.90},
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=Time)),
            _call_service=call_service,
            _trajectory_profile=lambda _: {"point_count": 1},
            _log_run_event=lambda event, **fields: events.append((event, fields)),
            _record_trajectory=lambda *args: recorded.append(args),
            _publish_trajectory=lambda *_: None,
            get_logger=lambda: SimpleNamespace(info=lambda *_: None),
        )
        with self.assertRaisesRegex(RuntimeError, "fraction"):
            GraspOperator._plan_cartesian(fake, Pose(), object(), threading.Event(), minimum_fraction=1.0)
        self.assertTrue(requests[0].avoid_collisions)
        self.assertEqual(events[0][1]["minimum_fraction"], 1.0)
        self.assertEqual(recorded[0][1], "cartesian-partial")

    def test_cylinder_collision_matches_fifty_mm_diameter(self):
        """Create the physical cylinder size used by the current scene.

        Returns:
            None.
        """
        fake = SimpleNamespace(config={"target_shape": "cylinder"},
                               base_frame="base_link")
        collision = GraspOperator._target_collision(fake, "cylinder-1", {"center": [0.0, 0.0, 0.025], "yaw": 0, "size": [0.10, 0.05, 0.05]})
        self.assertEqual(list(collision.primitives[0].dimensions), [0.10, 0.025])
        smaller = GraspOperator._target_collision(fake, "cylinder-1", {"center": [0.0, 0.0, 0.02], "yaw": 0, "size": [0.08, 0.04, 0.04]})
        self.assertEqual(list(smaller.primitives[0].dimensions), [0.08, 0.02])


if __name__ == "__main__":
    unittest.main()
