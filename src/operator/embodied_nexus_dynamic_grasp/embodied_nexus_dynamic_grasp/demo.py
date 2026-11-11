"""Run one dynamic grasp demonstration without resetting the simulation."""

import argparse
import time

import rclpy
from action_msgs.msg import GoalStatus
from embodied_nexus_dynamic_grasp_interfaces.action import DynamicGrasp
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String


class Demo(Node):
    """Coordinate the test fixture and the public dynamic grasp action."""

    def __init__(self, object_id):
        """Create fixture and action clients.

        Args:
            object_id: ID of the object configured in the fixture.

        Returns:
            None.
        """
        super().__init__("dynamic_grasp_demo")
        self.object_id = object_id
        self.events = []
        self.started = False
        self.prepare = self.create_publisher(String, "/embodied_nexus/dynamic_grasp/fixture_prepare", 10)
        self.start = self.create_publisher(String, "/embodied_nexus/dynamic_grasp/fixture_start", 10)
        self.release = self.create_publisher(String, "/embodied_nexus/dynamic_grasp/fixture_release", 10)
        self.create_subscription(String, "/embodied_nexus/dynamic_grasp/fixture_event", self.on_event, 10)
        self.client = ActionClient(self, DynamicGrasp, "/embodied_nexus/dynamic_grasp/grasp")

    def on_event(self, message):
        """Record fixture events belonging to the selected object.

        Args:
            message: Fixture event message.

        Returns:
            None.
        """
        if message.data.endswith(":" + self.object_id):
            self.events.append(message.data)
            print("fixture:", message.data, flush=True)

    def on_feedback(self, message):
        """Print action feedback and start the fixture at tracking.

        Args:
            message: Action feedback wrapper.

        Returns:
            None.
        """
        feedback = message.feedback
        print(f"{feedback.phase}: {feedback.detail}", flush=True)
        if feedback.phase == "track" and not self.started:
            self.start.publish(String(data=self.object_id))
            self.started = True

    def wait(self, predicate, timeout):
        """Spin until a condition is met or the deadline expires.

        Args:
            predicate: Callable indicating completion.
            timeout: Maximum wall-clock wait in seconds.

        Returns:
            True if the predicate became true, otherwise False.
        """
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not predicate() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        return bool(predicate())

    def run(self, target_class):
        """Prepare the fixture, execute the action and return its outcome.

        Args:
            target_class: Perception class configured at launch.

        Returns:
            True when the action succeeds and fixture motion was acknowledged.
        """
        goal_handle = None
        pending_goal = None
        result_future = None
        try:
            if not self.client.wait_for_server(timeout_sec=10.0):
                raise RuntimeError("dynamic grasp action unavailable")
            if not self.wait(lambda: self.prepare.get_subscription_count() > 0 and
                             self.start.get_subscription_count() > 0 and
                             self.release.get_subscription_count() > 0 and
                             self.count_publishers('/embodied_nexus/dynamic_grasp/fixture_event') > 0, 10.0):
                raise RuntimeError("motion fixture unavailable")
            self.prepare.publish(String(data=self.object_id))
            if not self.wait(lambda: any(e == 'ready:' + self.object_id or e.startswith('aborted_')
                                         for e in self.events), 30.0):
                raise RuntimeError("motion fixture preparation timed out")
            if 'ready:' + self.object_id not in self.events:
                raise RuntimeError("motion fixture preparation aborted")
            goal = DynamicGrasp.Goal()
            goal.target_class, goal.object_id = target_class, self.object_id
            goal.camera_id, goal.arm_id, goal.gripper_id = 'camera_front', 'arm_main', 'gripper_main'
            pending_goal = self.client.send_goal_async(goal, feedback_callback=self.on_feedback)
            if not self.wait(pending_goal.done, 10.0):
                raise RuntimeError("action goal response timed out")
            goal_handle = pending_goal.result()
            pending_goal = None
            if goal_handle is None or not goal_handle.accepted:
                raise RuntimeError("action goal rejected")
            result_future = goal_handle.get_result_async()
            if not self.wait(lambda: result_future.done() or any(e.startswith('aborted_') for e in self.events), 480.0):
                raise RuntimeError("action timed out")
            if not result_future.done():
                raise RuntimeError("motion fixture aborted")
            response = result_future.result()
            if response is None:
                raise RuntimeError("action result unavailable")
            print(f"result: status={response.status} success={response.result.success} "
                  f"grasp_verified={response.result.grasp_verified} message={response.result.message}", flush=True)
            return (response.status == GoalStatus.STATUS_SUCCEEDED and response.result.success and
                    not any(e.startswith('aborted_') for e in self.events) and
                    self.started and 'started:' + self.object_id in self.events)
        finally:
            if goal_handle is None and pending_goal is not None:
                # A late acceptance must not leave an unattended task running.
                def cancel_late(future):
                    """Cancel a late goal.

                    Args:
                        future: Future holding the goal response.

                    Returns:
                        None.
                    """
                    handle = future.result()
                    if handle is not None and handle.accepted:
                        handle.cancel_goal_async()
                pending_goal.add_done_callback(cancel_late)
                if not self.wait(pending_goal.done, 5.0):
                    print('Goal response unavailable; cancellation could not be confirmed.', flush=True)
            if goal_handle is not None and goal_handle.accepted and result_future is not None and not result_future.done():
                cancel = goal_handle.cancel_goal_async()
                self.wait(cancel.done, 5.0)
                if not self.wait(result_future.done, 10.0):
                    print('Action did not confirm termination after cancellation.', flush=True)
            self.release.publish(String(data=self.object_id))
            rclpy.spin_once(self, timeout_sec=0.1)


def main(args=None):
    """Run the command-line demonstration.

    Args:
        args: Optional command-line arguments.

    Returns:
        None; exits with a nonzero status if the demonstration fails.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--object-id', default='cylinder-1')
    parser.add_argument('--target-class', default='green cylinder')
    options = parser.parse_args(args)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Demo(options.object_id)
    try:
        success = node.run(options.target_class)
        if not success:
            print('Demo failed or fixture movement was not acknowledged.', flush=True)
        raise SystemExit(0 if success else 1)
    except (RuntimeError, KeyboardInterrupt) as exc:
        print(f'Demo interrupted or failed: {exc}', flush=True)
        raise SystemExit(1) from exc
    finally:
        node.client.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
