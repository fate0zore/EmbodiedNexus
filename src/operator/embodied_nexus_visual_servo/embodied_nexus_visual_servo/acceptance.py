"""Independent MuJoCo ground-truth evaluator for visual servo grasp trials."""

from __future__ import annotations

import argparse
import copy
import json
import math
import threading
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from embodied_nexus_visual_servo_interfaces.action import VisualServoGrasp
from embodied_nexus_sim_mujoco_interfaces.msg import SceneObjectStates
from embodied_nexus_sim_mujoco_interfaces.srv import ApplyScene
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String


class GraspAcceptance(Node):
    """Trigger grasp trials and evaluate them from MuJoCo-only state topics."""

    def __init__(self, object_id: str, camera_id: str, arm_id: str,
                 gripper_id: str, base_world_height_m: float,
                 target_class: str = "red rectangular box", scene_id: str = "visual_servo_contact") -> None:
        """Create the action, scene-reset, and ground-truth monitor clients.

        Args:
            object_id: MuJoCo scene object ID to evaluate.
            camera_id: Nexus camera instance ID for the action request.
            arm_id: Nexus arm instance ID for the action request.
            gripper_id: Nexus gripper instance ID for the action request.
            base_world_height_m: MuJoCo world Z offset of the robot base frame.
            target_class: YOLOE text prompt requested for each grasp trial.
            scene_id: MuJoCo scene to reset before each trial.

        Returns:
            None.
        """
        super().__init__("embodied_nexus_grasp_acceptance")
        self.object_id = str(object_id)
        self.camera_id = str(camera_id)
        self.arm_id = str(arm_id)
        self.gripper_id = str(gripper_id)
        self.base_world_height_m = float(base_world_height_m)
        self.target_class = str(target_class)
        self.scene_id = str(scene_id)
        self.callback_group = ReentrantCallbackGroup()
        self._condition = threading.Condition()
        self._latest: SceneObjectStates | None = None
        self._recording = False
        self._samples: list[SceneObjectStates] = []
        self._trial_feedback: list[dict] = []
        self._contact_diagnostics: list[dict] = []
        self.create_subscription(SceneObjectStates, "/sim/arachne/ground_truth/scene_objects",
                                 self._on_scene_state, 10, callback_group=self.callback_group)
        self.create_subscription(String, "/sim/arachne/gripper/contact_state",
                                 self._on_contact_state, 10, callback_group=self.callback_group)
        self.scene_client = self.create_client(ApplyScene, "/sim/arachne/mujoco/apply_scene",
                                               callback_group=self.callback_group)
        self.action_client = ActionClient(self, VisualServoGrasp, "/embodied_nexus/visual_servo/grasp",
                                          callback_group=self.callback_group)

    def _on_contact_state(self, message: String) -> None:
        """Record force diagnostics independently from the control path.

        Args:
            message: Simulator contact diagnostics JSON.

        Returns:
            None.
        """
        try:
            payload = json.loads(message.data)
            forces = payload.get('normal_forces_n', {}).get(self.object_id)
            if forces is None:
                return
            record = {'left': float(forces['left']), 'right': float(forces['right'])}
            if not all(math.isfinite(value) and value >= 0 for value in record.values()):
                return
        except (ValueError, TypeError, KeyError, json.JSONDecodeError):
            return
        with self._condition:
            if self._recording:
                self._contact_diagnostics.append(record)
                if len(self._contact_diagnostics) > 5000:
                    self._contact_diagnostics.pop(0)

    def _on_scene_state(self, message: SceneObjectStates) -> None:
        """Cache simulator ground truth and record samples during one trial.

        Args:
            message: Simulator-only object pose and finger-contact state.

        Returns:
            None.
        """
        sample = copy.deepcopy(message)
        with self._condition:
            self._latest = sample
            if self._recording:
                self._samples.append(sample)
                if len(self._samples) > 5000:
                    self._samples.pop(0)
            self._condition.notify_all()

    def _on_trial_feedback(self, message) -> None:
        """Record grasp state-machine feedback for the active acceptance trial.

        Args:
            message: Action feedback wrapper containing phase, detail, and confidence.

        Returns:
            None.
        """
        feedback = message.feedback
        with self._condition:
            if self._recording:
                record = {
                    "phase": str(feedback.phase),
                    "detail": str(feedback.detail),
                    "confidence": float(feedback.detection_confidence),
                    "at_monotonic": time.monotonic(),
                }
                if self._trial_feedback and record["phase"] == "moving" and (
                        self._trial_feedback[-1]["phase"] == "moving"):
                    self._trial_feedback[-1]["detail"] = record["detail"]
                    self._trial_feedback[-1]["at_monotonic"] = record["at_monotonic"]
                elif not self._trial_feedback or (
                        record["phase"], record["detail"]) != (
                            self._trial_feedback[-1]["phase"], self._trial_feedback[-1]["detail"]):
                    self._trial_feedback.append(record)

    def _wait_future(self, future, timeout: float):
        """Wait for a ROS future using a wall-clock deadline.

        Args:
            future: Asynchronous service or action future.
            timeout: Maximum wait in seconds.

        Returns:
            The completed future result.
        """
        deadline = time.monotonic() + float(timeout)
        while not future.done():
            if time.monotonic() >= deadline:
                raise TimeoutError("MuJoCo grasp acceptance operation timed out")
            time.sleep(0.02)
        return future.result()

    def _wait_truth(self, timeout: float = 5.0) -> SceneObjectStates:
        """Wait for a fresh simulator object-state sample.

        Args:
            timeout: Maximum wall-clock wait in seconds.

        Returns:
            Latest SceneObjectStates message.
        """
        deadline = time.monotonic() + float(timeout)
        with self._condition:
            while self._latest is None:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError("MuJoCo ground-truth scene topic is unavailable")
                self._condition.wait(remaining)
            return copy.deepcopy(self._latest)

    @staticmethod
    def _stamp_seconds(message: SceneObjectStates) -> float:
        """Convert a scene-state simulation timestamp to seconds.

        Args:
            message: MuJoCo scene object state carrying a ROS header timestamp.

        Returns:
            Simulation time in seconds.
        """
        stamp = message.header.stamp
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _wait_simulation_duration(self, duration: float, wall_timeout: float) -> None:
        """Wait until simulator time advances by a duration or a wall deadline expires.

        Args:
            duration: Required increase in MuJoCo simulation time, in seconds.
            wall_timeout: Maximum wall-clock wait in seconds.

        Returns:
            None; raises TimeoutError if the simulator does not advance.
        """
        start = self._stamp_seconds(self._wait_truth())
        deadline = time.monotonic() + float(wall_timeout)
        with self._condition:
            while self._latest is None or self._stamp_seconds(self._latest) - start < float(duration):
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError("MuJoCo simulation time did not advance for the hold interval")
                self._condition.wait(min(remaining, 0.2))

    def _reset_scene(self) -> None:
        """Reload the configured scene and allow the object to settle.

        Args:
            None.

        Returns:
            None.
        """
        if not self.scene_client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError("MuJoCo ApplyScene service is unavailable")
        request = ApplyScene.Request()
        request.scene_id = self.scene_id
        response = self._wait_future(self.scene_client.call_async(request), 15.0)
        if not response.success:
            raise RuntimeError(f"could not reset {self.scene_id} scene: {response.message}")
        time.sleep(1.0)

    def _pose_for_id(self, message: SceneObjectStates, object_id: str):
        """Return the simulator pose aligned with one object ID.

        Args:
            message: Simulator state containing parallel ID and pose arrays.
            object_id: Object ID to find.

        Returns:
            Pose for the requested ID.
        """
        try:
            return message.poses[list(message.object_ids).index(object_id)]
        except ValueError as exc:
            raise LookupError(f"MuJoCo scene does not contain object {object_id!r}") from exc

    def _evaluate_lift(self, baseline: SceneObjectStates, samples: list[SceneObjectStates],
                       minimum_lift_m: float, hold_sec: float) -> dict:
        """Check target lift, maintained contact, and stable target-to-gripper offset.

        Args:
            baseline: Simulator state before triggering the grasp.
            samples: Simulator states recorded while the action was running.
            minimum_lift_m: Minimum object rise required in meters.
            hold_sec: Required continuous lift-and-contact duration in seconds.

        Returns:
            Metrics and a pass/fail result for this trial.
        """
        initial = self._pose_for_id(baseline, self.object_id)
        initial_z = float(initial.position.z)
        active: list[tuple[float, np.ndarray]] = []
        maximum_lift = 0.0
        passed = False
        first_contact = None
        contact_samples = 0
        max_horizontal_displacement = 0.0
        for message in samples:
            pose = self._pose_for_id(message, self.object_id)
            max_horizontal_displacement = max(max_horizontal_displacement, math.hypot(
                pose.position.x - initial.position.x, pose.position.y - initial.position.y))
            if self.object_id in message.gripper_contact_object_ids:
                contact_samples += 1
                if first_contact is None:
                    first_contact = {
                        "object_world_m": [pose.position.x, pose.position.y, pose.position.z],
                        "gripper_world_m": [message.gripper_pose.position.x, message.gripper_pose.position.y,
                                            message.gripper_pose.position.z],
                    }
            lift = float(pose.position.z) - initial_z
            maximum_lift = max(maximum_lift, lift)
            if lift < minimum_lift_m or self.object_id not in message.gripper_contact_object_ids:
                if active:
                    passed = passed or self._stable_contact_segment(active, hold_sec)
                    active = []
                continue
            stamp = float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1e-9
            relative = np.array([
                pose.position.x - message.gripper_pose.position.x,
                pose.position.y - message.gripper_pose.position.y,
                pose.position.z - message.gripper_pose.position.z,
            ], dtype=np.float64)
            if active and stamp - active[-1][0] > 0.15:
                passed = passed or self._stable_contact_segment(active, hold_sec)
                active = []
            active.append((stamp, relative))
        passed = passed or self._stable_contact_segment(active, hold_sec)
        return {"passed": passed, "maximum_lift_m": maximum_lift,
                "contact_samples": contact_samples, "first_contact": first_contact,
                "max_horizontal_displacement_m": max_horizontal_displacement,
                "minimum_lift_m": float(minimum_lift_m), "hold_sec": float(hold_sec)}

    @staticmethod
    def _stable_contact_segment(samples: list[tuple[float, np.ndarray]], hold_sec: float) -> bool:
        """Confirm a contact segment lasts long enough with stable relative position.

        Args:
            samples: Ordered simulation-time and object-to-gripper translation samples.
            hold_sec: Minimum required duration in simulation seconds.

        Returns:
            True when duration and relative-pose stability pass.
        """
        if len(samples) < 2 or samples[-1][0] - samples[0][0] < float(hold_sec):
            return False
        offsets = np.stack([item[1] for item in samples])
        median = np.median(offsets, axis=0)
        return bool(np.max(np.linalg.norm(offsets - median, axis=1)) <= 0.02)

    def run_trials(self, trials: int, minimum_lift_m: float = 0.05,
                   hold_sec: float = 2.0,
                   action_timeout_sec: float = 360.0) -> list[dict]:
        """Reset the scene, run grasp goals, and evaluate independent ground truth.

        Args:
            trials: Number of sequential grasp trials.
            minimum_lift_m: Required object rise above its starting position.
            hold_sec: Required lift duration with maintained contact.
            action_timeout_sec: Maximum wall-clock time for each grasp action.

        Returns:
            One result record per trial.
        """
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("grasp action server is unavailable")
        records = []
        for index in range(int(trials)):
            self._reset_scene()
            baseline = self._wait_truth()
            self._pose_for_id(baseline, self.object_id)
            with self._condition:
                self._samples = []
                self._trial_feedback = []
                self._contact_diagnostics = []
                self._recording = True
            goal = VisualServoGrasp.Goal()
            goal.target_class = self.target_class
            goal.object_id = self.object_id
            goal.camera_id = self.camera_id
            goal.arm_id = self.arm_id
            goal.gripper_id = self.gripper_id
            goal_handle = self._wait_future(
                self.action_client.send_goal_async(goal, feedback_callback=self._on_trial_feedback), 10.0)
            if not goal_handle.accepted:
                with self._condition:
                    self._recording = False
                raise RuntimeError("grasp acceptance goal was rejected")
            result_future = goal_handle.get_result_async()
            try:
                wrapped = self._wait_future(result_future, action_timeout_sec)
            except Exception:
                cancel_future = goal_handle.cancel_goal_async()
                try:
                    self._wait_future(cancel_future, 5.0)
                except Exception:
                    pass
                with self._condition:
                    self._recording = False
                raise
            self._wait_simulation_duration(float(hold_sec) + 0.25,
                                           max(10.0, float(hold_sec) * 5.0))
            with self._condition:
                self._recording = False
                samples = list(self._samples)
                feedback_events = list(self._trial_feedback)
                diagnostics = list(self._contact_diagnostics)
            action_result = wrapped.result
            metrics = self._evaluate_lift(baseline, samples, minimum_lift_m, hold_sec)
            if action_result.object_pose.header.frame_id:
                estimate = action_result.object_pose.pose.position
                truth = self._pose_for_id(baseline, self.object_id).position
                estimated_world = np.array([estimate.x, estimate.y,
                                            estimate.z + self.base_world_height_m])
                truth_world = np.array([truth.x, truth.y, truth.z])
                metrics["position_error_m"] = float(np.linalg.norm(estimated_world - truth_world))
            else:
                metrics["position_error_m"] = None
            metrics["action_success"] = bool(action_result.success)
            metrics["scene_id"] = self.scene_id
            metrics["object_id"] = self.object_id
            metrics["target_class"] = self.target_class
            metrics["peak_normal_force_n"] = {
                side: max((sample[side] for sample in diagnostics), default=0.0)
                for side in ('left', 'right')}
            metrics["bilateral_force_samples"] = sum(
                min(sample.values()) >= 0.05 for sample in diagnostics)
            metrics["action_grasp_verified"] = bool(action_result.grasp_verified)
            metrics["action_message"] = str(action_result.message)
            phases = []
            for phase_index, event in enumerate(feedback_events):
                next_time = (feedback_events[phase_index + 1]["at_monotonic"]
                             if phase_index + 1 < len(feedback_events) else event["at_monotonic"])
                phases.append({key: value for key, value in event.items() if key != "at_monotonic"}
                              | {"duration_sec": round(max(0.0, next_time - event["at_monotonic"]), 3)})
            metrics["phase_feedback"] = phases
            metrics["trial"] = index + 1
            metrics["sample_count"] = len(samples)
            metrics["passed"] = bool(metrics["passed"] and action_result.success)
            records.append(metrics)
            print(json.dumps(metrics, ensure_ascii=False, allow_nan=False))
        return records


def main(args=None) -> int:
    """Parse acceptance options, run the requested trials, and report pass count.

    Args:
        args: Optional command-line arguments.

    Returns:
        Process exit code, zero only when every requested trial passes.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trials", type=int, default=5)
    parser.add_argument("--object-id", default="target_box")
    parser.add_argument("--camera-id", default="camera_front")
    parser.add_argument("--arm-id", default="arm_main")
    parser.add_argument("--gripper-id", default="gripper_main")
    parser.add_argument("--target-class", default="red rectangular box")
    parser.add_argument("--scene-id", default="visual_servo_contact")
    parser.add_argument("--minimum-lift-m", type=float, default=0.05)
    parser.add_argument("--hold-sec", type=float, default=2.0)
    parser.add_argument("--action-timeout-sec", type=float, default=360.0)
    parser.add_argument("--base-world-height-m", type=float, default=0.236)
    parsed, ros_args = parser.parse_known_args(args)
    if parsed.trials < 1:
        parser.error("--trials must be positive")
    rclpy.init(args=ros_args)
    node = GraspAcceptance(parsed.object_id, parsed.camera_id, parsed.arm_id,
                           parsed.gripper_id, parsed.base_world_height_m, parsed.target_class, parsed.scene_id)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, name="grasp-acceptance-executor", daemon=True)
    spinner.start()
    try:
        records = node.run_trials(parsed.trials, parsed.minimum_lift_m, parsed.hold_sec,
                                  parsed.action_timeout_sec)
        passed = sum(1 for record in records if record["passed"])
        print(json.dumps({"passed": passed, "trials": len(records)}, ensure_ascii=False))
        return 0 if passed == parsed.trials else 1
    except Exception as exc:
        print(json.dumps({"error": str(exc)}, ensure_ascii=False))
        return 1
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
