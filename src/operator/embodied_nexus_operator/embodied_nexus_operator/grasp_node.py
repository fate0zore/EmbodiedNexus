"""Planned RGB-D grasp action using YOLOE, MoveIt2, and Nexus device actions."""

from __future__ import annotations

import json
import math
import os
import threading
import time
import uuid
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (AllowedCollisionEntry, AllowedCollisionMatrix, AttachedCollisionObject,
                             CollisionObject, Constraints, JointConstraint, MoveItErrorCodes, OrientationConstraint,
                             PlanningScene, PlanningSceneComponents, PositionConstraint)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetPlanningScene, GetPositionIK
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory
from tf2_ros import Buffer, TransformException, TransformListener

from embodied_nexus_arm_interfaces.action import ExecuteArmTrajectory
from embodied_nexus_arm_interfaces.srv import (AcquireArmControl, GetArmState, ReleaseArmControl,
                                               RenewArmControl, SetArmMotionEnabled, StopArm)
from embodied_nexus_gripper_interfaces.action import MoveGripper
from embodied_nexus_gripper_interfaces.srv import (AcquireGripperControl, GetGripperState,
                                                   ReleaseGripperControl, RenewGripperControl,
                                                   SetGripperMotionEnabled, StopGripper)
from embodied_nexus_operator_interfaces.action import PlanGrasp

from .detector import YOLOEBoxDetector
from .geometry import (decode_image, depth_to_points, gripper_opening_upper_bound, grasp_closing_yaw,
                       measure_target_geometry, rotate_vector, top_down_quaternion, transform_points,
                       validate_target_mask)
from .http_server import OperatorHttpServer


class GraspCancelled(RuntimeError):
    """Signal that the current grasp action was explicitly cancelled."""


class GraspOperator(Node):
    """Coordinate perception, planning, lease-protected execution, and grasping."""

    def __init__(self) -> None:
        """Load operator configuration and create all ROS clients and triggers.

        Args:
            None.

        Returns:
            None.
        """
        super().__init__("embodied_nexus_grasp_operator")
        self.declare_parameter("config_file", "")
        config_path = str(self.get_parameter("config_file").value).strip()
        if not config_path:
            config_path = str(Path(get_package_share_directory("embodied_nexus_operator"))
                              / "config" / "grasp_cylinder.yaml")
        with Path(config_path).expanduser().open("r", encoding="utf-8") as stream:
            root = yaml.safe_load(stream) or {}
        self.config = dict(root.get("operator", {}))
        self.camera_id = str(self.config["camera_id"])
        self.arm_id = str(self.config["arm_id"])
        self.gripper_id = str(self.config["gripper_id"])
        self.base_frame = str(self.config["base_frame"])
        self.tip_frame = str(self.config["end_effector_frame"])
        self.gripper_opening_upper_bound_m = gripper_opening_upper_bound(
            get_package_share_directory("embodied_nexus_sim_description"))
        self.callback_group = ReentrantCallbackGroup()
        self.trajectory_publisher = self.create_publisher(
            JointTrajectory, "/embodied_nexus/operators/grasp/planned_trajectory",
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.trajectory_directory = Path(str(self.config.get(
            "trajectory_directory", "~/.cache/embodied-nexus/trajectories"))).expanduser()
        self.run_log_directory = Path(str(self.config.get(
            "run_log_directory", "~/.cache/embodied-nexus/logs/grasp-runs"))).expanduser()
        self._mujoco_diagnostics_lock = threading.Lock()
        self._latest_mujoco_diagnostics: dict | None = None
        self._latest_endpoint_diagnostics: dict | None = None
        self._diagnostic_endpoint: dict[str, float] = {}
        diagnostics_topic = str(self.config.get(
            "mujoco_diagnostics_topic", "/sim/arachne/mujoco/arm_diagnostics"))
        self._mujoco_diagnostics_subscription = self.create_subscription(
            String, diagnostics_topic, self._on_mujoco_diagnostics, 10,
            callback_group=self.callback_group)
        self._run_log_path: Path | None = None
        self._run_log_lock = threading.Lock()
        self._last_logged_phase: tuple[str, str] | None = None
        self._last_planned_trajectory = None
        self.create_timer(1.0, self._republish_trajectory)
        self._active_lock = threading.RLock()
        self._active = False
        self._active_cancel: threading.Event | None = None
        self._frame_condition = threading.Condition()
        self._frames: dict[str, tuple[object, float] | None] = {
            "color": None, "depth": None, "color_info": None, "depth_info": None,
        }
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        camera_root = f"/embodied_nexus/cameras/{self.camera_id}"
        self.create_subscription(Image, f"{camera_root}/color/image_raw", self._on_color,
                                 qos_profile_sensor_data)
        self.create_subscription(Image, f"{camera_root}/depth/image_raw", self._on_depth,
                                 qos_profile_sensor_data)
        self.create_subscription(CameraInfo, f"{camera_root}/color/camera_info", self._on_color_info,
                                 qos_profile_sensor_data)
        self.create_subscription(CameraInfo, f"{camera_root}/depth/camera_info", self._on_depth_info,
                                 qos_profile_sensor_data)
        self.arm_services = self._create_arm_clients(self.arm_id)
        self.gripper_services = self._create_gripper_clients(self.gripper_id)
        self.arm_trajectory_client = ActionClient(
            self, ExecuteArmTrajectory, f"/embodied_nexus/arms/{self.arm_id}/execute_trajectory",
            callback_group=self.callback_group)
        self.gripper_action_client = ActionClient(
            self, MoveGripper, f"/embodied_nexus/grippers/{self.gripper_id}/move",
            callback_group=self.callback_group)
        self.move_group_client = ActionClient(self, MoveGroup, "/move_action",
                                              callback_group=self.callback_group)
        self.apply_scene_client = self.create_client(ApplyPlanningScene, "/apply_planning_scene",
                                                     callback_group=self.callback_group)
        self.get_planning_scene_client = self.create_client(
            GetPlanningScene, "/get_planning_scene", callback_group=self.callback_group)
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik",
                                            callback_group=self.callback_group)
        self.cartesian_client = self.create_client(GetCartesianPath, "/compute_cartesian_path",
                                                   callback_group=self.callback_group)
        self.detector = YOLOEBoxDetector(
            str(self.config["model_weights"]), str(self.config["text_embeddings"]),
            float(self.config["detector_confidence"]), str(self.config["class_prompt"]))
        self.action_name = str(self.config["action_name"])
        self.action_server = ActionServer(
            self, PlanGrasp, self.action_name, execute_callback=self._execute_grasp,
            goal_callback=self._goal_callback, cancel_callback=self._cancel_callback,
            callback_group=self.callback_group)
        token_env = str(self.config["http_token_env"])
        token = os.environ.get(token_env, "")
        self.http_server = OperatorHttpServer(
            self, ActionClient(self, PlanGrasp, self.action_name, callback_group=self.callback_group),
            str(self.config["http_bind_host"]), int(self.config["http_port"]), token)
        self.get_logger().info(
            f"grasp action={self.action_name} HTTP={self.config['http_bind_host']}:{self.config['http_port']} "
            f"camera={self.camera_id} arm={self.arm_id} gripper={self.gripper_id}")

    def _create_arm_clients(self, arm_id: str) -> dict:
        """Create public ROS clients for one configured arm instance.

        Args:
            arm_id: Nexus arm instance ID.

        Returns:
            Mapping from service operation names to their ROS clients.
        """
        prefix = f"/embodied_nexus/arms/{arm_id}"
        return {
            "state": self.create_client(GetArmState, f"{prefix}/get_state", callback_group=self.callback_group),
            "acquire": self.create_client(AcquireArmControl, f"{prefix}/control/acquire", callback_group=self.callback_group),
            "renew": self.create_client(RenewArmControl, f"{prefix}/control/renew", callback_group=self.callback_group),
            "release": self.create_client(ReleaseArmControl, f"{prefix}/control/release", callback_group=self.callback_group),
            "enable": self.create_client(SetArmMotionEnabled, f"{prefix}/motion/enable", callback_group=self.callback_group),
            "stop": self.create_client(StopArm, f"{prefix}/stop", callback_group=self.callback_group),
        }

    def _create_gripper_clients(self, gripper_id: str) -> dict:
        """Create public ROS clients for one configured gripper instance.

        Args:
            gripper_id: Nexus gripper instance ID.

        Returns:
            Mapping from service operation names to their ROS clients.
        """
        prefix = f"/embodied_nexus/grippers/{gripper_id}"
        return {
            "state": self.create_client(GetGripperState, f"{prefix}/get_state", callback_group=self.callback_group),
            "acquire": self.create_client(AcquireGripperControl, f"{prefix}/control/acquire", callback_group=self.callback_group),
            "renew": self.create_client(RenewGripperControl, f"{prefix}/control/renew", callback_group=self.callback_group),
            "release": self.create_client(ReleaseGripperControl, f"{prefix}/control/release", callback_group=self.callback_group),
            "enable": self.create_client(SetGripperMotionEnabled, f"{prefix}/motion/enable", callback_group=self.callback_group),
            "stop": self.create_client(StopGripper, f"{prefix}/stop", callback_group=self.callback_group),
        }

    def _on_color(self, message: Image) -> None:
        """Cache the most recently received RGB frame and receipt time.

        Args:
            message: Color image from the selected public camera topic.

        Returns:
            None.
        """
        self._store_frame("color", message)

    def _on_depth(self, message: Image) -> None:
        """Cache the most recently received depth frame and receipt time.

        Args:
            message: Depth image from the selected public camera topic.

        Returns:
            None.
        """
        self._store_frame("depth", message)

    def _on_color_info(self, message: CameraInfo) -> None:
        """Cache color-camera calibration information.

        Args:
            message: Color CameraInfo from the selected camera.

        Returns:
            None.
        """
        self._store_frame("color_info", message)

    def _on_depth_info(self, message: CameraInfo) -> None:
        """Cache depth-camera calibration information.

        Args:
            message: Depth CameraInfo from the selected camera.

        Returns:
            None.
        """
        self._store_frame("depth_info", message)

    def _store_frame(self, key: str, message) -> None:
        """Store a sensor message and wake consumers waiting for synchronized data.

        Args:
            key: Cache key for color, depth, or CameraInfo.
            message: Received ROS sensor message.

        Returns:
            None.
        """
        with self._frame_condition:
            self._frames[key] = (message, time.monotonic())
            self._frame_condition.notify_all()

    @staticmethod
    def _stamp_seconds(message) -> float:
        """Convert a stamped ROS message header time into seconds.

        Args:
            message: ROS message carrying a standard header.

        Returns:
            Header timestamp in seconds.
        """
        stamp = message.header.stamp
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _goal_callback(self, goal: PlanGrasp.Goal) -> GoalResponse:
        """Accept only the configured target class and known device instances.

        Args:
            goal: Requested grasp action goal.

        Returns:
            GoalResponse.ACCEPT or GoalResponse.REJECT.
        """
        with self._active_lock:
            if self._active:
                self.get_logger().warning("Rejecting grasp goal: one task is already active")
                return GoalResponse.REJECT
            if (goal.camera_id != self.camera_id or goal.arm_id != self.arm_id or
                    goal.gripper_id != self.gripper_id):
                self.get_logger().warning("Rejecting grasp goal: device IDs differ from operator configuration")
                return GoalResponse.REJECT
            if str(goal.target_class).strip().lower() != str(self.config["class_prompt"]).lower():
                self.get_logger().warning("Rejecting grasp goal: requested class has no cached YOLOE prompt")
                return GoalResponse.REJECT
            if str(goal.object_id).strip() != str(self.config["object_id"]):
                self.get_logger().warning("Rejecting grasp goal with an unknown MuJoCo scene object ID")
                return GoalResponse.REJECT
            self._active = True
            self._active_cancel = threading.Event()
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle) -> CancelResponse:
        """Set the active task's cancellation event.

        Args:
            _goal_handle: Action goal handle requesting cancellation.

        Returns:
            CancelResponse.ACCEPT.
        """
        with self._active_lock:
            if self._active_cancel is not None:
                self._active_cancel.set()
        return CancelResponse.ACCEPT

    def _cancel_check(self, goal_handle, lease_lost: threading.Event | None = None) -> None:
        """Raise when the action is cancelled or a device lease has expired.

        Args:
            goal_handle: Current PlanGrasp goal handle.
            lease_lost: Optional event set by the lease renewal worker.

        Returns:
            None when the task remains authorized to continue.
        """
        if goal_handle.is_cancel_requested:
            raise GraspCancelled("grasp action cancelled")
        if lease_lost is not None and lease_lost.is_set():
            raise RuntimeError("device control lease renewal failed")

    def _publish_phase(self, goal_handle, phase: str, detail: str = "",
                       confidence: float = 0.0) -> None:
        """Publish progress feedback for the current grasp pipeline phase.

        Args:
            goal_handle: Current PlanGrasp goal handle.
            phase: Stable stage name for the algorithm state machine.
            detail: Human-readable progress detail.
            confidence: YOLOE confidence for the current target detection.

        Returns:
            None.
        """
        feedback = PlanGrasp.Feedback()
        feedback.phase = str(phase)
        feedback.detail = str(detail)
        feedback.detection_confidence = float(confidence)
        phase_key = (feedback.phase, feedback.detail)
        if feedback.phase != "moving" and phase_key != self._last_logged_phase:
            self.get_logger().info(
                f"Grasp phase={feedback.phase} detail={feedback.detail} "
                f"detection_confidence={feedback.detection_confidence:.3f}")
            self._log_run_event(
                "phase", phase=feedback.phase, detail=feedback.detail,
                detection_confidence=feedback.detection_confidence)
        self._last_logged_phase = phase_key
        try:
            goal_handle.publish_feedback(feedback)
        except Exception as exc:
            self.get_logger().debug(f"Could not publish grasp feedback: {exc}")

    def _call_service(self, client, request, timeout: float = 5.0, cancel_check=None):
        """Call one public ROS service and wait for its future with a deadline.

        Args:
            client: ROS service client.
            request: Typed ROS service request.
            timeout: Maximum wall-clock wait in seconds.
            cancel_check: Optional callable invoked while waiting.

        Returns:
            Typed ROS service response.
        """
        if not client.wait_for_service(timeout_sec=min(float(timeout), 1.0)):
            raise TimeoutError(f"ROS service {getattr(client, 'srv_name', 'client')} is unavailable")
        future = client.call_async(request)
        deadline = time.monotonic() + float(timeout)
        while not future.done():
            if cancel_check:
                cancel_check()
            if time.monotonic() >= deadline:
                raise TimeoutError(f"ROS service {getattr(client, 'srv_name', 'client')} timed out")
            time.sleep(0.02)
        response = future.result()
        if response is None:
            raise RuntimeError(f"ROS service {getattr(client, 'srv_name', 'client')} returned no response")
        return response

    def _wait_future(self, future, timeout: float, cancel_check=None):
        """Wait for a ROS future while checking cancellation and a wall-clock deadline.

        Args:
            future: ROS asynchronous action or service future.
            timeout: Maximum wait in seconds.
            cancel_check: Optional callable invoked while waiting.

        Returns:
            The completed future result.
        """
        deadline = time.monotonic() + float(timeout)
        while not future.done():
            if cancel_check:
                cancel_check()
            if time.monotonic() >= deadline:
                raise TimeoutError("ROS operation timed out")
            time.sleep(0.02)
        return future.result()

    def _read_device_states(self, cancel_check=None) -> tuple[dict, object]:
        """Read current public arm and gripper state for simulation gating.

        Args:
            cancel_check: Optional cancellation or lease-check callable.

        Returns:
            Parsed arm state JSON and the public GripperState message.
        """
        arm_response = self._call_service(self.arm_services["state"], GetArmState.Request(),
                                          cancel_check=cancel_check)
        gripper_response = self._call_service(self.gripper_services["state"], GetGripperState.Request(),
                                              cancel_check=cancel_check)
        arm_state = json.loads(arm_response.state_json)
        gripper_state = gripper_response.state
        return arm_state, gripper_state

    def _acquire_leases(self, owner: str, cancel_check=None) -> tuple[str, str]:
        """Acquire arm and gripper control leases under a shared operation owner.

        Args:
            owner: Unique operator task owner label.
            cancel_check: Optional cancellation callback.

        Returns:
            Arm and gripper lease identifiers.
        """
        ttl = float(self.config["lease_ttl_sec"])
        arm_request = AcquireArmControl.Request()
        arm_request.owner = owner
        arm_request.ttl_sec = ttl
        arm_response = self._call_service(self.arm_services["acquire"], arm_request,
                                          cancel_check=cancel_check)
        if not arm_response.success:
            raise PermissionError(f"arm control lease rejected: {arm_response.error}")
        gripper_request = AcquireGripperControl.Request()
        gripper_request.owner = owner
        gripper_request.ttl_sec = ttl
        try:
            gripper_response = self._call_service(self.gripper_services["acquire"], gripper_request,
                                                  cancel_check=cancel_check)
            if not gripper_response.success:
                raise PermissionError(f"gripper control lease rejected: {gripper_response.error}")
        except Exception:
            release = ReleaseArmControl.Request()
            release.lease_id = arm_response.lease_id
            self._call_service(self.arm_services["release"], release, timeout=2.0)
            raise
        return str(arm_response.lease_id), str(gripper_response.lease_id)

    def _renew_leases(self, stop_event: threading.Event, lost_event: threading.Event,
                      error_state: dict, leases: tuple[str, str]) -> None:
        """Renew both control leases until the active grasp task finishes.

        Args:
            stop_event: Event requesting the renewal worker to stop.
            lost_event: Event set when either device rejects renewal.
            error_state: Mutable mapping receiving the renewal failure reason.
            leases: Arm and gripper lease identifiers.

        Returns:
            None.
        """
        clients = (self.arm_services["renew"], self.gripper_services["renew"])
        interval = float(self.config["lease_renew_interval_sec"])
        while not stop_event.wait(interval):
            try:
                for client, lease_id in zip(clients, leases):
                    request = RenewArmControl.Request() if client is clients[0] else RenewGripperControl.Request()
                    request.lease_id = lease_id
                    response = self._call_service(client, request, timeout=max(2.0, interval * 2.0))
                    if not response.success:
                        raise PermissionError(response.error or "lease renewal was rejected")
            except Exception as exc:
                error_state["message"] = str(exc)
                self.get_logger().error(f"Device control lease renewal failed: {exc}")
                lost_event.set()
                return

    def _set_motion_enabled(self, arm_lease: str, gripper_lease: str, enabled: bool,
                            cancel_check=None) -> None:
        """Enable or disable motion through the public device services.

        Args:
            arm_lease: Arm control lease identifier.
            gripper_lease: Gripper control lease identifier.
            enabled: Whether device motion should be enabled.
            cancel_check: Optional cancellation callback.

        Returns:
            None.
        """
        arm_request = SetArmMotionEnabled.Request()
        arm_request.lease_id = arm_lease
        arm_request.enabled = bool(enabled)
        arm_response = self._call_service(self.arm_services["enable"], arm_request,
                                          cancel_check=cancel_check)
        if not arm_response.success:
            raise RuntimeError(f"arm motion enable failed: {arm_response.error}")
        gripper_request = SetGripperMotionEnabled.Request()
        gripper_request.lease_id = gripper_lease
        gripper_request.enabled = bool(enabled)
        gripper_response = self._call_service(self.gripper_services["enable"], gripper_request,
                                              cancel_check=cancel_check)
        if not gripper_response.success:
            raise RuntimeError(f"gripper motion enable failed: {gripper_response.error}")

    def _stop_devices(self, reason: str) -> None:
        """Request arm and gripper stops and log any unconfirmed stop response.

        Args:
            reason: Reason recorded by the device stop services.

        Returns:
            None.
        """
        arm_request = StopArm.Request()
        arm_request.reason = str(reason)
        gripper_request = StopGripper.Request()
        gripper_request.reason = str(reason)
        for client, request, name in (
            (self.arm_services["stop"], arm_request, "arm"),
            (self.gripper_services["stop"], gripper_request, "gripper"),
        ):
            try:
                response = self._call_service(client, request, timeout=2.0)
                if not response.success:
                    self.get_logger().error(f"{name} stop was not confirmed: {response.error}")
            except Exception as exc:
                self.get_logger().error(f"Could not request {name} stop: {exc}")

    def _release_leases(self, arm_lease: str, gripper_lease: str) -> None:
        """Release arm and gripper control leases after stopping motion.

        Args:
            arm_lease: Arm lease identifier, possibly empty.
            gripper_lease: Gripper lease identifier, possibly empty.

        Returns:
            None.
        """
        for client, request_type, lease_id, name in (
            (self.arm_services["release"], ReleaseArmControl.Request, arm_lease, "arm"),
            (self.gripper_services["release"], ReleaseGripperControl.Request, gripper_lease, "gripper"),
        ):
            if not lease_id:
                continue
            try:
                request = request_type()
                request.lease_id = lease_id
                response = self._call_service(client, request, timeout=2.0)
                if not response.success:
                    self.get_logger().warning(f"Could not release {name} lease: {response.error}")
            except Exception as exc:
                self.get_logger().warning(f"Could not release {name} lease: {exc}")

    def _wait_for_stopped_frame(self, goal_handle, lease_lost: threading.Event) -> tuple:
        """Wait for a stationary arm and a fresh, timestamp-matched RGB-D pair.

        Args:
            goal_handle: Current grasp action goal handle.
            lease_lost: Event set when device lease renewal fails.

        Returns:
            Color, depth, color CameraInfo, and depth CameraInfo messages.
        """
        check = lambda: self._cancel_check(goal_handle, lease_lost)
        self._publish_phase(goal_handle, "observe", "requesting stop and waiting for fresh RGB-D")
        stop_arm = StopArm.Request()
        stop_arm.reason = "visual grasp observation stop"
        stop_gripper = StopGripper.Request()
        stop_gripper.reason = "visual grasp observation stop"
        for client, request in ((self.arm_services["stop"], stop_arm),
                                (self.gripper_services["stop"], stop_gripper)):
            response = self._call_service(client, request, cancel_check=check)
            if not response.success:
                raise RuntimeError(f"device stop was not confirmed: {response.error}")
        self._set_motion_enabled_for_observation(goal_handle, lease_lost)
        stable_deadline = time.monotonic() + 4.0
        while time.monotonic() < stable_deadline:
            check()
            arm_state, _ = self._read_device_states(check)
            if arm_state.get("backend") != "ros2_sim":
                raise RuntimeError("planned grasp execution is currently restricted to the MuJoCo ros2_sim backend")
            if arm_state.get("feedback_valid") and arm_state.get("stationary_confirmed") is True:
                break
            time.sleep(0.08)
        else:
            raise TimeoutError("arm did not confirm a stationary state before image capture")
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            check()
            with self._frame_condition:
                color = self._frames["color"]
                depth = self._frames["depth"]
                color_info = self._frames["color_info"]
                depth_info = self._frames["depth_info"]
                if all(value is not None for value in (color, depth, color_info, depth_info)):
                    color_msg, color_received = color
                    depth_msg, depth_received = depth
                    color_info_msg = color_info[0]
                    depth_info_msg = depth_info[0]
                    delta = abs(self._stamp_seconds(color_msg) - self._stamp_seconds(depth_msg))
                    age = time.monotonic() - min(color_received, depth_received)
                    if (delta <= float(self.config["maximum_rgb_depth_delta_sec"]) and
                            age <= float(self.config["maximum_frame_age_sec"])):
                        return color_msg, depth_msg, color_info_msg, depth_info_msg
                self._frame_condition.wait(timeout=0.1)
        raise TimeoutError("no fresh RGB-D pair with matching simulation timestamps was received")

    def _set_motion_enabled_for_observation(self, goal_handle, lease_lost: threading.Event) -> None:
        """Enable lease-held device services after the initial stop request.

        Args:
            goal_handle: Current grasp action goal handle.
            lease_lost: Event set when lease renewal fails.

        Returns:
            None.
        """
        # The lease IDs are installed on the active action instance before this stage.
        arm_lease = getattr(self, "_active_arm_lease", "")
        gripper_lease = getattr(self, "_active_gripper_lease", "")
        if not arm_lease or not gripper_lease:
            raise RuntimeError("device leases were not acquired before observation")
        self._set_motion_enabled(arm_lease, gripper_lease, True,
                                cancel_check=lambda: self._cancel_check(goal_handle, lease_lost))

    def _estimate_target(self, color_msg: Image, depth_msg: Image,
                         color_info: CameraInfo, depth_info: CameraInfo,
                         goal_handle, lease_lost: threading.Event) -> tuple[dict, float, PoseStamped]:
        """Segment the requested object and estimate its base-frame box pose.

        Args:
            color_msg: Timestamped RGB image.
            depth_msg: Timestamped depth image.
            color_info: RGB camera intrinsic parameters.
            depth_info: Depth camera intrinsic parameters.
            goal_handle: Current grasp action goal handle.
            lease_lost: Event set when device lease renewal fails.

        Returns:
            Estimated box geometry, detector confidence, and stamped object pose.
        """
        self._cancel_check(goal_handle, lease_lost)
        self._publish_phase(goal_handle, "detect", "running pretrained YOLOE text-prompt segmentation")
        color_image = decode_image(color_msg)
        depth = decode_image(depth_msg)
        if color_image.shape[:2] != (depth_msg.height, depth_msg.width):
            raise ValueError("MuJoCo RGB and depth streams must share the same pixel dimensions")
        if color_msg.encoding.lower() == "bgr8":
            rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        elif color_msg.encoding.lower() == "rgb8":
            rgb = color_image
        else:
            raise ValueError(f"YOLOE requires rgb8 or bgr8 camera frames, got {color_msg.encoding}")
        mask, confidence = self.detector.segment(rgb)
        mask_count = validate_target_mask(mask, int(self.config["minimum_mask_points"]))
        self._publish_phase(goal_handle, "localize", "projecting masked depth and transforming with timestamped TF",
                            confidence)
        stamp = Time.from_msg(depth_msg.header.stamp)
        color_from_depth = self.tf_buffer.lookup_transform(
            color_msg.header.frame_id, depth_msg.header.frame_id, stamp, timeout=Duration(seconds=0.4))
        base_from_depth = self.tf_buffer.lookup_transform(
            self.base_frame, depth_msg.header.frame_id, stamp, timeout=Duration(seconds=0.4))
        points_depth = depth_to_points(depth, mask, depth_info, color_info, color_from_depth)
        if len(points_depth) < max(int(self.config["minimum_mask_points"]), int(mask_count * 0.3)):
            raise ValueError("insufficient valid depth for the segmented target")
        points_base = transform_points(points_depth, base_from_depth)
        estimate = measure_target_geometry(points_base, float(self.config["table_height_base_m"]),
                                           int(self.config["minimum_mask_points"]),
                                           str(self.config.get("target_shape", "box")))
        estimate["measurement"]["mask_pixels"] = mask_count
        estimate["measurement"]["valid_depth_points"] = len(points_depth)
        if estimate["size"][1] > self.gripper_opening_upper_bound_m:
            raise ValueError(f"measured target width {estimate['size'][1]:.3f} m exceeds "
                             f"the robot geometry opening upper bound {self.gripper_opening_upper_bound_m:.3f} m")
        pose = PoseStamped()
        pose.header.frame_id = self.base_frame
        pose.header.stamp = depth_msg.header.stamp
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = estimate["center"]
        half_yaw = estimate["yaw"] / 2.0
        pose.pose.orientation.z = math.sin(half_yaw)
        pose.pose.orientation.w = math.cos(half_yaw)
        return estimate, confidence, pose

    def _apply_planning_scene(self, scene: PlanningScene, goal_handle, lease_lost: threading.Event) -> None:
        """Apply a planning-scene diff through MoveIt's public service.

        Args:
            scene: PlanningScene diff with collision objects or attached objects.
            goal_handle: Current grasp action goal handle.
            lease_lost: Event set when device lease renewal fails.

        Returns:
            None.
        """
        request = ApplyPlanningScene.Request()
        request.scene = scene
        response = self._call_service(self.apply_scene_client, request, timeout=8.0,
                                      cancel_check=lambda: self._cancel_check(goal_handle, lease_lost))
        if not response.success:
            raise RuntimeError("MoveIt rejected the planning-scene update")

    def _box_collision(self, identifier: str, center: list[float], size: list[float],
                       yaw: float = 0.0) -> CollisionObject:
        """Build one oriented box collision object in the base frame.

        Args:
            identifier: Stable MoveIt collision-object ID.
            center: Box center XYZ in the robot base frame.
            size: Box dimensions XYZ in meters.
            yaw: Rotation about the base-frame Z axis in radians.

        Returns:
            CollisionObject with one box primitive.
        """
        collision = CollisionObject()
        collision.id = str(identifier)
        collision.header.frame_id = self.base_frame
        collision.operation = CollisionObject.ADD
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [float(value) for value in size]
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = (float(value) for value in center)
        pose.orientation.z = math.sin(float(yaw) / 2.0)
        pose.orientation.w = math.cos(float(yaw) / 2.0)
        collision.primitives.append(primitive)
        collision.primitive_poses.append(pose)
        return collision

    def _target_collision(self, identifier: str, estimate: dict) -> CollisionObject:
        """Create the configured target geometry using its estimated center and yaw.

        Args:
            identifier: Planning-scene object identifier.
            estimate: Vision-estimated center and horizontal long-axis yaw.

        Returns:
            A box or horizontal cylinder collision object in the base frame.
        """
        if self.config.get("target_shape", "box") != "cylinder":
            return self._box_collision(identifier, estimate["center"],
                                       list(estimate["size"]), float(estimate["yaw"]))
        size = [float(value) for value in estimate["size"]]
        collision = CollisionObject()
        collision.id = identifier
        collision.header.frame_id = self.base_frame
        collision.operation = CollisionObject.ADD
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.CYLINDER
        primitive.dimensions = [size[0], size[1] / 2.0]
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = estimate["center"]
        half_yaw = float(estimate["yaw"]) / 2.0
        half_pitch = math.pi / 4.0
        pose.orientation.x = -math.sin(half_yaw) * math.sin(half_pitch)
        pose.orientation.y = math.cos(half_yaw) * math.sin(half_pitch)
        pose.orientation.z = math.sin(half_yaw) * math.cos(half_pitch)
        pose.orientation.w = math.cos(half_yaw) * math.cos(half_pitch)
        collision.primitives.append(primitive)
        collision.primitive_poses.append(pose)
        return collision

    def _set_allowed_finger_contact(self, target_id: str, allowed: bool,
                                    goal_handle, lease_lost: threading.Event) -> None:
        """Set collision allowance only between the target and the two fingers.

        Args:
            target_id: Target collision-object ID.
            allowed: Whether finger-to-target contact is allowed for planning.
            goal_handle: Current grasp action goal handle.
            lease_lost: Event set when lease renewal fails.

        Returns:
            None.
        """
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        current = self._call_service(self.get_planning_scene_client, request, timeout=8.0,
                                     cancel_check=lambda: self._cancel_check(goal_handle, lease_lost))
        matrix = current.scene.allowed_collision_matrix
        names = list(matrix.entry_names)
        if len(matrix.entry_values) != len(names):
            raise RuntimeError("MoveIt returned a malformed allowed-collision matrix")
        for entry in matrix.entry_values:
            if len(entry.enabled) != len(names):
                raise RuntimeError("MoveIt returned a non-square allowed-collision matrix")
        for name in (target_id, "ms42dc_left_finger_link", "ms42dc_right_finger_link"):
            if name not in names:
                for entry in matrix.entry_values:
                    entry.enabled.append(False)
                matrix.entry_names.append(name)
                matrix.entry_values.append(AllowedCollisionEntry(enabled=[False] * len(matrix.entry_names)))
                names.append(name)
        target_index = names.index(target_id)
        for finger in ("ms42dc_left_finger_link", "ms42dc_right_finger_link"):
            finger_index = names.index(finger)
            matrix.entry_values[target_index].enabled[finger_index] = bool(allowed)
            matrix.entry_values[finger_index].enabled[target_index] = bool(allowed)
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.allowed_collision_matrix = matrix
        self._apply_planning_scene(scene, goal_handle, lease_lost)

    def _install_collision_scene(self, target_id: str, estimate: dict,
                                 goal_handle, lease_lost: threading.Event) -> None:
        """Add the tabletop and vision-estimated target to MoveIt's world model.

        Args:
            target_id: Target collision-object ID.
            estimate: Vision-based target center and yaw.
            goal_handle: Current grasp action goal handle.
            lease_lost: Event set when lease renewal fails.

        Returns:
            None.
        """
        table_center = [float(value) for value in self.config["table_center_base_m"]]
        table = self._box_collision("grasp_table", table_center,
                                    list(self.config["table_size_m"]))
        target = self._target_collision(target_id, estimate)
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        obstacles = []
        for obstacle in self.config.get("collision_obstacles", []):
            obstacle_id = str(obstacle["id"])
            if obstacle_id in {"grasp_table", target_id}:
                raise ValueError(f"collision obstacle ID is reserved: {obstacle_id}")
            obstacles.append(self._box_collision(
                obstacle_id, list(obstacle["center_base_m"]), list(obstacle["size_m"])))
        scene.world.collision_objects = [table, *obstacles, target]
        self._apply_planning_scene(scene, goal_handle, lease_lost)
        self._set_allowed_finger_contact(target_id, False, goal_handle, lease_lost)

    def _plan_pose(self, target: Pose, goal_handle, lease_lost: threading.Event):
        """Plan to a pose through collision-aware IK, with pose-constrained fallback.

        Args:
            target: Desired grasp-frame pose in the base frame.
            goal_handle: Current grasp action goal handle.
            lease_lost: Event set when lease renewal fails.

        Returns:
            Planned trajectory_msgs/RobotTrajectory.
        """
        if not self.move_group_client.wait_for_server(timeout_sec=2.0):
            raise RuntimeError("MoveIt /move_action server is unavailable")
        target_xyz = (float(target.position.x), float(target.position.y), float(target.position.z))
        target_quaternion = (float(target.orientation.x), float(target.orientation.y),
                             float(target.orientation.z), float(target.orientation.w))
        self.get_logger().info(
            f"Planning grasp-frame pose xyz={target_xyz} quat_xyzw={target_quaternion}")

        try:
            fraction, cartesian = self._plan_cartesian(target, goal_handle, lease_lost)
            self.get_logger().info(
                f"Collision-checked Cartesian path to pregrasp fraction={fraction:.3f}")
            return cartesian
        except GraspCancelled:
            raise
        except Exception as exc:
            self.get_logger().warning(
                f"Cartesian path to pregrasp was incomplete ({exc}); using MoveIt pose planning")
        self._cancel_check(goal_handle, lease_lost)

        ik_request = GetPositionIK.Request()
        ik_request.ik_request.group_name = "manipulator"
        ik_request.ik_request.robot_state.is_diff = True
        ik_request.ik_request.avoid_collisions = True
        ik_request.ik_request.ik_link_name = self.tip_frame
        ik_request.ik_request.pose_stamped.header.frame_id = self.base_frame
        ik_request.ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        ik_request.ik_request.pose_stamped.pose = target
        ik_request.ik_request.timeout = Duration(seconds=1.0).to_msg()
        try:
            ik_result = self._call_service(
                self.ik_client, ik_request, timeout=3.0,
                cancel_check=lambda: self._cancel_check(goal_handle, lease_lost))
        except Exception as exc:
            ik_result = None
            self.get_logger().warning(f"Collision-aware IK request failed; using pose planning: {exc}")
        if (ik_result is not None and
                ik_result.error_code.val == MoveItErrorCodes.SUCCESS):
            joint_state = ik_result.solution.joint_state
            joint_positions = dict(zip(joint_state.name, joint_state.position))
            arm_names = [str(name) for name in self.config["observation_joint_names"]]
            if all(name in joint_positions for name in arm_names):
                joint_goal = MoveGroup.Goal()
                joint_request = joint_goal.request
                joint_request.group_name = "manipulator"
                joint_request.num_planning_attempts = int(self.config["planning_attempts"])
                joint_request.allowed_planning_time = float(self.config["planning_time_sec"])
                joint_request.max_velocity_scaling_factor = float(self.config["moveit_velocity_scaling"])
                joint_request.max_acceleration_scaling_factor = float(self.config["moveit_acceleration_scaling"])
                joint_request.start_state.is_diff = True
                joint_constraints = Constraints()
                joint_constraints.name = "collision_aware_ik_joint_goal"
                for name in arm_names:
                    constraint = JointConstraint()
                    constraint.joint_name = name
                    constraint.position = float(joint_positions[name])
                    constraint.tolerance_above = 0.015
                    constraint.tolerance_below = 0.015
                    constraint.weight = 1.0
                    joint_constraints.joint_constraints.append(constraint)
                joint_request.goal_constraints = [joint_constraints]
                joint_goal.planning_options.plan_only = True
                joint_goal.planning_options.look_around = False
                joint_goal.planning_options.replan = False
                try:
                    joint_result = self._run_action(
                        self.move_group_client, joint_goal, 30.0,
                        lambda: self._cancel_check(goal_handle, lease_lost))
                    if (joint_result.error_code.val == MoveItErrorCodes.SUCCESS and
                            joint_result.planned_trajectory.joint_trajectory.points):
                        self.get_logger().info("MoveIt found a collision-free IK joint-space plan")
                        return joint_result.planned_trajectory
                    self.get_logger().warning(
                        "MoveIt could not plan to the collision-aware IK solution "
                        f"(error_code={joint_result.error_code.val}); retrying with pose constraints")
                except GraspCancelled:
                    raise
                except Exception as exc:
                    self.get_logger().warning(
                        f"MoveIt IK joint-space planning failed ({exc}); retrying with pose constraints")
                self._cancel_check(goal_handle, lease_lost)
        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name = "manipulator"
        request.num_planning_attempts = int(self.config["planning_attempts"])
        request.allowed_planning_time = float(self.config["planning_time_sec"])
        request.max_velocity_scaling_factor = float(self.config["moveit_velocity_scaling"])
        request.max_acceleration_scaling_factor = float(self.config["moveit_acceleration_scaling"])
        request.start_state.is_diff = True
        constraints = Constraints()
        constraints.name = "grasp_frame_pose"
        position = PositionConstraint()
        position.header.frame_id = self.base_frame
        position.header.stamp = self.get_clock().now().to_msg()
        position.link_name = self.tip_frame
        position.weight = 1.0
        region = SolidPrimitive()
        region.type = SolidPrimitive.SPHERE
        region.dimensions = [float(self.config.get("grasp_position_tolerance_m", 0.02))]
        position.constraint_region.primitives.append(region)
        region_pose = Pose()
        region_pose.position = target.position
        position.constraint_region.primitive_poses.append(region_pose)
        orientation = OrientationConstraint()
        orientation.header = position.header
        orientation.link_name = self.tip_frame
        orientation.orientation = target.orientation
        orientation.absolute_x_axis_tolerance = float(
            self.config.get("grasp_orientation_tolerance_rad", 0.15))
        orientation.absolute_y_axis_tolerance = float(
            self.config.get("grasp_orientation_tolerance_rad", 0.15))
        orientation.absolute_z_axis_tolerance = float(
            self.config.get("grasp_yaw_tolerance_rad", 0.20))
        orientation.weight = 1.0
        constraints.position_constraints.append(position)
        constraints.orientation_constraints.append(orientation)
        request.goal_constraints = [constraints]
        goal.planning_options.plan_only = True
        goal.planning_options.look_around = False
        goal.planning_options.replan = False
        last_error = None
        retries = max(1, int(self.config.get("pose_plan_retries", 2)))
        for attempt in range(retries):
            try:
                result = self._run_action(
                    self.move_group_client, goal, 30.0,
                    lambda: self._cancel_check(goal_handle, lease_lost))
                if result.error_code.val != MoveItErrorCodes.SUCCESS:
                    last_error = RuntimeError(
                        f"MoveIt failed to plan pose goal (error_code={result.error_code.val})")
                elif not result.planned_trajectory.joint_trajectory.points:
                    last_error = RuntimeError("MoveIt returned an empty joint trajectory")
                else:
                    return result.planned_trajectory
            except GraspCancelled:
                raise
            except Exception as exc:
                last_error = exc
            self._cancel_check(goal_handle, lease_lost)
            if attempt + 1 < retries:
                self.get_logger().warning(
                    f"MoveIt pose planning attempt {attempt + 1}/{retries} failed ({last_error}); retrying")
        raise RuntimeError(f"MoveIt failed to plan grasp pose after {retries} attempts: {last_error}") from last_error

    def _plan_joint_positions(self, positions: list[float], goal_handle,
                              lease_lost: threading.Event):
        """Ask MoveIt to plan to the configured camera observation joint pose.

        Args:
            positions: Target joint positions in the configured manipulator order.
            goal_handle: Current grasp action goal handle.
            lease_lost: Event set when device lease renewal fails.

        Returns:
            Planned trajectory_msgs/RobotTrajectory.
        """
        names = [str(name) for name in self.config["observation_joint_names"]]
        values = [float(value) for value in positions]
        if not names or len(names) != len(values):
            raise ValueError("observation joint names and positions must have equal nonzero lengths")
        if len(set(names)) != len(names):
            raise ValueError("observation joint names must be unique")
        if not self.move_group_client.wait_for_server(timeout_sec=2.0):
            raise RuntimeError("MoveIt /move_action server is unavailable")
        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name = "manipulator"
        request.num_planning_attempts = int(self.config["planning_attempts"])
        request.allowed_planning_time = float(self.config["planning_time_sec"])
        request.max_velocity_scaling_factor = float(self.config["moveit_velocity_scaling"])
        request.max_acceleration_scaling_factor = float(self.config["moveit_acceleration_scaling"])
        request.start_state.is_diff = True
        constraints = Constraints()
        constraints.name = "camera_observation_joint_goal"
        for name, value in zip(names, values):
            constraint = JointConstraint()
            constraint.joint_name = name
            constraint.position = value
            constraint.tolerance_above = 0.035
            constraint.tolerance_below = 0.035
            constraint.weight = 1.0
            constraints.joint_constraints.append(constraint)
        request.goal_constraints = [constraints]
        goal.planning_options.plan_only = True
        goal.planning_options.look_around = False
        goal.planning_options.replan = False
        result = self._run_action(self.move_group_client, goal, 30.0,
                                  lambda: self._cancel_check(goal_handle, lease_lost))
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(
                f"MoveIt failed to plan the camera observation pose (error_code={result.error_code.val})")
        if not result.planned_trajectory.joint_trajectory.points:
            raise RuntimeError("MoveIt returned an empty camera observation trajectory")
        return result.planned_trajectory

    def _plan_cartesian(self, waypoint: Pose, goal_handle, lease_lost: threading.Event,
                        minimum_fraction: float | None = None):
        """Request a collision-checked Cartesian path to one tool waypoint.

        Args:
            waypoint: Desired grasp-frame waypoint in the base frame.
            goal_handle: Current grasp action goal handle.
            lease_lost: Event set when lease renewal fails.
            minimum_fraction: Optional required fraction; defaults to the configured threshold.

        Returns:
            Cartesian path fraction and RobotTrajectory.
        """
        request = GetCartesianPath.Request()
        request.header.frame_id = self.base_frame
        request.header.stamp = self.get_clock().now().to_msg()
        request.start_state.is_diff = True
        request.group_name = "manipulator"
        request.link_name = self.tip_frame
        request.max_step = 0.005
        request.jump_threshold = 0.0
        request.avoid_collisions = True
        request.max_velocity_scaling_factor = float(self.config["moveit_velocity_scaling"])
        request.max_acceleration_scaling_factor = float(self.config["moveit_acceleration_scaling"])
        request.waypoints = [waypoint]
        response = self._call_service(
            self.cartesian_client, request, timeout=20.0,
            cancel_check=lambda: self._cancel_check(goal_handle, lease_lost))
        profile = self._trajectory_profile(response.solution.joint_trajectory)
        planning_diagnostics = {
            "fraction": float(response.fraction),
            "minimum_fraction": (float(self.config["cartesian_min_fraction"])
                                 if minimum_fraction is None else float(minimum_fraction)),
            "velocity_scaling": request.max_velocity_scaling_factor,
            "acceleration_scaling": request.max_acceleration_scaling_factor,
            "max_step_m": float(request.max_step),
            **profile,
        }
        self.get_logger().info(f"MoveIt Cartesian plan diagnostics={planning_diagnostics}")
        self._log_run_event("cartesian_plan", **planning_diagnostics)
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f"MoveIt Cartesian path failed (error_code={response.error_code.val})")
        if response.fraction < planning_diagnostics["minimum_fraction"]:
            if response.solution.joint_trajectory.points:
                self._record_trajectory(response.solution.joint_trajectory, "cartesian-partial", response.fraction)
                self._publish_trajectory(response.solution.joint_trajectory)
            raise RuntimeError(f"MoveIt Cartesian path fraction is only {response.fraction:.3f}")
        if not response.solution.joint_trajectory.points:
            raise RuntimeError("MoveIt returned an empty Cartesian trajectory")
        return float(response.fraction), response.solution

    @staticmethod
    def _trajectory_profile(trajectory: JointTrajectory) -> dict:
        """Summarize trajectory timing and peak joint derivatives.

        Args:
            trajectory: Joint trajectory returned by MoveIt or submitted to Nexus.

        Returns:
            Point count, duration, and peak absolute velocity and acceleration per joint.
        """
        points = list(trajectory.points)
        joint_names = list(trajectory.joint_names)
        peak_velocities = {name: 0.0 for name in joint_names}
        peak_accelerations = {name: 0.0 for name in joint_names}
        for point in points:
            for index, name in enumerate(joint_names):
                if index < len(point.velocities):
                    peak_velocities[name] = max(peak_velocities[name], abs(float(point.velocities[index])))
                if index < len(point.accelerations):
                    peak_accelerations[name] = max(
                        peak_accelerations[name], abs(float(point.accelerations[index])))
        duration = 0.0
        if points:
            last_time = points[-1].time_from_start
            duration = float(last_time.sec) + float(last_time.nanosec) * 1e-9
        return {"point_count": len(points), "duration_sec": duration,
                "peak_velocity_rad_sec_by_joint": peak_velocities,
                "peak_acceleration_rad_sec2_by_joint": peak_accelerations}

    def _run_action(self, client, goal, timeout: float, cancel_check=None, feedback_callback=None):
        """Send a ROS action goal, handle cancellation, and return its result.

        Args:
            client: ROS ActionClient.
            goal: Typed action goal.
            timeout: Maximum wall-clock wait for goal and result.
            cancel_check: Optional callable that raises if the grasp should stop.
            feedback_callback: Optional callback for the sub-action feedback.

        Returns:
            Typed action result from the accepted goal.
        """
        if not client.wait_for_server(timeout_sec=2.0):
            raise RuntimeError("required ROS action server is unavailable")
        future = client.send_goal_async(goal, feedback_callback=feedback_callback)
        handle = self._wait_future(future, timeout, cancel_check)
        if not handle.accepted:
            raise RuntimeError("required ROS action goal was rejected")
        result_future = handle.get_result_async()
        try:
            wrapped = self._wait_future(result_future, timeout, cancel_check)
        except GraspCancelled:
            handle.cancel_goal_async()
            raise
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            action_result = wrapped.result
            detail = str(getattr(action_result, "message", "")).strip()
            if not detail:
                detail = str(getattr(action_result, "error", "")).strip()
            if not detail and hasattr(action_result, "error_code"):
                detail = f"error_code={action_result.error_code.val}"
            if not detail and hasattr(action_result, "success"):
                detail = f"success={bool(action_result.success)}"
            suffix = f": {detail}" if detail else ""
            raise RuntimeError(f"required ROS action ended with status {wrapped.status}{suffix}")
        return wrapped.result

    def _execute_joint_trajectory(self, trajectory, arm_lease: str,
                                  goal_handle, lease_lost: threading.Event,
                                  stage: str = "motion") -> None:
        """Submit a MoveIt trajectory to Nexus arm execution under the active lease.

        Args:
            trajectory: Planned trajectory_msgs/RobotTrajectory.
            arm_lease: Active arm control lease identifier.
            goal_handle: Current PlanGrasp goal handle.
            lease_lost: Event set when lease renewal fails.
            stage: Grasp stage producing the trajectory.

        Returns:
            None.
        """
        action_goal = ExecuteArmTrajectory.Goal()
        action_goal.lease_id = arm_lease
        action_goal.trajectory = trajectory.joint_trajectory
        self._record_trajectory(action_goal.trajectory, stage)
        self._publish_trajectory(action_goal.trajectory)
        action_goal.speed_rad_sec = float(self.config["arm_speed_rad_sec"])
        action_goal.acceleration_rad_sec2 = float(self.config["arm_acceleration_rad_sec2"])
        last_time = action_goal.trajectory.points[-1].time_from_start
        duration = float(last_time.sec) + float(last_time.nanosec) * 1e-9
        timeout_multiplier = float(self.config.get("arm_trajectory_timeout_multiplier", 3.0))
        timeout_margin = float(self.config.get("arm_trajectory_timeout_margin_sec", 10.0))
        if timeout_multiplier < 1.0 or timeout_margin < 0.0:
            raise ValueError("arm trajectory timeout multiplier must be >= 1 and margin must be >= 0")
        action_goal.timeout_sec = max(
            float(self.config["arm_timeout_sec"]), duration * timeout_multiplier + timeout_margin)
        endpoint = dict(zip(action_goal.trajectory.joint_names,
                            action_goal.trajectory.points[-1].positions))
        with self._mujoco_diagnostics_lock:
            self._diagnostic_endpoint = {name: float(value) for name, value in endpoint.items()}
            self._latest_endpoint_diagnostics = None
        profile = self._trajectory_profile(action_goal.trajectory)
        execution_details = {
            "stage": stage, "endpoint_rad": endpoint,
            "requested_speed_rad_sec": action_goal.speed_rad_sec,
            "requested_acceleration_rad_sec2": action_goal.acceleration_rad_sec2,
            "timeout_sec": action_goal.timeout_sec, **profile,
        }
        self.get_logger().info(f"Submitting Nexus arm trajectory diagnostics={execution_details}")
        self._log_run_event("arm_trajectory_submitted", **execution_details)
        last_progress_log = 0.0
        progress_active = True

        def record_progress(message):
            """Publish action feedback and periodically persist the simulator snapshot.

            Args:
                message: ExecuteArmTrajectory feedback wrapper from Nexus.

            Returns:
                None.
            """
            nonlocal last_progress_log
            if not progress_active:
                return
            feedback = message.feedback
            self._publish_phase(goal_handle, "moving", f"Nexus arm trajectory {feedback.point_index}", 0.0)
            now = time.monotonic()
            if now - last_progress_log >= 2.0:
                self._log_run_event(
                    "arm_trajectory_progress", stage=stage, state=str(feedback.state),
                    point_index=int(feedback.point_index),
                    tracked_waypoint_error_rad=float(feedback.max_error_rad),
                    mujoco_diagnostics=self._current_mujoco_diagnostics())
                last_progress_log = now

        try:
            result = self._run_action(
                self.arm_trajectory_client, action_goal, action_goal.timeout_sec + 5.0,
                lambda: self._cancel_check(goal_handle, lease_lost),
                record_progress,
            )
        except Exception as exc:
            self.get_logger().error(f"Nexus arm trajectory failed at stage={stage}: {exc}")
            self._log_run_event(
                "arm_trajectory_failed", stage=stage, error=str(exc),
                mujoco_diagnostics=self._current_mujoco_diagnostics(),
                mujoco_endpoint_diagnostics=self._current_mujoco_diagnostics(endpoint_tracking=True),
                **profile)
            raise
        finally:
            progress_active = False
        if not result.success:
            self.get_logger().error(
                f"Nexus arm trajectory reported unsuccessful at stage={stage}: {result.message}")
            self._log_run_event(
                "arm_trajectory_failed", stage=stage, error=str(result.message),
                mujoco_diagnostics=self._current_mujoco_diagnostics(),
                mujoco_endpoint_diagnostics=self._current_mujoco_diagnostics(endpoint_tracking=True),
                **profile)
            raise RuntimeError(f"Nexus arm trajectory failed: {result.message}")
        self.get_logger().info(
            f"Nexus arm trajectory completed with final joint error="
            f"{result.max_error_rad:.4f} rad")
        self._log_run_event("arm_trajectory_completed", stage=stage,
                            final_max_error_rad=float(result.max_error_rad),
                            mujoco_diagnostics=self._current_mujoco_diagnostics(), **profile)

    def _on_mujoco_diagnostics(self, message: String) -> None:
        """Cache the latest simulator-side arm and contact diagnostic snapshot.

        Args:
            message: JSON diagnostic snapshot published by the MuJoCo simulator.

        Returns:
            None.
        """
        try:
            snapshot = json.loads(message.data)
            if not isinstance(snapshot, dict):
                return
        except (TypeError, json.JSONDecodeError):
            return
        snapshot["received_at_unix_sec"] = time.time()
        with self._mujoco_diagnostics_lock:
            self._latest_mujoco_diagnostics = snapshot
            try:
                joints = snapshot["joints"]
                if self._diagnostic_endpoint and all(
                    abs(float(joints[name]["target_rad"]) - target) <= 1e-4
                    for name, target in self._diagnostic_endpoint.items()
                ):
                    self._latest_endpoint_diagnostics = snapshot
            except (KeyError, TypeError, ValueError):
                pass

    def _current_mujoco_diagnostics(self, endpoint_tracking: bool = False) -> dict | None:
        """Return a simulator diagnostic snapshot with its wall age.

        Args:
            endpoint_tracking: Select the last snapshot whose controls matched the trajectory endpoint.

        Returns:
            A copied MuJoCo diagnostic snapshot, or None when the requested snapshot is unavailable.
        """
        with self._mujoco_diagnostics_lock:
            selected = (self._latest_endpoint_diagnostics if endpoint_tracking
                        else self._latest_mujoco_diagnostics)
            snapshot = dict(selected) if selected is not None else None
        if snapshot is not None:
            snapshot["age_sec"] = max(0.0, time.time() - snapshot["received_at_unix_sec"])
        return snapshot

    def _record_trajectory(self, trajectory: JointTrajectory, stage: str,
                           fraction: float | None = None) -> None:
        """Save a planned joint trajectory and its stage before execution.

        Args:
            trajectory: Planned joint positions, derivatives and arrival times.
            stage: Grasp stage that generated the trajectory.
            fraction: Completed fraction for a partial Cartesian path, if applicable.

        Returns:
            None.
        """
        self.trajectory_directory.mkdir(parents=True, exist_ok=True)
        record = {
            "stage": stage, "fraction": fraction, "recorded_at_unix_sec": time.time(),
            "frame_id": self.base_frame, "tip_frame": self.tip_frame,
            "trajectory_header": {"frame_id": trajectory.header.frame_id,
                                  "sec": trajectory.header.stamp.sec,
                                  "nanosec": trajectory.header.stamp.nanosec},
            "joint_names": list(trajectory.joint_names),
            "points": [{"positions": list(point.positions), "velocities": list(point.velocities),
                        "accelerations": list(point.accelerations), "effort": list(point.effort),
                        "time_from_start_sec": point.time_from_start.sec + point.time_from_start.nanosec * 1e-9}
                       for point in trajectory.points],
        }
        path = self.trajectory_directory / f"{time.time_ns()}-{stage}-{uuid.uuid4().hex[:8]}.json"
        path.write_text(json.dumps(record, ensure_ascii=False, indent=2), encoding="utf-8")
        self.get_logger().info(f"Recorded {stage} trajectory at {path}")
        self._log_run_event("trajectory_recorded", stage=stage, path=str(path),
                            **self._trajectory_profile(trajectory))

    def _log_run_event(self, event: str, **details) -> None:
        """Append one structured event to the active grasp run log.

        Args:
            event: Event name describing a grasp pipeline transition or operation.
            details: JSON-serializable event fields.

        Returns:
            None.
        """
        path = self._run_log_path
        if path is None:
            return
        record = {"recorded_at_unix_sec": time.time(), "action_goal_id": path.stem,
                  "event": str(event), **details}
        with self._run_log_lock:
            path.parent.mkdir(parents=True, exist_ok=True)
            with path.open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(record, ensure_ascii=False, default=str) + "\n")
                stream.flush()

    def _publish_trajectory(self, trajectory: JointTrajectory) -> None:
        """Publish and retain the latest planned joint trajectory.

        Args:
            trajectory: Planned joint trajectory to preview.

        Returns:
            None.
        """
        self._last_planned_trajectory = trajectory
        self.trajectory_publisher.publish(trajectory)

    def _republish_trajectory(self) -> None:
        """Republish the latest preview for clients using volatile subscriptions.

        Args:
            None.

        Returns:
            None.
        """
        if self._last_planned_trajectory is not None:
            self.trajectory_publisher.publish(self._last_planned_trajectory)

    def _ensure_tip_pose(self, target: Pose, stage: str, arm_lease: str,
                         goal_handle, lease_lost: threading.Event) -> None:
        """Verify the measured tool pose and apply collision-checked corrections if needed.

        Args:
            target: Desired grasp-frame pose in the base frame.
            stage: Motion stage name used in diagnostics.
            arm_lease: Active arm control lease identifier.
            goal_handle: Current grasp action goal handle.
            lease_lost: Event set when lease renewal fails.

        Returns:
            None.
        """
        correction_limit = max(0, int(self.config.get("grasp_pose_correction_attempts", 2)))
        position_tolerance = float(self.config.get(
            "grasp_pose_tracking_tolerance_m", self.config["grasp_position_tolerance_m"]))
        orientation_tolerance = float(self.config["grasp_orientation_tolerance_rad"])
        for correction_index in range(correction_limit + 1):
            self._cancel_check(goal_handle, lease_lost)
            actual_transform = self.tf_buffer.lookup_transform(
                self.base_frame, self.tip_frame, Time(), timeout=Duration(seconds=0.75))
            actual_position = actual_transform.transform.translation
            position_error = math.sqrt(sum((actual - desired) ** 2 for actual, desired in zip(
                (actual_position.x, actual_position.y, actual_position.z),
                (target.position.x, target.position.y, target.position.z))))
            actual_orientation = actual_transform.transform.rotation
            orientation_dot = abs(sum(actual * desired for actual, desired in zip(
                (actual_orientation.x, actual_orientation.y, actual_orientation.z, actual_orientation.w),
                (target.orientation.x, target.orientation.y,
                 target.orientation.z, target.orientation.w))))
            orientation_error = 2.0 * math.acos(min(1.0, orientation_dot))
            if (position_error <= position_tolerance and
                    orientation_error <= orientation_tolerance):
                self.get_logger().info(
                    f"{stage} grasp-frame tracking error position={position_error:.4f} m "
                    f"orientation={orientation_error:.4f} rad")
                return
            if correction_index >= correction_limit:
                raise RuntimeError(
                    f"{stage} grasp-frame tracking error exceeds tolerance after correction: "
                    f"position={position_error:.4f} m/{position_tolerance:.4f} m, "
                    f"orientation={orientation_error:.4f} rad/{orientation_tolerance:.4f} rad")
            self.get_logger().warning(
                f"{stage} grasp-frame tracking error position={position_error:.4f} m "
                f"orientation={orientation_error:.4f} rad; planning correction "
                f"{correction_index + 1}/{correction_limit}")
            _, correction = self._plan_cartesian(target, goal_handle, lease_lost)
            self._execute_joint_trajectory(correction, arm_lease, goal_handle, lease_lost)

    def _move_gripper(self, command: str, gripper_lease: str, object_id: str,
                      goal_handle, lease_lost: threading.Event):
        """Run one gripper open/close operation through its Nexus Action.

        Args:
            command: Gripper command, either open or close.
            gripper_lease: Active gripper control lease identifier.
            object_id: Expected MuJoCo object contact ID for close.
            goal_handle: Current PlanGrasp goal handle.
            lease_lost: Event set when lease renewal fails.

        Returns:
            MoveGripper action result.
        """
        action_goal = MoveGripper.Goal()
        action_goal.lease_id = gripper_lease
        action_goal.client_operation_id = uuid.uuid4().hex
        action_goal.command = str(command)
        action_goal.angle_degrees = 0.0
        action_goal.expected_contact_object_id = str(object_id) if command == "close" else ""
        return self._run_action(
            self.gripper_action_client, action_goal, float(self.config["gripper_timeout_sec"]) + 5.0,
            lambda: self._cancel_check(goal_handle, lease_lost),
            lambda message: self._publish_phase(
                goal_handle, command, f"gripper state: {message.feedback.state}", 0.0),
        )

    def _attach_target(self, target_id: str, estimate: dict, goal_handle,
                       lease_lost: threading.Event) -> None:
        """Attach the contacted target to the grasp frame for MoveIt lift planning.

        Args:
            target_id: Target collision-object ID.
            estimate: Vision-based target center and yaw.
            goal_handle: Current PlanGrasp goal handle.
            lease_lost: Event set when lease renewal fails.

        Returns:
            None.
        """
        attached = AttachedCollisionObject()
        attached.link_name = self.tip_frame
        attached.touch_links = [self.tip_frame, "ms42dc_base_link",
                                "ms42dc_left_finger_link", "ms42dc_right_finger_link"]
        attached.object = self._target_collision(target_id, estimate)
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        # MoveIt removes a matching world object as part of processing an
        # ADD AttachedCollisionObject. Sending a separate world REMOVE first
        # is not idempotent: the object may already have disappeared from the
        # world, and MoveIt then rejects the whole planning-scene update.
        scene.robot_state.attached_collision_objects = [attached]
        self._apply_planning_scene(scene, goal_handle, lease_lost)

    def _clear_target_scene(self, target_id: str) -> None:
        """Remove a prior target object and attachment from MoveIt's planning scene.

        Args:
            target_id: Target collision-object ID to remove.

        Returns:
            None.
        """
        current_request = GetPlanningScene.Request()
        current_request.components.components = (
            PlanningSceneComponents.WORLD_OBJECT_NAMES |
            PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS)
        try:
            current = self._call_service(self.get_planning_scene_client, current_request, timeout=2.0)
        except Exception as exc:
            self.get_logger().warning(f"Could not inspect target planning-scene object for cleanup: {exc}")
            return

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        world_ids = {item.id for item in current.scene.world.collision_objects}
        attached_ids = {
            item.object.id for item in current.scene.robot_state.attached_collision_objects
        }
        if target_id in world_ids:
            remove_world = CollisionObject()
            remove_world.id = target_id
            remove_world.header.frame_id = self.base_frame
            remove_world.operation = CollisionObject.REMOVE
            scene.world.collision_objects = [remove_world]
        if target_id in attached_ids:
            detach = AttachedCollisionObject()
            detach.object.id = target_id
            detach.object.operation = CollisionObject.REMOVE
            scene.robot_state.attached_collision_objects = [detach]
        if not scene.world.collision_objects and not scene.robot_state.attached_collision_objects:
            return
        request = ApplyPlanningScene.Request()
        request.scene = scene
        try:
            self._call_service(self.apply_scene_client, request, timeout=2.0)
        except Exception as exc:
            self.get_logger().warning(f"Could not clear target planning-scene object: {exc}")

    def _execute_grasp(self, goal_handle):
        """Run observation, detection, localization, planning, grasp, and lift stages.

        Args:
            goal_handle: Accepted PlanGrasp action goal.

        Returns:
            Action result with operation status and estimated object/grasp poses.
        """
        request = goal_handle.request
        result = PlanGrasp.Result()
        lease_lost = threading.Event()
        renew_stop = threading.Event()
        renew_error: dict[str, str] = {}
        started = time.monotonic()
        action_goal_id = bytes(goal_handle.goal_id.uuid).hex()
        self._run_log_path = self.run_log_directory / f"{action_goal_id}.jsonl"
        self._last_logged_phase = None
        with self._mujoco_diagnostics_lock:
            self._latest_mujoco_diagnostics = None
            self._latest_endpoint_diagnostics = None
            self._diagnostic_endpoint = {}
        arm_lease = ""
        gripper_lease = ""
        target_id = str(request.object_id)
        scene_installed = False
        self.get_logger().info(
            f"Starting grasp action_goal_id={action_goal_id} target_class={request.target_class} "
            f"object_id={target_id} camera_id={request.camera_id} arm_id={request.arm_id} "
            f"gripper_id={request.gripper_id} run_log={self._run_log_path}")
        self._log_run_event(
            "task_started", target_class=str(request.target_class), object_id=target_id,
            camera_id=str(request.camera_id), arm_id=str(request.arm_id),
            gripper_id=str(request.gripper_id))
        try:
            # Clear the previous task's latched preview before planning a new one.
            self._publish_trajectory(JointTrajectory())
            check = lambda: self._cancel_check(goal_handle, lease_lost)
            self._publish_phase(goal_handle, "observe", "checking MuJoCo device instances")
            arm_state, gripper_state = self._read_device_states(check)
            if arm_state.get("backend") != "ros2_sim" or gripper_state.backend != "ros2_sim":
                raise NotImplementedError("planned grasp currently supports only MuJoCo ros2_sim devices")
            owner = f"visual_grasp:{uuid.uuid4().hex}"
            arm_lease, gripper_lease = self._acquire_leases(owner, check)
            self._active_arm_lease = arm_lease
            self._active_gripper_lease = gripper_lease
            renew_thread = threading.Thread(
                target=self._renew_leases,
                args=(renew_stop, lease_lost, renew_error, (arm_lease, gripper_lease)),
                name="grasp-lease-renewal", daemon=True)
            renew_thread.start()
            self._set_motion_enabled(arm_lease, gripper_lease, True, check)
            self._stop_devices("move to visual grasp observation pose")
            stationary_deadline = time.monotonic() + 4.0
            while time.monotonic() < stationary_deadline:
                check()
                arm_state, _ = self._read_device_states(check)
                if arm_state.get("feedback_valid") and arm_state.get("stationary_confirmed") is True:
                    break
                time.sleep(0.08)
            else:
                raise TimeoutError("arm did not confirm a stationary state before observation planning")
            self._set_motion_enabled(arm_lease, gripper_lease, True, check)
            observation_targets = [float(value) for value in self.config["observation_joints_rad"]]
            joint_names = [str(name) for name in self.config["observation_joint_names"]]
            measured = dict(zip(arm_state.get("joint_names", []), arm_state.get("positions_rad", [])))
            if any(name not in measured for name in joint_names):
                raise RuntimeError("arm feedback is missing a configured observation joint")
            observation_error = max(abs(measured[name] - target)
                                    for name, target in zip(joint_names, observation_targets))
            tolerance = float(self.config.get("observation_joint_tolerance_rad", 0.04))
            self.get_logger().info(
                f"Camera observation pose maximum joint error={observation_error:.4f} rad "
                f"(tolerance={tolerance:.4f} rad)")
            if observation_error <= tolerance:
                self._publish_phase(goal_handle, "observe", "arm is already at the configured camera observation pose")
            else:
                self._publish_phase(goal_handle, "observe", "moving to the configured camera observation pose")
                observation = self._plan_joint_positions(observation_targets, goal_handle, lease_lost)
                self._execute_joint_trajectory(observation, arm_lease, goal_handle, lease_lost, "observe")
            color_msg, depth_msg, color_info, depth_info = self._wait_for_stopped_frame(
                goal_handle, lease_lost)
            estimate, confidence, object_pose = self._estimate_target(
                color_msg, depth_msg, color_info, depth_info, goal_handle, lease_lost)
            target_diagnostics = {
                "confidence": float(confidence),
                "center_base_m": [float(value) for value in estimate["center"]],
                "yaw_rad": float(estimate["yaw"]),
                "point_count": int(estimate["point_count"]),
                "size_m": estimate["size"],
                "measurement": estimate["measurement"],
                "rgb_stamp_ns": int(color_msg.header.stamp.sec) * 1_000_000_000
                + int(color_msg.header.stamp.nanosec),
                "depth_stamp_ns": int(depth_msg.header.stamp.sec) * 1_000_000_000
                + int(depth_msg.header.stamp.nanosec),
            }
            self.get_logger().info(f"Estimated target pose diagnostics={target_diagnostics}")
            self._log_run_event("target_estimated", **target_diagnostics)
            result.object_pose = object_pose
            box_size = [float(value) for value in estimate["size"]]
            target_yaw = grasp_closing_yaw(
                estimate["yaw"], box_size,
                float(self.config.get("symmetric_box_grasp_yaw_rad", 0.0)))
            grasp_pose = Pose()
            qx, qy, qz, qw = top_down_quaternion(target_yaw)
            finger_offset = [float(value) for value in
                             self.config.get("finger_contact_center_offset_grasp_m", [0.0, 0.0, -0.043])]
            if len(finger_offset) != 3:
                raise ValueError("finger_contact_center_offset_grasp_m must contain three values")
            contact_offset_base = rotate_vector(finger_offset, [qx, qy, qz, qw])
            grasp_pose.position.x = float(estimate["center"][0]) - float(contact_offset_base[0])
            grasp_pose.position.y = float(estimate["center"][1]) - float(contact_offset_base[1])
            grasp_pose.position.z = float(estimate["center"][2]) - float(contact_offset_base[2])
            grasp_pose.orientation.x = qx
            grasp_pose.orientation.y = qy
            grasp_pose.orientation.z = qz
            grasp_pose.orientation.w = qw
            result.grasp_pose.header.frame_id = self.base_frame
            result.grasp_pose.header.stamp = depth_msg.header.stamp
            result.grasp_pose.pose = grasp_pose
            self._install_collision_scene(target_id, estimate, goal_handle, lease_lost)
            scene_installed = True
            pregrasp = Pose()
            pregrasp.position.x = grasp_pose.position.x
            pregrasp.position.y = grasp_pose.position.y
            pregrasp.position.z = grasp_pose.position.z + float(self.config["pregrasp_distance_m"])
            pregrasp.orientation = grasp_pose.orientation
            grasp_geometry = {
                "target_size_m": box_size,
                "estimated_center_base_m": [float(value) for value in estimate["center"]],
                "grasp_position_base_m": [grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z],
                "pregrasp_position_base_m": [pregrasp.position.x, pregrasp.position.y, pregrasp.position.z],
                "finger_contact_center_offset_grasp_m": finger_offset,
            }
            self.get_logger().info(f"Grasp geometry diagnostics={grasp_geometry}")
            self._log_run_event("grasp_geometry", **grasp_geometry)
            self._publish_phase(goal_handle, "plan", "planning current pose to pregrasp with MoveIt2", confidence)
            planned = self._plan_pose(pregrasp, goal_handle, lease_lost)
            self._execute_joint_trajectory(planned, arm_lease, goal_handle, lease_lost, "pregrasp")
            self._ensure_tip_pose(pregrasp, "pregrasp", arm_lease, goal_handle, lease_lost)
            self._cancel_check(goal_handle, lease_lost)
            self._publish_phase(goal_handle, "open", "opening gripper at pregrasp", confidence)
            opened = self._move_gripper("open", gripper_lease, "", goal_handle, lease_lost)
            if not opened.success:
                raise RuntimeError(f"gripper open failed: {opened.error}")
            self._set_allowed_finger_contact(target_id, True, goal_handle, lease_lost)
            self._publish_phase(goal_handle, "approach", "planning collision-checked Cartesian approach", confidence)
            fraction, approach = self._plan_cartesian(grasp_pose, goal_handle, lease_lost,
                                                       minimum_fraction=1.0)
            self.get_logger().info(f"Cartesian approach fraction={fraction:.3f}")
            self._execute_joint_trajectory(approach, arm_lease, goal_handle, lease_lost, "approach")
            self._ensure_tip_pose(grasp_pose, "grasp", arm_lease, goal_handle, lease_lost)
            self._set_allowed_finger_contact(target_id, False, goal_handle, lease_lost)
            self._publish_phase(goal_handle, "close", "closing until MuJoCo reports target contact", confidence)
            closed = self._move_gripper("close", gripper_lease, target_id, goal_handle, lease_lost)
            if not closed.success or not closed.contact_confirmed or closed.contacted_object_id != target_id:
                raise RuntimeError(
                    "gripper did not confirm contact with the requested target object "
                    f"(error={closed.error!r}, contact_confirmed={closed.contact_confirmed}, "
                    f"contacted_object_id={closed.contacted_object_id!r}, "
                    f"expected_object_id={target_id!r})")
            self._attach_target(target_id, estimate, goal_handle, lease_lost)
            lift_pose = Pose()
            lift_pose.position.x = grasp_pose.position.x
            lift_pose.position.y = grasp_pose.position.y
            lift_pose.position.z = grasp_pose.position.z + float(self.config["lift_distance_m"])
            lift_pose.orientation = grasp_pose.orientation
            self._publish_phase(goal_handle, "lift", "planning and executing a 12 cm Cartesian lift", confidence)
            _, lift = self._plan_cartesian(lift_pose, goal_handle, lease_lost)
            self._execute_joint_trajectory(lift, arm_lease, goal_handle, lease_lost, "lift")
            self._cancel_check(goal_handle, lease_lost)
            _, lifted_gripper = self._read_device_states(check)
            contact_ids = list(lifted_gripper.feedback_contact_object_ids)
            if not lifted_gripper.feedback_contact_valid or target_id not in contact_ids:
                raise RuntimeError("target contact was not maintained through the commanded lift")
            result.success = True
            result.grasp_verified = False
            result.message = (
                "grasp sequence and lift command completed with target contact maintained; "
                "run the independent MuJoCo ground-truth acceptance check to verify object lift")
            self._publish_phase(goal_handle, "complete", result.message, confidence)
            self._log_run_event("task_sequence_completed", success=True,
                                grasp_verified=False, message=result.message)
            goal_handle.succeed()
        except GraspCancelled as exc:
            result.success = False
            result.grasp_verified = False
            result.message = str(exc)
            self._stop_devices("visual grasp cancelled")
            self._log_run_event("task_cancelled", message=str(exc))
            goal_handle.canceled()
        except Exception as exc:
            result.success = False
            result.grasp_verified = False
            result.message = str(exc)
            self.get_logger().error(f"Visual grasp failed: {exc}")
            self._log_run_event("task_failed", error_type=type(exc).__name__, message=str(exc))
            self._stop_devices(f"visual grasp failed: {exc}")
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
            else:
                goal_handle.abort()
        finally:
            if arm_lease or gripper_lease:
                self._stop_devices("visual grasp operation finished")
                try:
                    self._set_motion_enabled(arm_lease, gripper_lease, False)
                except Exception as exc:
                    self.get_logger().warning(f"Could not disable grasp device motion: {exc}")
                renew_stop.set()
                if "renew_thread" in locals():
                    renew_thread.join(timeout=1.0)
                self._release_leases(arm_lease, gripper_lease)
            if scene_installed:
                self._clear_target_scene(target_id)
            with self._active_lock:
                self._active = False
                self._active_cancel = None
                self._active_arm_lease = ""
                self._active_gripper_lease = ""
            elapsed = time.monotonic() - started
            self.get_logger().info(
                f"Finished grasp action_goal_id={action_goal_id} success={result.success} "
                f"grasp_verified={result.grasp_verified} elapsed_wall_sec={elapsed:.3f} "
                f"message={result.message}")
            self._log_run_event(
                "task_finished", success=bool(result.success),
                grasp_verified=bool(result.grasp_verified), elapsed_wall_sec=elapsed,
                message=str(result.message))
        return result

    def destroy_node(self):
        """Stop the HTTP trigger before destroying ROS resources.

        Args:
            None.

        Returns:
            None.
        """
        try:
            self.http_server.close()
        finally:
            return super().destroy_node()


def main(args=None) -> None:
    """Initialize ROS, run the operator, and close resources on shutdown.

    Args:
        args: Optional ROS command-line arguments.

    Returns:
        None.
    """
    rclpy.init(args=args)
    node = GraspOperator()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
