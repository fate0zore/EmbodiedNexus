"""Independent lease-protected visual servo grasp action server."""

import copy
import json
import math
import threading
import time
import uuid
from pathlib import Path

import numpy as np
import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from embodied_nexus_arm_interfaces.action import ExecuteArmTrajectory
from embodied_nexus_arm_interfaces.srv import (AcquireArmControl, GetArmState, ReleaseArmControl,
                                               RenewArmControl, SetArmJointVelocity, SetArmMotionEnabled, StopArm)
from embodied_nexus_gripper_interfaces.action import MoveGripper
from embodied_nexus_gripper_interfaces.srv import (AcquireGripperControl, GetGripperState,
    ReleaseGripperControl, RenewGripperControl, SetGripperMotionEnabled, StopGripper)
from embodied_nexus_visual_servo_interfaces.action import VisualServoGrasp
from embodied_nexus_visual_servo_interfaces.msg import VisualTarget
from geometry_msgs.msg import Pose, TwistStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (AllowedCollisionEntry, AttachedCollisionObject, CollisionObject, Constraints,
                             JointConstraint, MoveItErrorCodes, OrientationConstraint, PositionConstraint,
                             PlanningScene, PlanningSceneComponents)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetPlanningScene, GetPositionIK, GetStateValidity
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
from std_msgs.msg import Int8
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory

from .control import (JointVelocityTracker, angular_velocity, cartesian_velocity,
                      continuous_grasp_orientation, map_servo_velocities, orientation_error)


class Cancelled(RuntimeError):
    """Signal an explicitly cancelled grasp task."""


class VisualServoGraspNode(Node):
    """Coordinate vision, MoveIt Servo and Nexus device interfaces."""

    def __init__(self):
        """Initialize the independent grasp action and its ROS clients.

        Returns:
            None.
        """
        super().__init__("visual_servo_grasp")
        self.declare_parameter("config_file", "")
        self.declare_parameter("target_class", "green cylinder")
        path = self.get_parameter("config_file").value or str(
            Path(get_package_share_directory("embodied_nexus_visual_servo")) / "config/visual_servo.yaml")
        self.config = yaml.safe_load(Path(path).expanduser().read_text(encoding="utf-8"))["visual_servo"]
        self.target_class = str(self.get_parameter("target_class").value)
        if self.target_class not in self.config["targets"]:
            raise ValueError("target class is not configured")
        self.group = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        self.active = False
        self.target = None
        self.joint_command = None
        self.servo_status = None
        self.joint_feedback = None
        self.lost = threading.Event()
        self.saved_collision_matrix = None
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self, spin_thread=False)
        self.create_subscription(VisualTarget, "/embodied_nexus/visual_servo/target", self._target, 10,
                                 callback_group=self.group)
        self.create_subscription(JointTrajectory, "/embodied_nexus/visual_servo/joint_command",
                                 self._joint_command, 10, callback_group=self.group)
        self.create_subscription(Int8, "/embodied_nexus/visual_servo/servo_status", self._servo_status, 10,
                                 callback_group=self.group)
        self.create_subscription(JointState, f"/embodied_nexus/arms/{self.config['arm_id']}/joint_states",
                                 self._joint_feedback, 10, callback_group=self.group)
        self.twists = self.create_publisher(TwistStamped, "/embodied_nexus/visual_servo/twist", 10)
        self.scene_pub = self.create_publisher(PlanningScene, "/planning_scene", 10)
        arm = f"/embodied_nexus/arms/{self.config['arm_id']}"
        grip = f"/embodied_nexus/grippers/{self.config['gripper_id']}"
        types = {
            "arm_state": (GetArmState, arm + "/get_state"), "grip_state": (GetGripperState, grip + "/get_state"),
            "arm_acquire": (AcquireArmControl, arm + "/control/acquire"),
            "grip_acquire": (AcquireGripperControl, grip + "/control/acquire"),
            "arm_renew": (RenewArmControl, arm + "/control/renew"),
            "grip_renew": (RenewGripperControl, grip + "/control/renew"),
            "arm_release": (ReleaseArmControl, arm + "/control/release"),
            "grip_release": (ReleaseGripperControl, grip + "/control/release"),
            "arm_enable": (SetArmMotionEnabled, arm + "/motion/enable"),
            "grip_enable": (SetGripperMotionEnabled, grip + "/motion/enable"),
            "arm_stop": (StopArm, arm + "/stop"), "grip_stop": (StopGripper, grip + "/stop"),
            "velocity": (SetArmJointVelocity, arm + "/joint_velocity"),
            "apply": (ApplyPlanningScene, "/apply_planning_scene"),
            "scene": (GetPlanningScene, "/get_planning_scene"),
            "ik": (GetPositionIK, "/compute_ik"), "cartesian": (GetCartesianPath, "/compute_cartesian_path"),
            "state_validity": (GetStateValidity, "/check_state_validity"),
            "servo_start": (Trigger, "/visual_servo_moveit/start_servo"),
            "servo_stop": (Trigger, "/visual_servo_moveit/stop_servo"),
            "reset_tracker": (Trigger, "/embodied_nexus/visual_servo/reset_tracker"),
        }
        self.service_clients = {key: self.create_client(typ, name, callback_group=self.group)
                        for key, (typ, name) in types.items()}
        self.move_group = ActionClient(self, MoveGroup, "/move_action", callback_group=self.group)
        self.arm_action = ActionClient(self, ExecuteArmTrajectory, arm + "/execute_trajectory", callback_group=self.group)
        self.grip_action = ActionClient(self, MoveGripper, grip + "/move", callback_group=self.group)
        self.action = ActionServer(self, VisualServoGrasp, "/embodied_nexus/visual_servo/grasp",
                                   execute_callback=self._execute, goal_callback=self._goal,
                                   cancel_callback=self._cancel, callback_group=self.group)

    def _target(self, message):
        """Cache a vision target and local receive time.

        Args:
            message: Vision-only target estimate.

        Returns:
            None.
        """
        with self.lock:
            self.target = (message, time.monotonic())

    def _joint_command(self, message):
        """Cache a MoveIt Servo joint command and its receive time.

        Args:
            message: Servo joint trajectory output.

        Returns:
            None.
        """
        with self.lock:
            self.joint_command = (message, time.monotonic())

    def _servo_status(self, message):
        """Cache the MoveIt Servo status code.

        Args:
            message: MoveIt Servo status message.

        Returns:
            None.
        """
        with self.lock:
            self.servo_status = (int(message.data), time.monotonic())

    def _joint_feedback(self, message):
        """Cache public arm joint measurements and their receive time.

        Args:
            message: Arm joint state feedback.

        Returns:
            None.
        """
        with self.lock:
            self.joint_feedback = (message, time.monotonic())

    def _positions(self):
        """Read recent measured positions in configured joint order.

        Returns:
            List of measured joint positions.
        """
        with self.lock:
            sample = self.joint_feedback
        if not sample or time.monotonic() - sample[1] > 0.5:
            raise RuntimeError("arm joint feedback is stale")
        positions = dict(zip(sample[0].name, sample[0].position))
        if any(name not in positions for name in self.config["joint_names"]):
            raise RuntimeError("arm joint feedback is incomplete")
        return [float(positions[name]) for name in self.config["joint_names"]]

    def _check_velocity_state(self, positions, velocities, goal):
        """Check a short velocity projection against the MoveIt planning scene.

        Args:
            positions: Current measured joint positions.
            velocities: Proposed joint velocities.
            goal: Parent action goal handle.

        Returns:
            None when the projected joint state is collision-free.
        """
        request = GetStateValidity.Request()
        request.group_name = "manipulator"
        request.robot_state.is_diff = True
        request.robot_state.joint_state.name = self.config["joint_names"]
        request.robot_state.joint_state.position = [float(q + v * 0.2) for q, v in zip(positions, velocities)]
        response = self._call("state_validity", request, goal, 2.0)
        if not response.valid:
            pairs = [(item.contact_body_1, item.contact_body_2, float(item.depth)) for item in response.contacts]
            raise RuntimeError(f"predicted joint velocity state is in collision: {pairs}")

    def _goal(self, request):
        """Accept one configured MuJoCo grasp at a time.

        Args:
            request: Proposed action goal.

        Returns:
            ROS goal acceptance decision.
        """
        with self.lock:
            if (self.active or request.target_class != self.target_class or not request.object_id or
                request.camera_id != self.config["camera_id"] or request.arm_id != self.config["arm_id"] or
                request.gripper_id != self.config["gripper_id"]):
                return GoalResponse.REJECT
            self.active = True
        return GoalResponse.ACCEPT

    def _cancel(self, handle):
        """Accept cancellation of a running action.

        Args:
            handle: Action goal handle.

        Returns:
            ROS cancel acceptance decision.
        """
        return CancelResponse.ACCEPT

    def _check(self, goal):
        """Reject further work after cancellation or lease renewal failure.

        Args:
            goal: Current action goal handle.

        Returns:
            None.
        """
        if goal.is_cancel_requested:
            raise Cancelled("grasp cancelled")
        if self.lost.is_set():
            raise RuntimeError("device lease renewal failed")

    def _wait(self, future, timeout, goal=None):
        """Wait for a ROS future with a wall-clock deadline.

        Args:
            future: ROS future.
            timeout: Deadline in seconds.
            goal: Optional current goal for cancellation checks.

        Returns:
            The future result.
        """
        deadline = time.monotonic() + timeout
        while not future.done():
            if goal is not None:
                self._check(goal)
            if time.monotonic() > deadline:
                raise TimeoutError("ROS operation timed out")
            time.sleep(0.02)
        value = future.result()
        if value is None:
            raise RuntimeError("ROS operation returned no result")
        return value

    def _call(self, key, request, goal=None, timeout=5.0):
        """Call a public ROS service and validate its response.

        Args:
            key: Service client key.
            request: Typed request.
            goal: Optional action goal handle.
            timeout: Wall-clock deadline in seconds.

        Returns:
            Typed response.
        """
        client = self.service_clients[key]
        if not client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError(f"service unavailable: {key}")
        response = self._wait(client.call_async(request), timeout, goal)
        if hasattr(response, "success") and not response.success:
            raise RuntimeError(f"{key} failed: {getattr(response, 'error', getattr(response, 'message', ''))}")
        return response

    def _run_action(self, client, request, goal, timeout):
        """Execute a child action, cancelling it on failure or cancellation.

        Args:
            client: Child action client.
            request: Child action goal.
            goal: Parent goal handle.
            timeout: Wall-clock timeout in seconds.

        Returns:
            Child action result.
        """
        if not client.wait_for_server(timeout_sec=3.0):
            raise RuntimeError("required action server is unavailable")
        child = self._wait(client.send_goal_async(request), 10.0, goal)
        if not child.accepted:
            raise RuntimeError("child action rejected")
        try:
            result = self._wait(child.get_result_async(), timeout, goal)
        except Exception:
            child.cancel_goal_async()
            raise
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f"child action failed: {result.result}")
        return result.result

    def _feedback(self, goal, phase, detail, confidence=0.0):
        """Publish and log a task stage.

        Args:
            goal: Current action goal handle.
            phase: Stage name.
            detail: Human-readable stage detail.
            confidence: Latest detection confidence.

        Returns:
            None.
        """
        message = VisualServoGrasp.Feedback()
        message.phase, message.detail, message.detection_confidence = phase, detail, float(confidence)
        goal.publish_feedback(message)
        self.get_logger().info(f"{phase}: {detail}")

    def _renew(self, leases, stop):
        """Renew both device leases until the operation finishes.

        Args:
            leases: Pair of arm and gripper lease IDs.
            stop: Event ending the worker.

        Returns:
            None.
        """
        while not stop.wait(0.5):
            try:
                for key, typ, lease in (("arm_renew", RenewArmControl, leases[0]),
                                        ("grip_renew", RenewGripperControl, leases[1])):
                    request = typ.Request()
                    request.lease_id = lease
                    self._call(key, request, timeout=3.0)
            except Exception as exc:
                self.get_logger().error(f"lease renewal failed: {exc}")
                self.lost.set()
                return

    def _plan_joints(self, positions, goal):
        """Plan to a set of arm joint positions through MoveGroup.

        Args:
            positions: Joint positions in the configured arm order.
            goal: Parent action goal handle.

        Returns:
            MoveIt robot trajectory.
        """
        request = MoveGroup.Goal()
        request.request.group_name = "manipulator"
        request.request.start_state.is_diff = True
        request.request.num_planning_attempts = 10
        request.request.allowed_planning_time = 20.0
        request.request.max_velocity_scaling_factor = 0.25
        request.request.max_acceleration_scaling_factor = 0.25
        constraint = Constraints()
        for name, position in zip(self.config["joint_names"], positions):
            item = JointConstraint()
            item.joint_name, item.position = name, float(position)
            item.tolerance_above = item.tolerance_below = 0.015
            item.weight = 1.0
            constraint.joint_constraints.append(item)
        request.request.goal_constraints = [constraint]
        request.planning_options.plan_only = True
        result = self._run_action(self.move_group, request, goal, 45.0)
        if result.error_code.val != MoveItErrorCodes.SUCCESS or not result.planned_trajectory.joint_trajectory.points:
            raise RuntimeError(f"MoveIt planning failed: {result.error_code.val}")
        return result.planned_trajectory

    def _plan_pose(self, pose, goal):
        """Solve collision-aware IK and plan to its joint solution.

        Args:
            pose: Desired grasp-frame pose in base coordinates.
            goal: Parent action goal handle.

        Returns:
            Collision-checked MoveIt robot trajectory.
        """
        request = GetPositionIK.Request()
        ik = request.ik_request
        ik.group_name, ik.ik_link_name = "manipulator", self.config["tip_frame"]
        ik.robot_state.is_diff, ik.avoid_collisions = True, True
        ik.pose_stamped.header.frame_id = self.config["base_frame"]
        ik.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        ik.pose_stamped.pose = pose
        ik.timeout = Duration(seconds=2.0).to_msg()
        result = self._call("ik", request, goal, 5.0)
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            joints = dict(zip(result.solution.joint_state.name, result.solution.joint_state.position))
            return self._plan_joints([joints[name] for name in self.config["joint_names"]], goal)
        request = MoveGroup.Goal()
        request.request.group_name = "manipulator"
        request.request.start_state.is_diff = True
        request.request.num_planning_attempts = 10
        request.request.allowed_planning_time = 20.0
        request.request.max_velocity_scaling_factor = request.request.max_acceleration_scaling_factor = 0.25
        position = PositionConstraint()
        position.header.frame_id, position.link_name, position.weight = self.config["base_frame"], self.config["tip_frame"], 1.0
        sphere = SolidPrimitive()
        sphere.type, sphere.dimensions = SolidPrimitive.SPHERE, [0.02]
        region = Pose()
        region.position, region.orientation.w = pose.position, 1.0
        position.constraint_region.primitives = [sphere]
        position.constraint_region.primitive_poses = [region]
        orientation = OrientationConstraint()
        orientation.header.frame_id, orientation.link_name = self.config["base_frame"], self.config["tip_frame"]
        orientation.orientation = pose.orientation
        orientation.absolute_x_axis_tolerance = orientation.absolute_y_axis_tolerance = 0.15
        orientation.absolute_z_axis_tolerance, orientation.weight = 0.2, 1.0
        constraint = Constraints()
        constraint.position_constraints, constraint.orientation_constraints = [position], [orientation]
        request.request.goal_constraints = [constraint]
        request.planning_options.plan_only = True
        planned = self._run_action(self.move_group, request, goal, 45.0)
        if planned.error_code.val != MoveItErrorCodes.SUCCESS or not planned.planned_trajectory.joint_trajectory.points:
            raise RuntimeError(f"MoveIt pose planning failed: {planned.error_code.val}")
        return planned.planned_trajectory

    def _execute_trajectory(self, trajectory, lease, goal):
        """Execute a planned trajectory through the leased Nexus arm.

        Args:
            trajectory: MoveIt robot trajectory.
            lease: Arm control lease ID.
            goal: Parent action goal handle.

        Returns:
            None.
        """
        request = ExecuteArmTrajectory.Goal()
        request.lease_id = lease
        request.trajectory = trajectory.joint_trajectory
        request.speed_rad_sec, request.acceleration_rad_sec2 = 0.2, 0.3
        end = request.trajectory.points[-1].time_from_start
        request.timeout_sec = max(45.0, (end.sec + end.nanosec / 1e9) * 12 + 10)
        result = self._run_action(self.arm_action, request, goal, request.timeout_sec + 10)
        if not result.success or not result.stationary_confirmed:
            raise RuntimeError(f"arm trajectory failed: {result.message}")

    def _cartesian(self, pose, goal):
        """Plan a full collision-checked Cartesian segment.

        Args:
            pose: Desired grasp-frame pose.
            goal: Parent action goal handle.

        Returns:
            MoveIt robot trajectory.
        """
        request = GetCartesianPath.Request()
        request.header.frame_id = self.config["base_frame"]
        request.header.stamp = self.get_clock().now().to_msg()
        request.start_state.is_diff = True
        request.group_name, request.link_name = "manipulator", self.config["tip_frame"]
        request.max_step, request.jump_threshold, request.avoid_collisions = 0.005, 0.0, True
        request.max_velocity_scaling_factor = request.max_acceleration_scaling_factor = 0.25
        request.waypoints = [pose]
        response = self._call("cartesian", request, goal, 30.0)
        if response.error_code.val != MoveItErrorCodes.SUCCESS or response.fraction < 0.99:
            raise RuntimeError(f"Cartesian path incomplete: {response.fraction:.3f}")
        return response.solution

    @staticmethod
    def _slow_trajectory(trajectory, scale):
        """Slow a planned trajectory while retaining its collision-checked path.

        Args:
            trajectory: MoveIt robot trajectory to retime in place.
            scale: Positive speed factor no greater than one.

        Returns:
            The retimed trajectory.
        """
        if not 0.0 < scale <= 1.0:
            raise ValueError("trajectory speed scale must be in (0, 1]")
        for point in trajectory.joint_trajectory.points:
            nanoseconds = round((point.time_from_start.sec * 1_000_000_000 +
                                 point.time_from_start.nanosec) / scale)
            point.time_from_start.sec, point.time_from_start.nanosec = divmod(nanoseconds, 1_000_000_000)
            point.velocities = [value * scale for value in point.velocities]
            point.accelerations = [value * scale * scale for value in point.accelerations]
        return trajectory

    def _collision(self, identifier, center, dimensions, yaw=0.0, cylinder=False):
        """Create a vision-based target or ground collision primitive.

        Args:
            identifier: Planning-scene object ID.
            center: Object XYZ in the base frame.
            dimensions: Box XYZ dimensions or cylinder length, diameter, height.
            yaw: Horizontal long-axis orientation.
            cylinder: Whether to create a horizontal cylinder.

        Returns:
            A MoveIt collision object.
        """
        obj = CollisionObject()
        obj.id, obj.header.frame_id, obj.operation = identifier, self.config["base_frame"], CollisionObject.ADD
        primitive = SolidPrimitive()
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = [float(x) for x in center]
        if cylinder:
            primitive.type = SolidPrimitive.CYLINDER
            primitive.dimensions = [float(dimensions[0]), float(dimensions[1]) / 2]
            pose.orientation.x = -math.sin(yaw / 2) * math.sin(math.pi / 4)
            pose.orientation.y = math.cos(yaw / 2) * math.sin(math.pi / 4)
            pose.orientation.z = math.sin(yaw / 2) * math.cos(math.pi / 4)
            pose.orientation.w = math.cos(yaw / 2) * math.cos(math.pi / 4)
        else:
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = [float(x) for x in dimensions]
            pose.orientation.z, pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        obj.primitives, obj.primitive_poses = [primitive], [pose]
        return obj

    def _apply_scene(self, scene, goal=None):
        """Apply a scene diff to MoveGroup and publish it for MoveIt Servo.

        Args:
            scene: MoveIt planning scene diff.
            goal: Optional active action goal.

        Returns:
            None.
        """
        request = ApplyPlanningScene.Request()
        request.scene = scene
        self._call("apply", request, goal, 8.0)
        self.scene_pub.publish(scene)

    def _allow_contact(self, target_id, goal):
        """Allow only the two finger links to touch the target collision body.

        Args:
            target_id: Target collision object ID.
            goal: Parent action goal handle.

        Returns:
            None.
        """
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        matrix = self._call("scene", request, goal).scene.allowed_collision_matrix
        self.saved_collision_matrix = copy.deepcopy(matrix)
        names = list(matrix.entry_names)
        if len(matrix.entry_values) != len(names) or any(len(row.enabled) != len(names) for row in matrix.entry_values):
            raise RuntimeError("invalid MoveIt collision matrix")
        for name in (target_id, "ms42dc_left_finger_link", "ms42dc_right_finger_link",
                     "ms42dc_base_link", "aubo_wrist3_Link"):
            if name not in names:
                for row in matrix.entry_values:
                    row.enabled.append(False)
                names.append(name)
                matrix.entry_names.append(name)
                matrix.entry_values.append(AllowedCollisionEntry(enabled=[False] * len(names)))
        target = names.index(target_id)
        for name in ("ms42dc_left_finger_link", "ms42dc_right_finger_link"):
            finger = names.index(name)
            matrix.entry_values[target].enabled[finger] = True
            matrix.entry_values[finger].enabled[target] = True
        wrist, base = names.index("aubo_wrist3_Link"), names.index("ms42dc_base_link")
        # These links are connected through fixed joints; their separation is invariant.
        matrix.entry_values[wrist].enabled[base] = True
        matrix.entry_values[base].enabled[wrist] = True
        scene = PlanningScene()
        scene.is_diff = scene.robot_state.is_diff = True
        scene.allowed_collision_matrix = matrix
        self._apply_scene(scene, goal)

    def _latest_target(self, goal, timeout=3.0):
        """Wait for a recent target estimate from the requested detector.

        Args:
            goal: Parent action goal handle.
            timeout: Maximum wait in seconds.

        Returns:
            Most recent valid vision target message.
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self._check(goal)
            with self.lock:
                data = self.target
            if (data and data[0].target_class == self.target_class and data[0].geometry_ready and
                    time.monotonic() - data[1] <= self.config["target_freshness_sec"]):
                return data[0]
            time.sleep(0.05)
        raise TimeoutError("fresh vision target unavailable")

    def _gripper(self, command, lease, object_id, goal):
        """Execute a gripper command through its leased device action.

        Args:
            command: Open or close.
            lease: Gripper lease ID.
            object_id: Expected target ID for a close operation.
            goal: Parent action goal handle.

        Returns:
            Gripper action result.
        """
        request = MoveGripper.Goal()
        request.lease_id, request.client_operation_id = lease, uuid.uuid4().hex
        request.command = command
        request.expected_contact_object_id = object_id if command == "close" else ""
        result = self._run_action(self.grip_action, request, goal, 25.0)
        if not result.success:
            raise RuntimeError(f"gripper {command} failed: {result.error}")
        return result

    def _velocity(self, lease, values, goal=None):
        """Submit a joint velocity command and verify device acceptance.

        Args:
            lease: Active arm control lease ID.
            values: Joint velocities in configured order.
            goal: Optional parent goal.

        Returns:
            Device velocity response.
        """
        request = SetArmJointVelocity.Request()
        request.lease_id, request.velocity_rad_sec = lease, values
        response = self._call("velocity", request, goal, 2.0)
        if not response.controller_accepted:
            raise RuntimeError("Nexus velocity command not accepted by controller")
        return response

    def _wait_stationary(self, goal, timeout=4.0):
        """Wait for fresh arm feedback to confirm that velocity motion stopped.

        Args:
            goal: Parent action goal handle.
            timeout: Maximum wall-clock wait in seconds.

        Returns:
            None once stationary feedback is confirmed.
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            state = json.loads(self._call("arm_state", GetArmState.Request(), goal).state_json)
            if state.get("feedback_valid") and state.get("stationary_confirmed") is True:
                return
            time.sleep(0.1)
        raise TimeoutError("arm did not confirm stationary feedback")

    def _servo_approach(self, goal, lease, target_id, desired_orientation):
        """Continuously adjust tool position from fresh vision via MoveIt Servo.

        Args:
            goal: Parent action goal handle.
            lease: Active arm lease ID.
            target_id: Collision-object ID for the selected target.
            desired_orientation: Desired tool quaternion.

        Returns:
            Latest target estimate after convergence.
        """
        self._allow_contact(target_id, goal)
        with self.lock:
            self.joint_command = None
            self.servo_status = None
        self._call("servo_start", Trigger.Request(), goal)
        deadline = time.monotonic() + self.config["servo_timeout_sec"]
        startup_deadline = time.monotonic() + 3.0
        tracker = JointVelocityTracker(self._positions(), gain=4.0, limit=0.2)
        previous_time = self.get_clock().now().nanoseconds
        initial_distance = None
        stable_since = None
        scene_update_at = 0.0
        try:
            while time.monotonic() < deadline:
                self._check(goal)
                try:
                    target = self._latest_target(goal, timeout=0.1)
                except TimeoutError:
                    stable_since = None
                    # Stop the velocity stream before waiting for another visual observation.
                    halt = TwistStamped()
                    halt.header.frame_id = self.config["base_frame"]
                    halt.header.stamp = self.get_clock().now().to_msg()
                    self.twists.publish(halt)
                    stopped = self._velocity(lease, [0.0] * len(self.config["joint_names"]), goal)
                    if not stopped.stationary_confirmed:
                        self._wait_stationary(goal)
                    with self.lock:
                        self.target = None
                    self.get_logger().warning("visual target lost; stopped while waiting for reacquisition")
                    self._latest_target(goal, timeout=5.0)
                    tracker = JointVelocityTracker(self._positions(), gain=4.0, limit=0.2)
                    previous_time = self.get_clock().now().nanoseconds
                    continue
                transform = self.tf.lookup_transform(self.config["base_frame"], self.config["tip_frame"],
                                                     Time(), timeout=Duration(seconds=0.4))
                actual = transform.transform.translation
                position = target.pose.pose.position
                target_orientation = target.pose.pose.orientation
                yaw = 2 * math.atan2(target_orientation.z, target_orientation.w)
                previous_orientation = [desired_orientation.x, desired_orientation.y,
                                        desired_orientation.z, desired_orientation.w]
                wanted_quaternion = continuous_grasp_orientation(yaw, previous_orientation)
                (desired_orientation.x, desired_orientation.y,
                 desired_orientation.z, desired_orientation.w) = wanted_quaternion
                desired = [position.x, position.y, position.z + self.config["finger_contact_offset_m"]]
                velocity, reached = cartesian_velocity(desired, [actual.x, actual.y, actual.z],
                    self.config["servo_gain"], self.config["servo_speed_m_sec"],
                    self.config["servo_position_tolerance_m"])
                rotation = transform.transform.rotation
                actual_quaternion = [rotation.x, rotation.y, rotation.z, rotation.w]
                angle_error = orientation_error(wanted_quaternion, actual_quaternion)
                angular = angular_velocity(
                    wanted_quaternion, actual_quaternion, 0.05, 0.01)
                # Near the floor, finish lateral and angular alignment before
                # descending farther. Upward correction remains available.
                lateral_error = math.hypot(desired[0] - actual.x, desired[1] - actual.y)
                if (abs(desired[2] - actual.z) < 0.04 and velocity[2] < 0.0 and
                        (lateral_error > 0.004 or angle_error > math.radians(3.0))):
                    velocity[2] = 0.0
                distance = float(np.linalg.norm(np.asarray(desired) - [actual.x, actual.y, actual.z]))
                if initial_distance is None:
                    initial_distance = distance
                elif distance > initial_distance + 0.03:
                    raise RuntimeError("visual servo error increased by more than 3 cm")
                self.get_logger().info(f"servo error_m={[round(a-b, 4) for a, b in zip(desired, [actual.x, actual.y, actual.z])]} "
                                       f"orientation_error_deg={math.degrees(angle_error):.2f}",
                                       throttle_duration_sec=2.0)
                if time.monotonic() - scene_update_at >= 0.5:
                    obj = self._collision(target_id, [position.x, position.y, position.z],
                                          list(target.size_m), yaw,
                                          self.config['targets'][self.target_class]['shape'] == 'cylinder')
                    scene = PlanningScene()
                    scene.is_diff = scene.robot_state.is_diff = True
                    scene.world.collision_objects = [obj]
                    self._apply_scene(scene, goal)
                    scene_update_at = time.monotonic()
                if reached and angle_error <= math.radians(3.0):
                    if stable_since is None:
                        stable_since = time.monotonic()
                    response = self._velocity(lease, [0.0] * len(self.config["joint_names"]), goal)
                    halt = TwistStamped()
                    halt.header.frame_id = self.config['base_frame']
                    halt.header.stamp = self.get_clock().now().to_msg()
                    self.twists.publish(halt)
                    if time.monotonic() - stable_since >= 0.5:
                        if not response.stationary_confirmed:
                            self._wait_stationary(goal)
                        fresh = self._latest_target(goal, timeout=0.1)
                        current = self.tf.lookup_transform(self.config['base_frame'], self.config['tip_frame'],
                                                           Time(), timeout=Duration(seconds=0.4)).transform
                        p = fresh.pose.pose.position
                        error = np.linalg.norm(np.asarray([p.x, p.y, p.z + self.config['finger_contact_offset_m']]) -
                                               [current.translation.x, current.translation.y, current.translation.z])
                        q = current.rotation
                        fresh_orientation = fresh.pose.pose.orientation
                        fresh_yaw = 2 * math.atan2(fresh_orientation.z, fresh_orientation.w)
                        wanted_quaternion = continuous_grasp_orientation(fresh_yaw, wanted_quaternion)
                        if error <= self.config['servo_position_tolerance_m'] and orientation_error(
                                wanted_quaternion, [q.x, q.y, q.z, q.w]) <= math.radians(3.0):
                            return fresh
                        stable_since = None
                    time.sleep(0.1)
                    continue
                stable_since = None
                twist = TwistStamped()
                twist.header.frame_id = self.config["base_frame"]
                twist.header.stamp = self.get_clock().now().to_msg()
                twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z = velocity
                twist.twist.angular.x, twist.twist.angular.y, twist.twist.angular.z = angular
                self.twists.publish(twist)
                time.sleep(0.18)
                with self.lock:
                    command, status = self.joint_command, self.servo_status
                if (not status or not command or not command[0].points) and time.monotonic() < startup_deadline:
                    continue
                if not status or time.monotonic() - status[1] > 1.0 or status[0] in (-1, 2, 4, 5):
                    raise RuntimeError(f"MoveIt Servo stopped or stale: {status}")
                if not command or time.monotonic() - command[1] > 0.75 or not command[0].points:
                    raise RuntimeError("MoveIt Servo joint output unavailable")
                values = map_servo_velocities(command[0].joint_names, command[0].points[-1].velocities,
                                               self.config["joint_names"], 0.65)
                now = self.get_clock().now().nanoseconds
                elapsed = (now - previous_time) / 1e9
                previous_time = now
                if elapsed <= 0 or elapsed > 0.5:
                    raise RuntimeError("simulation clock did not advance normally")
                positions = self._positions()
                servo_values = list(values)
                values = tracker.step(positions, values, elapsed)
                self.get_logger().info(f"servo joints actual={[round(x, 3) for x in positions]} "
                                       f"servo={[round(x, 3) for x in servo_values]} "
                                       f"reference={[round(x, 3) for x in tracker.reference]} "
                                       f"command={[round(x, 3) for x in values]}", throttle_duration_sec=2.0)
                self._check_velocity_state(positions, values, goal)
                self._velocity(lease, values, goal)
            raise TimeoutError("visual servo approach timed out")
        finally:
            try:
                self._velocity(lease, [0.0] * len(self.config["joint_names"]))
            except Exception as exc:
                self.get_logger().error(f"zero velocity failed: {exc}")
            try:
                self._call("servo_stop", Trigger.Request())
            except Exception as exc:
                self.get_logger().warning(f"Servo stop failed: {exc}")

    def _execute(self, goal):
        """Run one simulation-only visual servo grasp and lift operation.

        Args:
            goal: Accepted visual servo action goal.

        Returns:
            Visual servo grasp action result.
        """
        result = VisualServoGrasp.Result()
        leases = ["", ""]
        renew_stop = threading.Event()
        worker = None
        target_id = goal.request.object_id
        installed = False
        attached_target = False
        self.lost.clear()
        try:
            arm_state = json.loads(self._call("arm_state", GetArmState.Request(), goal).state_json)
            gripper_state = self._call("grip_state", GetGripperState.Request(), goal).state
            if arm_state.get("backend") != "ros2_sim" or gripper_state.backend != "ros2_sim":
                raise RuntimeError("visual servo grasp requires simulation backends")
            if not arm_state.get("feedback_valid") or not gripper_state.feedback_valid:
                raise RuntimeError("device feedback is unavailable")
            for index, key, typ in ((0, "arm_acquire", AcquireArmControl),
                                    (1, "grip_acquire", AcquireGripperControl)):
                request = typ.Request()
                request.owner, request.ttl_sec = "visual_servo:" + uuid.uuid4().hex, self.config["lease_ttl_sec"]
                leases[index] = self._call(key, request, goal).lease_id
            worker = threading.Thread(target=self._renew, args=(leases, renew_stop), daemon=True)
            worker.start()
            for key, typ, lease in (("arm_enable", SetArmMotionEnabled, leases[0]),
                                    ("grip_enable", SetGripperMotionEnabled, leases[1])):
                request = typ.Request()
                request.lease_id, request.enabled = lease, True
                self._call(key, request, goal)
            self._feedback(goal, "observe", "planning observation pose")
            observation = self._plan_joints(self.config["observation_joints_rad"], goal)
            self._execute_trajectory(observation, leases[0], goal)
            # Discard estimates obtained before reaching the observation pose.
            self._call("reset_tracker", Trigger.Request(), goal)
            with self.lock:
                self.target = None
            self._feedback(goal, "detect", "waiting for YOLOE RGB-D target")
            target = self._latest_target(goal, timeout=15.0)
            result.object_pose = target.pose
            position = target.pose.pose.position
            orientation = target.pose.pose.orientation
            yaw = 2 * math.atan2(orientation.z, orientation.w)
            size = list(target.size_m)
            if not all(math.isfinite(x) and 0.005 <= x <= 0.5 for x in size):
                raise ValueError("invalid target dimensions")
            if size[1] > 0.06:
                raise ValueError("target is wider than configured grasp opening")
            closing = (yaw + math.pi / 2 + math.pi / 2) % math.pi - math.pi / 2
            grasp = Pose()
            grasp.position.x, grasp.position.y = position.x, position.y
            grasp.position.z = position.z + self.config["finger_contact_offset_m"]
            grasp.orientation.x, grasp.orientation.y = math.cos(closing / 2), math.sin(closing / 2)
            grasp.orientation.z = grasp.orientation.w = 0.0
            result.grasp_pose.header = target.pose.header
            result.grasp_pose.pose = grasp
            is_cylinder = self.config["targets"][self.target_class]["shape"] == "cylinder"
            obj = self._collision(target_id, [position.x, position.y, position.z], size, yaw, is_cylinder)
            ground = self._collision("visual_servo_ground", [0.88, 0.0, self.config["ground_height_base_m"] - 0.01],
                                     [0.6, 0.6, 0.02])
            scene = PlanningScene()
            scene.is_diff = scene.robot_state.is_diff = True
            scene.world.collision_objects = [ground, obj]
            self._apply_scene(scene, goal)
            installed = True
            pregrasp = Pose()
            pregrasp.position.x, pregrasp.position.y = grasp.position.x, grasp.position.y
            pregrasp.position.z = grasp.position.z + self.config["pregrasp_distance_m"]
            pregrasp.orientation = grasp.orientation
            self.get_logger().info(f"target center={[position.x, position.y, position.z]} size={size} "
                                   f"yaw={yaw:.3f} closing={closing:.3f}")
            self._feedback(goal, "plan", "planning pregrasp pose", target.confidence)
            try:
                trajectory = self._cartesian(pregrasp, goal)
            except Cancelled:
                raise
            except Exception as exc:
                self.get_logger().warning(f"Cartesian pregrasp unavailable: {exc}; trying IK")
                trajectory = self._plan_pose(pregrasp, goal)
            self._execute_trajectory(trajectory, leases[0], goal)
            self._feedback(goal, "open", "opening gripper", target.confidence)
            self._gripper("open", leases[1], target_id, goal)
            with self.lock:
                self.target = None
            self._latest_target(goal, timeout=5.0)
            self._feedback(goal, "servo", "approaching with continuous vision", target.confidence)
            target = self._servo_approach(goal, leases[0], target_id, grasp.orientation)
            position = target.pose.pose.position
            orientation = target.pose.pose.orientation
            yaw = 2 * math.atan2(orientation.z, orientation.w)
            obj = self._collision(target_id, [position.x, position.y, position.z],
                                  list(target.size_m), yaw, is_cylinder)
            self._feedback(goal, "close", "closing gripper on target", target.confidence)
            closed = self._gripper("close", leases[1], target_id, goal)
            if not closed.contact_confirmed or closed.contacted_object_id != target_id:
                raise RuntimeError("gripper did not confirm contact with target")
            attached = AttachedCollisionObject()
            attached.link_name = self.config["tip_frame"]
            attached.touch_links = [self.config["tip_frame"], "ms42dc_base_link",
                                    "ms42dc_left_finger_link", "ms42dc_right_finger_link"]
            attached.object = obj
            scene = PlanningScene()
            scene.is_diff = scene.robot_state.is_diff = True
            scene.robot_state.attached_collision_objects = [attached]
            self._apply_scene(scene, goal)
            attached_target = True
            current = self.tf.lookup_transform(self.config["base_frame"], self.config["tip_frame"],
                                               Time(), timeout=Duration(seconds=0.4)).transform
            lift = Pose()
            lift.position.x, lift.position.y = current.translation.x, current.translation.y
            lift.position.z = current.translation.z + self.config["lift_distance_m"]
            lift.orientation = current.rotation
            self._feedback(goal, "lift", "planning and executing lift", target.confidence)
            trajectory = self._slow_trajectory(self._cartesian(lift, goal), 0.25)
            self._execute_trajectory(trajectory, leases[0], goal)
            hold_start = self.get_clock().now().nanoseconds
            hold_deadline = time.monotonic() + 10.0
            while self.get_clock().now().nanoseconds - hold_start < 2_250_000_000:
                self._check(goal)
                if time.monotonic() > hold_deadline:
                    raise TimeoutError("simulation clock did not advance during contact hold")
                state = self._call("grip_state", GetGripperState.Request(), goal).state
                if not state.feedback_contact_valid or target_id not in state.feedback_contact_object_ids:
                    raise RuntimeError("target contact was not maintained during lift and hold")
                time.sleep(0.1)
            result.success = True
            result.grasp_verified = False
            result.message = "lift completed with contact; independent truth verification required"
            self._feedback(goal, "complete", result.message, target.confidence)
            goal.succeed()
        except Exception as exc:
            result.message = str(exc)
            self.get_logger().error(f"visual servo grasp failed: {exc}")
            if isinstance(exc, Cancelled) or goal.is_cancel_requested:
                goal.canceled()
            else:
                goal.abort()
        finally:
            for key, typ in (("arm_stop", StopArm), ("grip_stop", StopGripper)):
                try:
                    request = typ.Request()
                    request.reason = "visual servo grasp finished"
                    self._call(key, request, timeout=3.0)
                except Exception as exc:
                    self.get_logger().error(f"{key} failed: {exc}")
            renew_stop.set()
            if worker is not None:
                worker.join(timeout=4.0)
            for key, typ, lease in (("arm_release", ReleaseArmControl, leases[0]),
                                    ("grip_release", ReleaseGripperControl, leases[1])):
                if lease:
                    try:
                        request = typ.Request()
                        request.lease_id = lease
                        self._call(key, request, timeout=3.0)
                    except Exception as exc:
                        self.get_logger().error(f"{key} failed: {exc}")
            if installed:
                try:
                    if self.saved_collision_matrix is not None:
                        scene = PlanningScene()
                        scene.is_diff = scene.robot_state.is_diff = True
                        scene.allowed_collision_matrix = self.saved_collision_matrix
                        self._apply_scene(scene)
                        self.saved_collision_matrix = None
                    scene = PlanningScene()
                    scene.is_diff = scene.robot_state.is_diff = True
                    if attached_target:
                        detach = AttachedCollisionObject()
                        detach.object.id, detach.object.operation = target_id, CollisionObject.REMOVE
                        scene.robot_state.attached_collision_objects = [detach]
                        self._apply_scene(scene)
                    scene = PlanningScene()
                    scene.is_diff = scene.robot_state.is_diff = True
                    for name in (target_id, "visual_servo_ground"):
                        obj = CollisionObject()
                        obj.id, obj.header.frame_id, obj.operation = name, self.config["base_frame"], CollisionObject.REMOVE
                        scene.world.collision_objects.append(obj)
                    self._apply_scene(scene)
                except Exception as exc:
                    self.get_logger().warning(f"planning scene cleanup failed: {exc}")
            with self.lock:
                self.active = False
        return result


def main(args=None):
    """Run the independent grasp action server with concurrent callbacks.

    Args:
        args: Optional ROS command-line arguments.

    Returns:
        None.
    """
    rclpy.init(args=args)
    node = VisualServoGraspNode()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
