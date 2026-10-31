"""MuJoCo physics and ROS 2 topic bridge for the Arachne platform."""

from __future__ import annotations

import copy
import math
import subprocess
import threading
import time
from queue import Empty, Full, Queue
from pathlib import Path

import mujoco
from mujoco import viewer
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock as ClockMessage
from sensor_msgs.msg import CameraInfo, Image, JointState, PointCloud2
from sensor_msgs_py import point_cloud2
from trajectory_msgs.msg import JointTrajectory
from embodied_nexus_sim_mujoco_interfaces.srv import ApplyScene

from .camera_calibration import apply_camera_calibration, load_camera_calibration
from .control import COMMAND_TIMEOUT, Trajectory, make_trajectory, wheel_speeds
from .model import make_scene


SIM_NAMESPACE = "/sim/arachne"
ARM_JOINTS = (
    "aubo_shoulder_joint", "aubo_upperArm_joint", "aubo_foreArm_joint",
    "aubo_wrist1_joint", "aubo_wrist2_joint", "aubo_wrist3_joint",
)
GRIPPER_JOINTS = ("ms42dc_left_finger_joint", "ms42dc_right_finger_joint")
WHEEL_JOINTS = ("front_left_wheel", "rear_left_wheel",
                "front_right_wheel", "rear_right_wheel")
CAMERA_RATE_HZ = 10.0
LIDAR_RATE_HZ = 10.0
VIEWER_SYNC_RATE_HZ = 30.0
LIDAR_RAYS = 360


def _name_id(model: mujoco.MjModel, object_type: mujoco.mjtObj, name: str) -> int:
    """Resolve a MuJoCo object name or raise a descriptive error.

    Args:
        model: Compiled MuJoCo model.
        object_type: Kind of MuJoCo object to look up.
        name: Exact object name from the shared robot description.

    Returns:
        The numeric MuJoCo object identifier.
    """
    identifier = mujoco.mj_name2id(model, object_type, name)
    if identifier < 0:
        raise ValueError(f"MuJoCo model is missing {object_type} {name!r}")
    return identifier


def _stamp(seconds: float) -> Time:
    """Convert simulation seconds to a normalized ROS timestamp.

    Args:
        seconds: Simulation time in seconds.

    Returns:
        A ROS timestamp with nanoseconds normalized to one second.
    """
    whole = math.floor(seconds)
    return Time(sec=int(whole), nanosec=int(round((seconds - whole) * 1e9)))


def _expanded_urdf(description: Path, camera_calibration: dict[str, object]) -> str:
    """Expand the shared robot Xacro and apply the shared camera calibration.

    Args:
        description: Installed shared robot description directory.
        camera_calibration: Shared Gemini 335 mount and RGB-D intrinsics.

    Returns:
        Expanded Arachne URDF XML with calibrated color and depth frames.
    """
    source = description / "urdf" / "arachne.urdf.xacro"
    result = subprocess.run(
        ["xacro", str(source), "with_mimic_joints:=false", "with_ros2_control:=false",
         "gazebo_drive_wheels:=false", "with_ee_camera:=true", "with_lidar:=true"],
        check=True, capture_output=True, text=True,
    )
    return apply_camera_calibration(result.stdout, camera_calibration)


class MujocoSimulation(Node):
    """Run the shared Arachne model and expose the project simulation topics."""

    def __init__(self) -> None:
        """Create the MuJoCo model, ROS endpoints, and wall-clock update timer.

        Args:
            None.

        Returns:
            None.
        """
        super().__init__("mujoco", namespace=SIM_NAMESPACE,
                         parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.declare_parameter("gui", False)
        self.declare_parameter("scene", "default")
        self.declare_parameter("scene_directory", "")
        description = Path(get_package_share_directory("embodied_nexus_sim_description"))
        camera_package = Path(get_package_share_directory("embodied_nexus_camera"))
        self._description_directory = description
        self._camera_calibration = load_camera_calibration(description, camera_package)
        robot_description = _expanded_urdf(description, self._camera_calibration)
        self._robot_description = robot_description
        from .scene import SceneLibrary
        scene_directory = self.get_parameter("scene_directory").get_parameter_value().string_value
        if not scene_directory:
            scene_directory = str(Path(get_package_share_directory("embodied_nexus_sim_mujoco"))
                                  / "config" / "scenes")
        self._scene_directory = scene_directory
        self._scene_library = SceneLibrary(scene_directory)
        scene_name = self.get_parameter("scene").get_parameter_value().string_value
        scene_config = self._scene_library.get_scene(scene_name)
        self.model = mujoco.MjModel.from_xml_string(
            make_scene(robot_description, description, self._camera_calibration,
                       scene_config, scene_directory))
        self.get_logger().info(
            "Gemini 335 calibration loaded: "
            f"mount={self._camera_calibration['mount_name']}, "
            f"intrinsics={self._camera_calibration['factory_calibration_id']}"
        )
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)
        self._set_model_indices(self._model_indices(self.model))
        self._scene_name = scene_name
        self._base_x = float(self.data.qpos[self._root_qpos])
        self._base_y = float(self.data.qpos[self._root_qpos + 1])
        self._base_yaw = 0.0
        self._base_velocity = (0.0, 0.0)
        self._base_command_time = 0.0
        self._arm_trajectory: Trajectory | None = None
        self._gripper_trajectory: Trajectory | None = None
        self._next_camera = 0.0
        self._next_lidar = 0.0
        self._last_odom = 0.0
        self._renderer = None
        self._renderer_model = None
        self._renderer_failed = False
        self._camera_reported = False
        self._lidar_reported = False
        self._sensor_worker_reported = False
        self._sensor_publish_reported = False
        self._sensor_stop = object()
        self._sensor_inputs: Queue[object] = Queue(maxsize=1)
        self._sensor_outputs: Queue[object] = Queue(maxsize=1)
        self._model_generation = 0
        self._sensor_publish_lock = threading.Lock()
        self._sensor_thread = threading.Thread(
            target=self._sensor_worker, name="mujoco-sensors", daemon=False)
        self._viewer = None
        self._viewer_sync_stop = threading.Event()
        self._viewer_sync_thread: threading.Thread | None = None
        self._viewer_sync_reported = False
        self._sensor_publish_thread = threading.Thread(
            target=self._sensor_publisher_worker, name="mujoco-sensor-publisher", daemon=False)
        self._initialize_publishers()
        self.create_subscription(TwistStamped, f"{SIM_NAMESPACE}/base_controller/cmd_vel",
                                 self._on_base_command, 10)
        self.create_subscription(JointTrajectory, f"{SIM_NAMESPACE}/arm_controller/joint_trajectory",
                                 self._on_arm_trajectory, 10)
        self.create_subscription(JointTrajectory, f"{SIM_NAMESPACE}/gripper_controller/joint_trajectory",
                                 self._on_gripper_trajectory, 10)
        self._apply_scene_service = self.create_service(
            ApplyScene, f"{SIM_NAMESPACE}/mujoco/apply_scene", self._apply_scene)
        if self.get_parameter("gui").get_parameter_value().bool_value:
            self._viewer = viewer.launch_passive(self.model, self.data)
            self._start_viewer_sync()
        self._sensor_thread.start()
        self._sensor_publish_thread.start()
        self._timer = self.create_timer(0.01, self._tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(
            f"MuJoCo scene '{scene_name}' ready: {self.model.nbody} bodies, "
            f"{self.model.njnt} joints, {self.model.ngeom} collision geoms"
        )

    def _initialize_publishers(self) -> None:
        """Create publishers for clock, robot feedback, camera, and lidar.

        Args:
            None.

        Returns:
            None.
        """
        clock_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                               durability=DurabilityPolicy.VOLATILE)
        self._clock_pub = self.create_publisher(ClockMessage, "/clock", clock_qos)
        self._joint_pub = self.create_publisher(JointState, f"{SIM_NAMESPACE}/joint_states", 10)
        self._odom_pub = self.create_publisher(Odometry, f"{SIM_NAMESPACE}/base_controller/odom", 10)
        self._color_pub = self.create_publisher(Image, f"{SIM_NAMESPACE}/camera/image_raw", qos_profile_sensor_data)
        self._depth_pub = self.create_publisher(Image, f"{SIM_NAMESPACE}/camera/depth_image_raw", qos_profile_sensor_data)
        self._color_info_pub = self.create_publisher(CameraInfo, f"{SIM_NAMESPACE}/camera/color/camera_info", 10)
        self._depth_info_pub = self.create_publisher(CameraInfo, f"{SIM_NAMESPACE}/camera/depth/camera_info", 10)
        self._lidar_pub = self.create_publisher(PointCloud2, f"{SIM_NAMESPACE}/lidar/points", qos_profile_sensor_data)

    def _joint_qpos(self, name: str) -> int:
        """Return a joint's first generalized position index.

        Args:
            name: MuJoCo joint name.

        Returns:
            The joint position index.
        """
        joint = _name_id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
        return int(self.model.jnt_qposadr[joint])

    def _joint_qvel(self, name: str) -> int:
        """Return a joint's first generalized velocity index.

        Args:
            name: MuJoCo joint name.

        Returns:
            The joint velocity index.
        """
        joint = _name_id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
        return int(self.model.jnt_dofadr[joint])

    def _movable_joints(self, model: mujoco.MjModel | None = None) -> tuple[list[str], list[int], list[int]]:
        """Collect all scalar model joints for ROS joint-state feedback.

        Args:
            model: Optional compiled model; defaults to the active model.

        Returns:
            Joint names and corresponding position and velocity indices.
        """
        model = model or self.model
        names, positions, velocities = [], [], []
        for index in range(model.njnt):
            if model.jnt_type[index] == mujoco.mjtJoint.mjJNT_FREE:
                continue
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, index)
            names.append(name)
            positions.append(int(model.jnt_qposadr[index]))
            velocities.append(int(model.jnt_dofadr[index]))
        return names, positions, velocities

    def _model_indices(self, model: mujoco.MjModel) -> dict[str, object]:
        """Resolve all named indices needed by the simulation for one model.

        Args:
            model: Compiled MuJoCo model whose names should be indexed.

        Returns:
            A cache of root, joint, actuator, and lidar indices for that model.
        """
        root_joint = _name_id(model, mujoco.mjtObj.mjOBJ_JOINT, "root")
        return {
            "root_qpos": int(model.jnt_qposadr[root_joint]),
            "root_qvel": int(model.jnt_dofadr[root_joint]),
            "joints": self._movable_joints(model),
            "actuators": {
                name: _name_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
                for name in (*WHEEL_JOINTS, *ARM_JOINTS, *GRIPPER_JOINTS)
            },
            "joint_ids": {
                name: _name_id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
                for name in (*WHEEL_JOINTS, *ARM_JOINTS, *GRIPPER_JOINTS)
            },
            "lidar_body": _name_id(model, mujoco.mjtObj.mjOBJ_BODY, "lidar_link"),
        }

    def _set_model_indices(self, indices: dict[str, object]) -> None:
        """Install a previously resolved model index cache on the simulation.

        Args:
            indices: Model index cache returned by ``_model_indices``.

        Returns:
            None.
        """
        self._root_qpos = indices["root_qpos"]
        self._root_qvel = indices["root_qvel"]
        self._joint_names, self._joint_qpos_addrs, self._joint_qvel_addrs = indices["joints"]
        self._actuators = indices["actuators"]
        self._joint_ids = indices["joint_ids"]
        self._lidar_body = indices["lidar_body"]

    def _apply_scene(self, request: ApplyScene.Request, response: ApplyScene.Response) -> ApplyScene.Response:
        """Compile and activate a saved scene while preserving robot state and sim time.

        Args:
            request: Service request naming the saved scene to activate.
            response: Service response populated with the apply result and model counts.

        Returns:
            The populated service response.
        """
        try:
            scene = self._scene_library.get_scene(request.scene_id)
            xml = make_scene(self._robot_description, self._description_directory,
                             self._camera_calibration, scene, self._scene_directory)
            candidate_model = mujoco.MjModel.from_xml_string(xml)
            candidate_data = mujoco.MjData(candidate_model)
            candidate_indices = self._model_indices(candidate_model)
            self._copy_robot_state(candidate_model, candidate_data, candidate_indices)
            mujoco.mj_forward(candidate_model, candidate_data)
            self._replace_viewer(candidate_model, candidate_data)
        except Exception as exc:
            response.success = False
            response.message = f"场景应用失败，运行模型未切换：{exc}"
            return response

        self.model = candidate_model
        self.data = candidate_data
        self._set_model_indices(candidate_indices)
        self._scene_name = request.scene_id
        self._base_velocity = (0.0, 0.0)
        self._base_command_time = time.monotonic()
        self._arm_trajectory = None
        self._gripper_trajectory = None
        self._next_camera = float(self.data.time)
        self._next_lidar = float(self.data.time)
        self._last_odom = float(self.data.time)
        with self._sensor_publish_lock:
            self._model_generation += 1
            self._discard_pending_sensor_work()
        response.success = True
        response.message = f"场景 {request.scene_id} 已应用"
        response.body_count = candidate_model.nbody
        response.geom_count = candidate_model.ngeom
        self.get_logger().info(
            f"Applied MuJoCo scene '{request.scene_id}': "
            f"{candidate_model.nbody} bodies, {candidate_model.ngeom} geoms"
        )
        return response

    def _copy_robot_state(self, model: mujoco.MjModel, data: mujoco.MjData,
                          indices: dict[str, object]) -> None:
        """Copy robot joint positions into a newly compiled model and clear motion targets.

        Args:
            model: Candidate model that will replace the active model.
            data: New state object initialized from the candidate model.
            indices: Candidate model index cache.

        Returns:
            None.
        """
        old_model, old_data = self.model, self.data
        names = ("root", *WHEEL_JOINTS, *ARM_JOINTS, *GRIPPER_JOINTS)
        for name in names:
            old_joint = _name_id(old_model, mujoco.mjtObj.mjOBJ_JOINT, name)
            new_joint = _name_id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            joint_type = int(old_model.jnt_type[old_joint])
            if joint_type != model.jnt_type[new_joint]:
                raise ValueError(f"robot joint {name!r} changed type in candidate model")
            qpos_size, qvel_size = {
                int(mujoco.mjtJoint.mjJNT_FREE): (7, 6),
                int(mujoco.mjtJoint.mjJNT_BALL): (4, 3),
                int(mujoco.mjtJoint.mjJNT_SLIDE): (1, 1),
                int(mujoco.mjtJoint.mjJNT_HINGE): (1, 1),
            }[joint_type]
            old_qpos = int(old_model.jnt_qposadr[old_joint])
            new_qpos = int(model.jnt_qposadr[new_joint])
            old_qvel = int(old_model.jnt_dofadr[old_joint])
            new_qvel = int(model.jnt_dofadr[new_joint])
            data.qpos[new_qpos:new_qpos + qpos_size] = old_data.qpos[old_qpos:old_qpos + qpos_size]
            data.qvel[new_qvel:new_qvel + qvel_size] = old_data.qvel[old_qvel:old_qvel + qvel_size]

        root_qpos = indices["root_qpos"]
        root_qvel = indices["root_qvel"]
        data.qpos[root_qpos:root_qpos + 3] = (self._base_x, self._base_y, 0.236)
        data.qpos[root_qpos + 3:root_qpos + 7] = (
            math.cos(self._base_yaw / 2), 0.0, 0.0, math.sin(self._base_yaw / 2))
        data.qvel[root_qvel:root_qvel + 6] = 0.0
        data.time = float(old_data.time)
        actuators = indices["actuators"]
        joint_ids = indices["joint_ids"]
        for name in WHEEL_JOINTS:
            data.ctrl[actuators[name]] = 0.0
        for name in (*ARM_JOINTS, *GRIPPER_JOINTS):
            joint = joint_ids[name]
            qpos = int(model.jnt_qposadr[joint])
            data.ctrl[actuators[name]] = data.qpos[qpos]

    def _discard_pending_sensor_work(self) -> None:
        """Remove queued frames from models that have just been replaced.

        Args:
            None.

        Returns:
            None.
        """
        for pending in (self._sensor_inputs, self._sensor_outputs):
            while True:
                try:
                    pending.get_nowait()
                except Empty:
                    break

    def _start_viewer_sync(self) -> None:
        """Start the background synchronization loop for the current passive viewer.

        Args:
            None.

        Returns:
            None.
        """
        if self._viewer is None:
            return
        self._viewer_sync_stop = threading.Event()
        self._viewer_sync_reported = False
        self._viewer_sync_thread = threading.Thread(
            target=self._viewer_sync_worker, name="mujoco-viewer-sync", daemon=False)
        self._viewer_sync_thread.start()

    def _stop_viewer_sync(self) -> None:
        """Stop and join the current viewer synchronization loop.

        Args:
            None.

        Returns:
            None.
        """
        self._viewer_sync_stop.set()
        if self._viewer_sync_thread is not None:
            self._viewer_sync_thread.join()
            self._viewer_sync_thread = None

    def _replace_viewer(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        """Reopen the passive viewer for a candidate model, restoring it on failure.

        Args:
            model: Candidate model to display after the hot apply.
            data: Candidate model state to display after the hot apply.

        Returns:
            None.
        """
        if self._viewer is None:
            return
        old_viewer = self._viewer
        self._stop_viewer_sync()
        self._viewer = None
        try:
            old_viewer.close()
            self._viewer = viewer.launch_passive(model, data)
        except Exception:
            try:
                self._viewer = viewer.launch_passive(self.model, self.data)
                self._start_viewer_sync()
            except Exception as restore_error:
                self.get_logger().error(f"Could not restore MuJoCo viewer after apply failure: {restore_error}")
            raise
        self._start_viewer_sync()

    def _on_base_command(self, message: TwistStamped) -> None:
        """Store the most recent chassis velocity command.

        Args:
            message: Stamped linear and angular velocity command.

        Returns:
            None.
        """
        self._base_velocity = (float(message.twist.linear.x), float(message.twist.angular.z))
        self._base_command_time = time.monotonic()

    def _on_arm_trajectory(self, message: JointTrajectory) -> None:
        """Validate and schedule the requested arm joint trajectory.

        Args:
            message: Joint names and target positions from the arm backend.

        Returns:
            None.
        """
        self._arm_trajectory = self._trajectory(message, ARM_JOINTS)

    def _on_gripper_trajectory(self, message: JointTrajectory) -> None:
        """Validate and schedule the requested gripper joint trajectory.

        Args:
            message: Joint names and target positions from the gripper backend.

        Returns:
            None.
        """
        self._gripper_trajectory = self._trajectory(message, GRIPPER_JOINTS)

    def _trajectory(self, message: JointTrajectory, expected: tuple[str, ...]) -> Trajectory | None:
        """Capture measured start positions and make an ordered trajectory.

        Args:
            message: Trajectory command received from ROS.
            expected: Complete ordered joint set for this actuator group.

        Returns:
            A validated trajectory, or None when the message has no points.
        """
        if not message.points:
            return None
        point = message.points[0]
        duration = float(point.time_from_start.sec) + point.time_from_start.nanosec * 1e-9
        if duration <= 0:
            duration = 0.2
        measured = [self.data.qpos[self._joint_qpos(name)] for name in expected]
        return make_trajectory(message.joint_names, point.positions, expected, measured,
                               float(self.data.time), duration)

    def _tick(self) -> None:
        """Advance physics and publish the current ROS simulation state.

        Args:
            None.

        Returns:
            None.
        """
        lock = self._viewer.lock() if self._viewer is not None else _NullContext()
        with lock:
            self._advance()
            self._publish_clock_and_joints()
            if self.data.time - self._last_odom >= 0.05:
                self._publish_odometry()
                self._last_odom = self.data.time
            camera_due = self.data.time >= self._next_camera
            lidar_due = self.data.time >= self._next_lidar
            if camera_due:
                self._next_camera = self.data.time + 1.0 / CAMERA_RATE_HZ
            if lidar_due:
                self._next_lidar = self.data.time + 1.0 / LIDAR_RATE_HZ
            if camera_due or lidar_due:
                self._queue_sensor_snapshot()
    def _viewer_sync_worker(self) -> None:
        """Synchronize GUI state at a bounded rate outside the ROS timer callback.

        Args:
            None.

        Returns:
            None.
        """
        interval = 1.0 / VIEWER_SYNC_RATE_HZ
        while not self._viewer_sync_stop.wait(interval):
            viewer_handle = self._viewer
            if viewer_handle is None:
                return
            try:
                viewer_handle.sync()
            except Exception as exc:
                if not self._viewer_sync_reported:
                    self.get_logger().error(f"MuJoCo viewer synchronization failed: {exc}")
                    self._viewer_sync_reported = True
                return

    def _queue_sensor_snapshot(self) -> None:
        """Copy the current physics state and replace any waiting sensor snapshot.

        Args:
            None.

        Returns:
            None.
        """
        snapshot = copy.copy(self.data)
        self._replace_latest(self._sensor_inputs, (
            self._model_generation, self.model, float(snapshot.time), self._base_yaw, snapshot))

    @staticmethod
    def _replace_latest(target: Queue[object], item: object) -> None:
        """Store one item in a bounded queue, discarding its older waiting item.

        Args:
            target: One-item queue to update.
            item: Newest snapshot or sensor result to retain.

        Returns:
            None.
        """
        while True:
            try:
                target.put_nowait(item)
                return
            except Full:
                try:
                    target.get_nowait()
                except Empty:
                    continue

    def _sensor_worker(self) -> None:
        """Render camera and lidar messages from the latest copied simulation state.

        Args:
            None.

        Returns:
            None.
        """
        try:
            while True:
                job = self._sensor_inputs.get()
                if job is self._sensor_stop:
                    break
                try:
                    generation, model, sim_time, base_yaw, snapshot = job
                    if generation != self._model_generation:
                        continue
                    stamp = _stamp(sim_time)
                    color, depth, color_info, depth_info = self._render_camera_messages(
                        model, snapshot, stamp)
                    try:
                        lidar = self._build_lidar_message(model, snapshot, base_yaw, stamp)
                    except Exception as exc:
                        lidar = None
                        if not self._lidar_reported:
                            self.get_logger().error(f"MuJoCo lidar ray casting failed: {exc}")
                            self._lidar_reported = True
                except Exception as exc:
                    if not self._sensor_worker_reported:
                        self.get_logger().error(f"MuJoCo sensor processing failed: {exc}")
                        self._sensor_worker_reported = True
                    continue
                self._replace_latest(
                    self._sensor_outputs,
                    (generation, sim_time, color, depth, color_info, depth_info, lidar))
        finally:
            if self._renderer is not None:
                try:
                    self._renderer.close()
                except Exception as exc:
                    self.get_logger().error(f"MuJoCo camera renderer shutdown failed: {exc}")
                self._renderer = None
                self._renderer_model = None

    def _sensor_publisher_worker(self) -> None:
        """Publish completed sensor frames without blocking the physics timer.

        Args:
            None.

        Returns:
            None.
        """
        while True:
            result = self._sensor_outputs.get()
            if result is self._sensor_stop:
                return
            generation, _, color, depth, color_info, depth_info, lidar = result
            with self._sensor_publish_lock:
                if generation != self._model_generation:
                    continue
                try:
                    self._color_pub.publish(color)
                    self._depth_pub.publish(depth)
                    self._color_info_pub.publish(color_info)
                    self._depth_info_pub.publish(depth_info)
                    if lidar is not None:
                        self._lidar_pub.publish(lidar)
                except Exception as exc:
                    if not self._sensor_publish_reported:
                        self.get_logger().error(f"MuJoCo sensor message publication failed: {exc}")
                        self._sensor_publish_reported = True

    def _advance(self) -> None:
        """Step the model and apply commanded chassis and joint motion.

        Args:
            None.

        Returns:
            None.
        """
        dt = 0.01
        linear, angular = self._base_velocity
        if time.monotonic() - self._base_command_time > COMMAND_TIMEOUT:
            linear, angular = 0.0, 0.0
            self._base_velocity = (0.0, 0.0)
        left, right = wheel_speeds(linear, angular)
        for name, speed in zip(WHEEL_JOINTS, (left, left, right, right)):
            self.data.ctrl[self._actuators[name]] = speed
        self._apply_trajectory(self._arm_trajectory, ARM_JOINTS)
        self._apply_trajectory(self._gripper_trajectory, GRIPPER_JOINTS)
        steps = max(1, round(dt / self.model.opt.timestep))
        for _ in range(steps):
            mujoco.mj_step(self.model, self.data)
        self._base_yaw += angular * dt
        self._base_x += linear * math.cos(self._base_yaw) * dt
        self._base_y += linear * math.sin(self._base_yaw) * dt
        self.data.qpos[self._root_qpos:self._root_qpos + 3] = (self._base_x, self._base_y, 0.236)
        self.data.qpos[self._root_qpos + 3:self._root_qpos + 7] = (
            math.cos(self._base_yaw / 2), 0.0, 0.0, math.sin(self._base_yaw / 2))
        self.data.qvel[self._root_qvel:self._root_qvel + 6] = (
            linear * math.cos(self._base_yaw), linear * math.sin(self._base_yaw), 0.0,
            0.0, 0.0, angular)
        mujoco.mj_forward(self.model, self.data)

    def _apply_trajectory(self, trajectory: Trajectory | None, names: tuple[str, ...]) -> None:
        """Apply interpolated position targets to a joint group.

        Args:
            trajectory: Active trajectory or None.
            names: Ordered joint names mapped to the trajectory.

        Returns:
            None.
        """
        if trajectory is None:
            return
        targets = trajectory.sample(float(self.data.time))
        for name, target in zip(names, targets):
            self.data.ctrl[self._actuators[name]] = target
        if self.data.time >= trajectory.start_time + trajectory.duration:
            if names == ARM_JOINTS:
                self._arm_trajectory = None
            else:
                self._gripper_trajectory = None

    def _publish_clock_and_joints(self) -> None:
        """Publish /clock and measured positions and velocities.

        Args:
            None.

        Returns:
            None.
        """
        stamp = _stamp(float(self.data.time))
        clock = ClockMessage()
        clock.clock = stamp
        self._clock_pub.publish(clock)
        message = JointState()
        message.header.stamp = stamp
        message.name = self._joint_names
        message.position = [float(self.data.qpos[index]) for index in self._joint_qpos_addrs]
        message.velocity = [float(self.data.qvel[index]) for index in self._joint_qvel_addrs]
        self._joint_pub.publish(message)

    def _publish_odometry(self) -> None:
        """Publish the kinematic chassis pose and commanded twist.

        Args:
            None.

        Returns:
            None.
        """
        message = Odometry()
        message.header.stamp = _stamp(float(self.data.time))
        message.header.frame_id = "odom"
        message.child_frame_id = "base_link"
        message.pose.pose.position.x = self._base_x
        message.pose.pose.position.y = self._base_y
        message.pose.pose.position.z = 0.0
        message.pose.pose.orientation.z = math.sin(self._base_yaw / 2)
        message.pose.pose.orientation.w = math.cos(self._base_yaw / 2)
        message.twist.twist.linear.x, message.twist.twist.angular.z = self._base_velocity
        self._odom_pub.publish(message)

    def _render_camera_messages(self, model: mujoco.MjModel, data: mujoco.MjData,
                                stamp: Time) -> tuple[Image, Image, CameraInfo, CameraInfo]:
        """Render an RGB-D frame and build its calibration messages.

        Args:
            model: Compiled model corresponding to the copied sensor state.
            data: Independent MuJoCo state copied for this sensor frame.
            stamp: Simulation timestamp copied with the state.

        Returns:
            RGB, depth, color calibration, and depth calibration messages.
        """
        color_stream = self._camera_calibration["streams"]["color"]
        depth_stream = self._camera_calibration["streams"]["depth"]
        try:
            if self._renderer is not None and self._renderer_model is not model:
                self._renderer.close()
                self._renderer = None
                self._renderer_model = None
            if self._renderer is None:
                self._renderer = mujoco.Renderer(
                    model, height=color_stream["height"], width=color_stream["width"])
                self._renderer_model = model
            self._renderer.disable_depth_rendering()
            self._renderer.update_scene(data, camera="camera_color")
            color = np.asarray(self._renderer.render(), dtype=np.uint8).copy()
            self._renderer.enable_depth_rendering()
            self._renderer.update_scene(data, camera="camera_depth")
            depth = np.asarray(self._renderer.render(), dtype=np.float32).copy()
        except Exception as exc:
            renderer = self._renderer
            self._renderer = None
            self._renderer_model = None
            if renderer is not None:
                try:
                    renderer.close()
                except Exception as close_error:
                    self.get_logger().error(f"MuJoCo camera renderer close failed: {close_error}")
            if not self._renderer_failed:
                self.get_logger().error(f"MuJoCo camera rendering failed: {exc}")
                self._renderer_failed = True
            color = np.zeros((color_stream["height"], color_stream["width"], 3), dtype=np.uint8)
            depth = np.full((depth_stream["height"], depth_stream["width"]), np.inf, dtype=np.float32)
        color_message = self._image(
            stamp, "camera_color_optical_frame", "rgb8", color.tobytes(), "color", 3)
        depth_mm = np.clip(np.nan_to_num(depth * 1000.0, nan=0.0, posinf=0.0), 0, 65535).astype(np.uint16)
        depth_message = self._image(
            stamp, "camera_depth_optical_frame", "16UC1", depth_mm.tobytes(), "depth", 2)
        color_info = self._camera_info(stamp, "camera_color_optical_frame", "color")
        depth_info = self._camera_info(stamp, "camera_depth_optical_frame", "depth")
        if not self._camera_reported:
            self.get_logger().info("MuJoCo RGB-D camera topics are publishing")
            self._camera_reported = True
        return color_message, depth_message, color_info, depth_info

    def _image(self, stamp: Time, frame: str, encoding: str, data: bytes,
               channel: str, bytes_per_pixel: int) -> Image:
        """Build a ROS image message for an RGB or depth frame.

        Args:
            stamp: Simulation timestamp.
            frame: Optical frame associated with the image.
            encoding: ROS image encoding.
            data: Packed image bytes.
            channel: Calibrated stream name, either color or depth.
            bytes_per_pixel: Number of payload bytes per pixel.

        Returns:
            A populated ROS Image message.
        """
        message = Image()
        message.header.stamp = stamp
        message.header.frame_id = frame
        stream = self._camera_calibration["streams"][channel]
        message.height, message.width = stream["height"], stream["width"]
        message.encoding = encoding
        message.is_bigendian = False
        message.step = message.width * bytes_per_pixel
        message.data = data
        return message

    def _camera_info(self, stamp: Time, frame: str, channel: str) -> CameraInfo:
        """Build ideal rectified CameraInfo from the Gemini 335 factory profile.

        Args:
            stamp: Simulation timestamp.
            frame: Optical frame associated with the image.
            channel: Calibrated stream name, either color or depth.

        Returns:
            A populated ROS CameraInfo message.
        """
        stream = self._camera_calibration["streams"][channel]
        fx, fy = stream["fx"], stream["fy"]
        cx, cy = stream["cx"], stream["cy"]
        message = CameraInfo()
        message.header.stamp = stamp
        message.header.frame_id = frame
        message.width, message.height = stream["width"], stream["height"]
        message.distortion_model = "plumb_bob"
        message.d = [0.0] * 5
        message.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        message.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        message.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return message

    def _build_lidar_message(self, model: mujoco.MjModel, data: mujoco.MjData,
                             base_yaw: float, stamp: Time) -> PointCloud2:
        """Ray-cast a horizontal scan and build its XYZ point cloud message.

        Args:
            model: Compiled model corresponding to the copied sensor state.
            data: Independent MuJoCo state copied for this sensor frame.
            base_yaw: Chassis yaw captured with the state.
            stamp: Simulation timestamp copied with the state.

        Returns:
            The stamped lidar point cloud message.
        """
        lidar_body = _name_id(model, mujoco.mjtObj.mjOBJ_BODY, "lidar_link")
        origin = data.xpos[lidar_body].copy()
        hits = []
        geom = np.zeros(1, dtype=np.int32)
        for index in range(LIDAR_RAYS):
            angle = 2.0 * math.pi * index / LIDAR_RAYS + base_yaw
            direction = np.array([math.cos(angle), math.sin(angle), 0.0], dtype=np.float64)
            distance = mujoco.mj_ray(model, data, origin, direction, None, 1, -1, geom)
            if 1.0 <= distance <= 18.0:
                hits.append((origin + distance * direction).tolist())
        return point_cloud2.create_cloud_xyz32(
            header=self._sensor_header(stamp, "lidar_link"), points=hits)

    def _sensor_header(self, stamp: Time, frame: str):
        """Construct the common stamped header used by sensor messages.

        Args:
            stamp: Simulation timestamp.
            frame: Sensor frame name.

        Returns:
            A populated standard ROS header.
        """
        from std_msgs.msg import Header
        return Header(stamp=stamp, frame_id=frame)

    def close(self) -> None:
        """Close viewer and renderer resources before destroying the node.

        Args:
            None.

        Returns:
            None.
        """
        if self._sensor_thread.is_alive():
            self._replace_latest(self._sensor_inputs, self._sensor_stop)
            self._sensor_thread.join()
        self._replace_latest(self._sensor_outputs, self._sensor_stop)
        if self._sensor_publish_thread.is_alive():
            self._sensor_publish_thread.join()
        self._stop_viewer_sync()
        if self._viewer is not None:
            self._viewer.close()
            self._viewer = None


class _NullContext:
    """No-op context manager used when the optional MuJoCo viewer is closed."""

    def __enter__(self):
        """Enter the no-op context manager.

        Args:
            None.

        Returns:
            This context manager.
        """
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        """Exit the no-op context manager without suppressing errors.

        Args:
            exc_type: Exception type raised in the context, if any.
            exc_value: Exception value raised in the context, if any.
            traceback: Traceback for the exception, if any.

        Returns:
            False so exceptions propagate normally.
        """
        return False


def main(args=None) -> int:
    """Initialize ROS and spin the MuJoCo simulation node.

    Args:
        args: Optional ROS command-line arguments.

    Returns:
        Zero after the node shuts down cleanly.
    """
    rclpy.init(args=args)
    node = MujocoSimulation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0
