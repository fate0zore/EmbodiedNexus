"""MuJoCo physics and ROS 2 topic bridge for the Arachne platform."""

from __future__ import annotations

import math
import subprocess
import time
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
        description = Path(get_package_share_directory("embodied_nexus_sim_description"))
        camera_package = Path(get_package_share_directory("embodied_nexus_camera"))
        self._camera_calibration = load_camera_calibration(description, camera_package)
        robot_description = _expanded_urdf(description, self._camera_calibration)
        self.model = mujoco.MjModel.from_xml_string(
            make_scene(robot_description, description, self._camera_calibration))
        self.get_logger().info(
            "Gemini 335 calibration loaded: "
            f"mount={self._camera_calibration['mount_name']}, "
            f"intrinsics={self._camera_calibration['factory_calibration_id']}"
        )
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)
        self._root_qpos = self._joint_qpos("root")
        self._root_qvel = self._joint_qvel("root")
        self._joint_names, self._joint_qpos_addrs, self._joint_qvel_addrs = self._movable_joints()
        self._actuators = {
            name: _name_id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            for name in (*WHEEL_JOINTS, *ARM_JOINTS, *GRIPPER_JOINTS)
        }
        self._joint_ids = {
            name: _name_id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            for name in (*WHEEL_JOINTS, *ARM_JOINTS, *GRIPPER_JOINTS)
        }
        self._lidar_body = _name_id(self.model, mujoco.mjtObj.mjOBJ_BODY, "lidar_link")
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
        self._renderer_failed = False
        self._camera_reported = False
        self._viewer = None
        self._initialize_publishers()
        self.create_subscription(TwistStamped, f"{SIM_NAMESPACE}/base_controller/cmd_vel",
                                 self._on_base_command, 10)
        self.create_subscription(JointTrajectory, f"{SIM_NAMESPACE}/arm_controller/joint_trajectory",
                                 self._on_arm_trajectory, 10)
        self.create_subscription(JointTrajectory, f"{SIM_NAMESPACE}/gripper_controller/joint_trajectory",
                                 self._on_gripper_trajectory, 10)
        if self.get_parameter("gui").get_parameter_value().bool_value:
            self._viewer = viewer.launch_passive(self.model, self.data)
        self._timer = self.create_timer(0.01, self._tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(
            f"MuJoCo Arachne scene ready: {self.model.nbody} bodies, "
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

    def _movable_joints(self) -> tuple[list[str], list[int], list[int]]:
        """Collect all scalar model joints for ROS joint-state feedback.

        Args:
            None.

        Returns:
            Joint names and corresponding position and velocity indices.
        """
        names, positions, velocities = [], [], []
        for index in range(self.model.njnt):
            if self.model.jnt_type[index] == mujoco.mjtJoint.mjJNT_FREE:
                continue
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, index)
            names.append(name)
            positions.append(int(self.model.jnt_qposadr[index]))
            velocities.append(int(self.model.jnt_dofadr[index]))
        return names, positions, velocities

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
            if self.data.time >= self._next_camera:
                self._publish_camera()
                self._next_camera = self.data.time + 1.0 / CAMERA_RATE_HZ
            if self.data.time >= self._next_lidar:
                self._publish_lidar()
                self._next_lidar = self.data.time + 1.0 / LIDAR_RATE_HZ
            if self._viewer is not None:
                self._viewer.sync()

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

    def _publish_camera(self) -> None:
        """Render and publish RGB, depth, and camera calibration messages.

        Args:
            None.

        Returns:
            None.
        """
        stamp = _stamp(float(self.data.time))
        color_stream = self._camera_calibration["streams"]["color"]
        depth_stream = self._camera_calibration["streams"]["depth"]
        try:
            if self._renderer is None:
                self._renderer = mujoco.Renderer(
                    self.model, height=color_stream["height"], width=color_stream["width"])
            self._renderer.disable_depth_rendering()
            self._renderer.update_scene(self.data, camera="camera_color")
            color = np.asarray(self._renderer.render(), dtype=np.uint8).copy()
            self._renderer.enable_depth_rendering()
            self._renderer.update_scene(self.data, camera="camera_depth")
            depth = np.asarray(self._renderer.render(), dtype=np.float32).copy()
        except Exception as exc:
            if self._renderer is not None:
                self._renderer.close()
                self._renderer = None
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
        self._color_pub.publish(color_message)
        self._depth_pub.publish(depth_message)
        self._color_info_pub.publish(self._camera_info(stamp, "camera_color_optical_frame", "color"))
        self._depth_info_pub.publish(self._camera_info(stamp, "camera_depth_optical_frame", "depth"))
        if not self._camera_reported:
            self.get_logger().info("MuJoCo RGB-D camera topics are publishing")
            self._camera_reported = True

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

    def _publish_lidar(self) -> None:
        """Ray-cast a horizontal 360 degree scan and publish XYZ points.

        Args:
            None.

        Returns:
            None.
        """
        origin = self.data.xpos[self._lidar_body].copy()
        hits = []
        geom = np.zeros(1, dtype=np.int32)
        for index in range(LIDAR_RAYS):
            angle = 2.0 * math.pi * index / LIDAR_RAYS + self._base_yaw
            direction = np.array([math.cos(angle), math.sin(angle), 0.0], dtype=np.float64)
            distance = mujoco.mj_ray(self.model, self.data, origin, direction, None, 1, -1, geom)
            if 1.0 <= distance <= 18.0:
                hits.append((origin + distance * direction).tolist())
        message = point_cloud2.create_cloud_xyz32(
            header=self._sensor_header(_stamp(float(self.data.time)), "lidar_link"), points=hits)
        self._lidar_pub.publish(message)

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
        if self._viewer is not None:
            self._viewer.close()
            self._viewer = None
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None


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
