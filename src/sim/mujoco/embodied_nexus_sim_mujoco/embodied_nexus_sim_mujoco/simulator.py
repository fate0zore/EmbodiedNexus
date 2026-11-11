"""MuJoCo physics and ROS 2 topic bridge for the Arachne platform."""

from __future__ import annotations

import copy
import json
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
from geometry_msgs.msg import Pose, TwistStamped, WrenchStamped
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock as ClockMessage
from sensor_msgs.msg import CameraInfo, Image, JointState, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, String
from trajectory_msgs.msg import JointTrajectory
from embodied_nexus_sim_mujoco_interfaces.msg import SceneObjectStates
from embodied_nexus_sim_mujoco_interfaces.srv import ApplyScene

from .camera_calibration import apply_camera_calibration, load_camera_calibration
from .control import COMMAND_TIMEOUT, Trajectory, make_trajectory, wheel_speeds
from .model import make_scene
from .fin_ray import configure_fin_ray


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


def _contact_finger_side(model: mujoco.MjModel, contact, side: int) -> str | None:
    """Identify the finger attached to one rigid or flexible contact participant.

    Args:
        model: MuJoCo model containing finger bodies and optional pads.
        contact: MuJoCo contact containing geom and flex participant IDs.
        side: Participant index, either 0 or 1.

    Returns:
        Finger link name, or None when this participant is not a finger.
    """
    flex_id = int(contact.flex[side])
    if flex_id >= 0:
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_FLEX, flex_id) or ""
        for finger in ("left", "right"):
            if name == f"ms42dc_{finger}_pad" or name.startswith(f"ms42dc_{finger}_pad_"):
                return f"ms42dc_{finger}_finger_link"
    geom_id = int(contact.geom[side])
    if geom_id < 0 or geom_id >= model.ngeom:
        return None
    body_id = int(model.geom_bodyid[geom_id])
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id) or ""
    return name if name in ("ms42dc_left_finger_link", "ms42dc_right_finger_link") else None


def _finger_contact_forces(model: mujoco.MjModel, data: mujoco.MjData,
                           object_ids: set[str]) -> dict:
    """Sum each finger's normal force on each scene object.

    Args:
        model: Compiled simulation model.
        data: Current simulation state and contact forces.
        object_ids: Scene object IDs eligible for contact reporting.

    Returns:
        Object IDs mapped to left and right normal forces in newtons.
    """
    result = {item: {'left': 0.0, 'right': 0.0} for item in object_ids}
    for index in range(data.ncon):
        contact = data.contact[index]
        for finger_side, object_side in ((0, 1), (1, 0)):
            finger = _contact_finger_side(model, contact, finger_side)
            geom = int(contact.geom[object_side])
            if finger is None or geom < 0 or geom >= model.ngeom:
                continue
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom) or ''
            object_id = name[:-5] if name.endswith('_geom') else ''
            if object_id in result:
                force = np.zeros(6, dtype=float)
                mujoco.mj_contactForce(model, data, index, force)
                side = 'left' if '_left_' in finger else 'right'
                result[object_id][side] += max(0.0, float(force[0]))
    return result


def _bilateral_finger_contacts(model: mujoco.MjModel, data: mujoco.MjData,
                               object_ids: set[str], minimum_force_n: float = 0.05) -> set[str]:
    """Return scene objects simultaneously touched by both gripper fingers.

    Args:
        model: Active MuJoCo model containing the two named finger bodies.
        data: Current MuJoCo state and contact manifold.
        object_ids: IDs of scene objects eligible for grasp contact reporting.
        minimum_force_n: Minimum normal force on each finger, in newtons.

    Returns:
        Scene object IDs with at least one contact on each finger.
    """
    forces = _finger_contact_forces(model, data, object_ids)
    return {object_id for object_id, values in forces.items()
            if min(values.values()) >= minimum_force_n}


def _finger_visualization(model: mujoco.MjModel, data: mujoco.MjData) -> dict:
    """Describe actual finger collision boxes and contacts in each link frame.

    Args:
        model: Active MuJoCo model with finger bodies and collision geometry.
        data: Current MuJoCo transforms and contact manifold.

    Returns:
        JSON-serializable finger geometry and current contact positions.
    """
    fingers = []
    for name in ("ms42dc_left_finger_link", "ms42dc_right_finger_link"):
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        if body_id < 0:
            continue
        boxes, contacts, pads = [], [], []
        for geom_id in range(model.ngeom):
            if (int(model.geom_bodyid[geom_id]) != body_id or
                    not (model.geom_contype[geom_id] or model.geom_conaffinity[geom_id]) or
                    int(model.geom_type[geom_id]) != int(mujoco.mjtGeom.mjGEOM_BOX)):
                continue
            quat = model.geom_quat[geom_id]
            boxes.append({"position": model.geom_pos[geom_id].tolist(),
                          "quaternion": [float(quat[1]), float(quat[2]), float(quat[3]), float(quat[0])],
                          "size": (2.0 * model.geom_size[geom_id]).tolist()})
        rotation = data.xmat[body_id].reshape(3, 3)
        flex_prefix = name.replace("_finger_link", "_pad")
        for flex_id in range(model.nflex):
            flex_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_FLEX, flex_id) or ""
            if flex_name != flex_prefix and not flex_name.startswith(flex_prefix + "_"):
                continue
            start = int(model.flex_vertadr[flex_id])
            count = int(model.flex_vertnum[flex_id])
            vertices = (data.flexvert_xpos[start:start + count] - data.xpos[body_id]) @ rotation
            if int(model.flex_dim[flex_id]) == 2:
                elem = int(model.flex_elemdataadr[flex_id])
                triangles = model.flex_elem[elem:elem + 3 * int(model.flex_elemnum[flex_id])]
            else:
                shell = int(model.flex_shelldataadr[flex_id])
                triangles = model.flex_shell[shell:shell + 3 * int(model.flex_shellnum[flex_id])]
            edge = int(model.flex_edgeadr[flex_id])
            edge_count = int(model.flex_edgenum[flex_id])
            displacement = np.abs(data.flexedge_length[edge:edge + edge_count] -
                                  model.flexedge_length0[edge:edge + edge_count])
            pads.append({"name": flex_name, "vertices": vertices.tolist(),
                         "triangles": triangles.reshape(-1, 3).tolist(),
                         "max_edge_length_change_m": float(np.max(displacement)) if edge_count else 0.0})
        for index in range(data.ncon):
            contact = data.contact[index]
            for finger_side, other_side in ((0, 1), (1, 0)):
                if _contact_finger_side(model, contact, finger_side) != name:
                    continue
                position = rotation.T @ (contact.pos - data.xpos[body_id])
                other_geom = int(contact.geom[other_side])
                force = np.zeros(6, dtype=float)
                mujoco.mj_contactForce(model, data, index, force)
                contacts.append({"position": position.tolist(), "other":
                                 (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, other_geom) or "")
                                 if other_geom >= 0 else "",
                                 "normal_force_n": float(force[0]), "distance_m": float(contact.dist)})
        fingers.append({"link": name, "boxes": boxes, "contacts": contacts, "pads": pads})
    return {"fingers": fingers}


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
        self.declare_parameter("simplified_finger_collision", True)
        self.declare_parameter("finger_contact_box_half_size_m", [0.023, 0.052, 0.012])
        self._simplified_finger_collision = bool(
            self.get_parameter("simplified_finger_collision").value)
        self._finger_contact_box_half_size_m = tuple(
            float(value) for value in self.get_parameter("finger_contact_box_half_size_m").value)
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
                       scene_config, scene_directory, self._simplified_finger_collision,
                       self._finger_contact_box_half_size_m))
        configure_fin_ray(
            self.model,
            out_of_plane_stiffness_n_m=float(scene_config.get("finger_flex_pads", {}).get(
                "fin_ray_out_of_plane_stiffness_n_m", 500.0)))
        self.get_logger().info(
            "Gemini 335 calibration loaded: "
            f"mount={self._camera_calibration['mount_name']}, "
            f"intrinsics={self._camera_calibration['factory_calibration_id']}"
        )
        self.data = mujoco.MjData(self.model)
        self._set_model_indices(self._model_indices(self.model))
        self._apply_initial_arm_pose(scene_config)
        mujoco.mj_forward(self.model, self.data)
        self._scene_name = scene_name
        self._fixed_base = bool(scene_config.get("fixed_base", False))
        self._scene_objects = list(scene_config.get("objects", []))
        self._test_wrench = None
        self._base_x = float(self.data.qpos[self._root_qpos])
        self._base_y = float(self.data.qpos[self._root_qpos + 1])
        self._base_yaw = 0.0
        self._base_velocity = (0.0, 0.0)
        self._base_command_time = 0.0
        self._arm_trajectory: Trajectory | None = None
        self._arm_trajectory_sequence = 0
        self._last_arm_trajectory_event: dict | None = None
        self._last_flex_diagnostic_log = 0.0
        self._gripper_trajectory: Trajectory | None = None
        self._next_camera = 0.0
        self._next_lidar = 0.0
        self._last_odom = 0.0
        self._last_scene_state = -1.0
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
        self.create_subscription(WrenchStamped, f"{SIM_NAMESPACE}/mujoco/test_object_wrench",
                                 self._on_test_object_wrench, 10)
        self._apply_scene_service = self.create_service(
            ApplyScene, f"{SIM_NAMESPACE}/mujoco/apply_scene", self._apply_scene)
        if self.get_parameter("gui").get_parameter_value().bool_value:
            self._viewer = viewer.launch_passive(self.model, self.data)
            if scene_config.get("finger_flex_pads", {}).get("style") == "fin_ray":
                self._viewer.cam.lookat[:] = (0.55, 0.0, 0.24)
                self._viewer.cam.distance = 1.55
                self._viewer.cam.azimuth = 135
                self._viewer.cam.elevation = -22
            elif scene_config.get("finger_collision_geometry") == "convex":
                self._viewer.cam.lookat[:] = (0.88, 0.0, 0.13)
                self._viewer.cam.distance = 0.95
                self._viewer.cam.azimuth = 135
                self._viewer.cam.elevation = -20
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
        self._scene_state_pub = self.create_publisher(
            SceneObjectStates, f"{SIM_NAMESPACE}/ground_truth/scene_objects", 10)
        self._contact_pub = self.create_publisher(Bool, f"{SIM_NAMESPACE}/gripper/contact", 10)
        self._contact_state_pub = self.create_publisher(
            String, f"{SIM_NAMESPACE}/gripper/contact_state", 10)
        self._finger_visualization_pub = self.create_publisher(
            String, f"{SIM_NAMESPACE}/gripper/collision_visualization", 10)
        self._arm_diagnostics_pub = self.create_publisher(
            String, f"{SIM_NAMESPACE}/mujoco/arm_diagnostics", 10)
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
        """Collect named scalar model joints for ROS joint-state feedback.

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
            if not name:
                continue
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

    def _apply_initial_arm_pose(self, scene_config: dict[str, object]) -> None:
        """Apply optional scene-specific arm positions before the first physics step.

        Args:
            scene_config: Parsed MuJoCo scene configuration.

        Returns:
            None.
        """
        positions = scene_config.get("initial_arm_joint_positions_rad", {})
        if not isinstance(positions, dict):
            raise ValueError("initial_arm_joint_positions_rad must be a joint-to-position mapping")
        unknown = sorted(set(positions) - set(ARM_JOINTS))
        if unknown:
            raise ValueError(f"scene initial arm pose contains unknown joints: {unknown}")
        for name, raw_position in positions.items():
            position = float(raw_position)
            if not math.isfinite(position):
                raise ValueError(f"scene initial arm position for {name!r} must be finite")
            joint_id = self._joint_ids[name]
            if self.model.jnt_limited[joint_id]:
                lower, upper = self.model.jnt_range[joint_id]
                if not float(lower) <= position <= float(upper):
                    raise ValueError(
                        f"scene initial arm position for {name!r} is outside [{lower}, {upper}]")
            qpos = self._joint_qpos(name)
            qvel = int(self.model.jnt_dofadr[joint_id])
            self.data.qpos[qpos] = position
            self.data.qvel[qvel] = 0.0
            self.data.ctrl[self._actuators[name]] = position

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
                             self._camera_calibration, scene, self._scene_directory,
                             self._simplified_finger_collision,
                             self._finger_contact_box_half_size_m)
            candidate_model = mujoco.MjModel.from_xml_string(xml)
            configure_fin_ray(
                candidate_model,
                out_of_plane_stiffness_n_m=float(scene.get("finger_flex_pads", {}).get(
                    "fin_ray_out_of_plane_stiffness_n_m", 500.0)))
            candidate_data = mujoco.MjData(candidate_model)
            if request.scene_id.startswith("dynamic_grasp_"):
                # Test resets must not rewind the ROS clock while action clients
                # and TF listeners are running.
                candidate_data.time = self.data.time
            candidate_indices = self._model_indices(candidate_model)
            if scene.get("fixed_base", False):
                actual = self.data.qpos[self._root_qpos:self._root_qpos + 7]
                reference = candidate_model.qpos0[candidate_indices["root_qpos"]:
                                                  candidate_indices["root_qpos"] + 7]
                if (np.linalg.norm(actual[:3] - reference[:3]) > 1e-4 or
                        abs(float(np.dot(actual[3:], reference[3:]))) < 1.0 - 1e-6):
                    raise ValueError("fixed-base scene requires the base at its initial pose")
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
        self._fixed_base = bool(scene.get("fixed_base", False))
        self._scene_objects = list(scene.get("objects", []))
        self._test_wrench = None
        self._base_velocity = (0.0, 0.0)
        self._base_command_time = time.monotonic()
        if self._arm_trajectory is not None and self._last_arm_trajectory_event is not None:
            self._last_arm_trajectory_event = {
                **self._last_arm_trajectory_event,
                "state": "cleared_by_scene_apply",
                "cleared_simulation_time_sec": float(self.data.time),
            }
            self.get_logger().warning(
                f"MuJoCo arm trajectory cleared by scene apply: "
                f"{json.dumps(self._last_arm_trajectory_event, sort_keys=True)}")
        self._arm_trajectory = None
        self._gripper_trajectory = None
        self._next_camera = float(self.data.time)
        self._next_lidar = float(self.data.time)
        self._last_odom = float(self.data.time)
        self._last_scene_state = -1.0
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
        self._base_velocity = ((0.0, 0.0) if self._fixed_base else
                               (float(message.twist.linear.x), float(message.twist.angular.z)))
        self._base_command_time = time.monotonic()

    def _on_arm_trajectory(self, message: JointTrajectory) -> None:
        """Validate and schedule the requested arm joint trajectory.

        Args:
            message: Joint names and target positions from the arm backend.

        Returns:
            None.
        """
        previous = self._arm_trajectory
        try:
            trajectory = self._trajectory(message, ARM_JOINTS)
        except (ValueError, TypeError) as exc:
            self.get_logger().error(f"MuJoCo arm trajectory rejected: {exc}")
            raise
        self._arm_trajectory_sequence += 1
        stamp = message.header.stamp
        event = {
            "sequence": self._arm_trajectory_sequence,
            "state": "accepted" if trajectory is not None else "cleared",
            "message_stamp_sec": float(stamp.sec) + float(stamp.nanosec) * 1e-9,
            "received_simulation_time_sec": float(self.data.time),
            "point_count": len(message.points),
            "replaced_active_trajectory": previous is not None,
            "previous_sequence": (self._last_arm_trajectory_event or {}).get("sequence"),
            "duration_sec": trajectory.duration if trajectory is not None else None,
            "start_rad": dict(zip(ARM_JOINTS, trajectory.start)) if trajectory is not None else None,
            "endpoint_rad": dict(zip(ARM_JOINTS, trajectory.waypoints[-1])) if trajectory is not None else None,
        }
        self._last_arm_trajectory_event = event
        self._arm_trajectory = trajectory
        self.get_logger().info(f"MuJoCo arm trajectory received: {json.dumps(event, sort_keys=True)}")

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
        positions = [list(point.positions) for point in message.points]
        durations = [float(point.time_from_start.sec) + point.time_from_start.nanosec * 1e-9
                     for point in message.points]
        if len(durations) == 1 and durations[0] <= 0:
            durations[0] = 0.2
        measured = [self.data.qpos[self._joint_qpos(name)] for name in expected]
        return make_trajectory(message.joint_names, positions, expected, measured,
                               float(self.data.time), durations)

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
            if self.data.time - self._last_scene_state >= 0.05:
                self._publish_scene_state()
                self._last_scene_state = self.data.time
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

    def _on_test_object_wrench(self, message: WrenchStamped) -> None:
        """Accept bounded test forces only for a dynamic-grasp experiment scene.

        Args:
            message: World-frame wrench whose frame ID names a dynamic scene object.

        Returns:
            None.
        """
        if not self._scene_name.startswith("dynamic_grasp_"):
            return
        allowed = {str(item["id"]) for item in self._scene_objects if item.get("motion") == "dynamic"}
        if message.header.frame_id not in allowed:
            return
        wrench = message.wrench
        values = np.asarray([wrench.force.x, wrench.force.y, wrench.force.z,
                             wrench.torque.x, wrench.torque.y, wrench.torque.z], dtype=float)
        if not np.isfinite(values).all() or np.linalg.norm(values[:3]) > 10 or np.linalg.norm(values[3:]) > 1:
            return
        body = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, message.header.frame_id + "_body")
        if body >= 0:
            self._test_wrench = (body, values, time.monotonic())

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
        if self._test_wrench is not None:
            body, values, received = self._test_wrench
            self.data.xfrc_applied[body] = values if time.monotonic() - received <= 0.25 else 0.0
            if time.monotonic() - received > 0.25:
                self._test_wrench = None
        steps = max(1, round(dt / self.model.opt.timestep))
        mujoco.mj_step(self.model, self.data, nstep=steps)
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
                if self._last_arm_trajectory_event is not None:
                    self._last_arm_trajectory_event = {
                        **self._last_arm_trajectory_event,
                        "state": "targets_finished",
                        "finished_simulation_time_sec": float(self.data.time),
                        "finished_positions_rad": {
                            name: float(self.data.qpos[self._joint_qpos(name)]) for name in names},
                    }
                    self.get_logger().info(
                        f"MuJoCo arm trajectory targets finished: "
                        f"{json.dumps(self._last_arm_trajectory_event, sort_keys=True)}")
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

    def _publish_scene_state(self) -> None:
        """Publish simulator-only object truth and gripper contact diagnostics.

        Args:
            None.

        Returns:
            None.
        """
        stamp = _stamp(float(self.data.time))
        object_ids = {str(item["id"]) for item in self._scene_objects}
        contacts = _bilateral_finger_contacts(self.model, self.data, object_ids)
        state = SceneObjectStates()
        state.header.stamp = stamp
        state.header.frame_id = "mujoco_world"
        grasp_body = _name_id(self.model, mujoco.mjtObj.mjOBJ_BODY, "grasp_frame")
        grasp_position = self.data.xpos[grasp_body]
        grasp_quaternion = self.data.xquat[grasp_body]
        state.gripper_pose.position.x, state.gripper_pose.position.y, state.gripper_pose.position.z = (
            float(value) for value in grasp_position)
        state.gripper_pose.orientation.w = float(grasp_quaternion[0])
        state.gripper_pose.orientation.x = float(grasp_quaternion[1])
        state.gripper_pose.orientation.y = float(grasp_quaternion[2])
        state.gripper_pose.orientation.z = float(grasp_quaternion[3])
        for object_id in sorted(object_ids):
            body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, f"{object_id}_body")
            pose = Pose()
            if body_id >= 0:
                position = self.data.xpos[body_id]
                quaternion = self.data.xquat[body_id]
            else:
                geom_id = _name_id(self.model, mujoco.mjtObj.mjOBJ_GEOM, f"{object_id}_geom")
                position = self.data.geom_xpos[geom_id]
                quaternion = np.empty(4, dtype=float)
                mujoco.mju_mat2Quat(quaternion, self.data.geom_xmat[geom_id])
            pose.position.x, pose.position.y, pose.position.z = (float(value) for value in position)
            pose.orientation.w = float(quaternion[0])
            pose.orientation.x = float(quaternion[1])
            pose.orientation.y = float(quaternion[2])
            pose.orientation.z = float(quaternion[3])
            state.object_ids.append(object_id)
            state.poses.append(pose)
        state.gripper_contact_object_ids = sorted(contacts)
        state.gripper_contact = bool(contacts)
        self._scene_state_pub.publish(state)
        self._contact_pub.publish(Bool(data=bool(contacts)))
        finger_visualization = _finger_visualization(self.model, self.data)
        pad_diagnostics = [{"link": finger["link"], "contacts": finger["contacts"],
                            "pads": [{"name": pad["name"], "max_edge_length_change_m":
                                      pad["max_edge_length_change_m"]} for pad in finger["pads"]]}
                           for finger in finger_visualization["fingers"]]
        self._contact_state_pub.publish(String(data=json.dumps({
            "valid": True,
            "contact": bool(contacts),
            "object_ids": sorted(contacts),
            "normal_forces_n": _finger_contact_forces(self.model, self.data, object_ids),
            "joint_positions_rad": {name: float(self.data.qpos[self._joint_qpos(name)])
                                    for name in GRIPPER_JOINTS},
            "actuator_forces": {name: float(self.data.actuator_force[self._actuators[name]])
                                for name in GRIPPER_JOINTS},
            "fingers": pad_diagnostics,
        }, separators=(",", ":"))))
        self._finger_visualization_pub.publish(String(data=json.dumps(
            finger_visualization, separators=(",", ":"))))
        now = time.monotonic()
        if (any(finger["pads"] for finger in pad_diagnostics) and
                (contacts or self._gripper_trajectory is not None) and
                now - self._last_flex_diagnostic_log >= 0.5):
            self.get_logger().info("MuJoCo flexible finger contact: " + json.dumps({
                "simulation_time_sec": float(self.data.time), "object_ids": sorted(contacts),
                "fingers": pad_diagnostics}, separators=(",", ":")))
            self._last_flex_diagnostic_log = now
        self._publish_arm_diagnostics(stamp)

    def _publish_arm_diagnostics(self, stamp: Time) -> None:
        """Publish measured arm state, actuator effort, saturation, and contacts.

        Args:
            stamp: Current MuJoCo simulation timestamp.

        Returns:
            None.
        """
        joints = {}
        for name in ARM_JOINTS:
            actuator_id = self._actuators[name]
            joint_id = self._joint_ids[name]
            limited = bool(self.model.actuator_forcelimited[actuator_id])
            force_range = ([float(value) for value in self.model.actuator_forcerange[actuator_id]]
                           if limited else None)
            force = float(self.data.actuator_force[actuator_id])
            force_limit = max(abs(value) for value in force_range) if force_range else None
            joints[name] = {
                "target_rad": float(self.data.ctrl[actuator_id]),
                "position_rad": float(self.data.qpos[self._joint_qpos(name)]),
                "error_rad": float(self.data.ctrl[actuator_id] - self.data.qpos[self._joint_qpos(name)]),
                "velocity_rad_sec": float(self.data.qvel[self._joint_qvel(name)]),
                "actuator_force_nm": force,
                "force_range_nm": force_range,
                "force_saturated": bool(force_limit is not None and
                                         abs(force) >= 0.98 * force_limit),
                "joint_limited": bool(self.model.jnt_limited[joint_id]),
            }
        contacts = []
        for index in range(min(int(self.data.ncon), 40)):
            contact = self.data.contact[index]
            force = np.zeros(6, dtype=float)
            mujoco.mj_contactForce(self.model, self.data, index, force)
            participants = []
            for side in (0, 1):
                geom_id, flex_id = int(contact.geom[side]), int(contact.flex[side])
                if geom_id >= 0:
                    participants.append(mujoco.mj_id2name(
                        self.model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or str(geom_id))
                elif flex_id >= 0:
                    participants.append(mujoco.mj_id2name(
                        self.model, mujoco.mjtObj.mjOBJ_FLEX, flex_id) or f"flex:{flex_id}")
                else:
                    participants.append("")
            contacts.append({"geom1": participants[0], "geom2": participants[1],
                             "elements": [int(value) for value in contact.elem],
                             "distance_m": float(contact.dist),
                             "normal_force_n": float(force[0])})
        self._arm_diagnostics_pub.publish(String(data=json.dumps({
            "simulation_time_sec": float(stamp.sec) + float(stamp.nanosec) * 1e-9,
            "scene": self._scene_name,
            "arm_trajectory": self._last_arm_trajectory_event,
            "arm_trajectory_active": self._arm_trajectory is not None,
            "joints": joints,
            "contacts": contacts,
            "contact_count": int(self.data.ncon),
            "contacts_truncated": int(self.data.ncon) > len(contacts),
        }, separators=(",", ":"))))

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
