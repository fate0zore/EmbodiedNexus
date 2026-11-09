"""Launch the independent visual servo and optional MuJoCo device stack."""

from pathlib import Path
import json
import xml.etree.ElementTree as ET

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def _moveit_parameters(context):
    """Build matching planning and Servo collision geometry.

    Args:
        context: Launch context.

    Returns:
        MoveIt parameters for the selected simulation scene.
    """
    description = Path(get_package_share_directory("embodied_nexus_sim_description"))
    config = (MoveItConfigsBuilder("arachne", package_name="embodied_nexus_operator_moveit_config")
              .robot_description(file_path=str(description / "urdf/arachne.urdf.xacro"),
                                 mappings={"with_mimic_joints": "false", "with_ros2_control": "false",
                                           "gazebo_drive_wheels": "false", "with_ee_camera": "true",
                                           "with_lidar": "true"})
              .robot_description_semantic(file_path="config/arachne.srdf")
              .robot_description_kinematics(file_path="config/kinematics.yaml")
              .joint_limits(file_path="config/joint_limits.yaml")
              .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
              .trajectory_execution(file_path="config/moveit_controllers.yaml", moveit_manage_controllers=False)
              .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
              .to_moveit_configs())
    parameters = config.to_dict()
    if LaunchConfiguration("contact_finger_boxes").perform(context).lower() == "true":
        scene_directory = LaunchConfiguration("scene_directory").perform(context).strip()
        scene_name = LaunchConfiguration("scene").perform(context).strip()
        if not scene_directory:
            scene_directory = str(Path(get_package_share_directory("embodied_nexus_sim_mujoco")) / "config/scenes")
        scene = yaml.safe_load((Path(scene_directory) / f"{scene_name}.yaml").read_text(encoding="utf-8"))
        geometry = scene.get("finger_collision_geometry")
        if geometry not in ("boxes", "convex") or scene.get("finger_flex_pads", {}).get("enabled"):
            raise RuntimeError("contact_finger_boxes requires rigid finger contact boxes")
        robot = ET.fromstring(parameters["robot_description"])
        for side in ("left", "right"):
            link = robot.find(f"link[@name='ms42dc_{side}_finger_link']")
            if link is None:
                raise RuntimeError(f"missing {side} finger link")
            original = link.find("collision")
            original_origin = original.find("origin") if original is not None else None
            for collision in list(link.findall("collision")):
                link.remove(collision)
            if geometry == "convex":
                convex_dir = description / "meshes/gripper/ms42dc/convex"
                manifest = json.loads((convex_dir / "manifest.json").read_text(encoding="utf-8"))
                for index, filename in enumerate(manifest["fingers"][side]["parts"]):
                    collision = ET.SubElement(link, "collision", {"name": f"{side}_convex_{index}"})
                    if original_origin is not None:
                        ET.SubElement(collision, "origin", dict(original_origin.attrib))
                    shape = ET.SubElement(collision, "geometry")
                    ET.SubElement(shape, "mesh", {"filename": (convex_dir / filename).as_uri()})
                continue
            for index, part in enumerate(scene["finger_contact_boxes"][side]):
                collision = ET.SubElement(link, "collision", {"name": f"{side}_contact_box_{index}"})
                ET.SubElement(collision, "origin", {"xyz": " ".join(map(str, part["position"])),
                                                     "rpy": " ".join(map(str, part["rotation"]))})
                geometry = ET.SubElement(collision, "geometry")
                ET.SubElement(geometry, "box", {"size": " ".join(str(2 * value) for value in part["half_size"])})
        parameters["robot_description"] = ET.tostring(robot, encoding="unicode")
    return parameters


def _move_group(context):
    """Launch collision-checked planning with the selected geometry.

    Args:
        context: Launch context.

    Returns:
        List containing the planning-only MoveGroup node.
    """
    return [Node(package="moveit_ros_move_group", executable="move_group", output="screen",
                 parameters=[_moveit_parameters(context), {"use_sim_time": True, "allow_trajectory_execution": False}],
                 remappings=[("joint_states", "/sim/arachne/joint_states")])]


def _servo(context):
    """Create MoveIt Servo with the selected collision geometry.

    Args:
        context: Launch context.

    Returns:
        List containing the configured MoveIt Servo node.
    """
    parameters = _moveit_parameters(context)
    # Humble's Servo IK path permits approximate KDL solutions on every cycle;
    # their residual can accumulate even for a zero Cartesian component. Use
    # its Jacobian path for velocity commands while MoveGroup retains KDL.
    parameters.pop("robot_description_kinematics", None)
    servo_path = Path(get_package_share_directory("embodied_nexus_visual_servo")) / "config/servo.yaml"
    servo = yaml.safe_load(servo_path.read_text(encoding="utf-8"))
    return [Node(package="moveit_servo", executable="servo_node_main", name="visual_servo_moveit",
                 output="screen", parameters=[parameters, {"moveit_servo": servo, "use_sim_time": True}]),
            Node(package="embodied_nexus_visual_servo_filter", executable="collision_probe",
                 condition=IfCondition(LaunchConfiguration("collision_diagnostics")), output="screen",
                 parameters=[parameters, {"use_sim_time": True}])]


def generate_launch_description():
    """Launch simulation, public device nodes, planning and visual servo nodes.

    Returns:
        ROS launch description for the local MuJoCo demonstration.
    """
    def include(package, filename, arguments):
        """Include a package launch file.

        Args:
            package: ROS package name.
            filename: Launch file name.
            arguments: Mapping of launch argument names to values.

        Returns:
            Included launch action.
        """
        path = PathJoinSubstitution([FindPackageShare(package), "launch", filename])
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(path), launch_arguments=arguments.items())

    devices = []
    for package, config in (("embodied_nexus_arm", "aubo_i5_sim.yaml"),
                            ("embodied_nexus_gripper", "ms42dc_sim.yaml"),
                            ("embodied_nexus_camera", "gemini335_sim.yaml")):
        servo_configs = {"embodied_nexus_arm": "arm_sim_servo.yaml",
                         "embodied_nexus_gripper": "gripper_sim_servo.yaml"}
        config_path = (PathJoinSubstitution([FindPackageShare("embodied_nexus_visual_servo"), "config",
                                              servo_configs[package]]) if package in servo_configs
                       else PathJoinSubstitution([FindPackageShare(package), "config", config]))
        devices.append(GroupAction(scoped=True, actions=[
            include(package, package.removeprefix("embodied_nexus_") + ".launch.py",
                    {"config_file": config_path, "interface_mode": "ros2", "use_sim_time": "true"})]))
    devices.append(OpaqueFunction(function=_move_group))
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="false"),
        DeclareLaunchArgument("start_core", default_value="false"),
        DeclareLaunchArgument("scene", default_value="visual_servo_contact"),
        DeclareLaunchArgument("scene_directory", default_value=""),
        DeclareLaunchArgument("target_class", default_value="green cylinder"),
        DeclareLaunchArgument("collision_diagnostics", default_value="false"),
        DeclareLaunchArgument("contact_finger_boxes", default_value="true"),
        include("embodied_nexus_sim_mujoco", "sim.launch.py",
                {"gui": LaunchConfiguration("gui"), "scene": LaunchConfiguration("scene"),
                 "scene_directory": LaunchConfiguration("scene_directory"),
                 "start_core": LaunchConfiguration("start_core"), "simplified_finger_collision": "true"}),
        TimerAction(period=2.0, actions=devices),
        TimerAction(period=7.0, actions=[OpaqueFunction(function=_servo),
            Node(package="embodied_nexus_visual_servo", executable="visual_servo_perception",
                 output="screen", parameters=[{"use_sim_time": True,
                                                "target_class": LaunchConfiguration("target_class")}]),
            Node(package="embodied_nexus_visual_servo", executable="visual_servo_grasp",
                 output="screen", parameters=[{"use_sim_time": True,
                                                "target_class": LaunchConfiguration("target_class")}])]),
    ])
