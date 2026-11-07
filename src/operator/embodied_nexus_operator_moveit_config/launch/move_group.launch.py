from pathlib import Path
import math
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def _move_group(context):
    """Build MoveGroup parameters, optionally including the green scene pad envelope.

    Args:
        context: Launch context containing the pad option.

    Returns:
        List containing the planning-only MoveGroup node.
    """
    description = Path(get_package_share_directory("embodied_nexus_sim_description"))
    moveit_config = (
        MoveItConfigsBuilder("arachne", package_name="embodied_nexus_operator_moveit_config")
        .robot_description(
            file_path=str(description / "urdf" / "arachne.urdf.xacro"),
            mappings={"with_mimic_joints": "false", "with_ros2_control": "false",
                      "gazebo_drive_wheels": "false", "with_ee_camera": "true", "with_lidar": "true"},
        )
        .robot_description_semantic(file_path="config/arachne.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .trajectory_execution(file_path="config/moveit_controllers.yaml",
                              moveit_manage_controllers=False)
        .planning_scene_monitor(publish_robot_description=True,
                                publish_robot_description_semantic=True)
        .to_moveit_configs()
    )
    parameters = moveit_config.to_dict()
    if LaunchConfiguration("soft_finger_pads").perform(context).lower() == "true":
        robot = ET.fromstring(parameters["robot_description"])
        for side, sign in (("left", 1), ("right", -1)):
            link = robot.find(f"link[@name='ms42dc_{side}_finger_link']")
            if link is None:
                raise RuntimeError(f"missing {side} finger link")
            for index, (base_x, base_y, yaw, half_x, half_y, half_z) in enumerate((
                (sign * 0.003158, 0.007336, 0.0, 0.018825, 0.019796, 0.0114),
                (-sign * 0.013884, 0.056208, sign * 0.381773, 0.009478, 0.032755, 0.008627),
            )):
                distance = sign * (half_x + 0.003 / 2)
                x = base_x + math.cos(yaw) * distance
                y = base_y + math.sin(yaw) * distance
                collision = ET.SubElement(link, "collision", {"name": f"{side}_soft_pad_envelope_{index}"})
                ET.SubElement(collision, "origin", {"xyz": f"{x} {y} -0.0112", "rpy": f"0 0 {yaw}"})
                geometry = ET.SubElement(collision, "geometry")
                ET.SubElement(geometry, "box", {"size": f"0.0034 {2 * half_y + 0.0004} {2 * half_z + 0.0004}"})
        parameters["robot_description"] = ET.tostring(robot, encoding="unicode")
    return [
        Node(package="moveit_ros_move_group", executable="move_group", output="screen",
             parameters=[parameters, {
                 "use_sim_time": LaunchConfiguration("use_sim_time"),
                 # MoveIt provides plans; Nexus owns and executes device trajectories.
                 "allow_trajectory_execution": False,
             }],
             remappings=[("joint_states", "/sim/arachne/joint_states")]),
    ]


def generate_launch_description():
    """Return the MoveGroup launch description.

    Returns:
        Launch description for the MuJoCo planning-only MoveGroup server.
    """
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("soft_finger_pads", default_value="false"),
        OpaqueFunction(function=_move_group),
    ])
