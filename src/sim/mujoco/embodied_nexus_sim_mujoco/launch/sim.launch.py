"""Launch the MuJoCo model, ROS state publisher, and optional Core manager."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

from embodied_nexus_sim_mujoco.camera_calibration import (
    apply_camera_calibration,
    load_camera_calibration,
)


def _setup(context):
    """Prepare package paths and return the MuJoCo process actions.

    Args:
        context: Launch context containing resolved launch arguments.

    Returns:
        ROS launch actions for the simulator and optional Core manager.
    """
    description_share = Path(get_package_share_directory("embodied_nexus_sim_description"))
    camera_share = Path(get_package_share_directory("embodied_nexus_camera"))
    package_share = Path(get_package_share_directory("embodied_nexus_sim_mujoco"))
    camera_calibration = load_camera_calibration(description_share, camera_share)
    from xacro import process_file

    robot_description = process_file(
        str(description_share / "urdf" / "arachne.urdf.xacro"),
        mappings={"with_mimic_joints": "false", "with_ros2_control": "false",
                  "gazebo_drive_wheels": "false", "with_ee_camera": "true", "with_lidar": "true"},
    ).toxml()
    robot_description = apply_camera_calibration(robot_description, camera_calibration)
    simulation = Node(
        package="embodied_nexus_sim_mujoco", executable="mujoco_sim", name="mujoco",
        namespace="/sim/arachne", output="screen",
        parameters=[{"gui": LaunchConfiguration("gui"), "use_sim_time": True}],
    )
    state_publisher = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        namespace="/sim/arachne", name="robot_state_publisher", output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
        remappings=[("joint_states", "/sim/arachne/joint_states"), ("tf", "/tf"),
                    ("tf_static", "/tf_static")],
    )
    start_core = LaunchConfiguration("start_core").perform(context).strip().lower() in {"true", "1", "yes"}
    actions = [simulation, state_publisher]
    if start_core:
        platform_path = str(package_share / "config" / "platform_sim.yaml")
        core = Node(
            package="embodied_nexus_core", executable="nexus_core", name="nexus_core_sim",
            arguments=["--config", platform_path], output="screen",
            parameters=[{"use_sim_time": True}],
        )
        actions.append(TimerAction(period=4.0, actions=[core]))
    return actions


def generate_launch_description():
    """Declare launch options and assemble the MuJoCo launch description.

    Args:
        None.

    Returns:
        The launch description for the MuJoCo simulation profile.
    """
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="false"),
        DeclareLaunchArgument("start_core", default_value="true"),
        DeclareLaunchArgument("core_token", default_value=EnvironmentVariable(
            "EMBODIED_NEXUS_CORE_TOKEN", default_value="sim-local-token-change-me")),
        SetEnvironmentVariable("EMBODIED_NEXUS_CORE_TOKEN", LaunchConfiguration("core_token")),
        OpaqueFunction(function=_setup),
    ])
