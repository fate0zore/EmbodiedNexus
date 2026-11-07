from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch the grasp action server and its authenticated HTTP trigger.

    Args:
        None.

    Returns:
        Launch description for the algorithm operator.
    """
    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=PathJoinSubstitution([
            FindPackageShare("embodied_nexus_operator"), "config", "grasp_cylinder.yaml"])),
        Node(package="embodied_nexus_operator", executable="grasp_operator",
             name="embodied_nexus_grasp_operator", output="screen",
             parameters=[{"config_file": LaunchConfiguration("config_file"), "use_sim_time": True}]),
    ])
