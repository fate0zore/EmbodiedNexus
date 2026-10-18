from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    interface_mode = LaunchConfiguration("interface_mode")
    web_condition = IfCondition(PythonExpression(["'", interface_mode, "' in ['both', 'web']"]))
    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=PathJoinSubstitution([FindPackageShare("embodied_nexus_gripper"), "config", "ms42dc.yaml"]),
        ),
        DeclareLaunchArgument("interface_mode", default_value="both"),
        Node(
            package="embodied_nexus_gripper", executable="gripper_node",
            name="embodied_nexus_gripper", output="screen",
            parameters=[{"config_file": config_file}],
        ),
        Node(
            package="embodied_nexus_gripper", executable="gripper_web",
            name="embodied_nexus_gripper_web", output="screen",
            parameters=[{"config_file": config_file}], condition=web_condition,
        ),
    ])
