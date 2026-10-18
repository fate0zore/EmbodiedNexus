from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("embodied_nexus_camera"), "config", "gemini335.yaml"
            ]),
        ),
        DeclareLaunchArgument("interface_mode", default_value=""),
        DeclareLaunchArgument("web_bind_host", default_value=""),
        DeclareLaunchArgument("web_port", default_value="0"),
        Node(
            package="embodied_nexus_camera",
            executable="camera_node",
            name="embodied_nexus_camera",
            output="screen",
            parameters=[{
                "config_file": LaunchConfiguration("config_file"),
                "interface_mode": LaunchConfiguration("interface_mode"),
                "web.bind_host": LaunchConfiguration("web_bind_host"),
                "web.port": LaunchConfiguration("web_port"),
            }],
        ),
    ])
