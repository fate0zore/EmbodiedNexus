from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("embodied_nexus_camera"), "config", "gemini335.yaml"
            ]),
        ),
        DeclareLaunchArgument("interface_mode", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("web_bind_host", default_value=""),
        DeclareLaunchArgument("web_port", default_value="0"),
        DeclareLaunchArgument("web_auth_token_env", default_value=""),
        DeclareLaunchArgument("node_name", default_value="embodied_nexus_camera"),
        Node(
            package="embodied_nexus_camera",
            executable="camera_node",
            name=LaunchConfiguration("node_name"),
            output="screen",
            parameters=[{
                "config_file": LaunchConfiguration("config_file"),
                "interface_mode": LaunchConfiguration("interface_mode"),
                "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
                "web.bind_host": LaunchConfiguration("web_bind_host"),
                "web.port": LaunchConfiguration("web_port"),
                "web.auth_token_env": LaunchConfiguration("web_auth_token_env"),
            }],
        ),
    ])
