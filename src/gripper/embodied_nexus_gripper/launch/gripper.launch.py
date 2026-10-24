from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    interface_mode = LaunchConfiguration("interface_mode")
    config_file = LaunchConfiguration("config_file")
    web_condition = IfCondition(PythonExpression(["'", interface_mode, "' in ['both', 'web']"]))
    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=PathJoinSubstitution([FindPackageShare("embodied_nexus_gripper"), "config", "ms42dc.yaml"]),
        ),
        DeclareLaunchArgument("interface_mode", default_value="both"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("web_bind_host", default_value=""),
        DeclareLaunchArgument("web_port", default_value="0"),
        DeclareLaunchArgument("web_auth_token_env", default_value=""),
        DeclareLaunchArgument("node_name", default_value="embodied_nexus_gripper"),
        DeclareLaunchArgument("node_name_web", default_value="embodied_nexus_gripper_web"),
        Node(
            package="embodied_nexus_gripper", executable="gripper_node",
            name=LaunchConfiguration("node_name"), output="screen",
            parameters=[{"config_file": config_file, "interface_mode": interface_mode,
                         "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
                         "web.bind_host": LaunchConfiguration("web_bind_host"),
                         "web.port": LaunchConfiguration("web_port"),
                         "web.auth_token_env": LaunchConfiguration("web_auth_token_env")}],
        ),
        Node(
            package="embodied_nexus_gripper", executable="gripper_web",
            name=LaunchConfiguration("node_name_web"), output="screen",
            parameters=[{"config_file": config_file, "interface_mode": interface_mode,
                         "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
                         "web.bind_host": LaunchConfiguration("web_bind_host"),
                         "web.port": LaunchConfiguration("web_port"),
                         "web.auth_token_env": LaunchConfiguration("web_auth_token_env")}], condition=web_condition,
        ),
    ])
