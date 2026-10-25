from __future__ import annotations

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    description_package = get_package_share_directory("embodied_nexus_sim_description")
    model_file = f"{description_package}/urdf/arachne.urdf.xacro"
    robot_description = xacro.process_file(model_file).toxml()

    return LaunchDescription([
        DeclareLaunchArgument(
            "joint_states_topic",
            default_value="/embodied_nexus/arms/arm_main/joint_states",
            description="JointState topic used to animate the Aubo i5 model",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description, "use_sim_time": False}],
            remappings=[("joint_states", LaunchConfiguration("joint_states_topic"))],
        ),
    ])
