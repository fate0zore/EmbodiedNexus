from __future__ import annotations

import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


CAMERA_FROM_OPTICAL = (
    (0.0, 0.0, 1.0),
    (-1.0, 0.0, 0.0),
    (0.0, -1.0, 0.0),
)


def _apply_camera_extrinsics(robot_xml: str, extrinsics: dict) -> str:
    robot = ET.fromstring(robot_xml)
    optical_rotation = [row[:3] for row in extrinsics["matrix_4x4"][:3]]
    optical_to_camera = tuple(zip(*CAMERA_FROM_OPTICAL))
    camera_rotation = [
        [sum(optical_rotation[row][k] * optical_to_camera[k][col] for k in range(3))
         for col in range(3)]
        for row in range(3)
    ]

    pitch = math.atan2(-camera_rotation[2][0], math.hypot(camera_rotation[0][0], camera_rotation[1][0]))
    if abs(math.cos(pitch)) < 1e-8:
        roll = 0.0
        yaw = math.atan2(-camera_rotation[0][1], camera_rotation[1][1])
    else:
        roll = math.atan2(camera_rotation[2][1], camera_rotation[2][2])
        yaw = math.atan2(camera_rotation[1][0], camera_rotation[0][0])
    xyz = [extrinsics["matrix_4x4"][index][3] for index in range(3)]
    rpy = (roll, pitch, yaw)

    for joint_name in ("camera_color_frame_joint", "camera_depth_frame_joint"):
        joint = robot.find(f"joint[@name='{joint_name}']")
        if joint is None or joint.find("parent").get("link") != extrinsics["parent_frame"]:
            raise ValueError(f"{joint_name} does not use camera parent {extrinsics['parent_frame']}")
        origin = joint.find("origin")
        if origin is None:
            origin = ET.SubElement(joint, "origin")
        origin.set("xyz", " ".join(f"{float(value):.12g}" for value in xyz))
        origin.set("rpy", " ".join(f"{value:.12g}" for value in rpy))
    return ET.tostring(robot, encoding="unicode")


def generate_launch_description():
    description_package = get_package_share_directory("embodied_nexus_sim_description")
    model_file = f"{description_package}/urdf/arachne.urdf.xacro"
    camera_extrinsics = json.loads(
        (Path(description_package) / "config/gemini335_extrinsics.json").read_text(encoding="utf-8")
    )
    robot_description = _apply_camera_extrinsics(
        xacro.process_file(model_file).toxml(), camera_extrinsics,
    )

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
