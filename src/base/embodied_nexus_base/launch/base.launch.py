from __future__ import annotations

import os
from pathlib import Path
import re

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from embodied_nexus_base.config import load_config


def _setup(context, *args, **kwargs):
    config_path = Path(LaunchConfiguration("config_file").perform(context)).expanduser().resolve()
    config = load_config(config_path)
    interface_mode = LaunchConfiguration("interface_mode").perform(context).strip() or config["interface_mode"]
    if interface_mode not in {"ros2", "web", "both"}:
        raise RuntimeError("interface_mode must be ros2, web or both")
    actions = [Node(package="embodied_nexus_base", executable="base_node",
                    name="embodied_nexus_base", output="screen",
                    parameters=[{"config_file": str(config_path)}])]
    if interface_mode in {"both", "web"}:
        web = config["web"]
        token = os.environ.get(str(web["auth_token_env"]), "")
        if not token or not re.fullmatch(r"[A-Za-z0-9._~-]+", token):
            raise RuntimeError(f"Web mode requires a URL-safe token in {web['auth_token_env']}")
        actions.append(Node(package="embodied_nexus_base", executable="base_web",
                            name="embodied_nexus_base_web", output="screen",
                            parameters=[{"config_file": str(config_path)}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=PathJoinSubstitution([
            FindPackageShare("embodied_nexus_base"), "config", "scout_mock.yaml"])),
        DeclareLaunchArgument("interface_mode", default_value=""),
        OpaqueFunction(function=_setup),
    ])
