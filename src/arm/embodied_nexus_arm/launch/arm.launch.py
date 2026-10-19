from __future__ import annotations

import os
import re
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from embodied_nexus_arm.config import load_config


def _setup(context, *args, **kwargs):
    config_path = Path(LaunchConfiguration("config_file").perform(context)).expanduser().resolve()
    config = load_config(config_path)
    interface_mode = LaunchConfiguration("interface_mode").perform(context).strip() or config["interface_mode"]
    if interface_mode not in {"ros2", "web", "both"}:
        raise RuntimeError("interface_mode must be ros2, web or both")
    actions = [Node(package="embodied_nexus_arm", executable="arm_node",
                    name=LaunchConfiguration("node_name"), output="screen",
                    parameters=[{"config_file": str(config_path)}])]
    if interface_mode in {"both", "web"}:
        web = config["web"]
        bind_host = LaunchConfiguration("web_bind_host").perform(context).strip() or str(web["bind_host"])
        web_port = int(LaunchConfiguration("web_port").perform(context)) or int(web["port"])
        token_env = LaunchConfiguration("web_auth_token_env").perform(context).strip() or str(web["auth_token_env"])
        token = os.environ.get(token_env, "")
        if not token or not re.fullmatch(r"[A-Za-z0-9._~-]+", token):
            raise RuntimeError(f"Web mode requires a URL-safe token in {token_env}")
        actions.append(Node(package="embodied_nexus_arm", executable="arm_web",
                            name=LaunchConfiguration("node_name_web"), output="screen",
                            parameters=[{"config_file": str(config_path), "web.bind_host": bind_host,
                                         "web.port": web_port, "web.auth_token_env": token_env}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=PathJoinSubstitution([
            FindPackageShare("embodied_nexus_arm"), "config", "aubo_i5_mock.yaml"])),
        DeclareLaunchArgument("interface_mode", default_value=""),
        DeclareLaunchArgument("web_bind_host", default_value=""),
        DeclareLaunchArgument("web_port", default_value="0"),
        DeclareLaunchArgument("web_auth_token_env", default_value=""),
        DeclareLaunchArgument("node_name", default_value="embodied_nexus_arm"),
        DeclareLaunchArgument("node_name_web", default_value="embodied_nexus_arm_web"),
        OpaqueFunction(function=_setup),
    ])
