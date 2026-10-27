import re
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from embodied_nexus_lidar.cli import check_driver_available
from embodied_nexus_lidar.config import load_config, validate_config


def _resolve_topic(topic, namespace):
    """解析话题。

    Args:
        topic: 话题，作为解析话题所需的输入。
        namespace: namespace，作为解析话题所需的输入。

    Returns:
        本次处理产生的结果值。
    """
    return topic if topic.startswith("/") else f"{namespace}/{topic.strip('/')}"


def _setup(context, *args, **kwargs):
    """设置当前设备或操作。

    Args:
        context: context，作为设置当前设备或操作所需的输入。
        *args: 额外的位置参数集合。
        **kwargs: 额外的关键字参数集合。

    Returns:
        本次处理产生的结果值。
    """
    config_file = Path(LaunchConfiguration("config_file").perform(context)).expanduser().resolve()
    config = load_config(config_file)
    mode = LaunchConfiguration("interface_mode").perform(context).strip() or config["interface_mode"]
    if mode not in {"ros2", "web", "both"}:
        raise RuntimeError("interface_mode must be ros2, web, or both")
    config["interface_mode"] = mode
    web_port = int(LaunchConfiguration("web_port").perform(context))
    if web_port > 0:
        config["web"]["port"] = web_port
    bind_host = LaunchConfiguration("web_bind_host").perform(context).strip()
    if bind_host:
        config["web"]["bind_host"] = bind_host
    token_env = LaunchConfiguration("web_auth_token_env").perform(context).strip()
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).strip().lower() in {"true", "1", "yes"}
    if token_env:
        config["web"]["auth_token_env"] = token_env
    validate_config(config)
    device_id = str(config["device_id"])
    namespace = f"/embodied_nexus/lidars/{device_id}"
    input_topic = _resolve_topic(config["source"]["pointcloud_topic"], namespace)
    output_topics = {
        _resolve_topic(config["source"]["output_topic"], namespace),
        _resolve_topic(config["source"]["scan_output_topic"], namespace),
    }
    if input_topic in output_topics:
        raise RuntimeError("resolved PointCloud2 input and output topics must be distinct")
    if mode in {"both", "web"}:
        web = config["web"]
        token = os.environ.get(str(web.get("auth_token_env", "EMBODIED_NEXUS_LIDAR_TOKEN")), "")
        if not token or not re.fullmatch(r"[A-Za-z0-9._~-]+", token):
            raise RuntimeError("Web mode requires a URL-safe token in the configured environment variable")
    nodes = [
        Node(
            package="embodied_nexus_lidar", executable="lidar_node",
            name=LaunchConfiguration("node_name"), output="screen",
            parameters=[{"config_file": str(config_file), "interface_mode": mode,
                         "web_bind_host": bind_host, "web_port": web_port,
                         "web_auth_token_env": token_env, "use_sim_time": use_sim_time}],
        )
    ]
    if config["backend"] == "c16":
        check_driver_available(config)
        driver = config["driver"]
        nodes.append(Node(
            package="lslidar_driver", executable="lslidar_driver_node",
            name="lslidar_node", namespace=f"{namespace}/_driver", output="screen",
            respawn=True, respawn_delay=2.0,
            parameters=[{
                "device_ip": str(driver.get("device_ip", "192.168.1.200")),
                "msop_port": int(driver.get("msop_port", 2368)),
                "difop_port": int(driver.get("difop_port", 2369)),
                "pcl_type": False,
                "add_multicast": bool(driver.get("add_multicast", False)),
                "group_ip": str(driver.get("group_ip", "224.1.1.2")),
                "use_time_service": False,
                "min_range": float(driver.get("min_range_m", 0.15)),
                "max_range": float(driver.get("max_range_m", 200.0)),
                "angle_disable_min": int(driver.get("angle_disable_min", 0)),
                "angle_disable_max": int(driver.get("angle_disable_max", 0)),
                "distance_unit": float(driver.get("distance_unit", 0.4)),
                "horizontal_angle_resolution": float(driver.get("horizontal_angle_resolution_deg", 0.18)),
                "frame_id": str(driver.get("frame_id", "lidar_link")),
                "topic_name": f"{namespace}/_driver/points",
                "publish_scan": False,
                "coordinate_opt": bool(driver.get("coordinate_opt", False)),
            }],
        ))
    if mode in {"both", "web"}:
        nodes.append(Node(
            package="embodied_nexus_lidar", executable="lidar_web",
            name=LaunchConfiguration("node_name_web"), output="screen",
            parameters=[{"config_file": str(config_file), "web_port": web_port,
                         "interface_mode": mode, "web_bind_host": bind_host,
                         "web_auth_token_env": token_env}],
        ))
    return nodes


def generate_launch_description():
    """生成 ROS launch 启动描述。

    Args:
        无参数。

    Returns:
        本次处理产生的结果值。
    """
    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("embodied_nexus_lidar"), "config", "c16.yaml"
            ]),
        ),
        DeclareLaunchArgument("interface_mode", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("web_port", default_value="0"),
        DeclareLaunchArgument("web_bind_host", default_value=""),
        DeclareLaunchArgument("web_auth_token_env", default_value=""),
        DeclareLaunchArgument("node_name", default_value="embodied_nexus_lidar"),
        DeclareLaunchArgument("node_name_web", default_value="embodied_nexus_lidar_web"),
        OpaqueFunction(function=_setup),
    ])
