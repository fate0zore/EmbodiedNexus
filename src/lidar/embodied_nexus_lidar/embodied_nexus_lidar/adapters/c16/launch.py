"""C16-specific ROS driver launch configuration."""

from launch_ros.actions import Node

from ...cli import check_driver_available


def create_driver_node(config, namespace):
    """Construct the C16 driver process after checking device availability.

    Args:
        config: Validated lidar configuration.
        namespace: Device-specific ROS namespace.

    Returns:
        ROS launch node for the C16 driver.
    """
    check_driver_available(config)
    driver = config["driver"]
    return Node(
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
    )
