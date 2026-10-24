from __future__ import annotations

import xml.etree.ElementTree as ET
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


SIM_NAMESPACE = "/sim/arachne"
SIM_WORLD = "embodied_nexus_workcell"


def _inject_fortress_plugins(robot_xml: str, controllers_file: str) -> str:
    robot = ET.fromstring(robot_xml)
    # Match the upstream Arachne tire contact parameters. The low lateral
    # friction lets the four fixed-direction wheels scrub during differential
    # turns instead of translating while their encoders report a rotation.
    for wheel_link in ("front_left_wheel_link", "front_right_wheel_link",
                       "rear_left_wheel_link", "rear_right_wheel_link"):
        tire = ET.SubElement(robot, "gazebo", {"reference": wheel_link})
        for name, value in (("mu1", "1.2"), ("mu2", "0.25"),
                            ("kp", "1000000.0"), ("kd", "100.0")):
            ET.SubElement(tire, name).text = value

    gazebo = ET.SubElement(robot, "gazebo")
    plugin = ET.SubElement(gazebo, "plugin", {
        "filename": "gz_ros2_control-system",
        "name": "gz_ros2_control::GazeboSimROS2ControlPlugin",
    })
    ET.SubElement(plugin, "robot_param").text = "robot_description"
    ET.SubElement(plugin, "robot_param_node").text = "robot_state_publisher"
    ET.SubElement(plugin, "parameters").text = controllers_file
    ros = ET.SubElement(plugin, "ros")
    ET.SubElement(ros, "namespace").text = SIM_NAMESPACE

    camera = ET.SubElement(robot, "gazebo", {"reference": "ee_camera_link"})
    sensor = ET.SubElement(camera, "sensor", {"name": "gemini335", "type": "rgbd_camera"})
    for name, value in (("always_on", "true"), ("update_rate", "10"),
                        ("visualize", "false"), ("topic", f"{SIM_NAMESPACE}/camera")):
        ET.SubElement(sensor, name).text = value
    camera_description = ET.SubElement(sensor, "camera")
    ET.SubElement(camera_description, "horizontal_fov").text = "1.0471975512"
    image = ET.SubElement(camera_description, "image")
    ET.SubElement(image, "width").text = "640"
    ET.SubElement(image, "height").text = "480"
    ET.SubElement(image, "format").text = "R8G8B8"
    clip = ET.SubElement(camera_description, "clip")
    ET.SubElement(clip, "near").text = "0.05"
    ET.SubElement(clip, "far").text = "5.0"

    lidar_mount = ET.SubElement(robot, "gazebo", {"reference": "lidar_link"})
    for sensor_name, half, yaw in (("lslidar_c16_front", "front", "0"),
                                   ("lslidar_c16_rear", "rear", "3.1415926536")):
        lidar = ET.SubElement(lidar_mount, "sensor", {"name": sensor_name, "type": "gpu_lidar"})
        # Two 180-degree GPU ray sensors preserve a full 360-degree view in Fortress.
        ET.SubElement(lidar, "pose").text = f"0 0 0 0 0 {yaw}"
        for name, value in (("always_on", "true"), ("update_rate", "10"),
                            ("visualize", "false"),
                            ("topic", f"{SIM_NAMESPACE}/lidar_{half}")):
            ET.SubElement(lidar, name).text = value
        lidar_description = ET.SubElement(lidar, "lidar")
        scan = ET.SubElement(lidar_description, "scan")
        horizontal = ET.SubElement(scan, "horizontal")
        for name, value in (("samples", "180"), ("resolution", "1"),
                            ("min_angle", "-1.5707963268"), ("max_angle", "1.5707963268")):
            ET.SubElement(horizontal, name).text = value
        vertical = ET.SubElement(scan, "vertical")
        for name, value in (("samples", "16"), ("resolution", "1"),
                            ("min_angle", "-0.2617993878"), ("max_angle", "0.2617993878")):
            ET.SubElement(vertical, name).text = value
        ranges = ET.SubElement(lidar_description, "range")
        for name, value in (("min", "0.15"), ("max", "18.0"), ("resolution", "0.01")):
            ET.SubElement(ranges, name).text = value
    return ET.tostring(robot, encoding="unicode")


def _setup(context, *args, **kwargs):
    gazebo_share = get_package_share_directory("embodied_nexus_sim_gazebo")
    model_share = get_package_share_directory("embodied_nexus_sim_description")
    model_file = f"{model_share}/urdf/arachne.urdf.xacro"
    controller_file = f"{gazebo_share}/config/controllers.yaml"
    description = xacro.process_file(model_file, mappings={
        "with_ros2_control": "true",
        "ros2_control_hardware_plugin": "gz_ros2_control/GazeboSimSystem",
        "with_mimic_joints": "false",
        "gazebo_drive_wheels": "true",
        "with_ee_camera": "true",
        "with_lidar": "true",
        "lidar_parent": "base_link",
        "lidar_xyz": "0.6 0 0.6",
        "lidar_rpy": "0 0 0",
    }).toxml()
    robot_description = _inject_fortress_plugins(description, controller_file)
    world = f"{gazebo_share}/worlds/workcell.sdf"
    gui = LaunchConfiguration("gui").perform(context).strip().lower() in {"true", "1", "yes"}
    # Fortress otherwise selects OGRE2 for the GUI renderer even though the
    # WSL sensor system below is configured for OGRE1; both paths need the
    # software-compatible renderer on this WSL profile.
    gz_args = f"-r -v 3 --render-engine ogre {' ' if gui else '-s '}{world}"
    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f"{get_package_share_directory('ros_gz_sim')}/launch/gz_sim.launch.py"),
        launch_arguments={"gz_args": gz_args}.items(),
    )
    state_publisher = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        namespace=SIM_NAMESPACE, name="robot_state_publisher", output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
        remappings=[("joint_states", f"{SIM_NAMESPACE}/joint_states"),
                    ("tf", "/tf"), ("tf_static", "/tf_static")],
    )
    bridge_topics = [
        "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
        f"{SIM_NAMESPACE}/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image",
        f"{SIM_NAMESPACE}/camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image",
        f"{SIM_NAMESPACE}/lidar_front/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked",
        f"{SIM_NAMESPACE}/lidar_rear/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked",
        f"/world/{SIM_WORLD}/control@ros_gz_interfaces/srv/ControlWorld",
    ]
    bridge = Node(package="ros_gz_bridge", executable="parameter_bridge",
                  name="sim_sensor_bridge", arguments=bridge_topics, output="screen")
    clock_guard = Node(package="embodied_nexus_sim_gazebo", executable="sim_clock_guard",
                       name="sim_clock_guard", arguments=["--world-name", SIM_WORLD], output="screen")
    relay = Node(package="embodied_nexus_sim_gazebo", executable="sim_sensor_relay",
                 name="sim_sensor_relay", output="screen")
    camera_info = [
        Node(package="embodied_nexus_sim_gazebo", executable="sim_camera_info",
             name=f"sim_camera_info_{channel}", arguments=["--channel", channel], output="screen")
        for channel in ("color", "depth")
    ]
    spawn = Node(package="ros_gz_sim", executable="create", name="spawn_arachne",
                 arguments=["-topic", "/sim/arachne/robot_description", "-name", "arachne",
                            "-x", "0", "-y", "0", "-z", "0.236"], output="screen")
    manager = f"{SIM_NAMESPACE}/controller_manager"
    broadcaster = Node(package="controller_manager", executable="spawner",
                       arguments=["joint_state_broadcaster", "-c", manager,
                                  "--controller-manager-timeout", "120"], output="screen")
    drive = Node(package="controller_manager", executable="spawner",
                 arguments=["base_controller", "-c", manager,
                            "--controller-manager-timeout", "120"], output="screen")
    arm = Node(package="controller_manager", executable="spawner",
               arguments=["arm_controller", "-c", manager,
                          "--controller-manager-timeout", "120"], output="screen")
    gripper = Node(package="controller_manager", executable="spawner",
                   arguments=["gripper_controller", "-c", manager,
                              "--controller-manager-timeout", "120"], output="screen")
    start_controllers = RegisterEventHandler(OnProcessExit(
        target_action=broadcaster, on_exit=[drive, arm, gripper]))
    enable_core = TimerAction(period=12.0, actions=[Node(
        package="embodied_nexus_core", executable="nexus_core", name="nexus_core_sim",
        arguments=["--config", f"{gazebo_share}/config/platform_sim.yaml"], output="screen",
    )])
    return [AppendEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", os.path.dirname(model_share)),
            gz_launch, state_publisher, bridge, clock_guard, relay, *camera_info,
            TimerAction(period=3.0, actions=[spawn]),
            RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[broadcaster])),
            start_controllers, enable_core]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("core_token", default_value="sim-local-token-change-me"),
        SetEnvironmentVariable("EMBODIED_NEXUS_CORE_TOKEN", LaunchConfiguration("core_token")),
        OpaqueFunction(function=_setup),
    ])
