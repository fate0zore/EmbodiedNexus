from __future__ import annotations

import xml.etree.ElementTree as ET
import os
import tempfile
import json
import math
from pathlib import Path

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable, TimerAction
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from embodied_nexus_sim_gazebo.camera_geometry import camera_geometry, pose_xyz_rpy


SIM_NAMESPACE = "/sim/arachne"
SIM_WORLD = "embodied_nexus_workcell"


PERFORMANCE_PROFILES = {
    "balanced": {"width": 320, "height": 240, "rate": 5},
    "full": {"width": 640, "height": 480, "rate": 10},
}


def _inject_fortress_plugins(robot_xml: str, controllers_file: str, profile: str = "balanced") -> str:
    """为机器人描述注入 Gazebo Fortress 插件配置。

    Args:
        robot_xml: robotxml，作为执行 _inject_fortress_plugins 对应的处理流程所需的输入。 类型为 str。
        controllers_file: controllers文件，作为执行 _inject_fortress_plugins 对应的处理流程所需的输入。 类型为 str。
        profile: profile，作为执行 _inject_fortress_plugins 对应的处理流程所需的输入。 类型为 str。

    Returns:
        本次处理产生的结果值。 返回类型为 str。
    """
    settings = PERFORMANCE_PROFILES[profile]
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

    geometry = camera_geometry(settings["width"], settings["height"])
    for channel, sensor_type, topic in (("color", "camera", "image"), ("depth", "depth_camera", "depth_image")):
        xyz, rpy = pose_xyz_rpy(geometry[channel]["pose"])
        # Update the URDF and the actual Gazebo sensor together. Merely changing
        # the outgoing image frame_id would leave the rendered view incorrect.
        joint = robot.find(f"joint[@name='camera_{channel}_frame_joint']")
        if joint is not None:
            joint.find("parent").set("link", geometry["mount_parent"])
            origin = joint.find("origin")
            origin.set("xyz", " ".join(map(str, xyz)))
            origin.set("rpy", " ".join(map(str, rpy)))
        camera = ET.SubElement(robot, "gazebo", {"reference": geometry["mount_parent"]})
        sensor = ET.SubElement(camera, "sensor", {"name": f"gemini335_{channel}", "type": sensor_type})
        ET.SubElement(sensor, "pose").text = " ".join(map(str, [*xyz, *rpy]))
        for name, value in (("always_on", "true"), ("update_rate", str(settings["rate"])),
                            ("visualize", "false"), ("topic", f"{SIM_NAMESPACE}/camera/{topic}")):
            ET.SubElement(sensor, name).text = value
        camera_description = ET.SubElement(sensor, "camera")
        intrinsic = geometry[channel]["intrinsic"]
        ET.SubElement(camera_description, "horizontal_fov").text = str(2 * math.atan(settings["width"] / (2 * intrinsic["fx"])))
        lens = ET.SubElement(camera_description, "lens")
        ET.SubElement(lens, "type").text = "gnomonical"
        projection = ET.SubElement(lens, "intrinsics")
        for name in ("fx", "fy", "cx", "cy"):
            ET.SubElement(projection, name).text = str(intrinsic[name])
        ET.SubElement(projection, "s").text = "0"
        image = ET.SubElement(camera_description, "image")
        ET.SubElement(image, "width").text = str(settings["width"])
        ET.SubElement(image, "height").text = str(settings["height"])
        ET.SubElement(image, "format").text = "R8G8B8" if channel == "color" else "R_FLOAT32"
        clip = ET.SubElement(camera_description, "clip")
        ET.SubElement(clip, "near").text = "0.05"
        ET.SubElement(clip, "far").text = "5.0"

    lidar_mount = ET.SubElement(robot, "gazebo", {"reference": "lidar_link"})
    for sensor_name, half, yaw in (("lslidar_c16_front", "front", "0"),
                                   ("lslidar_c16_rear", "rear", "3.1415926536")):
        lidar = ET.SubElement(lidar_mount, "sensor", {"name": sensor_name, "type": "gpu_lidar"})
        # Two 180-degree GPU ray sensors preserve a full 360-degree view in Fortress.
        ET.SubElement(lidar, "pose").text = f"0 0 0 0 0 {yaw}"
        for name, value in (("always_on", "true"), ("update_rate", str(settings["rate"])),
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
    """设置当前设备或操作。

    Args:
        context: context，作为设置当前设备或操作所需的输入。
        *args: 额外的位置参数集合。
        **kwargs: 额外的关键字参数集合。

    Returns:
        本次处理产生的结果值。
    """
    profile = LaunchConfiguration("performance").perform(context)
    if profile not in PERFORMANCE_PROFILES:
        raise ValueError("performance must be balanced or full")
    settings = PERFORMANCE_PROFILES[profile]
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
        # Keep the shared model's lidar mount on top of rear_rack_link so
        # the visual, TF and GPU sensors describe the same installation.
    }).toxml()
    robot_description = _inject_fortress_plugins(description, controller_file, profile)
    world = f"{gazebo_share}/worlds/workcell.sdf"
    temporary_world = tempfile.TemporaryDirectory(prefix="nexus-gazebo-")
    if profile == "balanced":
        world_xml = ET.parse(world)
        for shadow in world_xml.findall(".//shadows") + world_xml.findall(".//cast_shadows"):
            shadow.text = "false"
        world = os.path.join(temporary_world.name, "workcell.sdf")
        world_xml.write(world, encoding="unicode")
    cleanup_world = RegisterEventHandler(OnShutdown(on_shutdown=[
        OpaqueFunction(function=lambda context: temporary_world.cleanup() or []),
    ]))
    platform_file = f"{gazebo_share}/config/platform_sim.yaml"
    # Keep public capture metadata and calibration consistent with rendered images.
    platform = yaml.safe_load(Path(platform_file).read_text(encoding="utf-8"))
    camera_file = Path(get_package_share_directory("embodied_nexus_camera")) / "config/gemini335_sim.yaml"
    camera_config = yaml.safe_load(camera_file.read_text(encoding="utf-8"))
    calibration_file = Path(temporary_world.name) / "camera_calibration.json"
    calibration_file.write_text(json.dumps(camera_geometry(settings["width"], settings["height"])["calibration"]), encoding="utf-8")
    camera_config["calibration"]["factory_file"] = str(calibration_file)
    for channel in ("color", "depth"):
        for field in ("width", "height"):
            camera_config["capture"][f"{channel}_{field}"] = settings[field]
        camera_config["capture"][f"{channel}_fps"] = settings["rate"]
        camera_config["streams"][f"{channel}_output_fps"] = settings["rate"]
        camera_config["streams"][f"web_{channel}_fps"] = settings["rate"]
    camera_file = Path(temporary_world.name) / "camera.yaml"
    camera_file.write_text(yaml.safe_dump(camera_config), encoding="utf-8")
    for device in platform["devices"]:
        if device["type"] == "camera":
            device["config"] = str(camera_file)
    platform_file = str(Path(temporary_world.name) / "platform.yaml")
    Path(platform_file).write_text(yaml.safe_dump(platform), encoding="utf-8")
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
             name=f"sim_camera_info_{channel}", arguments=["--channel", channel,
                "--width", str(settings["width"]), "--height", str(settings["height"]),
                "--rate", str(settings["rate"])], output="screen")
        for channel in ("color", "depth")
    ]
    spawn = Node(package="ros_gz_sim", executable="create", name="spawn_arachne",
                 arguments=["-topic", "/sim/arachne/robot_description", "-name", "arachne",
                            "-x", "0", "-y", "0", "-z", "0.236"], output="screen")
    # Start the bridge after the model has been inserted. Fortress GPU sensors
    # register their transport publishers during entity creation; starting the
    # bridge before that can leave it with ROS subscriptions but no GZ inputs.
    bridge_after_spawn = RegisterEventHandler(OnProcessExit(
        target_action=spawn,
        on_exit=[TimerAction(period=2.0, actions=[bridge])],
    ))
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
        arguments=["--config", platform_file], output="screen",
    )])
    return [cleanup_world, AppendEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", os.path.dirname(model_share)),
            gz_launch, state_publisher, clock_guard, relay, *camera_info, bridge_after_spawn,
            TimerAction(period=3.0, actions=[spawn]),
            RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[broadcaster])),
            start_controllers, enable_core]


def generate_launch_description():
    """生成 ROS launch 启动描述。

    Args:
        无参数。

    Returns:
        本次处理产生的结果值。
    """
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("performance", default_value="balanced"),
        DeclareLaunchArgument("core_token", default_value="sim-local-token-change-me"),
        SetEnvironmentVariable("EMBODIED_NEXUS_CORE_TOKEN", LaunchConfiguration("core_token")),
        OpaqueFunction(function=_setup),
    ])
