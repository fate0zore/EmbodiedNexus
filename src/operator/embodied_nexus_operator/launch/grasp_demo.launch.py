"""Start the MuJoCo grasp demo stack with all Nexus device services."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch MuJoCo, simulated devices, MoveIt2, and the grasp operator.

    Args:
        None.

    Returns:
        Launch description for the complete planned-grasp simulation stack.
    """
    sim_launch = PathJoinSubstitution([
        FindPackageShare("embodied_nexus_sim_mujoco"), "launch", "sim.launch.py"])
    camera_launch = PathJoinSubstitution([
        FindPackageShare("embodied_nexus_camera"), "launch", "camera.launch.py"])
    arm_launch = PathJoinSubstitution([
        FindPackageShare("embodied_nexus_arm"), "launch", "arm.launch.py"])
    gripper_launch = PathJoinSubstitution([
        FindPackageShare("embodied_nexus_gripper"), "launch", "gripper.launch.py"])
    moveit_launch = PathJoinSubstitution([
        FindPackageShare("embodied_nexus_operator_moveit_config"), "launch", "move_group.launch.py"])
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("scene", default_value="scene-1793463913737"),
        DeclareLaunchArgument("start_core", default_value="false"),
        DeclareLaunchArgument("simplified_finger_collision", default_value="true"),
        DeclareLaunchArgument("finger_contact_box_half_size_m", default_value="[0.010, 0.045, 0.011]"),
        DeclareLaunchArgument("operator_config", default_value=PathJoinSubstitution([
            FindPackageShare("embodied_nexus_operator"), "config", "grasp_cylinder.yaml"])),
        DeclareLaunchArgument("http_token", default_value=EnvironmentVariable(
            "EMBODIED_NEXUS_OPERATOR_HTTP_TOKEN", default_value="")),
        SetEnvironmentVariable("EMBODIED_NEXUS_OPERATOR_HTTP_TOKEN", LaunchConfiguration("http_token")),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(sim_launch),
            launch_arguments={"gui": LaunchConfiguration("gui"), "scene": LaunchConfiguration("scene"),
                              "start_core": LaunchConfiguration("start_core"),
                              "simplified_finger_collision": LaunchConfiguration(
                                  "simplified_finger_collision"),
                              "finger_contact_box_half_size_m": LaunchConfiguration(
                                  "finger_contact_box_half_size_m")}.items()),
        TimerAction(period=2.0, actions=[
            IncludeLaunchDescription(PythonLaunchDescriptionSource(arm_launch),
                launch_arguments={"config_file": PathJoinSubstitution([
                    FindPackageShare("embodied_nexus_arm"), "config", "aubo_i5_sim.yaml"]),
                    "interface_mode": "ros2", "use_sim_time": "true"}.items()),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(gripper_launch),
                launch_arguments={"config_file": PathJoinSubstitution([
                    FindPackageShare("embodied_nexus_gripper"), "config", "ms42dc_sim.yaml"]),
                    "interface_mode": "ros2", "use_sim_time": "true"}.items()),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(camera_launch),
                launch_arguments={"config_file": PathJoinSubstitution([
                    FindPackageShare("embodied_nexus_camera"), "config", "gemini335_sim.yaml"]),
                    "interface_mode": "ros2", "use_sim_time": "true"}.items()),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(moveit_launch),
                launch_arguments={"use_sim_time": "true"}.items()),
        ]),
        TimerAction(period=7.0, actions=[
            Node(package="embodied_nexus_operator", executable="grasp_operator",
                 name="embodied_nexus_grasp_operator", output="screen",
                 parameters=[{"config_file": LaunchConfiguration("operator_config"), "use_sim_time": True}]),
        ]),
    ])
