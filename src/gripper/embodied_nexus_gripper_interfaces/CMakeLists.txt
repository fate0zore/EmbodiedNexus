cmake_minimum_required(VERSION 3.8)
project(embodied_nexus_gripper_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GripperState.msg"
  "srv/GetGripperInfo.srv"
  "srv/GetGripperState.srv"
  "srv/GetGripperConfiguration.srv"
  "srv/ValidateGripperConfiguration.srv"
  "srv/ApplyGripperConfiguration.srv"
  "srv/AcquireGripperControl.srv"
  "srv/RenewGripperControl.srv"
  "srv/ReleaseGripperControl.srv"
  "srv/SetGripperMotionEnabled.srv"
  "srv/StopGripper.srv"
  "action/MoveGripper.action"
  DEPENDENCIES action_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
