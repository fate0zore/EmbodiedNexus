cmake_minimum_required(VERSION 3.8)
project(embodied_nexus_base_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BaseState.msg"
  "srv/GetBaseInfo.srv"
  "srv/GetBaseState.srv"
  "srv/GetBaseConfiguration.srv"
  "srv/ValidateBaseConfiguration.srv"
  "srv/ApplyBaseConfiguration.srv"
  "srv/AcquireBaseControl.srv"
  "srv/RenewBaseControl.srv"
  "srv/ReleaseBaseControl.srv"
  "srv/SetBaseMotionEnabled.srv"
  "srv/SetBaseVelocity.srv"
  "srv/StopBase.srv"
  "action/MoveBaseDistance.action"
  "action/TurnBaseRelative.action"
  DEPENDENCIES action_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
