cmake_minimum_required(VERSION 3.8)
project(embodied_nexus_arm_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetArmInfo.srv"
  "srv/GetArmState.srv"
  "srv/GetArmConfiguration.srv"
  "srv/ValidateArmConfiguration.srv"
  "srv/ApplyArmConfiguration.srv"
  "srv/AcquireArmControl.srv"
  "srv/RenewArmControl.srv"
  "srv/ReleaseArmControl.srv"
  "srv/SetArmMotionEnabled.srv"
  "srv/SetArmJointVelocity.srv"
  "srv/SetArmLifecycle.srv"
  "srv/SetArmTeachMode.srv"
  "srv/StopArm.srv"
  "action/MoveArmJoints.action"
  DEPENDENCIES action_msgs std_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
