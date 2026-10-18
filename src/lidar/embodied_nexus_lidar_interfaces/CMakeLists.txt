cmake_minimum_required(VERSION 3.8)
project(embodied_nexus_lidar_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LidarState.msg"
  "srv/GetLidarInfo.srv"
  "srv/GetLidarState.srv"
  "srv/GetLidarConfiguration.srv"
  "srv/ValidateLidarConfiguration.srv"
  "srv/ApplyLidarConfiguration.srv"
  "srv/SetLidarCapture.srv"
  "srv/AcquireLidarControl.srv"
  "srv/RenewLidarControl.srv"
  "srv/ReleaseLidarControl.srv"
  "srv/SetLidarMotor.srv"
  "srv/SetLidarMotorSpeed.srv"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
