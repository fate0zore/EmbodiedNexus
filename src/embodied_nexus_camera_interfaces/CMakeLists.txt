cmake_minimum_required(VERSION 3.8)
project(embodied_nexus_camera_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/SolveRgbCalibration.action"
  "srv/GetCameraConfiguration.srv"
  "srv/ValidateCameraConfiguration.srv"
  "srv/ApplyCameraConfiguration.srv"
  DEPENDENCIES action_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
