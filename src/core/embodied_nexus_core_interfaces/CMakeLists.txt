cmake_minimum_required(VERSION 3.8)
project(embodied_nexus_core_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetCoreStatus.srv"
  "srv/ManageCoreDevice.srv"
  "srv/ManageAllCoreDevices.srv"
  "srv/ValidatePlatformConfiguration.srv"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
