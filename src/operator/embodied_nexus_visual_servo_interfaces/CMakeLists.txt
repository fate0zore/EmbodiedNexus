cmake_minimum_required(VERSION 3.8)
project(embodied_nexus_visual_servo_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/VisualServoGrasp.action"
  "msg/VisualTarget.msg"
  DEPENDENCIES action_msgs geometry_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
