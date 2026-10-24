cmake_minimum_required(VERSION 3.16)
project(embodied_nexus_sim_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf config meshes third_party DESTINATION share/${PROJECT_NAME})
install(FILES UPSTREAM_README.md DESTINATION share/${PROJECT_NAME})
ament_package()
