cmake_minimum_required(VERSION 3.8)
project(ros_sim_robot)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch urdf config worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
