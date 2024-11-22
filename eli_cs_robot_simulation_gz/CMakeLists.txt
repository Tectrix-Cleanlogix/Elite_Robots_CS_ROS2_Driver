cmake_minimum_required(VERSION 3.5)
project(eli_cs_robot_simulation_gz)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
