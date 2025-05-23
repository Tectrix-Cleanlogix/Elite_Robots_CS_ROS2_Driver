cmake_minimum_required(VERSION 3.5)
project(eli_cs_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(controller_interface REQUIRED)
find_package(joint_trajectory_controller REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rcutils REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(eli_common_interface REQUIRED)

set(
  THIS_PACKAGE_DEPENDENCIES
  angles
  controller_interface
  joint_trajectory_controller
  lifecycle_msgs
  pluginlib
  rclcpp_lifecycle
  rcutils
  realtime_tools
  std_msgs
  std_srvs
  generate_parameter_library
  eli_common_interface
)

include_directories(include)


generate_parameter_library(
  gpio_controller_parameters
  src/gpio_controller_parameters.yaml
)

generate_parameter_library(
  speed_scaling_state_broadcaster_parameters
  src/speed_scaling_state_broadcaster_parameters.yaml
)

generate_parameter_library(
  scaled_joint_trajectory_controller_parameters
  src/scaled_joint_trajectory_controller_parameters.yaml
)

add_library(
  ${PROJECT_NAME} 
  SHARED
  src/scaled_joint_trajectory_controller.cpp
  src/speed_scaling_state_broadcaster.cpp
  src/gpio_controller.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)
target_link_libraries(
  ${PROJECT_NAME}
  gpio_controller_parameters
  speed_scaling_state_broadcaster_parameters
  scaled_joint_trajectory_controller_parameters
)
ament_target_dependencies(
  ${PROJECT_NAME}
  ${THIS_PACKAGE_DEPENDENCIES}
)

target_compile_options(${PROJECT_NAME} PRIVATE -Wpedantic -Werror=return-type)

# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(controller_interface controller_plugins.xml)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES controller_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${PROJECT_NAME}
)

ament_package()
