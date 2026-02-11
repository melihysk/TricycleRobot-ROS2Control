cmake_minimum_required(VERSION 3.16)
project(offset_tricycle_controller)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ackermann_msgs
  builtin_interfaces
  controller_interface
  geometry_msgs
  generate_parameter_library
  hardware_interface
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rcpputils
  realtime_tools
  std_srvs
  tf2
  tf2_msgs
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
add_compile_definitions(RCPPUTILS_VERSION_MAJOR=${rcpputils_VERSION_MAJOR})
add_compile_definitions(RCPPUTILS_VERSION_MINOR=${rcpputils_VERSION_MINOR})

generate_parameter_library(offset_tricycle_controller_parameters
  src/offset_tricycle_controller_parameter.yaml
)

add_library(offset_tricycle_controller SHARED
  src/offset_tricycle_controller.cpp
  src/odometry.cpp
  src/traction_limiter.cpp
  src/steering_limiter.cpp
)
target_compile_features(offset_tricycle_controller PUBLIC cxx_std_17)
target_include_directories(offset_tricycle_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/offset_tricycle_controller>
)
target_link_libraries(offset_tricycle_controller PUBLIC
                      offset_tricycle_controller_parameters
                      controller_interface::controller_interface
                      hardware_interface::hardware_interface
                      pluginlib::pluginlib
                      rclcpp::rclcpp
                      rclcpp_lifecycle::rclcpp_lifecycle
                      realtime_tools::realtime_tools
                      tf2::tf2
                      rcpputils::rcpputils
                      ${ackermann_msgs_TARGETS}
                      ${nav_msgs_TARGETS}
                      ${geometry_msgs_TARGETS}
                      ${tf2_msgs_TARGETS}
                      ${std_srvs_TARGETS}
                      ${builtin_interfaces_TARGETS})

pluginlib_export_plugin_description_file(controller_interface offset_tricycle_controller.xml)

# Tests disabled - no test files included
# if(BUILD_TESTING)
# endif()

install(
  DIRECTORY include/
  DESTINATION include/offset_tricycle_controller
)
install(
  TARGETS
    offset_tricycle_controller
    offset_tricycle_controller_parameters
  EXPORT export_offset_tricycle_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_offset_tricycle_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
