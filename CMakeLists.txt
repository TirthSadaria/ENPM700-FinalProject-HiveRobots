cmake_minimum_required(VERSION 3.8)
project(hive_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# ------------------------------------------------------------------------------
# Library: core state machine logic (HiveContext + states)
# ------------------------------------------------------------------------------

add_library(hive_core
  src/hive_controller.cpp
  src/hive_controller_node.cpp
  src/hive_state.cpp
)

target_include_directories(hive_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(hive_core
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
)

# ------------------------------------------------------------------------------
# Executable: ROS 2 node that wraps HiveContext
# ------------------------------------------------------------------------------

add_executable(hive_controller_node
  src/hive_controller_node.cpp
)

target_link_libraries(hive_controller_node
  hive_core
)

ament_target_dependencies(hive_controller_node
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
)

# ------------------------------------------------------------------------------
# Install targets and headers so ros2 run can find the node
# ------------------------------------------------------------------------------

install(
  TARGETS
    hive_core
    hive_controller_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

# ------------------------------------------------------------------------------
# Testing / linting
# ------------------------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
