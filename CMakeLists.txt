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
# Executable: Scan frame remapper for SLAM compatibility
# ------------------------------------------------------------------------------

add_executable(scan_frame_remapper
  src/scan_frame_remapper.cpp
)

ament_target_dependencies(scan_frame_remapper
  rclcpp
  sensor_msgs
)

# ------------------------------------------------------------------------------
# Install targets and headers so ros2 run can find the node
# ------------------------------------------------------------------------------

install(
  TARGETS
    hive_core
    hive_controller_node
    scan_frame_remapper
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  worlds  
  config
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  scripts/generate_world.py
  scripts/source_all_workspaces.sh
  scripts/verify_robot_publishing.sh
  DESTINATION lib/${PROJECT_NAME}
)

# ------------------------------------------------------------------------------
# Testing / linting
# ------------------------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  
  # Set copyright and cpplint to true to skip those checks for now
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  
  # Automatic linting
  ament_lint_auto_find_test_dependencies()
  
  # ------------------------------------------------------------------------------
  # Unit Tests
  # ------------------------------------------------------------------------------
  
  # Unit tests removed for cleanup
  
  # Integration tests for launch files (removed for cleanup)
  
endif()

ament_package()
