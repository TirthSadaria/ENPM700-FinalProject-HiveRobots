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
# Shared Library: hive_logic (core logic without main)
# ------------------------------------------------------------------------------

add_library(hive_logic SHARED
  src/hive_controller.cpp
  src/hive_state.cpp
)

target_include_directories(hive_logic PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(hive_logic
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
)

# ------------------------------------------------------------------------------
# Executable: ROS 2 node that wraps HiveController
# ------------------------------------------------------------------------------

add_executable(hive_controller_node
  src/main.cpp
  src/hive_controller_node.cpp
)

target_link_libraries(hive_controller_node
  hive_logic
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
    hive_logic
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
  scripts/generate_rviz_config.py
  scripts/world_frame_publisher.py
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
  # Unit Tests - Comprehensive test suite
  # ------------------------------------------------------------------------------
  
  # Main unit test suite (test_hive_control.cpp - existing tests)
  ament_add_gtest(test_hive_control
    test/test_hive_control.cpp
  )
  target_link_libraries(test_hive_control
    hive_logic
  )
  ament_target_dependencies(test_hive_control
    rclcpp
    geometry_msgs
    sensor_msgs
    nav_msgs
  )
  
  # Comprehensive unit test suite (test_hive_controller.cpp - new tests)
  ament_add_gtest(test_hive_controller
    test/test_hive_controller.cpp
  )
  target_link_libraries(test_hive_controller
    hive_logic
  )
  ament_target_dependencies(test_hive_controller
    rclcpp
    geometry_msgs
    sensor_msgs
  )
  
  # ------------------------------------------------------------------------------
  # Code Coverage Support
  # ------------------------------------------------------------------------------
  
  # Check for WANT_COVERAGE option
  if(WANT_COVERAGE)
    # Add coverage flags to test_hive_controller target
    target_compile_options(test_hive_controller PRIVATE
      -fprofile-arcs
      -ftest-coverage
    )
    target_link_options(test_hive_controller PRIVATE
      --coverage
    )
    
    # Also add coverage to the library being tested
    target_compile_options(hive_logic PRIVATE
      -fprofile-arcs
      -ftest-coverage
    )
    target_link_options(hive_logic PRIVATE
      --coverage
    )
    
    # Find lcov and gcovr
    find_program(LCOV_PATH lcov)
    find_program(GCOVR_PATH gcovr)
    
    if(LCOV_PATH AND GCOVR_PATH)
      # Custom target to generate coverage report
      # Note: Tests must be run first (via colcon test) to generate .gcda files
      add_custom_target(test_coverage
        COMMAND ${LCOV_PATH} --directory . --capture --output-file test_coverage.info
        COMMAND ${LCOV_PATH} --remove test_coverage.info '/usr/*' '*/test/*' '*/build/*' '*/install/*' --output-file test_coverage.info
        COMMAND ${LCOV_PATH} --list test_coverage.info
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
        COMMENT "Generating code coverage report"
      )
    else()
      message(WARNING "lcov or gcovr not found. Coverage target will not be available.")
    endif()
  endif()
  
endif()

ament_package()
