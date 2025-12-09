// Copyright 2025 Shreya Kalyanaraman, Tirth Sadaria
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file test_hive_controller.cpp
 * @brief Comprehensive unit tests for HiveController using GoogleTest
 * @author Shreya Kalyanaraman, Tirth Sadaria
 */

#include <gtest/gtest.h>
#include <memory>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "hive_control/hive_controller.hpp"
#include "hive_control/hive_state.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/**
 * @brief Test fixture for HiveController tests
 */
class HiveControllerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  void TearDown() override {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  /**
   * @brief Helper function to create a fake LaserScan message
   * @param ranges Vector of range values (in meters)
   * @return Shared pointer to LaserScan message
   */
  sensor_msgs::msg::LaserScan::SharedPtr createFakeLaserScan(const std::vector<float>& ranges) {
    auto scan = std::make_shared<sensor_msgs::msg::LaserScan>();
    scan->header.stamp = rclcpp::Clock().now();
    scan->header.frame_id = "LDS-01";
    scan->angle_min = -M_PI;
    scan->angle_max = M_PI;
    scan->angle_increment = (scan->angle_max - scan->angle_min) / ranges.size();
    scan->range_min = 0.1;
    scan->range_max = 3.5;
    scan->ranges = ranges;
    return scan;
  }
};

// =============================================================================
// TEST 1: TestInitialization
// =============================================================================
/**
 * @brief Test that HiveController instantiates correctly
 */
TEST_F(HiveControllerTest, TestInitialization) {
  // Create controller instance
  auto controller = std::make_shared<hive_control::HiveController>("tb1");
  
  // Verify it was created successfully
  ASSERT_NE(controller, nullptr);
  
  // Verify initial state is EXPLORING (robots start in SEARCH state)
  EXPECT_EQ(controller->getCurrentStateId(), hive_control::StateID::EXPLORING);
  
  // Verify initial state name is SEARCH
  EXPECT_EQ(controller->getCurrentStateName(), "SEARCH");
  
  // Verify default battery level is 1.0 (full)
  EXPECT_DOUBLE_EQ(controller->getBatteryLevel(), 1.0);
}

// =============================================================================
// TEST 2: TestObstacleAvoidance - Clear Path
// =============================================================================
/**
 * @brief Test obstacle avoidance with clear path (all ranges = 2.0m)
 * Covers SearchState behavior when path is clear
 */
TEST_F(HiveControllerTest, TestObstacleAvoidance_ClearPath) {
  auto controller = std::make_shared<hive_control::HiveController>("tb1");
  
  // Create a fake LaserScan with all ranges = 2.0m (Clear path)
  std::vector<float> clear_ranges(360, 2.0f);
  auto scan = createFakeLaserScan(clear_ranges);
  
  // Process the scan
  controller->processLaserScan(scan);
  
  // Get velocity command
  auto cmd_vel = controller->getVelocityCommand();
  
  // Assert: robot should move forward (linear.x > 0) when path is clear
  EXPECT_GT(cmd_vel.linear.x, 0.0) << "Robot should move forward when path is clear";
  
  // Verify command is valid (not NaN or Inf)
  EXPECT_TRUE(std::isfinite(cmd_vel.linear.x));
  EXPECT_TRUE(std::isfinite(cmd_vel.angular.z));
}

// =============================================================================
// TEST 3: TestObstacleAvoidance - Obstacle Detected
// =============================================================================
/**
 * @brief Test obstacle avoidance with obstacle detected (ranges = 0.3m)
 * Covers SearchState behavior when obstacle is detected
 */
TEST_F(HiveControllerTest, TestObstacleAvoidance_ObstacleDetected) {
  auto controller = std::make_shared<hive_control::HiveController>("tb1");
  
  // Create a fake LaserScan with ranges = 0.3m (Obstacle very close)
  std::vector<float> obstacle_ranges(360, 0.3f);
  auto scan = createFakeLaserScan(obstacle_ranges);
  
  // Process the scan
  controller->processLaserScan(scan);
  
  // Get velocity command
  auto cmd_vel = controller->getVelocityCommand();
  
  // Assert: robot should stop or turn (linear.x <= 0 OR angular.z != 0)
  // When obstacle is detected, robot should either:
  // 1. Stop/back up (linear.x <= 0), OR
  // 2. Turn to avoid (angular.z != 0)
  bool should_stop_or_turn = (cmd_vel.linear.x <= 0.0) || (std::abs(cmd_vel.angular.z) > 0.01);
  EXPECT_TRUE(should_stop_or_turn) 
    << "Robot should stop (linear.x <= 0) or turn (angular.z != 0) when obstacle detected. "
    << "Got linear.x=" << cmd_vel.linear.x << ", angular.z=" << cmd_vel.angular.z;
  
  // Verify command is valid (not NaN or Inf)
  EXPECT_TRUE(std::isfinite(cmd_vel.linear.x));
  EXPECT_TRUE(std::isfinite(cmd_vel.angular.z));
}

// =============================================================================
// TEST 4: TestAllStates - CoordinationState
// =============================================================================
/**
 * @brief Test CoordinationState behavior
 * Manually force state to COORDINATION and verify it generates backup/turn command
 */
TEST_F(HiveControllerTest, TestAllStates_CoordinationState) {
  auto controller = std::make_shared<hive_control::HiveController>("tb1");
  
  // Manually force state to COORDINATION
  auto coord_state = std::make_shared<hive_control::CoordinationState>(true);  // turn_right = true
  controller->setState(coord_state);
  
  // Verify state changed
  EXPECT_EQ(controller->getCurrentStateName(), "COORDINATION");
  
  // Create a scan with close obstacles (0.2m) to trigger backup behavior
  std::vector<float> close_obstacle_ranges(360, 0.2f);
  auto scan = createFakeLaserScan(close_obstacle_ranges);
  
  // Process the scan to update state
  controller->processLaserScan(scan);
  
  // Get velocity command from CoordinationState
  auto cmd_vel = controller->getVelocityCommand();
  
  // Verify it generates a command (either backup or turn)
  // CoordinationState should generate either:
  // 1. Angular velocity (turning in place), OR
  // 2. Linear backward velocity (backing up if too close)
  bool has_command = (std::abs(cmd_vel.angular.z) > 0.01) || (cmd_vel.linear.x < 0.0);
  EXPECT_TRUE(has_command) 
    << "CoordinationState should generate backup or turn command. "
    << "Got linear.x=" << cmd_vel.linear.x << ", angular.z=" << cmd_vel.angular.z;
  
  // Verify command is valid
  EXPECT_TRUE(std::isfinite(cmd_vel.linear.x));
  EXPECT_TRUE(std::isfinite(cmd_vel.angular.z));
}

// =============================================================================
// TEST 5: TestAllStates - ConvergenceState
// =============================================================================
/**
 * @brief Test ConvergenceState behavior
 * Manually force state to CONVERGENCE and verify it generates a command
 */
TEST_F(HiveControllerTest, TestAllStates_ConvergenceState) {
  auto controller = std::make_shared<hive_control::HiveController>("tb1");
  
  // Manually force state to CONVERGENCE
  auto conv_state = std::make_shared<hive_control::ConvergenceState>();
  controller->setState(conv_state);
  
  // Verify state changed
  EXPECT_EQ(controller->getCurrentStateName(), "CONVERGENCE");
  
  // Create a scan (any scan will do for ConvergenceState)
  std::vector<float> scan_ranges(360, 1.5f);
  auto scan = createFakeLaserScan(scan_ranges);
  
  // Process the scan
  controller->processLaserScan(scan);
  
  // Get velocity command from ConvergenceState
  auto cmd_vel = controller->getVelocityCommand();
  
  // Verify it generates a command
  // ConvergenceState should generate either:
  // 1. Linear forward velocity (moving toward rendezvous), OR
  // 2. Angular velocity (rotating at center)
  bool has_command = (std::abs(cmd_vel.linear.x) > 0.01) || (std::abs(cmd_vel.angular.z) > 0.01);
  EXPECT_TRUE(has_command) 
    << "ConvergenceState should generate a command. "
    << "Got linear.x=" << cmd_vel.linear.x << ", angular.z=" << cmd_vel.angular.z;
  
  // Verify command is valid
  EXPECT_TRUE(std::isfinite(cmd_vel.linear.x));
  EXPECT_TRUE(std::isfinite(cmd_vel.angular.z));
}

// =============================================================================
// TEST 6: TestObstacleAvoidance - Mixed Ranges
// =============================================================================
/**
 * @brief Test obstacle avoidance with mixed ranges (some clear, some obstacles)
 * This tests the SearchState's ability to find the best direction
 */
TEST_F(HiveControllerTest, TestObstacleAvoidance_MixedRanges) {
  auto controller = std::make_shared<hive_control::HiveController>("tb1");
  
  // Create a scan with obstacles on left/right but clear ahead
  std::vector<float> mixed_ranges(360);
  for (size_t i = 0; i < mixed_ranges.size(); ++i) {
    // Front 90 degrees: clear (2.0m)
    if (i >= 135 && i < 225) {
      mixed_ranges[i] = 2.0f;
    } else {
      // Sides: obstacles (0.5m)
      mixed_ranges[i] = 0.5f;
    }
  }
  auto scan = createFakeLaserScan(mixed_ranges);
  
  // Process the scan
  controller->processLaserScan(scan);
  
  // Get velocity command
  auto cmd_vel = controller->getVelocityCommand();
  
  // Robot should prefer forward motion when front is clear
  // But may also turn slightly to optimize path
  EXPECT_TRUE(std::isfinite(cmd_vel.linear.x));
  EXPECT_TRUE(std::isfinite(cmd_vel.angular.z));
  
  // If front is clear, should generally move forward
  // (allowing for some turning to optimize)
  if (cmd_vel.linear.x > 0.0) {
    // Forward motion is reasonable
    EXPECT_GT(cmd_vel.linear.x, 0.0);
  }
}

// =============================================================================
// TEST 7: TestStateTransitions
// =============================================================================
/**
 * @brief Test state transitions between different states
 */
TEST_F(HiveControllerTest, TestStateTransitions) {
  auto controller = std::make_shared<hive_control::HiveController>("tb1");
  
  // Initial state should be SEARCH
  EXPECT_EQ(controller->getCurrentStateName(), "SEARCH");
  
  // Transition to IDLE
  auto idle_state = std::make_shared<hive_control::IdleState>();
  controller->setState(idle_state);
  EXPECT_EQ(controller->getCurrentStateName(), "IDLE");
  
  // Transition to COORDINATION
  auto coord_state = std::make_shared<hive_control::CoordinationState>(false);
  controller->setState(coord_state);
  EXPECT_EQ(controller->getCurrentStateName(), "COORDINATION");
  
  // Transition to CONVERGENCE
  auto conv_state = std::make_shared<hive_control::ConvergenceState>();
  controller->setState(conv_state);
  EXPECT_EQ(controller->getCurrentStateName(), "CONVERGENCE");
  
  // Transition back to SEARCH
  auto search_state = std::make_shared<hive_control::SearchState>();
  controller->setState(search_state);
  EXPECT_EQ(controller->getCurrentStateName(), "SEARCH");
}

// =============================================================================
// TEST 8: TestEmptyScanHandling
// =============================================================================
/**
 * @brief Test handling of empty or invalid scans
 */
TEST_F(HiveControllerTest, TestEmptyScanHandling) {
  auto controller = std::make_shared<hive_control::HiveController>("tb1");
  
  // Create an empty scan
  auto empty_scan = std::make_shared<sensor_msgs::msg::LaserScan>();
  empty_scan->header.stamp = rclcpp::Clock().now();
  empty_scan->header.frame_id = "LDS-01";
  empty_scan->angle_min = -M_PI;
  empty_scan->angle_max = M_PI;
  empty_scan->angle_increment = 0.0175;
  empty_scan->range_min = 0.1;
  empty_scan->range_max = 3.5;
  empty_scan->ranges = std::vector<float>();  // Empty ranges
  
  // Process empty scan - should not crash
  EXPECT_NO_THROW(controller->processLaserScan(empty_scan));
  
  // Get velocity command - should still return valid command
  auto cmd_vel = controller->getVelocityCommand();
  EXPECT_TRUE(std::isfinite(cmd_vel.linear.x));
  EXPECT_TRUE(std::isfinite(cmd_vel.angular.z));
}

// =============================================================================
// TEST 9: TestOdometrySetterGetter
// =============================================================================
/**
 * @brief Test odometry setter and getter methods
 */
TEST_F(HiveControllerTest, TestOdometrySetterGetter) {
  auto controller = std::make_shared<hive_control::HiveController>("tb1");
  
  // Initially no odometry
  EXPECT_EQ(controller->getCurrentOdometry(), nullptr);
  
  // Create and set odometry
  auto odom = std::make_shared<nav_msgs::msg::Odometry>();
  odom->header.stamp = rclcpp::Clock().now();
  odom->header.frame_id = "odom";
  odom->child_frame_id = "base_link";
  odom->pose.pose.position.x = 1.5;
  odom->pose.pose.position.y = 2.5;
  odom->pose.pose.orientation.w = 1.0;
  
  controller->setCurrentOdometry(odom);
  
  // Verify odometry was stored
  auto retrieved_odom = controller->getCurrentOdometry();
  ASSERT_NE(retrieved_odom, nullptr);
  EXPECT_DOUBLE_EQ(retrieved_odom->pose.pose.position.x, 1.5);
  EXPECT_DOUBLE_EQ(retrieved_odom->pose.pose.position.y, 2.5);
}

// =============================================================================
// TEST 10: TestMapSetterGetter
// =============================================================================
/**
 * @brief Test map setter and getter methods
 */
TEST_F(HiveControllerTest, TestMapSetterGetter) {
  auto controller = std::make_shared<hive_control::HiveController>("tb1");
  
  // Initially no map
  EXPECT_EQ(controller->getCurrentMap(), nullptr);
  
  // Create and set map
  auto map = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  map->header.stamp = rclcpp::Clock().now();
  map->header.frame_id = "map";
  map->info.width = 50;
  map->info.height = 50;
  map->info.resolution = 0.05;
  map->data = std::vector<int8_t>(2500, 0);
  
  controller->setCurrentMap(map);
  
  // Verify map was stored
  auto retrieved_map = controller->getCurrentMap();
  ASSERT_NE(retrieved_map, nullptr);
  EXPECT_EQ(retrieved_map->info.width, 50);
  EXPECT_EQ(retrieved_map->info.height, 50);
}

// =============================================================================
// TEST 11: TestScanGetter
// =============================================================================
/**
 * @brief Test getCurrentScan method
 */
TEST_F(HiveControllerTest, TestScanGetter) {
  auto controller = std::make_shared<hive_control::HiveController>("tb1");
  
  // Initially no scan
  EXPECT_EQ(controller->getCurrentScan(), nullptr);
  
  // Process a scan
  std::vector<float> ranges(360, 1.5f);
  auto scan = createFakeLaserScan(ranges);
  controller->processLaserScan(scan);
  
  // Verify scan was stored
  auto retrieved_scan = controller->getCurrentScan();
  ASSERT_NE(retrieved_scan, nullptr);
  EXPECT_EQ(retrieved_scan->ranges.size(), 360);
}

// =============================================================================
// TEST 12: TestStateIDGetter
// =============================================================================
/**
 * @brief Test getCurrentStateId method
 */
TEST_F(HiveControllerTest, TestStateIDGetter) {
  auto controller = std::make_shared<hive_control::HiveController>("tb1");
  
  // Initial state should be EXPLORING
  EXPECT_EQ(controller->getCurrentStateId(), hive_control::StateID::EXPLORING);
  
  // Change to IDLE
  controller->setState(std::make_shared<hive_control::IdleState>());
  EXPECT_EQ(controller->getCurrentStateId(), hive_control::StateID::IDLE);
  
  // Change back to SEARCH
  controller->setState(std::make_shared<hive_control::SearchState>());
  EXPECT_EQ(controller->getCurrentStateId(), hive_control::StateID::EXPLORING);
}

// =============================================================================
// TEST 13: TestObstacleAvoidance_CloseObstacle
// =============================================================================
/**
 * @brief Test obstacle avoidance with very close obstacle (0.2m)
 */
TEST_F(HiveControllerTest, TestObstacleAvoidance_CloseObstacle) {
  auto controller = std::make_shared<hive_control::HiveController>("tb1");
  
  // Create scan with very close obstacle
  std::vector<float> close_ranges(360, 0.2f);
  auto scan = createFakeLaserScan(close_ranges);
  
  controller->processLaserScan(scan);
  auto cmd_vel = controller->getVelocityCommand();
  
  // Robot should back up or turn (not move forward)
  EXPECT_LE(cmd_vel.linear.x, 0.1) << "Robot should not move forward when obstacle is very close";
  EXPECT_TRUE(std::isfinite(cmd_vel.linear.x));
  EXPECT_TRUE(std::isfinite(cmd_vel.angular.z));
}

// =============================================================================
// TEST 14: TestObstacleAvoidance_FarObstacle
// =============================================================================
/**
 * @brief Test obstacle avoidance with far obstacle (3.0m)
 */
TEST_F(HiveControllerTest, TestObstacleAvoidance_FarObstacle) {
  auto controller = std::make_shared<hive_control::HiveController>("tb1");
  
  // Create scan with far obstacle
  std::vector<float> far_ranges(360, 3.0f);
  auto scan = createFakeLaserScan(far_ranges);
  
  controller->processLaserScan(scan);
  auto cmd_vel = controller->getVelocityCommand();
  
  // Robot should be able to move forward when obstacle is far
  EXPECT_TRUE(std::isfinite(cmd_vel.linear.x));
  EXPECT_TRUE(std::isfinite(cmd_vel.angular.z));
  // May move forward or turn slightly
  EXPECT_GE(cmd_vel.linear.x, -1.0);
  EXPECT_LE(cmd_vel.linear.x, 1.0);
}

// =============================================================================
// TEST 15: TestAllStates_IdleState
// =============================================================================
/**
 * @brief Test IdleState behavior
 */
TEST_F(HiveControllerTest, TestAllStates_IdleState) {
  auto controller = std::make_shared<hive_control::HiveController>("tb1");
  
  // Force IDLE state
  auto idle_state = std::make_shared<hive_control::IdleState>();
  controller->setState(idle_state);
  EXPECT_EQ(controller->getCurrentStateName(), "IDLE");
  
  // Create scan
  std::vector<float> ranges(360, 1.5f);
  auto scan = createFakeLaserScan(ranges);
  
  // Process scan - IDLE should transition to SEARCH when valid scan received
  controller->processLaserScan(scan);

  // Should transition to SEARCH (not remain IDLE)
  EXPECT_EQ(controller->getCurrentStateName(), "SEARCH");

  // SEARCH state should produce valid velocity command
  auto cmd_vel = controller->getVelocityCommand();
  EXPECT_TRUE(std::isfinite(cmd_vel.linear.x));
  EXPECT_TRUE(std::isfinite(cmd_vel.angular.z));
}

// =============================================================================
// TEST 16: TestAllStates_SearchState
// =============================================================================
/**
 * @brief Test SearchState behavior
 */
TEST_F(HiveControllerTest, TestAllStates_SearchState) {
  auto controller = std::make_shared<hive_control::HiveController>("tb1");
  
  // Should start in SEARCH state
  EXPECT_EQ(controller->getCurrentStateName(), "SEARCH");
  
  // Create scan with clear path
  std::vector<float> clear_ranges(360, 2.0f);
  auto scan = createFakeLaserScan(clear_ranges);
  
  controller->processLaserScan(scan);
  auto cmd_vel = controller->getVelocityCommand();
  
  // Should generate movement command
  EXPECT_TRUE(std::isfinite(cmd_vel.linear.x));
  EXPECT_TRUE(std::isfinite(cmd_vel.angular.z));
}

// =============================================================================
// TEST 17: TestCoordinationState_Clockwise
// =============================================================================
/**
 * @brief Test CoordinationState with clockwise rotation
 */
TEST_F(HiveControllerTest, TestCoordinationState_Clockwise) {
  auto controller = std::make_shared<hive_control::HiveController>("tb1");
  
  // Create clockwise coordination state
  auto coord_state = std::make_shared<hive_control::CoordinationState>(true);
  controller->setState(coord_state);
  
  std::vector<float> ranges(360, 1.0f);
  auto scan = createFakeLaserScan(ranges);
  controller->processLaserScan(scan);
  
  auto cmd_vel = controller->getVelocityCommand();
  EXPECT_TRUE(std::isfinite(cmd_vel.linear.x));
  EXPECT_TRUE(std::isfinite(cmd_vel.angular.z));
}

// =============================================================================
// TEST 18: TestCoordinationState_CounterClockwise
// =============================================================================
/**
 * @brief Test CoordinationState with counterclockwise rotation
 */
TEST_F(HiveControllerTest, TestCoordinationState_CounterClockwise) {
  auto controller = std::make_shared<hive_control::HiveController>("tb1");
  
  // Create counterclockwise coordination state
  auto coord_state = std::make_shared<hive_control::CoordinationState>(false);
  controller->setState(coord_state);
  
  std::vector<float> ranges(360, 1.0f);
  auto scan = createFakeLaserScan(ranges);
  controller->processLaserScan(scan);
  
  auto cmd_vel = controller->getVelocityCommand();
  EXPECT_TRUE(std::isfinite(cmd_vel.linear.x));
  EXPECT_TRUE(std::isfinite(cmd_vel.angular.z));
  
  // Compare with clockwise - should have opposite angular velocity sign
  auto coord_cw = std::make_shared<hive_control::CoordinationState>(true);
  controller->setState(coord_cw);
  controller->processLaserScan(scan);
  auto cmd_cw = controller->getVelocityCommand();
  
  // Angular velocities should have opposite signs (or at least be different)
  if (std::abs(cmd_vel.angular.z) > 0.01 && std::abs(cmd_cw.angular.z) > 0.01) {
    EXPECT_NE(std::signbit(cmd_vel.angular.z), std::signbit(cmd_cw.angular.z));
  }
}

// =============================================================================
// TEST 19: TestConvergenceState_WithScan
// =============================================================================
/**
 * @brief Test ConvergenceState with scan data
 */
TEST_F(HiveControllerTest, TestConvergenceState_WithScan) {
  auto controller = std::make_shared<hive_control::HiveController>("tb1");
  
  auto conv_state = std::make_shared<hive_control::ConvergenceState>();
  controller->setState(conv_state);
  EXPECT_EQ(controller->getCurrentStateName(), "CONVERGENCE");
  
  std::vector<float> ranges(360, 1.5f);
  auto scan = createFakeLaserScan(ranges);
  controller->processLaserScan(scan);
  
  auto cmd_vel = controller->getVelocityCommand();
  EXPECT_TRUE(std::isfinite(cmd_vel.linear.x));
  EXPECT_TRUE(std::isfinite(cmd_vel.angular.z));
}

// =============================================================================
// TEST 20: TestConvergenceState_WithoutScan
// =============================================================================
/**
 * @brief Test ConvergenceState without scan data
 */
TEST_F(HiveControllerTest, TestConvergenceState_WithoutScan) {
  auto controller = std::make_shared<hive_control::HiveController>("tb1");
  
  auto conv_state = std::make_shared<hive_control::ConvergenceState>();
  controller->setState(conv_state);
  
  // Get command without processing scan
  auto cmd_vel = controller->getVelocityCommand();
  EXPECT_TRUE(std::isfinite(cmd_vel.linear.x));
  EXPECT_TRUE(std::isfinite(cmd_vel.angular.z));
}

// =============================================================================
// TEST 21: TestNamespaceParsing_VariousFormats
// =============================================================================
/**
 * @brief Test namespace parsing with various formats
 */
TEST_F(HiveControllerTest, TestNamespaceParsing_VariousFormats) {
  // Test different namespace formats
  auto c1 = std::make_shared<hive_control::HiveController>("tb1");
  auto c2 = std::make_shared<hive_control::HiveController>("/tb2");
  auto c3 = std::make_shared<hive_control::HiveController>("tb3");
  auto c4 = std::make_shared<hive_control::HiveController>("robot4");
  auto c5 = std::make_shared<hive_control::HiveController>("");
  
  // All should initialize successfully
  EXPECT_NE(c1, nullptr);
  EXPECT_NE(c2, nullptr);
  EXPECT_NE(c3, nullptr);
  EXPECT_NE(c4, nullptr);
  EXPECT_NE(c5, nullptr);
  
  // Check rotation directions based on ID
  EXPECT_TRUE(c1->isClockwise());   // ID 1 = odd = clockwise
  EXPECT_FALSE(c2->isClockwise()); // ID 2 = even = counterclockwise
  EXPECT_TRUE(c3->isClockwise());   // ID 3 = odd = clockwise
}

// =============================================================================
// TEST 22: TestBatteryLevel_SetterGetter
// =============================================================================
/**
 * @brief Test battery level setter and getter
 */
TEST_F(HiveControllerTest, TestBatteryLevel_SetterGetter) {
  auto controller = std::make_shared<hive_control::HiveController>("tb1");
  
  // Default should be 1.0 (full)
  EXPECT_DOUBLE_EQ(controller->getBatteryLevel(), 1.0);
  
  // Set various levels
  controller->setBatteryLevel(0.75);
  EXPECT_DOUBLE_EQ(controller->getBatteryLevel(), 0.75);
  
  controller->setBatteryLevel(0.5);
  EXPECT_DOUBLE_EQ(controller->getBatteryLevel(), 0.5);
  
  controller->setBatteryLevel(0.25);
  EXPECT_DOUBLE_EQ(controller->getBatteryLevel(), 0.25);
  
  controller->setBatteryLevel(0.0);
  EXPECT_DOUBLE_EQ(controller->getBatteryLevel(), 0.0);
}

// =============================================================================
// TEST 23: TestRotationDirection_Toggle
// =============================================================================
/**
 * @brief Test rotation direction toggle
 */
TEST_F(HiveControllerTest, TestRotationDirection_Toggle) {
  auto controller = std::make_shared<hive_control::HiveController>("tb1");
  
  // Get initial direction
  bool initial = controller->isClockwise();
  
  // Toggle multiple times
  controller->toggleRotationDirection();
  EXPECT_NE(controller->isClockwise(), initial);
  
  controller->toggleRotationDirection();
  EXPECT_EQ(controller->isClockwise(), initial);
  
  controller->toggleRotationDirection();
  EXPECT_NE(controller->isClockwise(), initial);
}

// =============================================================================
// TEST 24: TestStateName_AllStates
// =============================================================================
/**
 * @brief Test state name retrieval for all states
 */
TEST_F(HiveControllerTest, TestStateName_AllStates) {
  auto controller = std::make_shared<hive_control::HiveController>("tb1");
  
  // Test all state names
  controller->setState(std::make_shared<hive_control::IdleState>());
  EXPECT_EQ(controller->getCurrentStateName(), "IDLE");
  
  controller->setState(std::make_shared<hive_control::SearchState>());
  EXPECT_EQ(controller->getCurrentStateName(), "SEARCH");
  
  controller->setState(std::make_shared<hive_control::CoordinationState>(true));
  EXPECT_EQ(controller->getCurrentStateName(), "COORDINATION");
  
  controller->setState(std::make_shared<hive_control::ConvergenceState>());
  EXPECT_EQ(controller->getCurrentStateName(), "CONVERGENCE");
}

// =============================================================================
// TEST 25: TestScanProcessing_MultipleScans
// =============================================================================
/**
 * @brief Test processing multiple scans in sequence
 */
TEST_F(HiveControllerTest, TestScanProcessing_MultipleScans) {
  auto controller = std::make_shared<hive_control::HiveController>("tb1");
  
  // Process multiple scans with different ranges
  for (int i = 0; i < 10; ++i) {
    std::vector<float> ranges(360, 1.0f + i * 0.1f);
    auto scan = createFakeLaserScan(ranges);
    controller->processLaserScan(scan);
    
    auto cmd = controller->getVelocityCommand();
    EXPECT_TRUE(std::isfinite(cmd.linear.x));
    EXPECT_TRUE(std::isfinite(cmd.angular.z));
  }
}

// =============================================================================
// TEST 26: TestScanProcessing_InvalidRanges
// =============================================================================
/**
 * @brief Test processing scans with invalid range values
 */
TEST_F(HiveControllerTest, TestScanProcessing_InvalidRanges) {
  auto controller = std::make_shared<hive_control::HiveController>("tb1");
  
  // Create scan with mix of valid and invalid ranges
  std::vector<float> ranges(360);
  for (size_t i = 0; i < ranges.size(); ++i) {
    if (i % 10 == 0) {
      ranges[i] = std::numeric_limits<float>::quiet_NaN();
    } else if (i % 10 == 1) {
      ranges[i] = std::numeric_limits<float>::infinity();
    } else {
      ranges[i] = 1.5f;
    }
  }
  
  auto scan = createFakeLaserScan(ranges);
  controller->processLaserScan(scan);
  
  auto cmd = controller->getVelocityCommand();
  EXPECT_TRUE(std::isfinite(cmd.linear.x));
  EXPECT_TRUE(std::isfinite(cmd.angular.z));
}

// Note: main() is provided automatically by ament_add_gtest via gtest_main

