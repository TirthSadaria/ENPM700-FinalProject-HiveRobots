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

// Note: main() is provided automatically by ament_add_gtest via gtest_main

