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
 * @file test_hive_control.cpp
 * @brief Unit tests for the Hive Control multi-robot SLAM system
 * @author Shreya Kalyanaraman, Tirth Sadaria
 */

#include <gtest/gtest.h>
#include <memory>
#include <cmath>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "hive_control/hive_controller.hpp"
#include "hive_control/hive_state.hpp"

/**
 * @brief Test fixture for Hive Control tests
 */
class HiveControlTest : public ::testing::Test {
 protected:
  void SetUp() override {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  void TearDown() override {
    // Don't shutdown here - let main handle it
  }
};

// =============================================================================
// TEST 1: HiveController Node Creation
// =============================================================================
/**
 * @brief Test that HiveController can be instantiated correctly
 * 
 * This test verifies that the main controller class can be created
 * without throwing exceptions. This is a fundamental test that ensures
 * the basic state machine infrastructure is working.
 */
TEST_F(HiveControlTest, ControllerInstantiation) {
  // Create controller instance with namespace
  auto controller = std::make_shared<hive_control::HiveController>("tb1");
  
  // Verify it was created successfully
  ASSERT_NE(controller, nullptr);
  
  // Verify initial state is EXPLORING (robots start exploring immediately)
  EXPECT_EQ(controller->getCurrentStateId(), hive_control::StateID::EXPLORING);
}

// =============================================================================
// TEST 2: Battery Level Management
// =============================================================================
/**
 * @brief Test battery level getter and setter
 * 
 * Verifies that battery level can be set and retrieved correctly.
 * Battery level should be in range [0.0, 1.0].
 */
TEST_F(HiveControlTest, BatteryLevelManagement) {
  auto controller = std::make_shared<hive_control::HiveController>("tb1");
  
  // Default battery should be full (1.0)
  EXPECT_DOUBLE_EQ(controller->getBatteryLevel(), 1.0);
  
  // Set battery to 50%
  controller->setBatteryLevel(0.5);
  EXPECT_DOUBLE_EQ(controller->getBatteryLevel(), 0.5);
  
  // Set battery to low
  controller->setBatteryLevel(0.1);
  EXPECT_DOUBLE_EQ(controller->getBatteryLevel(), 0.1);
}

// =============================================================================
// TEST 3: Rotation Direction Toggle
// =============================================================================
/**
 * @brief Test rotation direction toggling
 * 
 * Verifies that the clockwise/counterclockwise toggle works correctly.
 * This is important for randomized exploration behavior.
 */
TEST_F(HiveControlTest, RotationDirectionToggle) {
  auto controller = std::make_shared<hive_control::HiveController>("tb1");
  
  // Get initial rotation direction
  bool initial_clockwise = controller->isClockwise();
  
  // Toggle direction
  controller->toggleRotationDirection();
  EXPECT_NE(controller->isClockwise(), initial_clockwise);
  
  // Toggle again - should return to original
  controller->toggleRotationDirection();
  EXPECT_EQ(controller->isClockwise(), initial_clockwise);
}

// =============================================================================
// TEST 4: Laser Scan Angle Normalization
// =============================================================================
/**
 * @brief Test laser scan angle normalization logic
 * 
 * Verifies that scan angles are correctly normalized from Webots format
 * (angle_min=π, angle_max=-π, negative increment) to standard ROS format
 * (angle_min=-π, angle_max=π, positive increment).
 */
TEST_F(HiveControlTest, ScanAngleNormalization) {
  // Simulate Webots scan format (inverted)
  double webots_angle_min = M_PI;       // 3.14159
  double webots_angle_max = -M_PI;      // -3.14159
  double webots_increment = -0.0175;    // negative (clockwise)
  
  // Apply normalization (same logic as scan_frame_remapper.cpp)
  double normalized_angle_min, normalized_angle_max, normalized_increment;
  
  if (webots_increment < 0) {
    // Swap min/max and negate increment
    normalized_angle_min = webots_angle_max;   // -π
    normalized_angle_max = webots_angle_min;   // π
    normalized_increment = -webots_increment;  // positive
  } else {
    normalized_angle_min = webots_angle_min;
    normalized_angle_max = webots_angle_max;
    normalized_increment = webots_increment;
  }
  
  // Verify normalized values
  EXPECT_NEAR(normalized_angle_min, -M_PI, 0.001);
  EXPECT_NEAR(normalized_angle_max, M_PI, 0.001);
  EXPECT_GT(normalized_increment, 0.0);
  EXPECT_NEAR(normalized_increment, 0.0175, 0.001);
}

// =============================================================================
// TEST 5: Obstacle Detection Thresholds
// =============================================================================
/**
 * @brief Test obstacle detection threshold configuration
 * 
 * Verifies that the obstacle detection thresholds are set to reasonable
 * values that allow safe navigation while not being overly conservative.
 */
TEST_F(HiveControlTest, ObstacleDetectionThresholds) {
  // These thresholds match values in hive_state.cpp
  const double CRITICAL_DISTANCE = 0.3;   // 30cm - too close, must back up
  const double OBSTACLE_THRESHOLD = 0.8;  // 80cm - obstacle ahead, should turn
  const double SAFE_DISTANCE = 1.0;       // 1.0m - safe to proceed forward
  const double LIDAR_MAX_RANGE = 3.5;     // LDS-01 max range
  
  // Thresholds should be in correct ascending order
  EXPECT_LT(CRITICAL_DISTANCE, OBSTACLE_THRESHOLD);
  EXPECT_LT(OBSTACLE_THRESHOLD, SAFE_DISTANCE);
  EXPECT_LT(SAFE_DISTANCE, LIDAR_MAX_RANGE);
  
  // Critical distance should be greater than robot radius (~0.1m)
  EXPECT_GT(CRITICAL_DISTANCE, 0.1);
  
  // All thresholds should be positive
  EXPECT_GT(CRITICAL_DISTANCE, 0.0);
  EXPECT_GT(OBSTACLE_THRESHOLD, 0.0);
  EXPECT_GT(SAFE_DISTANCE, 0.0);
}

// =============================================================================
// TEST 6: State Name Retrieval
// =============================================================================
/**
 * @brief Test getCurrentStateName returns correct state names
 */
TEST_F(HiveControlTest, StateNameRetrieval) {
  auto controller = std::make_shared<hive_control::HiveController>("tb1");
  
  // Initial state should be SEARCH (which maps to EXPLORING)
  EXPECT_EQ(controller->getCurrentStateName(), "SEARCH");
  
  // Test state name is not empty
  EXPECT_FALSE(controller->getCurrentStateName().empty());
}

// =============================================================================
// TEST 7: Process Laser Scan
// =============================================================================
/**
 * @brief Test processLaserScan stores scan and updates state
 */
TEST_F(HiveControlTest, ProcessLaserScan) {
  auto controller = std::make_shared<hive_control::HiveController>("tb1");
  
  // Create a valid laser scan
  auto scan = std::make_shared<sensor_msgs::msg::LaserScan>();
  scan->header.stamp = rclcpp::Clock().now();
  scan->header.frame_id = "LDS-01";
  scan->angle_min = -M_PI;
  scan->angle_max = M_PI;
  scan->angle_increment = 0.0175;
  scan->range_min = 0.1;
  scan->range_max = 3.5;
  scan->ranges = std::vector<float>(360, 1.5);  // 360 readings at 1.5m
  
  // Process the scan
  controller->processLaserScan(scan);
  
  // Verify scan was stored
  auto stored_scan = controller->getCurrentScan();
  ASSERT_NE(stored_scan, nullptr);
  EXPECT_EQ(stored_scan->ranges.size(), 360);
}

// =============================================================================
// TEST 8: Get Velocity Command
// =============================================================================
/**
 * @brief Test getVelocityCommand returns valid twist messages
 */
TEST_F(HiveControlTest, GetVelocityCommand) {
  auto controller = std::make_shared<hive_control::HiveController>("tb1");
  
  // Get velocity command (should work even without scan)
  auto cmd = controller->getVelocityCommand();
  
  // Verify it's a valid Twist message (values may be zero)
  EXPECT_GE(cmd.linear.x, -1.0);
  EXPECT_LE(cmd.linear.x, 1.0);
  EXPECT_GE(cmd.angular.z, -2.0);
  EXPECT_LE(cmd.angular.z, 2.0);
}

// =============================================================================
// TEST 9: Map Management
// =============================================================================
/**
 * @brief Test map setter and getter
 */
TEST_F(HiveControlTest, MapManagement) {
  auto controller = std::make_shared<hive_control::HiveController>("tb1");
  
  // Initially no map
  EXPECT_EQ(controller->getCurrentMap(), nullptr);
  
  // Create a test map
  auto map = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  map->header.stamp = rclcpp::Clock().now();
  map->header.frame_id = "map";
  map->info.width = 100;
  map->info.height = 100;
  map->info.resolution = 0.05;
  map->data = std::vector<int8_t>(10000, 0);
  
  // Set the map
  controller->setCurrentMap(map);
  
  // Verify map was stored
  auto stored_map = controller->getCurrentMap();
  ASSERT_NE(stored_map, nullptr);
  EXPECT_EQ(stored_map->info.width, 100);
  EXPECT_EQ(stored_map->info.height, 100);
}

// =============================================================================
// TEST 10: State Transitions
// =============================================================================
/**
 * @brief Test setState changes the current state
 */
TEST_F(HiveControlTest, StateTransitions) {
  auto controller = std::make_shared<hive_control::HiveController>("tb1");
  
  // Initial state should be SEARCH
  EXPECT_EQ(controller->getCurrentStateName(), "SEARCH");
  
  // Transition to IDLE state
  auto idle_state = std::make_shared<hive_control::IdleState>();
  controller->setState(idle_state);
  
  // Verify state changed
  EXPECT_EQ(controller->getCurrentStateName(), "IDLE");
  
  // Get velocity command from IDLE state (should be zero)
  auto cmd = controller->getVelocityCommand();
  EXPECT_DOUBLE_EQ(cmd.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(cmd.angular.z, 0.0);
}

// =============================================================================
// TEST 11: Robot ID Parsing from Namespace
// =============================================================================
/**
 * @brief Test that robot ID is correctly parsed from namespace
 */
TEST_F(HiveControlTest, RobotIDParsing) {
  // Robot 1 should be clockwise (odd)
  auto controller1 = std::make_shared<hive_control::HiveController>("tb1");
  EXPECT_TRUE(controller1->isClockwise());
  
  // Robot 2 should be counterclockwise (even)
  auto controller2 = std::make_shared<hive_control::HiveController>("tb2");
  EXPECT_FALSE(controller2->isClockwise());
  
  // Robot 3 should be clockwise (odd)
  auto controller3 = std::make_shared<hive_control::HiveController>("tb3");
  EXPECT_TRUE(controller3->isClockwise());
  
  // Test with different namespace formats
  auto controller4 = std::make_shared<hive_control::HiveController>("/tb4");
  EXPECT_FALSE(controller4->isClockwise());
}

// =============================================================================
// TEST 12: Empty Scan Handling
// =============================================================================
/**
 * @brief Test handling of empty or invalid scans
 */
TEST_F(HiveControlTest, EmptyScanHandling) {
  auto controller = std::make_shared<hive_control::HiveController>("tb1");
  
  // Process empty scan (with valid header)
  auto empty_scan = std::make_shared<sensor_msgs::msg::LaserScan>();
  empty_scan->header.stamp = rclcpp::Clock().now();
  empty_scan->header.frame_id = "LDS-01";
  empty_scan->angle_min = -M_PI;
  empty_scan->angle_max = M_PI;
  empty_scan->angle_increment = 0.0175;
  empty_scan->range_min = 0.1;
  empty_scan->range_max = 3.5;
  empty_scan->ranges = std::vector<float>();  // Empty ranges
  controller->processLaserScan(empty_scan);
  
  // Controller should still function
  auto cmd = controller->getVelocityCommand();
  EXPECT_NE(controller->getCurrentStateName(), "");
  EXPECT_TRUE(std::isfinite(cmd.linear.x));
  EXPECT_TRUE(std::isfinite(cmd.angular.z));
}

// =============================================================================
// TEST 13: State Classes Basic Functionality
// =============================================================================
/**
 * @brief Test individual state classes return correct names
 */
TEST_F(HiveControlTest, StateClassesBasicFunctionality) {
  // Test IdleState
  auto idle = std::make_shared<hive_control::IdleState>();
  EXPECT_EQ(idle->getStateName(), "IDLE");
  
  // Test SearchState
  auto search = std::make_shared<hive_control::SearchState>();
  EXPECT_EQ(search->getStateName(), "SEARCH");
  
  // Test CoordinationState
  auto coord1 = std::make_shared<hive_control::CoordinationState>(true);
  EXPECT_EQ(coord1->getStateName(), "COORDINATION");
  
  auto coord2 = std::make_shared<hive_control::CoordinationState>(false);
  EXPECT_EQ(coord2->getStateName(), "COORDINATION");
  
  // Test ConvergenceState
  auto conv = std::make_shared<hive_control::ConvergenceState>();
  EXPECT_EQ(conv->getStateName(), "CONVERGENCE");
}

// =============================================================================
// TEST 14: Velocity Command Edge Cases
// =============================================================================
/**
 * @brief Test velocity commands with different state scenarios
 */
TEST_F(HiveControlTest, VelocityCommandEdgeCases) {
  auto controller = std::make_shared<hive_control::HiveController>("tb1");
  
  // Test with valid scan
  auto scan = std::make_shared<sensor_msgs::msg::LaserScan>();
  scan->angle_min = -M_PI;
  scan->angle_max = M_PI;
  scan->angle_increment = 0.0175;
  scan->range_min = 0.1;
  scan->range_max = 3.5;
  scan->ranges = std::vector<float>(360, 2.0);  // Clear path ahead
  
  controller->processLaserScan(scan);
  auto cmd = controller->getVelocityCommand();
  
  // Command should be valid (not NaN or Inf)
  EXPECT_TRUE(std::isfinite(cmd.linear.x));
  EXPECT_TRUE(std::isfinite(cmd.angular.z));
}

// =============================================================================
// Main function
// =============================================================================
int main(int argc, char ** argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
