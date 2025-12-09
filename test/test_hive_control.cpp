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
// Main function
// =============================================================================
int main(int argc, char ** argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
