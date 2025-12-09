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
 * @file hive_controller.cpp
 * @brief Implementation of HiveController and state management
 * @author Shreya Kalyanaraman, Tirth Sadaria
 */

#include "hive_control/hive_controller.hpp"
#include "hive_control/hive_state.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace hive_control
{

// ============================================================================
// HiveController Implementation
// ============================================================================

/**
 * @brief Construct a new HiveController object
 * @param namespace_str ROS2 namespace string (e.g., "tb1" or "/tb1")
 *
 * Initializes the controller in SEARCH state for autonomous exploration.
 * Extracts robot ID from namespace to determine rotation direction:
 * - Odd-numbered robots (1, 3, 5...) rotate clockwise
 * - Even-numbered robots (2, 4, 6...) rotate counterclockwise
 * This alternation helps prevent robots from bunching together.
 */
HiveController::HiveController(const std::string & namespace_str)
: clockwise_(true), battery_level_(1.0), current_state_id_(StateID::EXPLORING),
  latest_map_(nullptr), latest_odom_(nullptr)
{
  // Initialize in search state for autonomous exploration
  current_state_ = std::make_shared<SearchState>();

  // Parse robot ID from namespace string
  // Supports formats: "tb1", "/tb1", "robot1", etc.
  int robot_id = 0;

  try {
    // Extract numeric suffix from namespace
    size_t last_index = namespace_str.find_last_not_of("0123456789");
    if (last_index != std::string::npos && last_index + 1 < namespace_str.length()) {
      robot_id = std::stoi(namespace_str.substr(last_index + 1));
    } else if (!namespace_str.empty()) {
      // Handle cases where namespace contains only digits
      size_t first_digit = namespace_str.find_first_of("0123456789");
      if (first_digit != std::string::npos) {
        robot_id = std::stoi(namespace_str.substr(first_digit));
      }
    }
  } catch (...) {
    // Default to robot 1 if parsing fails
    robot_id = 1;
  }

  // Alternate rotation direction based on robot ID to prevent bunching
  // Odd robots: clockwise, Even robots: counterclockwise
  clockwise_ = (robot_id % 2 != 0);
}

/**
 * @brief Process incoming laser scan data
 * @param scan Shared pointer to laser scan message
 *
 * Stores the scan and delegates processing to the current state.
 * Automatically transitions from IDLE to SEARCH when valid scan data
 * is received to prevent robots from remaining idle when sensors are active.
 */
void HiveController::processLaserScan(
  const sensor_msgs::msg::LaserScan::SharedPtr & scan)
{
  // Store scan for state classes to access
  latest_scan_ = scan;

  // Auto-transition from IDLE to SEARCH when valid scan data arrives
  // This ensures robots begin exploration when sensors become active
  if (current_state_id_ == StateID::IDLE && scan && !scan->ranges.empty()) {
    // Validate scan data quality
    bool has_valid_data = false;
    for (const auto& range : scan->ranges) {
      if (std::isfinite(range) && range > scan->range_min &&
          range < scan->range_max) {
        has_valid_data = true;
        break;
      }
    }

    // Transition to exploration mode if scan data is valid
    if (has_valid_data) {
      current_state_ = std::make_shared<SearchState>();
      current_state_id_ = StateID::EXPLORING;
    }
  }

  // Delegate scan processing to current state
  if (current_state_) {
    current_state_->handle(this, scan);
  }
}

/**
 * @brief Get velocity command from current state
 * @return Twist message with linear and angular velocities
 *
 * Delegates to the current state to compute velocity commands based on
 * sensor data and state logic.
 */
geometry_msgs::msg::Twist HiveController::getVelocityCommand() const
{
  if (current_state_) {
    // States need non-const access to retrieve scan data
    return current_state_->getVelocityCommand(
      const_cast<HiveController*>(this));
  }
  return geometry_msgs::msg::Twist();
}

/**
 * @brief Transition to a new state
 * @param new_state Shared pointer to the new state object
 *
 * Updates both the state object and the corresponding state ID enum
 * to maintain consistency between the state machine and high-level state tracking.
 */
void HiveController::setState(std::shared_ptr<HiveState> new_state)
{
  current_state_ = std::move(new_state);

  // Synchronize state ID enum with state object
  if (current_state_) {
    std::string state_name = current_state_->getStateName();
    if (state_name == "IDLE") {
      current_state_id_ = StateID::IDLE;
    } else if (state_name == "SEARCH") {
      current_state_id_ = StateID::EXPLORING;
    } else if (state_name == "CONVERGENCE") {
      current_state_id_ = StateID::RETURN;
    } else {
      // COORDINATION and other states map to EXPLORING
      current_state_id_ = StateID::EXPLORING;
    }
  }
}

/**
 * @brief Get the name of the current state
 * @return State name as string (e.g., "SEARCH", "IDLE", "COORDINATION")
 */
std::string HiveController::getCurrentStateName() const
{
  if (current_state_) {
    return current_state_->getStateName();
  }
  return "UNKNOWN";
}

}  // namespace hive_control
