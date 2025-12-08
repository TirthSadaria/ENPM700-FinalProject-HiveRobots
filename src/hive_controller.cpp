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
 * @brief Implementation of HiveContext and state classes
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

HiveController::HiveController(const std::string & namespace_str)
: clockwise_(true), battery_level_(1.0), current_state_id_(StateID::EXPLORING), latest_map_(nullptr)
{
  // Initialize in search state for HIVE swarm behavior
  current_state_ = std::make_shared<SearchState>();
  
  // Parse robot ID from namespace instead of static counter
  // Namespace format is usually "/tbX" or "tbX"
  int robot_id = 0;
  
  try {
    // Find the last number in the string
    size_t last_index = namespace_str.find_last_not_of("0123456789");
    if (last_index != std::string::npos && last_index + 1 < namespace_str.length()) {
      robot_id = std::stoi(namespace_str.substr(last_index + 1));
    } else if (!namespace_str.empty()) {
      // If namespace is all digits (e.g., "1" or "tb1" without separator)
      // Try to extract number from the end
      size_t first_digit = namespace_str.find_first_of("0123456789");
      if (first_digit != std::string::npos) {
        robot_id = std::stoi(namespace_str.substr(first_digit));
      }
    }
  } catch (...) {
    // Fallback if parsing fails
    robot_id = 1;
  }
  
  // Use the ID to determine behavior
  // Robot 1: clockwise (true), Robot 2: counterclockwise (false), Robot 3: clockwise (true)...
  clockwise_ = (robot_id % 2 != 0);
  
  // Note: Logging would require ROS2 node, so we skip it here
  // The node can log the robot ID if needed
}

void HiveController::processLaserScan(
  const sensor_msgs::msg::LaserScan::SharedPtr & scan)
{
  // Store the latest scan for state classes to access
  latest_scan_ = scan;
  
  if (current_state_) {
    current_state_->handle(this, scan);
  }
}

geometry_msgs::msg::Twist HiveController::getVelocityCommand() const
{
  if (current_state_) {
    // Pass const_cast to allow state to access context (states need non-const for getCurrentScan)
    return current_state_->getVelocityCommand(const_cast<HiveController*>(this));
  }
  return geometry_msgs::msg::Twist();
}

void HiveController::setState(std::shared_ptr<HiveState> new_state)
{
  current_state_ = std::move(new_state);
}

std::string HiveController::getCurrentStateName() const
{
  if (current_state_) {
    return current_state_->getStateName();
  }
  return "UNKNOWN";
}

}  // namespace hive_control
