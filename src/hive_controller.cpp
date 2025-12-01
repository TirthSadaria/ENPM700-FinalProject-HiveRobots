// Copyright 2025 Shreya Kalyanaraman
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
 * @author Shreya
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

HiveController::HiveController()
: clockwise_(true)
{
  // Initialize in exploring state (equivalent to ForwardState in walker)
  current_state_ = std::make_shared<ExploringState>();
}

void HiveController::processLaserScan(
  const sensor_msgs::msg::LaserScan::SharedPtr & scan)
{
  if (current_state_) {
    current_state_->handle(this, scan);
  }
}

geometry_msgs::msg::Twist HiveController::getVelocityCommand() const
{
  if (current_state_) {
    return current_state_->getVelocityCommand();
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
