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
 * @file hive_state.cpp
 * @brief Implementation of HiveState and concrete states
 * @author Shreya Kalyanaraman, Tirth Sadaria
 */
#include "hive_control/hive_state.hpp"
#include "hive_control/hive_controller.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace hive_control
{

// ================= IdleState =================
void IdleState::handle(
  HiveController * context,
  const sensor_msgs::msg::LaserScan::SharedPtr & scan)
{
  // No-op for now
}
geometry_msgs::msg::Twist IdleState::getVelocityCommand()
{
  return geometry_msgs::msg::Twist();
}

// ================= ExploringState =================
ExploringState::ExploringState() = default;
void ExploringState::handle(
  HiveController * context,
  const sensor_msgs::msg::LaserScan::SharedPtr & scan)
{
  // Example: transition to ReturnState if obstacle detected
  if (isObstacleDetected(scan)) {
    context->setState(std::make_shared<ReturnState>(context->isClockwise()));
    context->toggleRotationDirection();
  }
}
geometry_msgs::msg::Twist ExploringState::getVelocityCommand()
{
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = linear_velocity_;
  return cmd;
}
bool ExploringState::isObstacleDetected(const sensor_msgs::msg::LaserScan::SharedPtr & scan)
{
  for (const auto & range : scan->ranges) {
    if (range < obstacle_distance_) {
      return true;
    }
  }
  return false;
}

// ================= ReturnState =================
ReturnState::ReturnState(bool clockwise)
: clockwise_(clockwise) {}
void ReturnState::handle(
  HiveController * context,
  const sensor_msgs::msg::LaserScan::SharedPtr & scan)
{
  if (isPathClear(scan)) {
    context->setState(std::make_shared<ExploringState>());
  }
}
geometry_msgs::msg::Twist ReturnState::getVelocityCommand()
{
  geometry_msgs::msg::Twist cmd;
  cmd.angular.z = clockwise_ ? angular_velocity_ : -angular_velocity_;
  return cmd;
}
bool ReturnState::isPathClear(const sensor_msgs::msg::LaserScan::SharedPtr & scan)
{
  for (const auto & range : scan->ranges) {
    if (range < clear_distance_) {
      return false;
    }
  }
  return true;
}

} // namespace hive_control
