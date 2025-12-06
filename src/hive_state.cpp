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

// ================= SearchState =================
SearchState::SearchState() = default;

void SearchState::handle(
  HiveController * context,
  const sensor_msgs::msg::LaserScan::SharedPtr & scan)
{
  search_duration_counter_++;
  
  // Transition to coordination if obstacle detected
  if (isObstacleDetected(scan)) {
    context->setState(std::make_shared<CoordinationState>(context->isClockwise()));
    return;
  }
  
  // Transition to convergence after maximum search time
  if (search_duration_counter_ > MAX_SEARCH_TIME || needsCoordination()) {
    context->setState(std::make_shared<ConvergenceState>());
  }
}

geometry_msgs::msg::Twist SearchState::getVelocityCommand()
{
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = linear_velocity_;
  return cmd;
}

bool SearchState::isObstacleDetected(const sensor_msgs::msg::LaserScan::SharedPtr & scan)
{
  // Check front-facing sensors (middle third of scan)
  size_t start_idx = scan->ranges.size() / 3;
  size_t end_idx = 2 * scan->ranges.size() / 3;
  
  for (size_t i = start_idx; i < end_idx; ++i) {
    if (scan->ranges[i] < obstacle_distance_) {
      return true;
    }
  }
  return false;
}

bool SearchState::needsCoordination() const
{
  // Simple heuristic: coordinate periodically to avoid robot conflicts
  return (search_duration_counter_ % 50) == 0;
}

// ================= CoordinationState =================
CoordinationState::CoordinationState(bool turn_right)
: turn_right_(turn_right) {}

void CoordinationState::handle(
  HiveController * context,
  const sensor_msgs::msg::LaserScan::SharedPtr & scan)
{
  coordination_timer_++;
  
  // Return to search if path is clear and coordination time elapsed
  if (isPathClear(scan) && coordination_timer_ > COORDINATION_TIME) {
    context->setState(std::make_shared<SearchState>());
    context->toggleRotationDirection(); // Vary behavior for swarm diversity
  }
}

geometry_msgs::msg::Twist CoordinationState::getVelocityCommand()
{
  geometry_msgs::msg::Twist cmd;
  cmd.angular.z = turn_right_ ? -angular_velocity_ : angular_velocity_;
  return cmd;
}

bool CoordinationState::isPathClear(const sensor_msgs::msg::LaserScan::SharedPtr & scan)
{
  // Check if front path is clear for resuming search
  size_t start_idx = scan->ranges.size() / 3;
  size_t end_idx = 2 * scan->ranges.size() / 3;
  
  for (size_t i = start_idx; i < end_idx; ++i) {
    if (scan->ranges[i] < clear_distance_) {
      return false;
    }
  }
  return true;
}

// ================= ConvergenceState =================
ConvergenceState::ConvergenceState() = default;

void ConvergenceState::handle(
  HiveController * context,
  const sensor_msgs::msg::LaserScan::SharedPtr & scan)
{
  // Simple convergence behavior: move toward center, then rotate
  if (isAtRendezvous()) {
    rotating_to_center_ = true;
    // In a real system, this would trigger map merging
  }
}

geometry_msgs::msg::Twist ConvergenceState::getVelocityCommand()
{
  geometry_msgs::msg::Twist cmd;
  
  if (rotating_to_center_) {
    cmd.angular.z = angular_velocity_;
  } else {
    cmd.linear.x = linear_velocity_;
  }
  
  return cmd;
}

bool ConvergenceState::isAtRendezvous() const
{
  // Simplified: assume we've reached rendezvous after moving for some time
  // In real system, this would check odometry/position
  return false; // Placeholder - would use actual position check
}

} // namespace hive_control
