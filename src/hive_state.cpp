// Copyright 2025 Shreya Kalyanaraman
#include "hive_control/hive_state.hpp"
#include "hive_control/hive_controller.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace hive_control {

// ================= IdleState =================
void IdleState::handle(HiveController * context, const sensor_msgs::msg::LaserScan::SharedPtr & scan) {
  // No-op for now
}
geometry_msgs::msg::Twist IdleState::getVelocityCommand() {
  return geometry_msgs::msg::Twist();
}

// ================= ExploringState =================
ExploringState::ExploringState() = default;
void ExploringState::handle(HiveController * context, const sensor_msgs::msg::LaserScan::SharedPtr & scan) {
  // Example: transition to ReturnState if obstacle detected
  if (isObstacleDetected(scan)) {
    context->setState(std::make_shared<ReturnState>(context->isClockwise()));
    context->toggleRotationDirection();
  }
}
geometry_msgs::msg::Twist ExploringState::getVelocityCommand() {
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = linear_velocity_;
  return cmd;
}
bool ExploringState::isObstacleDetected(const sensor_msgs::msg::LaserScan::SharedPtr & scan) {
  for (const auto & range : scan->ranges) {
    if (range < obstacle_distance_) {
      return true;
    }
  }
  return false;
}

// ================= ReturnState =================
ReturnState::ReturnState(bool clockwise) : clockwise_(clockwise) {}
void ReturnState::handle(HiveController * context, const sensor_msgs::msg::LaserScan::SharedPtr & scan) {
  if (isPathClear(scan)) {
    context->setState(std::make_shared<ExploringState>());
  }
}
geometry_msgs::msg::Twist ReturnState::getVelocityCommand() {
  geometry_msgs::msg::Twist cmd;
  cmd.angular.z = clockwise_ ? angular_velocity_ : -angular_velocity_;
  return cmd;
}
bool ReturnState::isPathClear(const sensor_msgs::msg::LaserScan::SharedPtr & scan) {
  for (const auto & range : scan->ranges) {
    if (range < clear_distance_) {
      return false;
    }
  }
  return true;
}

} // namespace hive_control
