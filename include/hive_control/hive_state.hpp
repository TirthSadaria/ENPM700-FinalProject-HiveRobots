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
 * @file hive_state.hpp
 * @brief State pattern interface and concrete states for HIVE robot
 * @author Shreya K.
 */

#ifndef HIVE_CONTROL__HIVE_STATE_HPP_
#define HIVE_CONTROL__HIVE_STATE_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace hive_control
{

// Forward declaration
class HiveController;

/**
 * @class HiveState
 * @brief Abstract base class for HIVE states using the State design pattern
 */
class HiveState
{
public:
  virtual ~HiveState() = default;

  /**
   * @brief Handle state logic and potential transitions
   * @param context Pointer to HIVE controller context
   * @param scan Latest laser scan data
   */
  virtual void handle(
    HiveController * context,
    const sensor_msgs::msg::LaserScan::SharedPtr & scan) = 0;

  /**
   * @brief Get velocity command for current state
   * @return Twist message with velocity commands
   */
  virtual geometry_msgs::msg::Twist getVelocityCommand() = 0;

  /**
   * @brief Get name of current state (for debugging)
   * @return State name as string
   */
  virtual std::string getStateName() const = 0;
};

/**
 * @class IdleState
 * @brief State for holding position / waiting for mission start
 */
class IdleState : public HiveState
{
public:
  void handle(
    HiveController * context,
    const sensor_msgs::msg::LaserScan::SharedPtr & scan) override;

  geometry_msgs::msg::Twist getVelocityCommand() override;

  std::string getStateName() const override {return "IDLE";}
};

/**
 * @class SearchState
 * @brief State for distributed search pattern in HIVE swarm
 */
class SearchState : public HiveState
{
public:
  SearchState();
  void handle(
    HiveController * context,
    const sensor_msgs::msg::LaserScan::SharedPtr & scan) override;

  geometry_msgs::msg::Twist getVelocityCommand() override;

  std::string getStateName() const override {return "SEARCH";}

private:
  bool isObstacleDetected(const sensor_msgs::msg::LaserScan::SharedPtr & scan);
  bool needsCoordination() const;
  
  int search_duration_counter_ = 0;
  const double obstacle_distance_ = 0.5;
  const double linear_velocity_ = 0.1;
  static constexpr int MAX_SEARCH_TIME = 100;
};

/**
 * @class CoordinationState
 * @brief State for coordinating with other robots to avoid conflicts
 */
class CoordinationState : public HiveState
{
public:
  explicit CoordinationState(bool turn_right = true);
  void handle(
    HiveController * context,
    const sensor_msgs::msg::LaserScan::SharedPtr & scan) override;

  geometry_msgs::msg::Twist getVelocityCommand() override;

  std::string getStateName() const override {return "COORDINATION";}

private:
  bool isPathClear(const sensor_msgs::msg::LaserScan::SharedPtr & scan);
  
  bool turn_right_;
  int coordination_timer_ = 0;
  const double angular_velocity_ = 0.5;
  const double clear_distance_ = 0.7;
  static constexpr int COORDINATION_TIME = 30;
};

/**
 * @class ConvergenceState
 * @brief State for converging to rendezvous point for map merging
 */
class ConvergenceState : public HiveState
{
public:
  ConvergenceState();
  void handle(
    HiveController * context,
    const sensor_msgs::msg::LaserScan::SharedPtr & scan) override;

  geometry_msgs::msg::Twist getVelocityCommand() override;

  std::string getStateName() const override {return "CONVERGENCE";}

private:
  bool isAtRendezvous() const;
  
  bool rotating_to_center_ = false;
  const double linear_velocity_ = 0.1;
  const double angular_velocity_ = 0.5;
};

}  // namespace hive_control

#endif  // HIVE_CONTROL__HIVE_STATE_HPP_
