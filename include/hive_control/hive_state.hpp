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
 * @class ExploringState
 * @brief State for actively exploring the environment
 *
 * Initially this can reuse the "move forward until obstacle" behavior
 * from the Walker ForwardState, then expand to frontier-based logic later.
 */
class ExploringState : public HiveState
{
public:
  ExploringState();
  void handle(
    HiveController * context,
    const sensor_msgs::msg::LaserScan::SharedPtr & scan) override;

  geometry_msgs::msg::Twist getVelocityCommand() override;

  std::string getStateName() const override {return "EXPLORING";}

private:
  /**
   * @brief Check if obstacle is detected
   * @param scan Laser scan data
   * @return true if obstacle detected within threshold
   */
  bool isObstacleDetected(
    const sensor_msgs::msg::LaserScan::SharedPtr & scan);

  const double obstacle_distance_ = 0.5;   ///< Obstacle detection threshold (m)
  const double linear_velocity_ = 0.2;     ///< Forward velocity (m/s)
};

/**
 * @class ReturnState
 * @brief State for returning to deployment / home zone
 *
 * For now this can mirror RotateState behavior (rotate until path clear),
 * and later be extended to follow a planned path home.
 */
class ReturnState : public HiveState
{
public:
  /**
   * @brief Constructor
   * @param clockwise Direction of rotation
   */
  explicit ReturnState(bool clockwise);

  void handle(
    HiveController * context,
    const sensor_msgs::msg::LaserScan::SharedPtr & scan) override;

  geometry_msgs::msg::Twist getVelocityCommand() override;

  std::string getStateName() const override
  {
    return clockwise_ ? "RETURN_CLOCKWISE" : "RETURN_COUNTERCLOCKWISE";
  }

private:
  /**
   * @brief Check if path ahead is clear
   * @param scan Laser scan data
   * @return true if path is clear
   */
  bool isPathClear(
    const sensor_msgs::msg::LaserScan::SharedPtr & scan);

  bool clockwise_;                        ///< Rotation direction
  const double angular_velocity_ = 0.5;   ///< Rotation speed (rad/s)
  const double clear_distance_ = 0.8;     ///< Minimum clear distance (m)
};

}  // namespace hive_control

#endif  // HIVE_CONTROL__HIVE_STATE_HPP_
