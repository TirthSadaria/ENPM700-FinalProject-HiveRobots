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
 * @file hive_controller.hpp
 * @brief HIVE context class managing state transitions
 * @author Shreya
 */

#ifndef HIVE_CONTROL__HIVE_CONTROLLER_HPP_
#define HIVE_CONTROL__HIVE_CONTROLLER_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace hive_control
{

class HiveState;  // forward declaration for State pattern

/**
 * @enum StateID
 * @brief High-level identifiers for controller states
 */
enum class StateID
{
  IDLE,
  EXPLORING,
  RETURN
};

/**
 * @class HiveController
 * @brief Context class for State design pattern, manages robot behavior
 *
 * This class maintains the current state and handles state transitions
 * for the HIVE robot. It can also track battery level and choose
 * behaviors like exploring vs returning home.
 */
class HiveController
{
public:
  /**
   * @brief Constructor - initializes in IDLE state
   * @param namespace_str ROS2 namespace (e.g., "/tb1" or "tb1") to extract robot ID
   */
  explicit HiveController(const std::string & namespace_str = "");

  /**
   * @brief Process laser scan and update state (main update entrypoint)
   * @param scan Latest laser scan data
   */
  void processLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr & scan);

  /**
   * @brief Get current velocity command
   * @return Twist message for robot movement
   */
  geometry_msgs::msg::Twist getVelocityCommand() const;

  /**
   * @brief Transition to a new state
   * @param new_state Shared pointer to new state
   */
  void setState(std::shared_ptr<HiveState> new_state);

  /**
   * @brief Get current rotation direction
   * @return true for clockwise, false for counterclockwise
   */
  bool isClockwise() const {return clockwise_;}

  /**
   * @brief Toggle rotation direction for next rotation
   */
  void toggleRotationDirection() {clockwise_ = !clockwise_;}

  /**
   * @brief Get current state name (for debugging/logging)
   * @return State name as string
   */
  std::string getCurrentStateName() const;

  /**
   * @brief Set the simulated battery level [0.0, 1.0]
   */
  void setBatteryLevel(double level) {battery_level_ = level;}

  /**
   * @brief Get current battery level
   */
  double getBatteryLevel() const {return battery_level_;}

  /**
   * @brief Get current high-level state identifier
   */
  StateID getCurrentStateId() const {return current_state_id_;}

  /**
   * @brief Get the latest laser scan data
   * @return Shared pointer to latest scan, or nullptr if no scan received yet
   */
  sensor_msgs::msg::LaserScan::SharedPtr getCurrentScan() const {return latest_scan_;}

  /**
   * @brief Set the latest map data (for frontier exploration)
   * @param map Shared pointer to latest map
   */
  void setCurrentMap(const nav_msgs::msg::OccupancyGrid::SharedPtr & map) {latest_map_ = map;}

  /**
   * @brief Get the latest map data
   * @return Shared pointer to latest map, or nullptr if no map received yet
   */
  nav_msgs::msg::OccupancyGrid::SharedPtr getCurrentMap() const {return latest_map_;}

private:
  std::shared_ptr<HiveState> current_state_;  ///< Current concrete state
  bool clockwise_;                            ///< Rotation direction flag
  double battery_level_;                      ///< Simulated battery level
  StateID current_state_id_;                  ///< High-level state label
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;  ///< Latest laser scan data
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;  ///< Latest map data (for frontier exploration)
};

}  // namespace hive_control

#endif  // HIVE_CONTROL__HIVE_CONTROLLER_HPP_
