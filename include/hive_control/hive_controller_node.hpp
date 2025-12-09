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
 * @file hive_controller_node.hpp
 * @brief ROS2 node class for HIVE robot behavior
 * @author Shreya Kalyanaraman, Tirth Sadaria
 */

#ifndef HIVE_CONTROL__HIVE_CONTROLLER_NODE_HPP_
#define HIVE_CONTROL__HIVE_CONTROLLER_NODE_HPP_

#include <memory>
#include <vector>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "hive_control/hive_controller.hpp"

/**
 * @struct MapGrowthData
 * @brief Data structure for tracking map growth over time
 */
struct MapGrowthData {
  int known_cell_count = 0;
  rclcpp::Time timestamp;
};

/**
 * @class HiveControllerNode
 * @brief ROS2 node implementing robot behavior via HiveController
 *
 * This node subscribes to laser scan data and publishes velocity commands
 * by delegating decision logic to the State-pattern-based HiveController.
 */
class HiveControllerNode : public rclcpp::Node
{
public:
  /**
   * @brief Constructor - sets up publishers, subscribers, and timer
   */
  HiveControllerNode();

private:
  /**
   * @brief Callback for laser scan messages
   * @param msg Laser scan data
   */
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  /**
   * @brief Callback for map messages (for frontier exploration and completion detection)
   */
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  /**
   * @brief Save the merged map as PNG file when mapping is complete
   */
  void saveMapAsPNG();

  /**
   * @brief Timer callback to publish velocity commands
   */
  void timerCallback();

  /// HIVE controller context (State pattern brain)
  std::unique_ptr<hive_control::HiveController> controller_;

  /// Publisher for velocity commands
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;

  /// Subscriber for laser scan
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  
  /// Subscriber for map (for frontier exploration)
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  
  /// Latest map data
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
  
  /// Map completion detection (growth rate monitoring)
  bool mapping_complete_ = false;
  bool enable_map_completion_ = true;

  // Map growth monitoring (better than unknown percentage)
  std::vector<MapGrowthData> map_growth_history_;  // Store history for growth rate calculation
  static constexpr double GROWTH_CHECK_INTERVAL = 30.0;  // Check growth every 30 seconds
  static constexpr double MIN_GROWTH_RATE = 0.01;  // 1% growth threshold
  static constexpr double MIN_EXPLORATION_TIME = 300.0;  // Minimum 300 seconds (5 minutes)

  /// Timer for publishing commands
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // HIVE_CONTROL__HIVE_CONTROLLER_NODE_HPP_

