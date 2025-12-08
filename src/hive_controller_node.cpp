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
 * @file hive_controller_node.cpp
 * @brief ROS2 node for HIVE robot behavior (State pattern wrapper)
 * @author Shreya
 */

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "hive_control/hive_controller.hpp"

using std::chrono_literals::operator""ms;

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
 * @brief ROS2 node implementing robot behavior via HiveContext
 *
 * This node subscribes to laser scan data and publishes velocity commands
 * by delegating decision logic to the State-pattern-based HiveContext.
 */
class HiveControllerNode : public rclcpp::Node
{
public:
  /**
   * @brief Constructor - sets up publishers, subscribers, and timer
   */
  HiveControllerNode()
  : Node("hive_controller_node")
  {
    // Get namespace to pass to controller for robot ID extraction
    std::string namespace_str = this->get_namespace();
    
    // Initialize HIVE controller context with namespace for robot ID parsing
    controller_ = std::make_unique<hive_control::HiveController>(namespace_str);
    
    // Log robot ID for debugging
    // Extract robot ID from namespace for logging
    int robot_id = 0;
    try {
      size_t last_index = namespace_str.find_last_not_of("0123456789");
      if (last_index != std::string::npos && last_index + 1 < namespace_str.length()) {
        robot_id = std::stoi(namespace_str.substr(last_index + 1));
      } else if (!namespace_str.empty()) {
        size_t first_digit = namespace_str.find_first_of("0123456789");
        if (first_digit != std::string::npos) {
          robot_id = std::stoi(namespace_str.substr(first_digit));
        }
      }
    } catch (...) {
      robot_id = 1;
    }
    RCLCPP_INFO(this->get_logger(), "Initialized Robot %d (Clockwise: %s)", 
                robot_id, controller_->isClockwise() ? "YES" : "NO");

    // Declare parameters for topic names (for flexibility / multi-robot use)
    // Use relative names (not absolute) so they work with namespaces
    this->declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");
    this->declare_parameter<std::string>("scan_topic", "scan");
    this->declare_parameter<std::string>("map_topic", "map");
    this->declare_parameter<bool>("enable_map_completion", true);

    const std::string cmd_vel_topic =
      this->get_parameter("cmd_vel_topic").as_string();
    const std::string scan_topic =
      this->get_parameter("scan_topic").as_string();
    const std::string map_topic =
      this->get_parameter("map_topic").as_string();
    enable_map_completion_ =
      this->get_parameter("enable_map_completion").as_bool();

    // Create publisher for velocity commands with BestEffort QoS
    // BestEffort prevents blocking when controllers are temporarily unavailable
    rclcpp::QoS cmd_vel_qos(10);
    cmd_vel_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    cmd_vel_qos.durability(rclcpp::DurabilityPolicy::Volatile);
    velocity_pub_ =
      this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, cmd_vel_qos);

    // Create subscriber for laser scan with SensorData QoS (required for sensor topics)
    rclcpp::QoS scan_qos = rclcpp::SensorDataQoS();
    
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic, scan_qos,
      std::bind(&HiveControllerNode::laserCallback, this, std::placeholders::_1));
    
    // Subscribe to map for frontier-based exploration and completion detection
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic, 10,
      std::bind(&HiveControllerNode::mapCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Subscribing to: %s (will resolve to namespace)", scan_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing to: %s (will resolve to namespace)", cmd_vel_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Initial state: %s", controller_->getCurrentStateName().c_str());

    // Create timer for publishing velocity commands at fixed rate
    timer_ = this->create_wall_timer(
      100ms, std::bind(&HiveControllerNode::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "HIVE controller node initialized");
  }

private:
  /**
   * @brief Callback for laser scan messages
   * @param msg Laser scan data
   */
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // Validate scan data before processing
    if (!msg || msg->ranges.empty()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                           "Received empty or invalid scan!");
      return;
    }
    
    // Count valid ranges
    size_t valid_ranges = 0;
    double min_range = msg->range_max;
    double max_range = 0.0;
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      float range = msg->ranges[i];
      if (std::isfinite(range) && range > msg->range_min && range < msg->range_max) {
        valid_ranges++;
        if (range < min_range) min_range = range;
        if (range > max_range) max_range = range;
      }
    }
    
    // Process scan and update state
    controller_->processLaserScan(msg);

    // Log state changes and scan reception
    static std::string last_state = "";
    const std::string current_state = controller_->getCurrentStateName();
    if (current_state != last_state) {
      RCLCPP_INFO(this->get_logger(), "State: %s", current_state.c_str());
      last_state = current_state;
    }
    
    // Debug: Log scan reception with validation info
    static int scan_count = 0;
    scan_count++;
    if (scan_count == 1) {
      RCLCPP_INFO(this->get_logger(), 
                  "First scan received! Total ranges: %zu, Valid: %zu, Min: %.2fm, Max: %.2fm", 
                  msg->ranges.size(), valid_ranges, min_range, max_range);
    }
    if (scan_count % 50 == 0) {  // Log every 50th scan to avoid spam
      RCLCPP_INFO(this->get_logger(), 
                  "Scan #%d: Valid ranges: %zu/%zu, Min: %.2fm, Max: %.2fm", 
                  scan_count, valid_ranges, msg->ranges.size(), min_range, max_range);
    }
  }

  /**
   * @brief Callback for map messages (for frontier exploration and completion detection)
   */
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    latest_map_ = msg;
    // Also update the controller with the latest map for frontier exploration
    if (controller_ && msg) {
      controller_->setCurrentMap(msg);
    }
    
    if (enable_map_completion_ && msg && !mapping_complete_) {
      // Improved map completion detection: Monitor growth rate instead of unknown percentage
      // This detects when exploration stops (no new areas discovered)
      
      // Count known cells (not unknown, i.e., not -1)
      int known_count = 0;
      for (int8_t cell : msg->data) {
        if (cell != -1) {  // Known cell (free or occupied)
          known_count++;
        }
      }
      
      // Store current map state with timestamp
      rclcpp::Time current_time = this->get_clock()->now();
      MapGrowthData current_data;
      current_data.known_cell_count = known_count;
      current_data.timestamp = current_time;
      
      map_growth_history_.push_back(current_data);
      
      // Keep only recent history (last 2 minutes worth)
      rclcpp::Time cutoff_time = current_time - rclcpp::Duration::from_seconds(120.0);
      map_growth_history_.erase(
        std::remove_if(map_growth_history_.begin(), map_growth_history_.end(),
          [cutoff_time](const MapGrowthData& data) {
            return data.timestamp < cutoff_time;
          }),
        map_growth_history_.end()
      );
      
      // Check if we have enough history and enough time has passed
      if (map_growth_history_.size() >= 2) {
        // Find data from GROWTH_CHECK_INTERVAL seconds ago
        rclcpp::Time check_time = current_time - rclcpp::Duration::from_seconds(GROWTH_CHECK_INTERVAL);
        
        // Find closest historical data point (from GROWTH_CHECK_INTERVAL seconds ago)
        MapGrowthData* old_data = nullptr;
        for (size_t i = 0; i < map_growth_history_.size(); ++i) {
          if (map_growth_history_[i].timestamp <= check_time) {
            old_data = &map_growth_history_[i];
          } else {
            break;
          }
        }
        
        if (old_data) {
          // Calculate growth rate
          int growth = current_data.known_cell_count - old_data->known_cell_count;
          double growth_rate = (old_data->known_cell_count > 0) ? 
            static_cast<double>(growth) / old_data->known_cell_count : 0.0;
          
          // Check if enough time has passed and growth is minimal
          double elapsed_time = (current_time - map_growth_history_[0].timestamp).seconds();
          
          if (elapsed_time >= MIN_EXPLORATION_TIME && growth_rate < MIN_GROWTH_RATE) {
            mapping_complete_ = true;
            RCLCPP_INFO(this->get_logger(), 
                       "Map completion detected! Growth rate: %.2f%% over last %.1fs - Stopping exploration",
                       growth_rate * 100.0, GROWTH_CHECK_INTERVAL);
            
            // Save the map as PNG when complete
            saveMapAsPNG();
          }
        }
      }
    }
  }
  
  /**
   * @brief Save the merged map as PNG file when mapping is complete
   */
  void saveMapAsPNG()
  {
    static bool map_saved = false;  // Only save once
    if (map_saved) {
      return;
    }
    map_saved = true;
    
    // Create results directory if it doesn't exist
    std::string results_dir = "results";
    std::string mkdir_cmd = "mkdir -p " + results_dir;
    system(mkdir_cmd.c_str());
    
    // Save individual robot map (merged map may not be available)
    std::string namespace_str = this->get_namespace();
    std::string map_topic = namespace_str + "/map";
    std::string output_file = results_dir + "/" + namespace_str.substr(1) + "_map_complete";  // Save to results folder
    
    RCLCPP_INFO(this->get_logger(), "Saving map to %s from topic %s", output_file.c_str(), map_topic.c_str());
    
    // Use map_saver_cli to save the map (requires nav2_map_server package)
    // Run in background so it doesn't block
    std::string command = "ros2 run nav2_map_server map_saver_cli -f " + output_file + " --ros-args -r map:=" + map_topic + " &";
    system(command.c_str());
    
    // Also try to save merged map if available (non-blocking, may fail silently)
    std::string merged_command = "ros2 run nav2_map_server map_saver_cli -f " + results_dir + "/merged_map_complete --ros-args -r map:=/map_merged &";
    system(merged_command.c_str());
  }
  
  /**
   * @brief Timer callback to publish velocity commands
   */
  void timerCallback()
  {
    // Stop if mapping is complete
    if (mapping_complete_) {
      geometry_msgs::msg::Twist cmd;  // Zero velocity = stop
      velocity_pub_->publish(cmd);
      return;
    }
    
    auto cmd = controller_->getVelocityCommand();
    
    // Debug: Log velocity commands (INFO level for easier debugging)
    static int cmd_count = 0;
    cmd_count++;
    if (cmd.linear.x != 0.0 || cmd.angular.z != 0.0) {
      if (cmd_count % 10 == 0) {  // Log every 10th command to avoid spam
        RCLCPP_INFO(this->get_logger(), "Publishing cmd: linear.x=%.2f m/s, angular.z=%.2f rad/s", 
                     cmd.linear.x, cmd.angular.z);
      }
    }
    
    velocity_pub_->publish(cmd);
  }

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
  static constexpr double MIN_EXPLORATION_TIME = 120.0;  // Minimum 120 seconds (2 minutes) before checking - balanced for coverage and speed
  
  /// Timer for publishing commands
  rclcpp::TimerBase::SharedPtr timer_;
};

/**
 * @brief Main function
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HiveControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
