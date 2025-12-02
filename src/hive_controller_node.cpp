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

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "hive_control/hive_controller.hpp"

using std::chrono_literals::operator""ms;

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
    // Initialize HIVE controller context
    controller_ = std::make_unique<hive_control::HiveController>();

    // Declare parameters for topic names (for flexibility / multi-robot use)
    this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    this->declare_parameter<std::string>("scan_topic", "/scan");

    const std::string cmd_vel_topic =
      this->get_parameter("cmd_vel_topic").as_string();
    const std::string scan_topic =
      this->get_parameter("scan_topic").as_string();

    // Create publisher for velocity commands
    velocity_pub_ =
      this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);

    // Create subscriber for laser scan
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic, 10,
      std::bind(&HiveControllerNode::laserCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Subscribing to: %s", scan_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing to: %s", cmd_vel_topic.c_str());

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
    // Process scan and update state
    controller_->processLaserScan(msg);

    // Log state changes (optional)
    static std::string last_state = "";
    const std::string current_state = controller_->getCurrentStateName();
    if (current_state != last_state) {
      RCLCPP_INFO(this->get_logger(), "State: %s", current_state.c_str());
      last_state = current_state;
    }
  }

  /**
   * @brief Timer callback to publish velocity commands
   */
  void timerCallback()
  {
    auto cmd = controller_->getVelocityCommand();
    velocity_pub_->publish(cmd);
  }

  /// HIVE controller context (State pattern brain)
  std::unique_ptr<hive_control::HiveController> controller_;

  /// Publisher for velocity commands
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;

  /// Subscriber for laser scan
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;

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
