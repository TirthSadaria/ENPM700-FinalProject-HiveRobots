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
 * @file scan_frame_remapper.cpp
 * @brief Remaps laser scan frame_id from LDS-01 to {namespace}/LDS-01 for SLAM compatibility
 */

#include <memory>
#include <string>
#include <algorithm>  // For std::reverse, std::swap
#include <vector>
#include <cmath>  // For M_PI
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class ScanFrameRemapper : public rclcpp::Node
{
public:
  ScanFrameRemapper()
  : Node("scan_frame_remapper")
  {
    // Get namespace from node namespace
    std::string ns = this->get_namespace();
    
    // Remove leading slash if present
    if (!ns.empty() && ns[0] == '/') {
      ns = ns.substr(1);
    }
    
    // If still empty, try to get from parameter
    if (ns.empty()) {
      this->declare_parameter<std::string>("robot_namespace", "");
      ns = this->get_parameter("robot_namespace").as_string();
      if (!ns.empty() && ns[0] == '/') {
        ns = ns.substr(1);
      }
    }
    
    // Build remapped frame_id
    if (!ns.empty()) {
      remapped_frame_id_ = ns + "/LDS-01";
    } else {
      // Fallback: try to get from environment or use default
      remapped_frame_id_ = "LDS-01";
      RCLCPP_WARN(this->get_logger(), 
                  "No namespace detected! Remapping LDS-01 -> LDS-01 (no namespace). "
                  "This node should be launched with a namespace (e.g., /tb1/scan_frame_remapper)");
    }
    
    // Create subscriber and publisher
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan_in", 10,
      std::bind(&ScanFrameRemapper::scanCallback, this, std::placeholders::_1));
    
    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
      "scan", 10);
    
    RCLCPP_INFO(this->get_logger(), 
                "Scan frame remapper: LDS-01 -> %s", remapped_frame_id_.c_str());
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // Skip empty scans
    if (msg->ranges.empty()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                            "Received empty scan, skipping");
      return;
    }
    
    // Create a copy and remap the frame_id
    auto remapped_msg = std::make_shared<sensor_msgs::msg::LaserScan>(*msg);
    remapped_msg->header.frame_id = remapped_frame_id_;
    
    // CRITICAL FIX: Use current node time instead of original scan timestamp
    remapped_msg->header.stamp = this->now();
    
    // CRITICAL FIX: Normalize scan angles for SLAM compatibility
    // Webots lidar outputs: angle_min=π, angle_max=-π, angle_increment=-0.017 (inverted)
    // SLAM expects: angle_min=-π, angle_max=π, angle_increment=+0.017 (standard)
    // 
    // SLAM toolbox calculates expected_count = (angle_max - angle_min) / angle_increment
    // If angle_increment is negative, this gives a negative number which overflows to ~4 billion
    // So we MUST ensure angle_increment is positive!
    
    bool needs_reversal = (remapped_msg->angle_increment < 0);
    
    if (needs_reversal) {
      // Reverse the ranges array
      std::reverse(remapped_msg->ranges.begin(), remapped_msg->ranges.end());
      
      // Also reverse intensities if present
      if (!remapped_msg->intensities.empty()) {
        std::reverse(remapped_msg->intensities.begin(), remapped_msg->intensities.end());
      }
      
      // Swap angle_min and angle_max
      std::swap(remapped_msg->angle_min, remapped_msg->angle_max);
      
      // Make increment positive
      remapped_msg->angle_increment = std::abs(remapped_msg->angle_increment);
    }
    
    // ADDITIONAL SAFETY: Ensure angle conventions are correct
    // Standard convention: angle_min < angle_max with positive increment
    if (remapped_msg->angle_min > remapped_msg->angle_max) {
      std::swap(remapped_msg->angle_min, remapped_msg->angle_max);
      if (!needs_reversal) {
        // If we didn't reverse before, reverse now
        std::reverse(remapped_msg->ranges.begin(), remapped_msg->ranges.end());
        if (!remapped_msg->intensities.empty()) {
          std::reverse(remapped_msg->intensities.begin(), remapped_msg->intensities.end());
        }
      }
    }
    
    // Ensure increment is always positive
    remapped_msg->angle_increment = std::abs(remapped_msg->angle_increment);
    
    // Validate the scan before publishing
    size_t expected_count = static_cast<size_t>(
      std::round((remapped_msg->angle_max - remapped_msg->angle_min) / remapped_msg->angle_increment) + 1
    );
    
    // Log first scan for debugging
    static bool first_scan = true;
    if (first_scan) {
      RCLCPP_INFO(this->get_logger(), 
                  "First remapped scan: angle_min=%.3f, angle_max=%.3f, increment=%.4f, "
                  "ranges=%zu, expected=%zu",
                  remapped_msg->angle_min, remapped_msg->angle_max, 
                  remapped_msg->angle_increment, remapped_msg->ranges.size(), expected_count);
      first_scan = false;
    }
    
    // Publish the remapped message
    scan_pub_->publish(*remapped_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  std::string remapped_frame_id_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ScanFrameRemapper>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

