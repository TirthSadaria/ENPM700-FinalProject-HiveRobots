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
 * @brief Implementation of HiveState and concrete state classes
 *
 * Implements the State design pattern for robot behavior management.
 * States include: IdleState, SearchState, CoordinationState, and ConvergenceState.
 * Each state implements autonomous exploration, obstacle avoidance, and coordination logic.
 *
 * @author Shreya Kalyanaraman, Tirth Sadaria
 */
#include "hive_control/hive_state.hpp"
#include "hive_control/hive_controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>
#include <random>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace hive_control
{

// ================= IdleState =================
void IdleState::handle(
  HiveController * /* context */,
  const sensor_msgs::msg::LaserScan::SharedPtr & /* scan */)
{
  // No-op for now
}
geometry_msgs::msg::Twist IdleState::getVelocityCommand(HiveController * /* context */)
{
  return geometry_msgs::msg::Twist();
}

// ================= SearchState =================
/**
 * @brief Random number generators for turn direction randomization
 *
 * Prevents the "lemming effect" where all robots turn in the same direction,
 * which causes robots to bunch together. Randomization ensures diverse
 * exploration patterns across the robot swarm.
 */
static std::random_device rd;
static std::mt19937 gen(rd());
static std::uniform_int_distribution<> turn_direction_dist(0, 1);  // 0 = left, 1 = right
static std::uniform_int_distribution<> persistence_dist(10, 30);  // Persist for 10-30 cycles
static std::uniform_int_distribution<> turn_duration_dist(10, 20);  // 1.0-2.0 seconds

SearchState::SearchState() 
: turn_direction_persistence_(0), current_turn_direction_(0), turn_remaining_cycles_(0), is_turning_(false),
  stuck_counter_(0), last_min_distance_(0.0)
{
  // Initialize with random turn direction
  current_turn_direction_ = turn_direction_dist(gen);
  turn_direction_persistence_ = persistence_dist(gen);
}

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
  
  // Periodic coordination helps avoid robot conflicts
  // Triggers occasionally (after 50 iterations) to prevent robots from
  // interfering with each other during exploration
  if (needsCoordination() && search_duration_counter_ > 50) {
    context->setState(std::make_shared<CoordinationState>(context->isClockwise()));
    return;
  }

  // Remain in SEARCH state for continuous exploration
  // Robots use collision avoidance and frontier detection while exploring
}

geometry_msgs::msg::Twist SearchState::getVelocityCommand(HiveController * context)
{
  geometry_msgs::msg::Twist cmd;
  
  // Get the latest scan from context
  if (!context) {
    return cmd;
  }
  
  auto scan = context->getCurrentScan();
  
  // Handle case where no scan data is received yet
  if (!scan || scan->ranges.empty()) {
    cmd.linear.x = 0.1;  // Slow forward motion
    return cmd;
  }
  
  // Use map data for frontier-based exploration
  // Find unmapped areas (frontiers) in the map and steer toward them
  auto map = context->getCurrentMap();
  double frontier_angle = 0.0;
  bool has_frontier = false;
  
  if (map && !map->data.empty()) {
    // Analyze map to find frontiers (boundaries between known and unknown)
    // This helps robots explore unmapped areas instead of getting stuck
    has_frontier = findFrontierDirection(map, scan, frontier_angle);
    
    // Frontier detection is performed via findFrontierDirection()
    // This helps robots explore unmapped areas instead of revisiting known regions
  }
  
  // Find the best direction to move (frontier-based exploration)
  // Look for the direction with the longest clear path
  double angle_increment = scan->angle_increment;
  double angle_min = scan->angle_min;
  
  // Check all directions and find the one with maximum clear distance
  double max_clear_distance = 0.0;
  size_t best_direction_idx = scan->ranges.size() / 2;  // Default: forward
  
  // Validate scan data
  if (scan->ranges.empty() || scan->angle_increment <= 0.0) {
    cmd.linear.x = 0.1;  // Slow forward if scan invalid
    return cmd;
  }
  
  // Count valid ranges to determine scan quality
  size_t valid_ranges = 0;
  double min_valid_range = scan->range_max;
  double max_valid_range = 0.0;
  
  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    float range = scan->ranges[i];
    // Check if range is valid (finite, within sensor limits, not NaN/Inf)
    if (std::isfinite(range) && range > scan->range_min && range < scan->range_max) {
      valid_ranges++;
      if (range < min_valid_range) {
        min_valid_range = range;
      }
      if (range > max_valid_range) {
        max_valid_range = range;
      }
      // Prefer directions with longer clear paths (frontier exploration)
      if (range > max_clear_distance) {
        max_clear_distance = range;
        best_direction_idx = i;
      }
    }
  }
  
  // If no valid ranges, drive forward slowly
  if (valid_ranges == 0) {
    cmd.linear.x = 0.1;
    return cmd;
  }
  
  // Use min_valid_range for collision detection
  double min_distance = min_valid_range;
  
  // Collision avoidance thresholds (in meters)
  const double too_close_threshold = 0.5;  // 50cm - too close, back up immediately
  const double obstacle_threshold = 0.8;   // 80cm - obstacle ahead, turn before hitting
  const double safe_distance = 1.2;        // 1.2m - preferred distance from walls

  // Stuck detection uses multiple checks for reliability:
  // 1. Position-based: Check if robot has actually moved (using odometry)
  // 2. Obstacle pattern: Detect if robot sees same distances repeatedly
  // 3. Proximity check: Detect if robot is very close to wall
  
  bool position_stuck = false;
  if (context) {
    auto odom = context->getCurrentOdometry();
    if (odom) {
      double current_x = odom->pose.pose.position.x;
      double current_y = odom->pose.pose.position.y;
      
      if (has_last_position_) {
        // Calculate distance moved since last check
        double dx = current_x - last_position_x_;
        double dy = current_y - last_position_y_;
        double distance_moved = std::sqrt(dx * dx + dy * dy);
        
        // If robot hasn't moved enough, it's stuck
        if (distance_moved < MIN_MOVEMENT_DISTANCE) {
          position_stuck = true;
        }
      }
      
      // Update last position
      last_position_x_ = current_x;
      last_position_y_ = current_y;
      has_last_position_ = true;
    }
  }
  
  // Obstacle-based checks (fallback if odometry not available)
  const double stuck_distance_threshold = 0.15;  // 15cm threshold (increased for better detection)
  bool same_obstacle_pattern = (std::abs(min_distance - last_min_distance_) < stuck_distance_threshold);
  bool very_close_to_wall = (min_distance < too_close_threshold);
  
  // Combine checks: stuck if position-based OR (obstacle pattern AND very close to wall)
  // Position-based detection is most reliable, but we keep obstacle checks as fallback
  if (position_stuck || (same_obstacle_pattern && very_close_to_wall)) {
    stuck_counter_++;
  } else {
    stuck_counter_ = 0;  // Reset if we're making progress
  }
  last_min_distance_ = min_distance;
  
  // If stuck for too long, force aggressive recovery (reduced threshold for faster response)
  if (stuck_counter_ > (STUCK_THRESHOLD / 2)) {  // Faster stuck detection (2.5s instead of 5s)
    stuck_counter_ = 0;  // Reset counter
    stuck_recovery_count_++;  // Track how many times we've tried to recover
    
    // If we've tried multiple recoveries without success, enter SUPER STUCK mode
    if (stuck_recovery_count_ >= SUPER_STUCK_THRESHOLD) {
      in_super_stuck_recovery_ = true;
      super_stuck_phase_ = 0;  // Start with spin
      super_stuck_cycles_ = 60;  // ~6 seconds per phase
      stuck_recovery_count_ = 0;  // Reset for next time
      RCLCPP_WARN(rclcpp::get_logger("SearchState"), 
                  "SUPER STUCK! Starting aggressive exploration recovery.");
    }
    
    // Force a long turn maneuver to escape
    is_turning_ = true;
    turn_remaining_cycles_ = 30;  // 3 seconds of turning
    current_turn_direction_ = turn_direction_dist(gen);  // Random direction
    cmd.linear.x = -0.08;  // Back up while turning (reduced to avoid maxVelocity)
    cmd.angular.z = (current_turn_direction_ == 0) ? 0.35 : -0.35;  // Turn (reduced for safe wheel vel)
    return cmd;
  }
  
  // SUPER STUCK recovery - aggressive exploration pattern
  if (in_super_stuck_recovery_) {
    super_stuck_cycles_--;
    
    if (super_stuck_cycles_ <= 0) {
      // Move to next phase
      super_stuck_phase_++;
      super_stuck_cycles_ = 60;  // Reset cycles for next phase
      
      if (super_stuck_phase_ > 2) {
        // Finished all phases, exit super stuck mode
        in_super_stuck_recovery_ = false;
        super_stuck_phase_ = 0;
        RCLCPP_INFO(rclcpp::get_logger("SearchState"), 
                    "Super stuck recovery complete - resuming normal exploration.");
      }
    }
    
    // Execute current phase
    switch (super_stuck_phase_) {
      case 0:  // Phase 0: Spin 360° to scan surroundings
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.35;  // Full spin
        break;
      case 1:  // Phase 1: Move forward in new direction
        cmd.linear.x = 0.15;  // Forward
        cmd.angular.z = 0.0;
        break;
      case 2:  // Phase 2: Turn to random direction and move
        cmd.linear.x = 0.1;
        cmd.angular.z = (current_turn_direction_ == 0) ? 0.2 : -0.2;
        break;
      default:
        in_super_stuck_recovery_ = false;
        break;
    }
    return cmd;
  }
  
  // If too close to wall, back up and turn toward clear direction
  // Uses randomized turn direction to prevent multiple robots from bunching together
  if (min_distance < too_close_threshold) {
    cmd.linear.x = -0.1;  // Back up (reduced to prevent maxVelocity overflow)
    // Find direction with most clearance
    double best_clearance = 0.0;
    size_t best_escape_idx = scan->ranges.size() / 2;
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
      float range = scan->ranges[i];
      if (std::isfinite(range) && range > scan->range_min && range < scan->range_max) {
        if (range > best_clearance) {
          best_clearance = range;
          best_escape_idx = i;
        }
      }
    }
    // Turn toward the direction with most clearance
    // Randomization prevents all robots from turning the same direction simultaneously
    double escape_angle = angle_min + best_escape_idx * angle_increment;
    
    // If escape angle is ambiguous (near 0), use random direction
    if (std::abs(escape_angle) < 0.2) {
      if (turn_direction_persistence_ <= 0) {
        current_turn_direction_ = turn_direction_dist(gen);
        turn_direction_persistence_ = persistence_dist(gen);
      } else {
        turn_direction_persistence_--;
      }
      escape_angle = (current_turn_direction_ == 0) ? 0.8 : -0.8;
    }
    
    // Proportional turn based on angle
    cmd.angular.z = std::clamp(escape_angle * 0.8, -0.35, 0.35);  // Smooth turn (clamped for safe wheel vel)
    return cmd;
  }
  
  // Check center 90 degrees for immediate obstacles (wider detection)
  size_t center_idx = scan->ranges.size() / 2;
  size_t check_range = static_cast<size_t>(45.0 * M_PI / 180.0 / angle_increment);  // ±45 degrees
  size_t start_check = (center_idx > check_range) ? center_idx - check_range : 0;
  size_t end_check = std::min(center_idx + check_range, scan->ranges.size() - 1);
  
  bool obstacle_ahead = false;
  double min_ahead_distance = scan->range_max;
  size_t valid_ahead_ranges = 0;
  
  for (size_t i = start_check; i <= end_check; ++i) {
    float range = scan->ranges[i];
    if (std::isfinite(range) && range > scan->range_min && range < scan->range_max) {
      valid_ahead_ranges++;
      if (range < min_ahead_distance) {
        min_ahead_distance = range;
      }
      if (range < obstacle_threshold) {
        obstacle_ahead = true;
      }
    }
  }
  
  // If no valid ranges ahead, assume clear path (sensor might be blocked)
  if (valid_ahead_ranges == 0) {
    obstacle_ahead = false;
    min_ahead_distance = safe_distance;  // Assume safe distance
  }
  
  // Check if we're in the middle of a turn maneuver (aggressive scattering)
  if (is_turning_ && turn_remaining_cycles_ > 0) {
    // Continue turning for the remaining duration
    turn_remaining_cycles_--;
    cmd.linear.x = 0.05;  // Very slow forward while turning
    // Use the persistent random direction
    cmd.angular.z = (current_turn_direction_ == 0) ? 0.3 : -0.3;  // Turn (safe wheel vel)
    return cmd;
  } else if (is_turning_ && turn_remaining_cycles_ <= 0) {
    // Turn complete, resume normal exploration
    is_turning_ = false;
  }
  
  if (obstacle_ahead) {
    // Obstacle detected ahead - turn toward best direction smoothly
    // Prefer frontier direction if available, otherwise use best lidar direction
    double best_angle = angle_min + best_direction_idx * angle_increment;
    
    if (has_frontier) {
      // Use frontier direction to escape from obstacles
      cmd.angular.z = std::clamp(frontier_angle * 0.6, -0.3, 0.3);  // Turn toward frontier
      cmd.linear.x = 0.08;  // Slow forward while turning toward frontier
    } else {
      // Fall back to lidar-based best direction
      double turn_strength = std::clamp((obstacle_threshold - min_ahead_distance) / obstacle_threshold, 0.3, 1.0);
      
      if (std::abs(best_angle) > 0.1) {
        cmd.angular.z = (best_angle > 0) ? (0.3 * turn_strength) : (-0.3 * turn_strength);
        cmd.linear.x = 0.05;  // Slow forward while turning
      } else {
        // Best direction is forward but obstacle there - turn away
        // Start aggressive turn maneuver (1.0-2.0 seconds) for better scattering
        if (turn_direction_persistence_ <= 0) {
          // Time to pick a new random direction
          current_turn_direction_ = turn_direction_dist(gen);
          turn_direction_persistence_ = persistence_dist(gen);
        } else {
          turn_direction_persistence_--;  // Decrement persistence counter
        }
        
        // Start a turn maneuver that lasts 1.0-2.0 seconds (10-20 cycles at ~10Hz)
        is_turning_ = true;
        turn_remaining_cycles_ = turn_duration_dist(gen);
        
        // Use the persistent random direction (0 = left/positive, 1 = right/negative)
        cmd.angular.z = (current_turn_direction_ == 0) ? 0.3 : -0.3;  // Safe wheel vel
        cmd.linear.x = 0.05;  // Slow forward while turning
      }
    }
  } else {
    // Path is clear - drive forward smoothly (DEFAULT: Always drive forward if clear)
    // Adjust speed based on distance to nearest obstacle (smooth deceleration)
    double speed_factor = std::min(min_distance / safe_distance, 1.0);
    cmd.linear.x = 0.15 * speed_factor;  // Max 0.15 m/s (reduced for safe wheel vel)

    // Prefer frontier direction if available, otherwise use best lidar direction
    if (has_frontier) {
      // Use map-based frontier direction (stronger bias toward unmapped areas)
      cmd.angular.z = std::clamp(frontier_angle * 0.6, -0.3, 0.3);  // Turn toward frontier
    } else {
      // Fall back to lidar-based best direction
      double best_angle = angle_min + best_direction_idx * angle_increment;
      if (std::abs(best_angle) > 0.15) {  // If best direction is off-center
        cmd.angular.z = std::clamp(best_angle * 0.4, -0.25, 0.25);  // Smooth, gentle turn
      } else {
        // Path is clear and straight ahead - drive forward (default behavior)
        cmd.angular.z = 0.0;
        cmd.linear.x = 0.15;  // Reduced max forward speed
      }
    }
  }
  
  return cmd;
}

/**
 * @brief Find frontier direction from map data
 * @param map Occupancy grid map
 * @param scan Laser scan (for coordinate transformation)
 * @param frontier_angle Output: angle to turn toward frontier (in radians)
 * @return true if frontier found, false otherwise
 */
bool SearchState::findFrontierDirection(
  const nav_msgs::msg::OccupancyGrid::SharedPtr & map,
  const sensor_msgs::msg::LaserScan::SharedPtr & scan,
  double & frontier_angle)
{
  if (!map || !scan || map->data.empty()) {
    return false;
  }
  
  // Frontier detection: Find unknown cells and determine best direction
  // This helps robots explore unmapped areas instead of revisiting known regions
  const int width = map->info.width;
  const int height = map->info.height;
  const double resolution = map->info.resolution;

  if (width <= 0 || height <= 0 || resolution <= 0.0) {
    return false;
  }

  // Frontier detection algorithm:
  // 1. Find all frontiers (unknown cells adjacent to known free cells) in the map
  // 2. Use lidar to determine which direction has clear path + unknown areas
  // 3. Return the angle toward the best frontier direction
  // 3. This works even when robot has moved from origin
  
  // Step 1: Find all frontier cells (unknown cells adjacent to known free cells)
  std::vector<std::pair<int, int>> frontier_cells;  // (x, y) in map coordinates
  
  for (int y = 1; y < height - 1; ++y) {
    for (int x = 1; x < width - 1; ++x) {
      int idx = y * width + x;
      if (idx < 0 || idx >= static_cast<int>(map->data.size())) {
        continue;
      }
      
      // Check if this is an unknown cell
      if (map->data[idx] == -1) {
        // Check if it's adjacent to a known free cell (0) - this is a frontier
        bool is_frontier = false;
        for (int dy = -1; dy <= 1 && !is_frontier; ++dy) {
          for (int dx = -1; dx <= 1 && !is_frontier; ++dx) {
            if (dx == 0 && dy == 0) continue;
            
            int nx = x + dx;
            int ny = y + dy;
            if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
              int nidx = ny * width + nx;
              if (nidx >= 0 && nidx < static_cast<int>(map->data.size())) {
                if (map->data[nidx] == 0) {  // Adjacent to free space
                  is_frontier = true;
                }
              }
            }
          }
        }
        
        if (is_frontier) {
          frontier_cells.push_back({x, y});
        }
      }
    }
  }
  
  if (frontier_cells.empty()) {
    return false;  // No frontiers found
  }
  
  // Step 2: Use lidar to find which direction has clear path
  // Find the lidar direction with longest clear distance
  double max_lidar_range = 0.0;
  size_t best_lidar_idx = scan->ranges.size() / 2;  // Default: forward
  
  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    float range = scan->ranges[i];
    if (std::isfinite(range) && range > scan->range_min && range < scan->range_max) {
      if (range > max_lidar_range) {
        max_lidar_range = range;
        best_lidar_idx = i;
      }
    }
  }
  
  // Step 3: Find frontier closest to the direction with clear path
  // Convert lidar index to angle
  double lidar_angle = scan->angle_min + best_lidar_idx * scan->angle_increment;
  
  // Find frontier in the general direction of clear path
  // We'll use a simple heuristic: prefer frontiers in forward direction
  // For simplicity, we'll just pick a frontier and use lidar to guide us
  
  // Use the lidar direction as the target (simpler and more effective)
  // If we have clear path in a direction, explore that direction
  frontier_angle = lidar_angle;
  
  // If we found frontiers, return true (we'll explore in the clear direction)
  return !frontier_cells.empty();
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
  
  // Check if stuck (very close to wall for too long)
  double min_distance = scan->range_max;
  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    float range = scan->ranges[i];
    if (std::isfinite(range) && range > scan->range_min && range < scan->range_max) {
      if (range < min_distance) {
        min_distance = range;
      }
    }
  }
  
  // If stuck at wall, try to escape immediately
  if (min_distance < 0.35 && coordination_timer_ > 10) {
    // Back up more aggressively and return to search
    context->setState(std::make_shared<SearchState>());
    return;
  }
  
  // Return to search if path is clear and coordination time elapsed (reduced time)
  if (isPathClear(scan) && coordination_timer_ > 15) {  // Reduced from COORDINATION_TIME
    context->setState(std::make_shared<SearchState>());
    context->toggleRotationDirection(); // Vary behavior for swarm diversity
  }
}

geometry_msgs::msg::Twist CoordinationState::getVelocityCommand(HiveController * context)
{
  geometry_msgs::msg::Twist cmd;
  
  // If context available, check if we're stuck
  if (context) {
    auto scan = context->getCurrentScan();
    if (scan && !scan->ranges.empty()) {
      double min_distance = scan->range_max;
      for (size_t i = 0; i < scan->ranges.size(); ++i) {
        float range = scan->ranges[i];
        if (std::isfinite(range) && range > scan->range_min && range < scan->range_max) {
          if (range < min_distance) {
            min_distance = range;
          }
        }
      }
      
      // If very close to wall, back up while turning more aggressively
      if (min_distance < 0.5) {
        cmd.linear.x = -0.08;  // Back up (safe for wheel vel)
        // Find direction with most clearance
        double best_clearance = 0.0;
        size_t best_escape_idx = scan->ranges.size() / 2;
        for (size_t i = 0; i < scan->ranges.size(); ++i) {
          float range = scan->ranges[i];
          if (std::isfinite(range) && range > scan->range_min && range < scan->range_max) {
            if (range > best_clearance) {
              best_clearance = range;
              best_escape_idx = i;
            }
          }
        }
        // Turn toward direction with most clearance
        double escape_angle = scan->angle_min + best_escape_idx * scan->angle_increment;
        cmd.angular.z = std::clamp(escape_angle * 0.8, -0.3, 0.3);  // Safe turn
        return cmd;
      }
    }
  }
  
  // Normal coordination: turn in place
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
  HiveController * /* context */,
  const sensor_msgs::msg::LaserScan::SharedPtr & /* scan */)
{
  // Simple convergence behavior: move toward center, then rotate
  if (isAtRendezvous()) {
    rotating_to_center_ = true;
    // In a real system, this would trigger map merging
  }
}

geometry_msgs::msg::Twist ConvergenceState::getVelocityCommand(HiveController * /* context */)
{
  geometry_msgs::msg::Twist cmd;
  
  // Move slowly toward rendezvous point
  if (rotating_to_center_) {
    cmd.angular.z = angular_velocity_;
  } else {
    cmd.linear.x = linear_velocity_;  // 0.1 m/s - slow forward
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
