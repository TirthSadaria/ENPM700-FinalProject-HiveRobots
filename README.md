# ENPM700 Final Project: Multi-Robot SLAM System

[![Build Status](https://github.com/TirthSadaria/ENPM700-FinalProject-HiveRobots/actions/workflows/ci.yml/badge.svg)](https://github.com/TirthSadaria/ENPM700-FinalProject-HiveRobots/actions/workflows/ci.yml)
[![codecov](https://codecov.io/gh/TirthSadaria/ENPM700-FinalProject-HiveRobots/graph/badge.svg?token=PLACEHOLDER_TOKEN)](https://codecov.io/gh/TirthSadaria/ENPM700-FinalProject-HiveRobots)

---

## Overview

This project implements a **fully functional multi-robot SLAM (Simultaneous Localization and Mapping) system** using ROS 2 Humble, Webots R2025a, and slam_toolbox. The system supports 1-10 robots with dynamic spawning, collaborative mapping, automatic map merging, and frontier-based exploration.

## Key Features

- ✅ **Multi-Robot SLAM**: Each robot runs independent SLAM with automatic map merging
- ✅ **Dynamic Robot Spawning**: Supports 1-10 robots with predefined spawn positions
- ✅ **High-Quality Maps**: 3cm resolution maps with fast 0.5s update intervals
- ✅ **Frontier-Based Exploration**: Intelligent exploration using frontier detection
- ✅ **Collision Avoidance**: Advanced obstacle avoidance with randomized turns
- ✅ **Stuck Detection**: Automatic recovery when robots get stuck
- ✅ **RViz Visualization**: Pre-configured visualization with merged map display
- ✅ **Automatic Map Saving**: Maps saved to `results/` directory on completion

## Team

- **Shreya Kalyanaraman** (Driver)
- **Tirth Sadaria** (Navigator)

## Sprint Planning & Backlog

- **Sprint Plan (Google Doc):** [Sprint Document](https://docs.google.com/document/d/1GciBjBSQgnnl1llGWNddDnd60kpwyzJ1LujE0GWnF4o/edit?usp=sharing)
  - Includes sprint goals, driver/navigator logs, AIP categories, iteration notes, and meeting summaries.

- **AIP Backlog (Google Sheet):** [Backlog & AIP Spreadsheet](https://docs.google.com/spreadsheets/d/18LbFuGbSA6lyf5C_sq3-1zMn88slf8hpBHtjBDwPiCw/edit?usp=sharing)
  - Contains the full product backlog, task durations, priorities, and AIP tags.

---

## Quick Start

### Prerequisites

```bash
# Install ROS2 Humble dependencies
sudo apt install ros-humble-webots-ros2-driver \
                 ros-humble-slam-toolbox \
                 ros-humble-multirobot-map-merge \
                 ros-humble-nav2-map-server \
                 ros-humble-tf2-tools
```

### Build the Workspace

```bash
# Clone the repository
git clone https://github.com/TirthSadaria/ENPM700-FinalProject-HiveRobots.git
cd ENPM700-FinalProject-HiveRobots

# Build the workspace
colcon build
source install/setup.bash
```

### Run Multi-Robot SLAM

**Terminal 1 - Launch SLAM System:**
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch hive_control hive_slam.launch.py num_robots:=2
```

**Terminal 2 - Launch RViz Visualization (Optional):**
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch hive_control visualize.launch.py
```

### Verify System Health

```bash
# Quick system check
./quick_check.sh

# Detailed transform verification
./check_transform_integration.sh

# Map quality verification
./verify_map_quality.sh
```

---

## Launch Files

### Main Launch File: `hive_slam.launch.py`

**Purpose**: Complete multi-robot SLAM system with dynamic robot spawning.

**Usage**:
```bash
ros2 launch hive_control hive_slam.launch.py [num_robots:=N]
```

**Arguments**:
- `num_robots` (default: 3): Number of robots to spawn (1-10)
- `enable_slam` (default: true): Enable SLAM for all robots
- `enable_map_merge` (default: true): Enable map merging
- `use_sim_time` (default: true): Use simulation time

**What it launches**:
- Webots simulation with N robots
- Robot controllers (hive_controller_node) for each robot
- SLAM nodes (slam_toolbox) for each robot
- Map merge node (multirobot_map_merge)
- Static transforms for coordinate system alignment

### Visualization: `visualize.launch.py`

**Purpose**: Launch RViz2 with pre-configured view for multi-robot SLAM.

**Usage**:
```bash
ros2 launch hive_control visualize.launch.py
```

**Configuration**: Uses `config/hive_debug.rviz` with:
- Fixed Frame: `world`
- Merged Map topic: `/map_merged` (TransientLocal QoS)
- Individual robot maps (optional)

### Alternative SLAM: `hive_slam_cartographer.launch.py`

**Purpose**: Alternative implementation using Cartographer instead of slam_toolbox.

**Usage**:
```bash
ros2 launch hive_control hive_slam_cartographer.launch.py num_robots:=2
```

**Note**: Requires Cartographer ROS2 package installation.

---

## Project Structure

```
ENPM700-FinalProject-HiveRobots/
├── src/                          # C++ source files
│   ├── hive_controller_node.cpp  # ROS2 node wrapper
│   ├── hive_controller.cpp       # Main controller (state machine)
│   ├── hive_state.cpp            # State implementations
│   └── scan_frame_remapper.cpp   # Laser scan frame remapper
├── include/hive_control/         # Header files
│   ├── hive_controller.hpp
│   └── hive_state.hpp
├── launch/                       # Launch files
│   ├── hive_slam.launch.py       # Main launch file ⭐
│   ├── visualize.launch.py      # RViz visualization
│   └── hive_slam_cartographer.launch.py  # Alternative SLAM
├── config/                       # Configuration files
│   ├── slam_params.yaml         # SLAM toolbox parameters
│   ├── hive_debug.rviz          # RViz configuration
│   ├── turtlebot_webots_no_imu.urdf  # Robot URDF
│   └── cartographer_2d.lua      # Cartographer config (optional)
├── scripts/                      # Utility scripts
│   ├── generate_world.py        # Generate Webots world file
│   ├── source_all_workspaces.sh
│   └── verify_robot_publishing.sh
├── results/                      # Generated maps (created at runtime)
│   ├── tb1_map_complete.pgm/yaml
│   ├── tb2_map_complete.pgm/yaml
│   └── merged_map_complete.pgm/yaml
├── test_slam.sh                  # Quick SLAM test
├── check_transform_integration.sh  # Transform verification
├── quick_check.sh                # System health check
├── verify_map_quality.sh         # Map quality verification
├── GEMINI_UPDATE_DEC2024.md     # Comprehensive update document
└── README.md                     # This file
```

---

## Key Components

### 1. State Machine Controller

**File**: `src/hive_state.cpp`

**States**:
- **IdleState**: Initial state, transitions to SearchState
- **SearchState**: Main exploration with frontier detection, collision avoidance, and stuck recovery
- **CoordinationState**: (Future: inter-robot coordination)
- **ConvergenceState**: (Future: return to start)

**Features**:
- Randomized turn directions to prevent "lemming effect"
- Stuck detection (50 cycles = ~5 seconds) with aggressive recovery
- Frontier-based exploration using lidar data
- Minimum exploration time: 120 seconds

### 2. SLAM Configuration

**File**: `config/slam_params.yaml`

**Key Parameters**:
- Resolution: **0.03m (3cm)** - High resolution for sharp maps
- Map update interval: **0.5s** - Fast updates for responsive mapping
- Scan buffer size: **30** - More scan history for better matching
- Link match minimum response: **0.4** - Stricter matching for quality

### 3. Map Merging

**Package**: `multirobot_map_merge`

**Configuration** (in `hive_slam.launch.py`):
- World frame: `world` - Common frame for all maps
- Known init poses: `False` - Automatic pose estimation
- Estimation confidence: `0.3` - Threshold for accepting matches
- Merging rate: `1.0` - Faster merging
- Estimation rate: `2.0` - Faster pose estimation

### 4. Coordinate System

**Transform Chain**:
```
world -> tb{i}/map -> tb{i}/odom -> tb{i}/base_link -> tb{i}/LDS-01
```

**Static Transforms**:
- Published from `world` to each robot's `map` frame using spawn positions
- X coordinate inverted to fix RViz display alignment
- Launched at 5 seconds (before SLAM at 15 seconds)

---

## Verification & Testing

### Quick Tests

```bash
# Test SLAM setup (checks odometry, transforms, maps)
./test_slam.sh

# Check transform integration
./check_transform_integration.sh

# Verify map quality parameters
./verify_map_quality.sh

# Quick system health check
./quick_check.sh
```

### Manual Verification

```bash
# Check odometry
ros2 topic echo /tb1/odom --once

# Check transforms
ros2 run tf2_ros tf2_echo world tb1/map

# View complete TF tree
ros2 run tf2_tools view_frames
# Opens frames.pdf

# Check map topics
ros2 topic list | grep map
# Should see: /map_merged, /tb1/map, /tb2/map, etc.

# Check merged map
ros2 topic echo /map_merged --once

# Check SLAM parameters
ros2 param list /tb1/slam_toolbox
ros2 param get /tb1/slam_toolbox resolution
```

---

## Troubleshooting

### Maps Not Appearing in RViz

1. **Check Fixed Frame**: Set to `world` in RViz Global Options
2. **Check Map Topic**: Should be `/map_merged` with TransientLocal QoS
3. **Verify Transforms**: Run `./check_transform_integration.sh`
4. **Check Map Merge**: Verify `/map_merged` topic is publishing

### Robots Getting Stuck

- The system includes automatic stuck detection and recovery
- If robots still get stuck, check:
  - Obstacle detection thresholds in `hive_state.cpp`
  - Turn duration and persistence settings
  - Minimum exploration time (120 seconds)

### Map Merge Not Working

1. **Check Confidence**: Verify `estimation_confidence: 0.3` in launch file
2. **Check Transforms**: Ensure `world -> tb{i}/map` transforms exist
3. **Check Map Topics**: Verify individual maps are publishing
4. **Check Logs**: Look for map_merge confidence values (should be > 0.3)

### Transform Errors

1. **Wait for Transforms**: Transforms launch at 5 seconds, SLAM at 15 seconds
2. **Check World Frame**: First `world -> tb1/map` transform creates `world` frame
3. **Verify Timing**: Use `./check_transform_integration.sh` after system is running

---

## Documentation

- **GEMINI_UPDATE_DEC2024.md**: Comprehensive update document with all features, fixes, and technical details
- **This README**: Quick start and overview
- **Code Comments**: Detailed inline documentation in source files

---

## Dependencies

### ROS2 Packages
- `webots_ros2_driver`: Webots integration
- `webots_ros2_turtlebot`: TurtleBot3 model
- `slam_toolbox`: SLAM implementation
- `multirobot_map_merge`: Map merging
- `nav2_map_server`: Map saving
- `tf2_ros`: Transform library
- `rviz2`: Visualization

### System Requirements
- **ROS2**: Humble Hawksbill
- **Webots**: R2025a (or compatible)
- **OS**: Ubuntu 22.04 (recommended)
- **Python**: 3.10+

---

## Recent Updates (December 2024)

- ✅ Fixed map merge confidence threshold (1.0 → 0.3)
- ✅ Fixed coordinate system alignment (X inversion for RViz)
- ✅ Improved map quality (0.03m resolution, faster updates)
- ✅ Enhanced robot exploration (randomized turns, stuck detection)
- ✅ Reduced exploration time (180s → 120s)
- ✅ Cleaned up repository (removed old launch files, debug images)
- ✅ Created comprehensive documentation

See `GEMINI_UPDATE_DEC2024.md` for detailed changelog.

---

## License

This project is licensed under the Apache-2.0 License - see the [LICENSE](LICENSE) file for details.

---

## Acknowledgments

- ROS2 Humble community
- Webots development team
- slam_toolbox maintainers
- multirobot_map_merge contributors
