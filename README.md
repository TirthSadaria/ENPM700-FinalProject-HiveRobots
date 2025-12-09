# ENPM700 Final Project: Multi-Robot SLAM System

[![Build Status](https://github.com/TirthSadaria/ENPM700-FinalProject-HiveRobots/actions/workflows/run-unit-test-and-upload-codecov.yml/badge.svg?branch=phase-2)](https://github.com/TirthSadaria/ENPM700-FinalProject-HiveRobots/actions/workflows/run-unit-test-and-upload-codecov.yml)

[![codecov](https://codecov.io/gh/TirthSadaria/ENPM700-FinalProject-HiveRobots/branch/phase-2/graph/badge.svg)](https://codecov.io/gh/TirthSadaria/ENPM700-FinalProject-HiveRobots)

## Overview

A **multi-robot SLAM (Simultaneous Localization and Mapping) system** using ROS 2 Humble, Webots R2025a, and slam_toolbox. Multiple TurtleBot3 robots collaboratively explore and map an environment, with their individual maps merged in real-time.

## Team

- **Shreya Kalyanaraman** (Driver)
- **Tirth Sadaria** (Navigator)

## Project Documentation

### Sprint Planning & Backlog
- **Sprint Plan:** [Google Doc](https://docs.google.com/document/d/1GciBjBSQgnnl1llGWNddDnd60kpwyzJ1LujE0GWnF4o/edit?usp=sharing)
- **AIP Backlog:** [Google Sheet](https://docs.google.com/spreadsheets/d/18LbFuGbSA6lyf5C_sq3-1zMn88slf8hpBHtjBDwPiCw/edit?usp=sharing)

### UML & Design Diagrams
- **UML Class Diagram:** [Link TBD]
- **Activity Diagram:** [Link TBD]
- **Sequence Diagram:** [Link TBD]

---

## Prerequisites

### System Requirements
- **OS**: Ubuntu 22.04
- **ROS2**: Humble Hawksbill
- **Webots**: R2025a

### Install Dependencies

```bash
sudo apt update
sudo apt install -y \
    ros-humble-webots-ros2-driver \
    ros-humble-webots-ros2-turtlebot \
    ros-humble-slam-toolbox \
    ros-humble-multirobot-map-merge \
    ros-humble-nav2-map-server \
    ros-humble-tf2-tools \
    ros-humble-tf2-ros
```

---

## Build & Run

### 1. Clone and Build

```bash
cd ~/your_workspace
git clone https://github.com/TirthSadaria/ENPM700-FinalProject-HiveRobots.git
cd ENPM700-FinalProject-HiveRobots
colcon build
source install/setup.bash
```

### 2. Launch Multi-Robot SLAM

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch hive_control hive_slam_final.launch.py num_robots:=2 enable_rviz:=true
```

### Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `num_robots` | 2 | Number of robots (1-10) |
| `enable_rviz` | false | Launch RViz with visualization |
| `enable_slam` | true | Enable SLAM for all robots |
| `enable_map_merge` | true | Enable real-time map merging |
| `use_sim_time` | true | Use simulation time |

---

## What Happens

1. **Webots** launches with N TurtleBot3 robots in a warehouse environment
2. Each robot runs **independent SLAM** (slam_toolbox)
3. Robots **explore autonomously** using frontier-based exploration
4. Individual maps are **merged in real-time** into `/map_merged`
5. **RViz** displays the combined map (if enabled)

---

## Project Structure

```
ENPM700-FinalProject-HiveRobots/
├── src/                          # C++ source files
│   ├── hive_controller_node.cpp  # ROS2 node interface
│   ├── hive_controller.cpp       # State machine controller
│   ├── hive_state.cpp            # Robot behavior states
│   └── scan_frame_remapper.cpp   # Lidar scan processing
├── include/hive_control/         # Header files
│   ├── hive_controller.hpp
│   └── hive_state.hpp
├── test/
│   └── test_hive_control.cpp     # Unit tests (gtest)
├── launch/
│   └── hive_slam_final.launch.py # Main launch file
├── config/
│   ├── slam_params.yaml          # SLAM configuration
│   ├── map_merge_params.yaml     # Map merge configuration
│   ├── hive_debug.rviz           # RViz configuration
│   └── turtlebot_webots_no_imu.urdf
├── scripts/
│   └── generate_world.py         # Dynamic world generation
├── worlds/
│   └── warehouse.wbt             # Webots world file
└── README.md
```

---

## Key Features

### Robot Behavior
- **Frontier-based exploration**: Robots move toward unmapped areas
- **Collision avoidance**: Lidar-based obstacle detection
- **Stuck detection**: Automatic recovery when robots get stuck
- **Randomized turns**: Prevents robots from bunching together

### SLAM System
- **Independent SLAM per robot**: Each robot builds its own map
- **Real-time map merging**: Maps combined using feature matching
- **Coordinate alignment**: Static transforms anchor maps to world frame

### Configuration
- **Map resolution**: 0.05m (5cm)
- **Map update interval**: 2.0s
- **Scan processing**: Angle normalization for Webots lidar compatibility

---

## Verification

### Check Topics

```bash
# List map topics
ros2 topic list | grep map

# Expected output:
# /map_merged
# /tb1/map
# /tb2/map
```

### Check Transforms

```bash
# View TF tree
ros2 run tf2_tools view_frames
# Creates frames.pdf

# Check specific transform
ros2 run tf2_ros tf2_echo world tb1/map
```

### Check Map Data

```bash
# Individual robot map
ros2 topic echo /tb1/map --once | grep -E "width|height|resolution"

# Merged map
ros2 topic echo /map_merged --once | grep -E "width|height|resolution"
```

---

## Testing

The project includes **Level 1 and Level 2 unit tests** using Google Test (gtest) framework to ensure code quality and reliability. The test suite covers core functionality including controller instantiation, battery management, rotation direction toggling, lidar scan angle normalization, and obstacle detection threshold validation.

```bash
# Build and run tests
colcon build
source install/setup.bash

# Run unit tests directly
./build/hive_control/test_hive_control

# Or run all tests with colcon
colcon test --packages-select hive_control
colcon test-result --verbose
```

---

## Dependencies

| Package | Purpose |
|---------|---------|
| webots_ros2_driver | Webots-ROS2 integration |
| webots_ros2_turtlebot | TurtleBot3 support |
| slam_toolbox | SLAM implementation |
| multirobot_map_merge | Map fusion |
| nav2_map_server | Map utilities |
| tf2_ros | Transform system |

---

## License

Apache-2.0 License - see [LICENSE](LICENSE) file.

---

## Acknowledgments

- ROS2 Humble community
- Webots development team
- slam_toolbox and multirobot_map_merge maintainers
