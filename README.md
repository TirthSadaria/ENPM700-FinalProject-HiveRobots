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
- **UML Class Diagram:** [View Diagram](UML/class_diagram.jpeg)
- **Activity Diagram:** [View Diagram](UML/activity_diagram.jpeg)
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

## Quick Start

### 1. Clone Repository

```bash
cd ~/your_workspace
git clone https://github.com/TirthSadaria/ENPM700-FinalProject-HiveRobots.git
cd ENPM700-FinalProject-HiveRobots
```

### 2. Build Package

```bash
# Source ROS2 Humble
source /opt/ros/humble/setup.bash

# Build the workspace
colcon build

# Source the workspace
source install/setup.bash
```

### 3. Launch Multi-Robot SLAM

**Basic launch (2 robots, no RViz):**
```bash
ros2 launch hive_control hive_slam_final.launch.py
```

**With RViz visualization:**
```bash
ros2 launch hive_control hive_slam_final.launch.py num_robots:=2 enable_rviz:=true
```

**Custom number of robots:**
```bash
ros2 launch hive_control hive_slam_final.launch.py num_robots:=3 enable_rviz:=true
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

## System Workflow

When you launch the system, the following sequence occurs:

1. **Webots Simulation** (0-5s): Webots launches with N TurtleBot3 robots in a warehouse environment
2. **Controller Initialization** (5-10s): Robot controllers spawn and begin publishing odometry
3. **SLAM Start** (25s): Each robot starts independent SLAM (slam_toolbox) and begins mapping
4. **Map Merging** (35s): Map merger node starts combining individual maps into `/map_merged`
5. **Autonomous Exploration**: Robots explore using frontier-based exploration with collision avoidance
6. **Real-time Visualization**: RViz displays the merged map and robot positions (if enabled)

**Note**: The system uses delayed launches to ensure proper TF tree initialization before SLAM starts.

---

## Project Structure

```
ENPM700-FinalProject-HiveRobots/
├── src/                          # C++ source files
│   ├── hive_controller_node.cpp  # ROS2 node interface (subscribers/publishers)
│   ├── hive_controller.cpp       # State machine controller implementation
│   ├── hive_state.cpp            # Robot behavior states (SEARCH, COORDINATION, etc.)
│   └── scan_frame_remapper.cpp   # Lidar scan frame remapping and angle normalization
├── include/hive_control/         # Header files
│   ├── hive_controller.hpp       # Controller class definition
│   ├── hive_state.hpp            # State classes definition
│   └── hive_controller_node.hpp  # ROS2 node class definition
├── test/                         # Unit tests
│   ├── test_hive_control.cpp     # Comprehensive test suite (40 tests)
│   └── test_hive_controller.cpp  # Controller-specific tests (26 tests)
├── launch/
│   └── hive_slam_final.launch.py # Main launch file for multi-robot SLAM
├── config/
│   ├── slam_params.yaml          # SLAM configuration (slam_toolbox)
│   ├── map_merge_params.yaml     # Map merge configuration
│   ├── hive_debug.rviz           # RViz visualization configuration
│   └── turtlebot_webots_no_imu.urdf  # Robot URDF description
├── scripts/
│   ├── generate_world.py         # Dynamic Webots world generation
│   ├── generate_rviz_config.py  # Dynamic RViz config generation
│   └── world_frame_publisher.py  # World frame TF publisher
├── worlds/
│   └── warehouse.wbt             # Webots world file template
└── README.md
```

## Architecture

The system uses a **State Design Pattern** for robot behavior management:

- **HiveController**: Context class that maintains current state and manages state transitions
- **State Classes**: 
  - `SearchState`: Autonomous exploration with frontier detection and collision avoidance
  - `CoordinationState`: Robot coordination to avoid conflicts
  - `ConvergenceState`: Rendezvous behavior for map merging
  - `IdleState`: Waiting/holding state

**Multi-Robot SLAM Architecture:**
- Each robot runs independent SLAM (decentralized approach)
- Static transforms anchor each robot's map frame to a shared world frame
- Map merger combines individual maps using feature-based estimation
- World frame publisher ensures proper TF tree connectivity

---

## Key Features

### Robot Behavior
- **Frontier-based exploration**: Robots move toward unmapped areas using map data
- **Collision avoidance**: Lidar-based obstacle detection with multiple safety thresholds
- **Stuck detection**: Position-based and pattern-based detection with automatic recovery
- **Randomized turns**: Prevents "lemming effect" where all robots turn the same direction
- **Super stuck recovery**: Multi-phase recovery mechanism for persistent stuck situations

### SLAM System
- **Independent SLAM per robot**: Each robot runs its own slam_toolbox instance
- **Real-time map merging**: Maps combined using feature-based estimation
- **Coordinate alignment**: Static transforms anchor each robot's map frame to world frame
- **World frame establishment**: Dedicated node ensures proper TF tree connectivity

### Configuration
- **Map resolution**: 0.05m (5cm per pixel)
- **Map update interval**: 1.0s (configurable in slam_params.yaml)
- **Scan processing**: Automatic angle normalization for Webots lidar compatibility
- **Lidar rate**: 3 Hz (reduced from 5 Hz to prevent queue overflow)
- **Stuck detection**: 5cm minimum movement threshold, 5-second timeout

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

The project includes comprehensive **unit tests** using Google Test (gtest) framework to ensure code quality and reliability. The test suite provides extensive coverage of:

- **Controller Functionality**: State management, battery tracking, rotation direction
- **State Machine**: All state transitions (IDLE, SEARCH, COORDINATION, CONVERGENCE)
- **Obstacle Avoidance**: Collision detection, stuck recovery, edge cases
- **Sensor Processing**: Laser scan handling, odometry integration, map management
- **Frontier Exploration**: Map-based exploration, frontier detection
- **Edge Cases**: Invalid data, empty scans, boundary conditions

**Test Statistics:**
- 40 tests in `test_hive_control.cpp`
- 26 tests in `test_hive_controller.cpp`
- All tests passing with comprehensive coverage

```bash
# Build and run tests
colcon build
source install/setup.bash

# Run unit tests directly
./build/hive_control/test_hive_control
./build/hive_control/test_hive_controller

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
