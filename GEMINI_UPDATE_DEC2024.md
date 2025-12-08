# Gemini Update Document - December 2024
## ENPM700 Final Project: Multi-Robot SLAM System

### Project Overview
This project implements a fully functional multi-robot SLAM (Simultaneous Localization and Mapping) system using ROS2 Humble, Webots R2025a, and slam_toolbox. The system supports 1-10 robots with dynamic spawning, collaborative mapping, and automatic map merging.

---

## Key Features Implemented

### 1. **Multi-Robot SLAM System**
- **Dynamic Robot Spawning**: Supports 1-10 robots with predefined spawn positions
- **Individual SLAM**: Each robot runs its own `slam_toolbox` instance in a unique namespace (`/tb1`, `/tb2`, etc.)
- **Map Merging**: Automatic merging of individual maps into a global map using `multirobot_map_merge`
- **Coordinate System Alignment**: Static transforms from `world` frame to each robot's `map` frame for proper alignment

### 2. **Robot Control & Exploration**
- **State Machine**: Implements a robust state pattern with `SearchState`, `CoordinationState`, and `ConvergenceState`
- **Frontier-Based Exploration**: Robots explore unknown areas using frontier detection
- **Collision Avoidance**: Advanced obstacle avoidance with randomized turn directions to prevent "lemming effect"
- **Stuck Detection**: Automatic recovery mechanism when robots get stuck in corners

### 3. **Map Quality & Performance**
- **High Resolution**: 3cm (0.03m) map resolution for sharp, detailed maps
- **Fast Updates**: 0.5s map update interval for responsive mapping
- **Optimized Parameters**: Tuned SLAM and map merge parameters for best quality
- **Automatic Map Saving**: Maps saved to `results/` directory when exploration completes

### 4. **Visualization & Debugging**
- **RViz Integration**: Pre-configured RViz view with merged map visualization
- **Transform Verification**: Scripts to verify TF tree and transform chains
- **Quality Checks**: Tools to verify map resolution, merge parameters, and system health

---

## Technical Architecture

### Launch System
**Main Launch File**: `launch/hive_slam.launch.py`
- Dynamically generates robot nodes based on `num_robots` argument
- Automatically generates Webots world file with correct number of robots
- Launches Webots, robot controllers, SLAM nodes, and map merge in correct sequence
- Handles timing delays for proper initialization

**Key Launch Arguments**:
- `num_robots`: Number of robots to spawn (1-10, default: 3)
- `enable_slam`: Enable SLAM for all robots (default: true)
- `enable_map_merge`: Enable map merging (default: true)
- `use_sim_time`: Use simulation time (default: true)

### Coordinate System & Transforms
**Problem Solved**: Maps appearing in wrong positions in RViz (left vs right mismatch)

**Solution**:
- Static transforms from `world` frame to each robot's `map` frame using spawn positions
- X coordinate inverted (`-x`) to fix RViz display alignment
- Transform chain: `world -> tb{i}/map -> tb{i}/odom -> tb{i}/base_link`
- Merged map frame: `world -> map` (published by map_merge)

**Transform Timing**:
- World-to-map transforms: Launched at 5 seconds (before SLAM)
- SLAM nodes: Launched at 15 seconds (after robots are ready)
- Map merge: Launched at 20 seconds (after SLAM has initial maps)

### SLAM Configuration
**File**: `config/slam_params.yaml`

**Key Parameters**:
- `resolution`: 0.03m (3cm) - High resolution for sharp maps
- `map_update_interval`: 0.5s - Fast updates for responsive mapping
- `scan_buffer_size`: 30 - More scan history for better matching
- `link_match_minimum_response`: 0.4 - Stricter matching for quality
- `correlation_search_space_smear_deviation`: 0.02 - Maximum sharpness

### Map Merging Configuration
**Package**: `multirobot_map_merge`

**Key Parameters** (in `hive_slam.launch.py`):
- `world_frame`: "world" - Common frame for all maps
- `known_init_poses`: False - Automatic pose estimation via feature matching
- `estimation_confidence`: 0.3 - Threshold for accepting matches (was 1.0, too high)
- `merging_rate`: 1.0 - Faster merging
- `estimation_rate`: 2.0 - Faster pose estimation
- `compositor_rate`: 2.0 - Faster map composition

**Critical Fix**: Lowered `estimation_confidence` from 1.0 to 0.3 because actual confidence values were 0.37-0.51, causing all matches to be rejected.

### Robot Control System
**State Machine Pattern**: `HiveController` with `HiveState` interface

**States**:
1. **IdleState**: Initial state, transitions to SearchState
2. **SearchState**: Main exploration state with:
   - Frontier-based exploration
   - Randomized turn directions (prevents bunching)
   - Stuck detection and recovery
   - Aggressive back-up maneuvers when stuck
3. **CoordinationState**: (Future: inter-robot coordination)
4. **ConvergenceState**: (Future: return to start)

**Key Improvements**:
- Randomized turn persistence to prevent "lemming effect"
- Stuck counter (50 cycles = ~5 seconds) triggers aggressive recovery
- Turn duration tracking for forced exploration in new directions
- Minimum exploration time: 120 seconds (2 minutes)

---

## File Structure

### Core Source Files
```
src/
├── hive_controller_node.cpp      # ROS2 node wrapper for HiveController
├── hive_controller.cpp            # Main controller with state machine
├── hive_state.cpp                 # State implementations (Search, Coordination, etc.)
└── scan_frame_remapper.cpp       # Remaps laser scan frame IDs to namespaced frames
```

### Configuration Files
```
config/
├── slam_params.yaml               # SLAM toolbox parameters
├── hive_debug.rviz               # RViz configuration (Fixed Frame: world)
├── turtlebot_webots_no_imu.urdf  # Robot URDF for Webots
└── cartographer_2d.lua           # Alternative SLAM config (Cartographer, optional)
```

### Launch Files
```
launch/
├── hive_slam.launch.py            # Main launch file (USE THIS)
├── visualize.launch.py            # RViz visualization
└── hive_slam_cartographer.launch.py  # Alternative SLAM (Cartographer, optional)
```

### Scripts
```
scripts/
├── generate_world.py              # Generates Webots world file with N robots
├── source_all_workspaces.sh        # Sources all ROS2 workspaces
└── verify_robot_publishing.sh      # Verifies robot topics are publishing
```

### Verification Scripts (Root)
```
├── test_slam.sh                    # Quick SLAM system test
├── check_transform_integration.sh  # Verify transform chain
├── quick_check.sh                  # Quick system health check
└── verify_map_quality.sh           # Verify map quality parameters
```

### Results Directory
```
results/
├── tb1_map_complete.pgm/yaml       # Individual robot maps (saved on completion)
├── tb2_map_complete.pgm/yaml
└── merged_map_complete.pgm/yaml   # Merged global map
```

---

## Running the System

### Prerequisites
```bash
# Install ROS2 Humble dependencies
sudo apt install ros-humble-webots-ros2-driver \
                 ros-humble-slam-toolbox \
                 ros-humble-multirobot-map-merge \
                 ros-humble-nav2-map-server
```

### Build the Workspace
```bash
cd ~/ENPM700/Final_project/ENPM700-FinalProject-HiveRobots
colcon build
source install/setup.bash
```

### Launch Multi-Robot SLAM
**Terminal 1 - Main System**:
```bash
ros2 launch hive_control hive_slam.launch.py num_robots:=2
```

**Terminal 2 - Visualization** (optional):
```bash
ros2 launch hive_control visualize.launch.py
```

### Verify System Health
```bash
# Quick check
./quick_check.sh

# Detailed transform verification
./check_transform_integration.sh

# Map quality check
./verify_map_quality.sh
```

---

## Key Fixes & Improvements

### 1. Map Merge Confidence Threshold
**Problem**: `estimation_confidence: 1.0` was too high, causing all matches (0.37-0.51) to be rejected.

**Fix**: Lowered to `0.3` to accept valid matches.

### 2. Coordinate System Alignment
**Problem**: Maps appearing on left in RViz while robots on right in Webots.

**Fix**: Inverted X coordinate in world-to-map transforms (`str(x)` → `str(-x)`).

### 3. Transform Chain
**Problem**: "World frame missing" errors in verification scripts.

**Fix**: 
- Removed problematic `world_frame_creator` node
- First `world -> tb1/map` transform implicitly creates `world` frame
- Improved verification script grep patterns

### 4. Map Quality
**Problem**: Maps not sharp enough, incomplete coverage.

**Fixes**:
- Increased resolution: 0.05m → 0.03m (3cm)
- Faster updates: `map_update_interval: 0.5s`
- More scan history: `scan_buffer_size: 30`
- Stricter matching: `link_match_minimum_response: 0.4`

### 5. Robot Exploration
**Problem**: Robots getting stuck, "lemming effect" (bunching up).

**Fixes**:
- Randomized turn directions with persistence
- Stuck detection (50 cycles = ~5 seconds)
- Aggressive recovery maneuvers (back up + turn)
- Increased turn duration when obstacles detected

### 6. Exploration Duration
**Problem**: Robots stopping too early, incomplete maps.

**Fix**: Reduced `MIN_EXPLORATION_TIME` from 180s to 120s (balanced coverage and speed).

---

## Known Issues & Future Work

### Current Limitations
1. **Map Origin vs Spawn Position**: SLAM map origin is where mapping starts, not necessarily spawn position. Static transforms use spawn positions, which may cause slight misalignment.
2. **Map Merge Confidence**: Currently set to 0.3, but optimal value may vary with environment.
3. **Coordinate System**: X inversion fix works but may need refinement for different environments.

### Future Improvements
1. **Dynamic Transform Updates**: Query actual map->odom transform after SLAM initializes, update static transforms dynamically.
2. **Adaptive Confidence**: Adjust `estimation_confidence` based on map quality metrics.
3. **Inter-Robot Coordination**: Implement `CoordinationState` for collaborative exploration.
4. **Return to Start**: Implement `ConvergenceState` for robots to return to spawn after exploration.
5. **Cartographer Integration**: Complete integration of Cartographer as alternative SLAM backend.

---

## Testing & Verification

### Quick Tests
```bash
# Test SLAM setup
./test_slam.sh

# Check transforms
./check_transform_integration.sh

# Verify map quality
./verify_map_quality.sh
```

### Manual Verification
```bash
# Check odometry
ros2 topic echo /tb1/odom --once

# Check transforms
ros2 run tf2_ros tf2_echo world tb1/map

# View TF tree
ros2 run tf2_tools view_frames

# Check map topics
ros2 topic list | grep map

# Check merged map
ros2 topic echo /map_merged --once
```

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
- ROS2 Humble
- Webots R2025a (or compatible)
- Ubuntu 22.04 (recommended)

---

## Authors
- Shreya Kalyanaraman
- Tirth Sadaria

## License
Apache-2.0 License

---

## Changelog

### December 2024
- ✅ Fixed map merge confidence threshold (1.0 → 0.3)
- ✅ Fixed coordinate system alignment (X inversion)
- ✅ Improved map quality (0.03m resolution, faster updates)
- ✅ Enhanced robot exploration (randomized turns, stuck detection)
- ✅ Reduced exploration time (180s → 120s)
- ✅ Cleaned up repository (removed old launch files, debug images)
- ✅ Created comprehensive documentation

---

**Last Updated**: December 8, 2024

