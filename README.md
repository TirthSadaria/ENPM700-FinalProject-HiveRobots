# ENPM700 Final Project: Hive Robots

[![Build Status](https://github.com/TirthSadaria/ENPM700-FinalProject-HiveRobots/actions/workflows/ci.yml/badge.svg)](https://github.com/TirthSadaria/ENPM700-FinalProject-HiveRobots/actions/workflows/ci.yml)
[![codecov](https://codecov.io/gh/TirthSadaria/ENPM700-FinalProject-HiveRobots/graph/badge.svg?token=PLACEHOLDER_TOKEN)](https://codecov.io/gh/TirthSadaria/ENPM700-FinalProject-HiveRobots)

---
## Overview
This project implements a multi-robot hive simulation using ROS 2 and Webots. The goal is to develop collaborative robot behaviors, state machines, and mapping in a shared environment, following agile development practices.

## Team (Phase 1)
- Shreya K (Driver)
- Tirth Sadaria (Navigator)

## Sprint Plan
- One-week sprints (3 iterations)
- Pair programming, AIP, and regular commits
- Backlog and sprint planning managed via Google Docs/Sheets

## Sprint Planning & Backlog

- **Sprint Plan (Google Doc):** [Sprint Document](https://docs.google.com/document/d/1GciBjBSQgnnl1llGWNddDnd60kpwyzJ1LujE0GWnF4o/edit?usp=sharing)
  - Includes sprint goals, driver/navigator logs, AIP categories, iteration notes, and meeting summaries.

- **AIP Backlog (Google Sheet):** [Backlog & AIP Spreadsheet](https://docs.google.com/spreadsheets/d/18LbFuGbSA6lyf5C_sq3-1zMn88slf8hpBHtjBDwPiCw/edit?usp=sharing)
  - Contains the full product backlog, task durations, priorities, and AIP tags (Ini, CL, IA, Ref, Test, Act, Bas, Rel, Ex).

## Features
- Multi-robot simulation in Webots
- Unique namespaces for each robot (`/tb1`, `/tb2`)
- Extern controller integration for ROS 2
- State machine controller (C++)
- Initial and revised UML diagrams
- Automated unit testing and CodeCov integration


## Setup Instructions
1. Clone the repository:
   ```bash
   git clone https://github.com/TirthSadaria/ENPM700-FinalProject-HiveRobots.git
   cd ENPM700-FinalProject-HiveRobots
   ```
2. Install dependencies:
   ```bash
   sudo apt install ros-humble-webots-ros2-driver ros-humble-slam-toolbox ros-humble-map-merge
   ```
3. Build the workspace:
   ```bash
   colcon build
   source install/setup.bash
   ```
4. Launch the simulation:
   
   **Basic multi-robot simulation:**
   ```bash
   ros2 launch hive_control spawn_robots.launch.py
   ```
   
   **Multi-robot SLAM with mapping:**
   ```bash
   ros2 launch hive_control multi_robot_slam.launch.py
   ```
   
   **Multi-robot SLAM with RViz visualization:**
   ```bash
   ros2 launch hive_control multi_robot_slam.launch.py rviz:=true
   ```

## UML Diagrams
- See `UML/initial/` for initial class and activity diagrams (PDF).
- Revised diagrams will be added to `UML/revised/` as design evolves.

## Testing & CI
- Unit tests in `test/`
- GitHub Actions for CI
- CodeCov for coverage reporting

## Custom ROS Messages/Services
- [List any new messages/services here]

## License

This project is licensed under the Apache-2.0 License - see the [LICENSE](LICENSE) file for details
