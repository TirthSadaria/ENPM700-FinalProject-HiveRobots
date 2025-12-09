#!/usr/bin/env python3
"""
Generate RViz configuration file dynamically based on number of robots.

Each robot gets a laser scan display with a unique color.
"""

import sys
import os

# Color palette for robot laser scans (RGB values)
ROBOT_COLORS = [
    (255, 0, 0),      # Red - Robot 1
    (0, 255, 0),      # Green - Robot 2
    (0, 0, 255),      # Blue - Robot 3
    (255, 255, 0),    # Yellow - Robot 4
    (255, 0, 255),    # Magenta - Robot 5
    (0, 255, 255),    # Cyan - Robot 6
    (255, 128, 0),    # Orange - Robot 7
    (128, 0, 255),    # Purple - Robot 8
    (255, 192, 203),  # Pink - Robot 9
    (128, 128, 128),  # Gray - Robot 10
]


def generate_laser_scan_display(robot_num, color_rgb):
    """Generate RViz LaserScan display configuration for a robot."""
    r, g, b = color_rgb
    return f"""    - Alpha: 1
      Autocompute Intensity Bounds: true
      Autocompute Value Bounds:
        Max Value: 10
        Min Value: -10
        Value: true
      Axis: Z
      Channel Name: intensity
      Class: rviz_default_plugins/LaserScan
      Color: {r}; {g}; {b}
      Color Transformer: FlatColor
      Decay Time: 0
      Enabled: true
      Invert Rainbow: false
      Max Color: 255; 255; 255
      Max Intensity: 4096
      Min Color: 0; 0; 0
      Min Intensity: 0
      Name: TB{robot_num} Scan
      Position Transformer: XYZ
      Selectable: true
      Size (Pixels): 5
      Size (m): 0.05
      Style: Flat Squares
      Topic:
        Depth: 10
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: BestEffort
        Value: /tb{robot_num}/scan
      Use Fixed Frame: true
      Use rainbow: false
      Value: true"""


def generate_rviz_config(num_robots):
    """Generate complete RViz configuration file."""
    # Generate laser scan displays for all robots
    laser_scans = []
    for i in range(1, num_robots + 1):
        color_idx = (i - 1) % len(ROBOT_COLORS)
        laser_scans.append(generate_laser_scan_display(i, ROBOT_COLORS[color_idx]))
    
    laser_scans_yaml = "\n".join(laser_scans)
    
    # Generate expanded list for display tree
    expanded_list = ["/Global Options1", "/Status1", "/Merged Map1"]
    for i in range(1, num_robots + 1):
        expanded_list.append(f"/TB{i} Scan1")
    
    expanded_yaml = "\n".join([f"        - {item}" for item in expanded_list])
    
    rviz_config = f"""Panels:
  - Class: rviz_common/Displays
    Help Height: 78
    Name: Displays
    Property Tree Widget:
      Expanded:
{expanded_yaml}
      Splitter Ratio: 0.5
    Tree Height: 557
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.03
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 10
      Reference Frame: <Fixed Frame>
      Value: true
{laser_scans_yaml}
    - Alpha: 0.7
      Class: rviz_default_plugins/Map
      Color Scheme: map
      Draw Behind: true
      Enabled: true
      Name: Merged Map
      Topic:
        Depth: 10
        Durability Policy: TransientLocal
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /map_merged
      Value: true
    - Class: rviz_default_plugins/TF
      Enabled: true
      Frame Timeout: 30
      Frames:
        All Enabled: true
        Base Frame: world
      Marker Scale: 0.3
      Name: TF
      Show Arrows: true
      Show Axes: true
      Show Names: true
      Tree:
        {{}}
      Update Interval: 0.1
      Value: true
    - Class: rviz_default_plugins/RobotModel
      Description File: ""
      Description Source: Topic
      Description Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /robot_description
      Enabled: true
      Links:
        All Links Enabled: true
        Expand Joint Details: false
        Expand Link Details: false
        Expand Tree: false
        Link Tree Style: Links in Alphabetic Order
        base_link:
          Alpha: 1
          Show Axes: false
          Show Trail: false
        Value: true
      Name: Robot Model
      TF Prefix: ""
      Update Interval: 0
      Value: true
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: world
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
      Hide Inactive Objects: true
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
      Line color: 128; 128; 0
    - Class: rviz_default_plugins/SetInitialPose
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /initialpose
    - Class: rviz_default_plugins/PublishPoint
      Single click: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /clicked_point
    - Class: rviz_default_plugins/SetGoal
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /goal_pose
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 12
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.06
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Focal Point:
        X: 0
        Y: 0
        Z: 0.2
      Focal Shape Fixed Size: true
      Focal Shape Size: 0.05
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.01
      Pitch: 1.1
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz)
      Yaw: 0.5
    Saved: ~
"""
    
    return rviz_config


def main():
    if len(sys.argv) != 3:
        print("Usage: generate_rviz_config.py <num_robots> <output_file>")
        sys.exit(1)
    
    num_robots = int(sys.argv[1])
    output_file = sys.argv[2]
    
    if num_robots < 1 or num_robots > 10:
        print("Error: num_robots must be between 1 and 10")
        sys.exit(1)
    
    config = generate_rviz_config(num_robots)
    
    # Create directory if it doesn't exist
    os.makedirs(os.path.dirname(output_file), exist_ok=True)
    
    with open(output_file, 'w') as f:
        f.write(config)
    
    print(f"Generated RViz config for {num_robots} robots: {output_file}")


if __name__ == "__main__":
    main()

