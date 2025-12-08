#!/usr/bin/env python3

"""
Multi-Robot SLAM Launch File using Cartographer (Alternative to slam_toolbox)
Dynamically launches N robots with Cartographer SLAM and map merging.
Supports 1-10 robots with predefined spawn positions.

Usage:
    source /opt/ros/humble/setup.bash
    source ~/ros_deps_ws/install/setup.bash  # If map_merge is there
    source install/setup.bash
    ros2 launch hive_control hive_slam_cartographer.launch.py num_robots:=2
"""

import os
import sys
import tempfile
import subprocess

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, OpaqueFunction, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController

# Import shared spawn positions and helper functions from main launch file
# For now, we'll duplicate the essential parts
SPAWN_POSITIONS = [
    (0.0, 0.0, 0.0),      # Robot 1: Center
    (1.5, 0.0, 0.0),      # Robot 2: +X
    (-1.5, 0.0, 0.0),     # Robot 3: -X
    (0.0, 1.5, 0.0),      # Robot 4: +Y
    (0.0, -1.5, 0.0),     # Robot 5: -Y
    (1.5, 1.5, 0.0),      # Robot 6: +X +Y
    (-1.5, -1.5, 0.0),    # Robot 7: -X -Y
    (1.5, -1.5, 0.0),     # Robot 8: +X -Y
    (-1.5, 1.5, 0.0),     # Robot 9: -X +Y
    (3.0, 0.0, 0.0),      # Robot 10: +2X
]


def launch_setup(context, *args, **kwargs):
    """Dynamically generate robot nodes based on num_robots argument."""
    num_robots = int(context.perform_substitution(LaunchConfiguration("num_robots")))
    use_sim_time = context.perform_substitution(LaunchConfiguration("use_sim_time")) == "true"
    enable_slam = context.perform_substitution(LaunchConfiguration("enable_slam")) == "true"
    enable_map_merge = context.perform_substitution(LaunchConfiguration("enable_map_merge")) == "true"
    
    # Get package paths
    hive_control_package = get_package_share_directory("hive_control")
    cartographer_config = os.path.join(hive_control_package, "config", "cartographer_2d.lua")
    
    # Generate world file dynamically
    # ... (same world generation logic as hive_slam.launch.py)
    
    # Create Cartographer nodes for each robot
    cartographer_nodes = []
    occupancy_grid_nodes = []
    
    for i in range(1, num_robots + 1):
        namespace = f"tb{i}"
        
        # Cartographer node
        cartographer_node = Node(
            package="cartographer_ros",
            executable="cartographer_node",
            name="cartographer_node",
            namespace=namespace,
            parameters=[{
                "use_sim_time": use_sim_time,
            }],
            arguments=[
                "-configuration_directory", os.path.dirname(cartographer_config),
                "-configuration_basename", os.path.basename(cartographer_config),
            ],
            remappings=[
                ("/scan", f"/{namespace}/scan"),
                ("/odom", f"/{namespace}/odom"),
            ],
            output="screen",
            condition=IfCondition(enable_slam),
        )
        cartographer_nodes.append(cartographer_node)
        
        # Occupancy grid node (converts Cartographer submaps to ROS map)
        occupancy_grid_node = Node(
            package="cartographer_ros",
            executable="occupancy_grid_node",
            name="occupancy_grid_node",
            namespace=namespace,
            parameters=[{
                "use_sim_time": use_sim_time,
                "resolution": 0.05,  # 5cm resolution (same as slam_toolbox)
                "publish_period_sec": 1.0,
            }],
            remappings=[
                ("/map", f"/{namespace}/map"),
            ],
            output="screen",
            condition=IfCondition(enable_slam),
        )
        occupancy_grid_nodes.append(occupancy_grid_node)
    
    # Note: This is a simplified version - you'd need to include all the robot spawning,
    # world generation, and map merge logic from hive_slam.launch.py
    
    return [
        # Cartographer nodes
        *cartographer_nodes,
        *occupancy_grid_nodes,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "num_robots",
            default_value="2",
            description="Number of robots to spawn (1-10)",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time",
        ),
        DeclareLaunchArgument(
            "enable_slam",
            default_value="true",
            description="Enable SLAM for all robots",
        ),
        DeclareLaunchArgument(
            "enable_map_merge",
            default_value="true",
            description="Enable map merging",
        ),
        OpaqueFunction(function=launch_setup),
    ])

