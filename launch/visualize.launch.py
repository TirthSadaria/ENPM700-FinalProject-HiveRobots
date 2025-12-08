#!/usr/bin/env python3

"""
RViz Visualization Launch File
Launches RViz2 with a pre-configured view for multi-robot SLAM debugging.
Shows merged map (global) and Robot 1's local data (map + scan).
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('hive_control')
    rviz_config_file = os.path.join(pkg_share, 'config', 'hive_debug.rviz')

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        )
    ])

