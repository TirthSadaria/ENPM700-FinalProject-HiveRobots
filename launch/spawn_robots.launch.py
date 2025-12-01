#!/usr/bin/env python3
"""
HIVE Multi-Robot Launch File
Spawns two TurtleBot robots in your existing Webots world with namespaces /tb1 and /tb2
Integrates with your existing hive_control package
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    GroupAction,
    TimerAction,
    ExecuteProcess,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package directory
    hive_control_dir = FindPackageShare('hive_control')

    # Webots world file (NOTE: .wbt extension)
    world_file = PathJoinSubstitution([
        hive_control_dir,
        'worlds',
        'turtlebot3_burger.wbt',
    ])

    # Launch arguments
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Start Webots GUI (informational only)'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # Launch Webots directly using the `webots` binary
    webots_launch = ExecuteProcess(
        cmd=[
            'webots',
            world_file,
        ],
        output='screen'
    )

    # TB1 group
    tb1_group = GroupAction([
        PushRosNamespace('tb1'),

        Node(
            package='hive_control',
            executable='hive_controller_node',
            name='hive_controller_node',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'cmd_vel_topic': '/tb1/cmd_vel',
                'scan_topic': '/tb1/scan',
            }],
            remappings=[
                ('/tf', '/tb1/tf'),
                ('/tf_static', '/tb1/tf_static'),
            ]
        ),
    ])

    # TB2 group
    tb2_group = GroupAction([
        PushRosNamespace('tb2'),

        Node(
            package='hive_control',
            executable='hive_controller_node',
            name='hive_controller_node',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'cmd_vel_topic': '/tb2/cmd_vel',
                'scan_topic': '/tb2/scan',
            }],
            remappings=[
                ('/tf', '/tb2/tf'),
                ('/tf_static', '/tb2/tf_static'),
            ]
        ),
    ])

    return LaunchDescription([
        gui_arg,
        use_sim_time_arg,

        LogInfo(msg=['Starting HIVE Multi-Robot Simulation']),
        LogInfo(msg=['World file: ', world_file]),
        LogInfo(msg=['Robot namespaces: /tb1, /tb2']),

        webots_launch,

        TimerAction(
            period=3.0,
            actions=[tb1_group],
        ),
        TimerAction(
            period=3.5,
            actions=[tb2_group],
        ),
    ])
