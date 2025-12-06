#!/usr/bin/env python3

"""
Multi-robot simulation launch file using WebotsController.
Launches ONE Webots instance and spawns 2 TurtleBot3 robots with namespaces: tb1, tb2
Each robot operates independently with its own topics and TF tree.
Based on improved controller management patterns.
"""

import os
import launch
import yaml
import tempfile
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from launch_ros.substitutions import FindPackageShare


def get_controller_spawners(controller_params_file, namespace):
    """Helper function to create controller spawners from YAML file."""
    with open(controller_params_file, 'r') as f:
        controller_params = yaml.safe_load(f)

    controller_names = []

    # Extract controller names from the YAML structure
    # Handle both namespaced (namespace: {controller_manager: ...}) and non-namespaced formats
    if namespace in controller_params:
        # Namespaced format: namespace: {controller_manager: ...}
        if 'controller_manager' in controller_params[namespace]:
            controller_names = list(controller_params[namespace]['controller_manager']['ros__parameters'].keys())
    elif 'controller_manager' in controller_params:
        # Non-namespaced format: {controller_manager: ...}
        controller_names = list(controller_params['controller_manager']['ros__parameters'].keys())

    # Filter out non-controller parameters
    controller_names = [name for name in controller_names
                        if name not in ['update_rate', 'publish_rate']]

    # Create controller spawners
    controller_spawners = []
    for controller_name in controller_names:
        controller_spawners.append(Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            arguments=[
                controller_name,
                '--controller-manager-timeout', '50',
                '--controller-manager', 'controller_manager',
            ],
            namespace=namespace
        ))
    return controller_spawners


def generate_launch_description():
    # HOTFIX: https://github.com/cyberbotics/webots_ros2/issues/567
    if 'LD_LIBRARY_PATH' in os.environ:
        os.environ['LD_LIBRARY_PATH'] += ':/opt/ros/humble/lib/controller'
    else:
        os.environ['LD_LIBRARY_PATH'] = '/opt/ros/humble/lib/controller'

    # Get package directories
    webots_ros2_turtlebot = get_package_share_directory('webots_ros2_turtlebot')
    hive_control_dir = FindPackageShare('hive_control')

    # Launch arguments
    world = LaunchConfiguration('world', default='turtlebot3_burger.wbt')
    mode = LaunchConfiguration('mode', default='realtime')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Webots world file
    world_file = PathJoinSubstitution([
        hive_control_dir,
        'worlds',
        world,
    ])

    # Robot configurations
    robots = [
        {'name': 'tb1', 'namespace': 'tb1'},
        {'name': 'tb2', 'namespace': 'tb2'},
    ]

    # Launch Webots ONCE with the world file
    webots = WebotsLauncher(
        world=world_file,
        mode=mode,
        ros2_supervisor=True
    )

    # Get paths
    robot_description_path = os.path.join(webots_ros2_turtlebot, 'resource', 'turtlebot_webots.urdf')
    ros2_control_params_path = os.path.join(webots_ros2_turtlebot, 'resource', 'ros2control.yml')

    # Load the original YAML and create namespaced versions for each robot
    with open(ros2_control_params_path, 'r') as f:
        original_params = yaml.safe_load(f)

    def create_namespaced_params_file(namespace):
        """Create a temporary YAML file with namespace at root level."""
        namespaced_params = {namespace: original_params}
        temp_file = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.yml')
        yaml.dump(namespaced_params, temp_file, default_flow_style=False)
        temp_file.close()
        return temp_file.name

    # Create namespaced parameter files for each robot
    tb1_params_file = create_namespaced_params_file('tb1')
    tb2_params_file = create_namespaced_params_file('tb2')

    # Map robot to its parameter file
    robot_params_map = {
        'tb1': tb1_params_file,
        'tb2': tb2_params_file,
    }

    # Create robot controller nodes and spawners
    robot_nodes = []
    all_spawners = []

    for robot in robots:
        # Topic remappings - map from namespaced controller topics to relative topics
        use_twist_stamped = 'ROS_DISTRO' in os.environ and (os.environ['ROS_DISTRO'] in ['rolling', 'jazzy'])
        if use_twist_stamped:
            mappings = [
                (f'/{robot["namespace"]}/diffdrive_controller/cmd_vel', 'cmd_vel'),
                (f'/{robot["namespace"]}/diffdrive_controller/odom', 'odom'),
                # Lidar topics: remap robot-specific LDS topic to generic 'scan'
                (f'/{robot["namespace"]}/LDS-01', 'scan'),
                ('/imu', 'imu'),
            ]
        else:
            mappings = [
                (f'/{robot["namespace"]}/diffdrive_controller/cmd_vel_unstamped', 'cmd_vel'),
                (f'/{robot["namespace"]}/diffdrive_controller/odom', 'odom'),
                # Lidar topics: remap robot-specific LDS topic to generic 'scan'
                (f'/{robot["namespace"]}/LDS-01', 'scan'),
                ('/imu', 'imu'),
            ]

        # Use WebotsController - pass namespaced parameter file
        webots_controller = WebotsController(
            robot_name=robot['name'],
            namespace=robot['namespace'],
            parameters=[
                {
                    'robot_description': robot_description_path,
                    'use_sim_time': True,
                },
                robot_params_map[robot['namespace']]  # Use namespaced parameter file
            ],
            remappings=mappings,
            respawn=True
        )
        robot_nodes.append(webots_controller)

        # Static TF from base_link to laser frame for each robot namespace.
        # Each Webots lidar should be uniquely named in the world.
        # The LaserScan header.frame_id will match these names.
        static_laser_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'{robot["namespace"]}_laser_tf',
            namespace=robot['namespace'],
            arguments=[
                '0', '0', '0',  # x y z
                '0', '0', '0',  # roll pitch yaw
                f'{robot["namespace"]}/base_link',  # parent frame
                f'LDS-01',  # child (laser) frame (matches LaserScan frame_id)
            ],
            output='screen',
        )
        robot_nodes.append(static_laser_tf)

        # Add your hive controller for each robot
        hive_controller = Node(
            package='hive_control',
            executable='hive_controller_node',
            name='hive_controller_node',
            namespace=robot['namespace'],
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
            }],
            remappings=[
                ('cmd_vel', 'cmd_vel'),
                ('scan', 'scan'),
            ]
        )
        robot_nodes.append(hive_controller)

        # Get controller spawners for this robot
        spawners = get_controller_spawners(robot_params_map[robot['namespace']], robot['namespace'])
        all_spawners.extend(spawners)

    # Delay spawners to ensure controller_manager is ready
    # Give WebotsController nodes time to initialize (5 seconds should be enough)
    delayed_spawners = TimerAction(
        period=5.0,
        actions=all_spawners
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='turtlebot3_burger.wbt',
            description='Webots world file (must contain 2 robots: tb1, tb2)'
        ),
        DeclareLaunchArgument(
            'mode',
            default_value='realtime',
            description='Webots startup mode (realtime, pause, fast)'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        # Launch Webots once
        webots,
        webots._supervisor,
        # Launch all robot controllers
        *robot_nodes,
        # Launch all controller spawners with delay
        delayed_spawners,
        # Shutdown when Webots exits
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ])
