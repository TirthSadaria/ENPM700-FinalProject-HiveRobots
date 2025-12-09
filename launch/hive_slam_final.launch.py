#!/usr/bin/env python3

"""
Decentralized Multi-Robot SLAM Launch File
Based on slam_toolbox decentralized architecture documentation.
Each robot runs its own SLAM instance, anchored to world frame via static transforms.

This implements the "Shared Reference Frame" approach where:
- Each robot has its own map frame (tbX/map)
- Static transforms anchor each map to world frame
- Map merge combines individual maps into global view

Reference: https://github.com/SteveMacenski/slam_toolbox/blob/ros2/docs/decentralized_multi_robot_slam.md
"""

import os
import sys
import tempfile
import yaml
import subprocess

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
import os


# ---------------------------------------------------------
# CONFIGURATION: MATCH THIS TO generate_world.py
# ---------------------------------------------------------
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


def get_controller_spawners(controller_params_file, namespace):
    """Extract controller spawners from params file."""
    with open(controller_params_file, "r") as f:
        controller_params = yaml.safe_load(f)
    controller_names = []
    if namespace in controller_params:
        if "controller_manager" in controller_params[namespace]:
            controller_names = list(
                controller_params[namespace]["controller_manager"]["ros__parameters"].keys()
            )
    elif "controller_manager" in controller_params:
        controller_names = list(
            controller_params["controller_manager"]["ros__parameters"].keys()
        )
    controller_names = [
        name
        for name in controller_names
        if name not in ["update_rate", "publish_rate"]
    ]
    sorted_controller_names = sorted(
        controller_names, 
        key=lambda x: (x != "joint_state_broadcaster", x)
    )
    controller_spawners = []
    for controller_name in sorted_controller_names:
        controller_spawners.append(
            Node(
                package="controller_manager",
                executable="spawner",
                output="screen",
                arguments=[
                    controller_name,
                    "--controller-manager-timeout",
                    "100",
                    "--controller-manager",
                    "controller_manager",
                ],
                namespace=namespace,
            )
        )
    return controller_spawners


def launch_setup(context, *args, **kwargs):
    """Launch setup for decentralized multi-robot SLAM."""
    # HOTFIX: https://github.com/cyberbotics/webots_ros2/issues/567
    if "LD_LIBRARY_PATH" in os.environ:
        os.environ["LD_LIBRARY_PATH"] += ":/opt/ros/humble/lib/controller"
    else:
        os.environ["LD_LIBRARY_PATH"] = "/opt/ros/humble/lib/controller"

    # Get package directories
    webots_ros2_turtlebot = get_package_share_directory("webots_ros2_turtlebot")
    hive_control_package = get_package_share_directory("hive_control")
    
    # Launch configurations
    num_robots = int(LaunchConfiguration("num_robots").perform(context))
    world = LaunchConfiguration("world")
    mode = LaunchConfiguration("mode")
    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_slam = LaunchConfiguration("enable_slam", default="true")
    enable_map_merge = LaunchConfiguration("enable_map_merge", default="true")

    # Generate world file
    workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(hive_control_package))))
    generate_world_script = os.path.join(workspace_root, "scripts", "generate_world.py")
    
    script_locations = [
        generate_world_script,
        os.path.join(hive_control_package, "lib", "hive_control", "generate_world.py"),
        os.path.join(hive_control_package, "scripts", "generate_world.py"),
    ]
    
    for script_path in script_locations:
        if os.path.exists(script_path):
            generate_world_script = script_path
            break
    
    temp_world_file = tempfile.NamedTemporaryFile(
        mode='w', delete=False, suffix='.wbt', prefix='hive_slam_final_'
    )
    temp_world_file.close()
    
    try:
        result = subprocess.run(
            [sys.executable, generate_world_script, str(num_robots), temp_world_file.name],
            check=True,
            capture_output=True,
            text=True
        )
        print(f"[HIVE_SLAM_FINAL] Generated world file with {num_robots} robots: {temp_world_file.name}")
    except subprocess.CalledProcessError as e:
        print(f"[ERROR] Failed to generate world file: {e}")
        temp_world_file.name = os.path.join(hive_control_package, "worlds", "warehouse.wbt")
        print(f"[WARN] Using fallback world file: {temp_world_file.name}")
    
    world_file_path = temp_world_file.name

    # Webots launcher
    webots = WebotsLauncher(
        world=world_file_path,
        mode=mode,
        ros2_supervisor=True,
    )

    # Robot description and control params
    robot_description_path = os.path.join(
        hive_control_package, "config", "turtlebot_webots_no_imu.urdf"
    )
    ros2_control_params_path = os.path.join(
        webots_ros2_turtlebot, "resource", "ros2control.yml"
    )

    with open(ros2_control_params_path, "r") as f:
        original_params = yaml.safe_load(f)

    def create_namespaced_params_file(namespace: str) -> str:
        # Deep copy the original params
        import copy
        params = copy.deepcopy(original_params)
        
        # CRITICAL FIX: Set frame IDs WITHOUT namespace prefix
        # The diffdrive_controller already runs in the namespace, so it will 
        # automatically add the namespace prefix to frame names.
        # Using "odom" will become "tb1/odom" when running in tb1 namespace.
        if "diffdrive_controller" in params:
            if "ros__parameters" not in params["diffdrive_controller"]:
                params["diffdrive_controller"]["ros__parameters"] = {}
            params["diffdrive_controller"]["ros__parameters"]["odom_frame_id"] = "odom"
            params["diffdrive_controller"]["ros__parameters"]["base_frame_id"] = "base_link"
            params["diffdrive_controller"]["ros__parameters"]["publish_odom_tf"] = True
        
        namespaced_params = {namespace: params}
        temp_file = tempfile.NamedTemporaryFile(
            mode="w", delete=False, suffix=".yml"
        )
        yaml.dump(namespaced_params, temp_file, default_flow_style=False)
        temp_file.close()
        return temp_file.name

    # Create namespaced parameter files
    robot_params_map = {}
    for i in range(1, num_robots + 1):
        namespace = f"tb{i}"
        robot_params_map[namespace] = create_namespaced_params_file(namespace)

    # SLAM params file
    slam_params_file = PathJoinSubstitution([
        hive_control_package, "config", "slam_params.yaml"
    ])

    nodes = []
    robot_nodes = []
    slam_nodes = []
    all_spawners = []

    # =========================================================
    # 1. LAUNCH THE MAP MERGER (Centralized Visualization)
    # =========================================================
    # This takes the N decentralized maps and displays one global view
    # CRITICAL FIX 1: Map Merging Node Setup
    # - known_init_poses: True - USE KNOWN SPAWN COORDINATES!
    # - Init poses passed directly to avoid YAML format issues
    map_merge_params_file = os.path.join(
        get_package_share_directory("hive_control"),
        "config",
        "map_merge_params.yaml"
    )
    
    # Map merge node with estimation mode (auto-aligns maps)
    # Reference: https://github.com/robo-friends/m-explore-ros2
    # Using estimation mode avoids init_pose parameter format issues
    map_merge_node = Node(
        package="multirobot_map_merge",
        executable="map_merge",
        name="map_merge",
        parameters=[
            map_merge_params_file,
            {
                "use_sim_time": use_sim_time,
                "merged_map_topic": "map_merged",
                "known_init_poses": False,  # Use feature-based estimation
                "estimation_confidence": 0.1,  # Very low threshold for faster matching
            },
        ],
        output="screen",
        condition=IfCondition(enable_map_merge),
    )

    # Note: No world->map transform needed - merged map uses frame_id: world directly

    # =========================================================
    # 2. LAUNCH ROBOTS LOOP
    # =========================================================
    for i in range(1, num_robots + 1):
        robot_name = f"tb{i}"
        namespace = robot_name
        
        # Get spawn position
        if i <= len(SPAWN_POSITIONS):
            x, y, z = SPAWN_POSITIONS[i - 1]
        else:
            x, y, z = (0.0, 0.0, 0.0)  # Fallback
        
        yaw = 0.0  # All robots start with 0 yaw (facing forward)

        # Webots Controller
        use_twist_stamped = "ROS_DISTRO" in os.environ and (
            os.environ["ROS_DISTRO"] in ["rolling", "jazzy"]
        )

        if use_twist_stamped:
            mappings = [
                (f'/{namespace}/diffdrive_controller/cmd_vel', "cmd_vel"),
                (f'/{namespace}/diffdrive_controller/odom', "odom"),
                ("/scan", f'/{namespace}/scan_raw'),
                ("/scan/point_cloud", f'/{namespace}/scan/point_cloud'),
            ]
        else:
            mappings = [
                (f'/{namespace}/diffdrive_controller/cmd_vel_unstamped', "cmd_vel"),
                (f'/{namespace}/diffdrive_controller/odom', "odom"),
                ("/scan", f'/{namespace}/scan_raw'),
                ("/scan/point_cloud", f'/{namespace}/scan/point_cloud'),
            ]

        webots_controller = WebotsController(
            robot_name=f"TurtleBot3Burger_{i}",
            namespace=namespace,
            parameters=[
                {
                    "robot_description": robot_description_path,
                    "use_sim_time": True,
                },
                robot_params_map[namespace],
            ],
            remappings=mappings,
            respawn=True,
        )
        robot_nodes.append(webots_controller)

        # =========================================================
        # A. STATIC TRANSFORM (The "Anchor")
        # =========================================================
        # This implements the "Shared Reference Frame" from the docs
        # It tells the system: "tbX/map is located HERE in the world"
        # CRITICAL: Invert X coordinate to fix left/right mismatch in RViz
        world_to_map_tf = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=f'{robot_name}_world_anchor',
            arguments=[
                str(x), str(y), str(z),  # x y z (NO inversion - use actual spawn position)
                "0", "0", "0", "1",  # qx qy qz qw (identity quaternion)
                "world",
                f'{robot_name}/map'
            ],
            parameters=[{"use_sim_time": use_sim_time}],
            output="screen",
        )
        robot_nodes.append(world_to_map_tf)

        # TF: base_link -> LDS-01 (required for SLAM to find lidar frame)
        lidar_tf = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=f'{robot_name}_lidar_tf',
            arguments=["0", "0", "0", "0", "0", "0", "1",
                       f'{robot_name}/base_link', f'{robot_name}/LDS-01'],
            parameters=[{"use_sim_time": use_sim_time}],
            output="screen",
        )
        robot_nodes.append(lidar_tf)

        # Scan Frame Remapper - corrects Webots inverted lidar angles
        # Input: angle_min=+π, angle_max=-π, increment=-0.017
        # Output: angle_min=-π, angle_max=+π, increment=+0.017
        frame_remapper = Node(
            package="hive_control",
            executable="scan_frame_remapper",
            name="scan_frame_remapper",
            namespace=namespace,
            parameters=[{"use_sim_time": use_sim_time}],  # CRITICAL: Use sim_time for this->now()
            remappings=[
                ("scan_in", "scan_raw"),  # Subscribe to raw scan from Webots
                ("scan", "scan"),  # Publish corrected scan for SLAM
            ],
            output="screen",
        )
        robot_nodes.append(frame_remapper)

        # =========================================================
        # B. DECENTRALIZED SLAM - CRITICAL FIX 3
        # =========================================================
        # CRITICAL FIX 3: SLAM Mode & Odometry Robustness
        # - Uses async_slam_toolbox_node for real-time operation
        # - Parameters tuned for odometry drift mitigation
        # - Huber loss function configured for outlier rejection
        slam_node = Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",  # CRITICAL: Async mode for real-time operation
            name="slam_toolbox",
            namespace=namespace,
            parameters=[
                slam_params_file,
                {
                    "use_sim_time": use_sim_time,
                    "base_frame": f'{namespace}/base_link',
                    "odom_frame": f'{namespace}/odom',
                    "map_frame": f'{namespace}/map',
                    "scan_topic": "scan",
                    "minimum_laser_range": 0.1,
                    "max_laser_range": 3.5,
                },
            ],
            remappings=[
                ("/scan", f"/{namespace}/scan"),
                ("/odom", f"/{namespace}/odom"),
                ("/map", f"/{namespace}/map"),
                ("/map_metadata", f"/{namespace}/map_metadata"),
            ],
            output="screen",
            condition=IfCondition(enable_slam),
        )
        slam_nodes.append(slam_node)

        # Controller spawners
        spawners = get_controller_spawners(robot_params_map[namespace], namespace)
        all_spawners.extend(spawners)

        # Hive Controller
        hive_controller = Node(
            package="hive_control",
            executable="hive_controller_node",
            name="hive_controller_node",
            namespace=namespace,
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "cmd_vel_topic": "cmd_vel",
                "scan_topic": "scan",
                "map_topic": "map",
                "enable_map_completion": True,
            }],
        )
        robot_nodes.append(hive_controller)

    # CRITICAL FIX 1: Ensure transforms are published early
    # World anchor transforms must be published before SLAM starts
    # This establishes the world frame and map frame relationships
    
    # Delay spawners (let Webots controllers initialize first)
    # Controller spawners set up diff_drive_controller which publishes odom TF
    delayed_spawners = TimerAction(
        period=10.0,
        actions=all_spawners
    )

    # Delay SLAM launch significantly (after TF tree is fully established)
    # CRITICAL: SLAM needs odom->base_link TF from diff_drive_controller
    # diff_drive_controller starts ~5 seconds after spawners
    # So we need to wait: 10s (spawners) + 5s (controller init) + 10s (buffer) = 25s
    delayed_slam = TimerAction(
        period=25.0,  # Increased from 15s to 25s for TF stability
        actions=slam_nodes,
        condition=IfCondition(enable_slam),
    )

    # map_merge starts after SLAM has had time to generate initial maps
    delayed_map_merge = TimerAction(
        period=35.0,  # 10s after SLAM starts
        actions=[map_merge_node],
        condition=IfCondition(enable_map_merge),
    )

    # Optional: Launch RViz for visualization
    enable_rviz_arg = LaunchConfiguration("enable_rviz")
    pkg_share = get_package_share_directory("hive_control")
    rviz_config_file = os.path.join(pkg_share, "config", "hive_debug.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen",
        condition=IfCondition(enable_rviz_arg),
    )

    nodes_list = [
        webots,
        webots._supervisor,
        *robot_nodes,
        delayed_spawners,
        delayed_slam,
        delayed_map_merge,
    ]
    
    if rviz_node:
        nodes_list.append(rviz_node)
    
    return nodes_list + [
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[
                    launch.actions.EmitEvent(
                        event=launch.events.Shutdown()
                    )
                ],
            )
        ),
    ]


def generate_launch_description():
    hive_control_package = get_package_share_directory("hive_control")
    slam_config = os.path.join(hive_control_package, "config", "slam_params.yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "num_robots",
            default_value="3",
            description="Number of robots to spawn (1-10)",
        ),
        DeclareLaunchArgument(
            "world",
            default_value="warehouse.wbt",
            description="Webots world file",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time",
        ),
        DeclareLaunchArgument(
            "mode",
            default_value="realtime",
            description="Webots startup mode",
        ),
        DeclareLaunchArgument(
            "enable_slam",
            default_value="true",
            description="Enable SLAM",
        ),
        DeclareLaunchArgument(
            "enable_map_merge",
            default_value="true",
            description="Enable map merging",
        ),
        DeclareLaunchArgument(
            "enable_rviz",
            default_value="false",
            description="Enable RViz visualization",
        ),
        DeclareLaunchArgument(
            "slam_params_file",
            default_value=slam_config,
            description="Path to SLAM parameters file",
        ),
        OpaqueFunction(function=launch_setup),
    ])

