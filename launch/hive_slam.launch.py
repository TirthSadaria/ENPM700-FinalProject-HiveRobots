#!/usr/bin/env python3

"""
Complete Multi-Robot SLAM Launch File
Dynamically launches N robots with SLAM and map merging.
Supports 1-10 robots with predefined spawn positions.

IMPORTANT: If map_merge is in a separate workspace (~/ros_deps_ws), make sure to source it:
    source ~/ros_deps_ws/install/setup.bash
    source install/setup.bash
    ros2 launch hive_control hive_slam.launch.py num_robots:=3
"""

import os
import sys
import tempfile
import yaml
import subprocess

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, OpaqueFunction, GroupAction, ExecuteProcess, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


# Predefined spawn positions (10 positions, spaced from center)
# Format: (x, y, z) - positions relative to center (0, 0, 0)
# Spacing: 1.5m between robots to prevent collisions
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
    """Helper function to create controller spawners from YAML file."""
    with open(controller_params_file, "r") as f:
        controller_params = yaml.safe_load(f)

    controller_names = []

    # Extract controller names from the YAML structure
    if namespace in controller_params:
        if "controller_manager" in controller_params[namespace]:
            controller_names = list(
                controller_params[namespace]["controller_manager"]["ros__parameters"].keys()
            )
    elif "controller_manager" in controller_params:
        controller_names = list(
            controller_params["controller_manager"]["ros__parameters"].keys()
        )

    # Filter out non-controller parameters
    controller_names = [
        name
        for name in controller_names
        if name not in ["update_rate", "publish_rate"]
    ]

    # Create controller spawners
    # IMPORTANT: Spawn joint_state_broadcaster first, then diffdrive_controller
    # This ensures proper initialization order
    controller_spawners = []
    
    # Sort to ensure joint_state_broadcaster comes first
    sorted_controller_names = sorted(
        controller_names, 
        key=lambda x: (x != "joint_state_broadcaster", x)
    )
    
    for controller_name in sorted_controller_names:
        controller_spawners.append(
            Node(
                package="controller_manager",
                executable="spawner",
                output="screen",
                arguments=[
                    controller_name,
                    "--controller-manager-timeout",
                    "100",  # Increased timeout for better reliability
                    "--controller-manager",
                    "controller_manager",
                ],
                namespace=namespace,
            )
        )
    return controller_spawners


def launch_setup(context, *args, **kwargs):
    """Dynamically generate robot nodes based on num_robots argument."""
    # HOTFIX: https://github.com/cyberbotics/webots_ros2/issues/567
    if "LD_LIBRARY_PATH" in os.environ:
        os.environ["LD_LIBRARY_PATH"] += ":/opt/ros/humble/lib/controller"
    else:
        os.environ["LD_LIBRARY_PATH"] = "/opt/ros/humble/lib/controller"

    # Get the number of robots from the launch argument
    num_robots = int(context.launch_configurations['num_robots'])
    
    # Validate num_robots
    if num_robots < 1 or num_robots > 10:
        raise ValueError("num_robots must be between 1 and 10")
    
    # Get package directories
    webots_ros2_turtlebot = get_package_share_directory("webots_ros2_turtlebot")
    hive_control_package = get_package_share_directory("hive_control")
    
    # Launch configurations
    world = LaunchConfiguration("world")
    mode = LaunchConfiguration("mode")
    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_slam = LaunchConfiguration("enable_slam")
    enable_map_merge = LaunchConfiguration("enable_map_merge")

    # Generate world file with only num_robots robots
    # This ensures Webots only spawns the robots we need
    # Try multiple locations to find the script
    # Get the workspace root by going up from package share directory
    # install/hive_control/share/hive_control -> install/hive_control/share -> install/hive_control -> install -> workspace_root
    workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(hive_control_package))))
    source_script_path = os.path.join(workspace_root, "scripts", "generate_world.py")
    
    script_locations = [
        # Source directory (for development) - workspace_root/scripts/generate_world.py
        source_script_path,
        # Install directory (lib) - if script was installed
        os.path.join(hive_control_package, "lib", "hive_control", "generate_world.py"),
        # Install directory (share) - fallback
        os.path.join(hive_control_package, "scripts", "generate_world.py"),
    ]
    
    generate_world_script = None
    for script_path in script_locations:
        if os.path.exists(script_path):
            generate_world_script = script_path
            print(f"[INFO] Found generate_world.py at: {script_path}")
            break
    
    if not generate_world_script:
        # Last resort: use source directory even if not found (will fail with clear error)
        generate_world_script = script_locations[0]
        print(f"[WARN] Script not found in any location, using: {generate_world_script}")
    temp_world_file = tempfile.NamedTemporaryFile(
        mode='w', delete=False, suffix='.wbt', prefix='warehouse_'
    )
    temp_world_file.close()
    
    # Generate world file with only num_robots robots
    try:
        result = subprocess.run(
            [sys.executable, generate_world_script, str(num_robots), temp_world_file.name],
            check=True,
            capture_output=True,
            text=True
        )
        print(f"[INFO] Generated world file with {num_robots} robots: {temp_world_file.name}")
        if result.stdout:
            print(result.stdout)
    except subprocess.CalledProcessError as e:
        print(f"[ERROR] Failed to generate world file: {e}")
        if e.stderr:
            print(f"[ERROR] stderr: {e.stderr}")
        # Fallback to original world file
        world_name = context.launch_configurations.get('world', 'warehouse.wbt')
        temp_world_file.name = os.path.join(
            hive_control_package, "worlds", world_name
        )
        print(f"[WARN] Using fallback world file: {temp_world_file.name}")
    
    # Use generated world file
    world_file_path = temp_world_file.name

    # Launch Webots ONCE with the generated world file
    webots = WebotsLauncher(
        world=world_file_path,
        mode=mode,
        ros2_supervisor=True,
    )

    # Get paths
    robot_description_path = os.path.join(
        hive_control_package, "config", "turtlebot_webots_no_imu.urdf"
    )
    ros2_control_params_path = os.path.join(
        webots_ros2_turtlebot, "resource", "ros2control.yml"
    )

    # Load the original YAML and create namespaced versions for each robot
    with open(ros2_control_params_path, "r") as f:
        original_params = yaml.safe_load(f)

    def create_namespaced_params_file(namespace: str) -> str:
        """Create a temporary YAML file with namespace at root level."""
        namespaced_params = {namespace: original_params}
        temp_file = tempfile.NamedTemporaryFile(
            mode="w", delete=False, suffix=".yml"
        )
        yaml.dump(namespaced_params, temp_file, default_flow_style=False)
        temp_file.close()
        return temp_file.name

    # Create namespaced parameter files for each robot dynamically
    robot_params_map = {}
    for i in range(1, num_robots + 1):
        namespace = f"tb{i}"
        robot_params_map[namespace] = create_namespaced_params_file(namespace)

    # Create robot controller nodes and spawners
    robot_nodes = []
    all_spawners = []

    # Dynamic robot generation based on num_robots
    for i in range(1, num_robots + 1):
        namespace = f"tb{i}"
        robot_name = f"TurtleBot3Burger_{i}"
        
        # Get spawn position (use predefined positions, wrap around if > 10)
        spawn_pos = SPAWN_POSITIONS[(i - 1) % len(SPAWN_POSITIONS)]
        
        # Topic remappings
        use_twist_stamped = "ROS_DISTRO" in os.environ and (
            os.environ["ROS_DISTRO"] in ["rolling", "jazzy"]
        )

        if use_twist_stamped:
            mappings = [
                (
                    f'/{namespace}/diffdrive_controller/cmd_vel',
                    "cmd_vel",
                ),
                (
                    f'/{namespace}/diffdrive_controller/odom',
                    "odom",
                ),
                ("/scan", f'/{namespace}/scan_raw'),
                ("/scan/point_cloud", f'/{namespace}/scan/point_cloud'),
            ]
        else:
            mappings = [
                (
                    f'/{namespace}/diffdrive_controller/cmd_vel_unstamped',
                    "cmd_vel",
                ),
                (
                    f'/{namespace}/diffdrive_controller/odom',
                    "odom",
                ),
                ("/scan", f'/{namespace}/scan_raw'),
                ("/scan/point_cloud", f'/{namespace}/scan/point_cloud'),
            ]

        # Use WebotsController - pass namespaced parameter file
        webots_controller = WebotsController(
            robot_name=robot_name,
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

        # Static TF from base_link to laser frame for each robot namespace
        # CRITICAL: Add 180° rotation around Z to fix backwards lidar orientation
        # Using quaternion format: x y z qx qy qz qw
        # For 180° rotation around Z: qx=0, qy=0, qz=1, qw=0
        static_laser_tf = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=f'{namespace}_laser_tf',
            namespace=namespace,
            arguments=[
                "0",
                "0",
                "0",  # x y z
                "0",
                "0",
                "1",
                "0",  # qx qy qz qw (180° rotation around Z)
                f'{namespace}/base_link',  # parent frame
                f'{namespace}/LDS-01',  # child (laser) frame - namespaced
            ],
            output="screen",
        )
        robot_nodes.append(static_laser_tf)
        
        # Frame remapper: remaps scan frame_id from LDS-01 to {namespace}/LDS-01
        frame_remapper = Node(
            package="hive_control",
            executable="scan_frame_remapper",
            name="scan_frame_remapper",
            namespace=namespace,
            remappings=[
                ("scan_in", "scan_raw"),  # Subscribe to raw scan from WebotsController
                ("scan", "scan"),  # Publish remapped scan
            ],
            output="screen",
        )
        robot_nodes.append(frame_remapper)

        # Add hive controller for each robot
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

        # Get controller spawners for this robot
        spawners = get_controller_spawners(
            robot_params_map[namespace], namespace
        )
        all_spawners.extend(spawners)

    # Delay spawners to ensure controller_manager is ready
    # Increased delay to 10 seconds to give Webots and hardware more time to initialize
    # The controller manager needs time to fully configure before controllers can be activated
    delayed_spawners = TimerAction(
        period=10.0,  # Increased to 10.0 to allow hardware to fully activate
        actions=all_spawners
    )

    # ============================================================================
    # SLAM Nodes (launched with delay to ensure robots are ready)
    # ============================================================================
    
    # Path to SLAM parameters file
    slam_params_file = PathJoinSubstitution([
        hive_control_package, "config", "slam_params.yaml"
    ])
    
    # Create SLAM nodes for each robot dynamically
    slam_nodes = []
    
    for i in range(1, num_robots + 1):
        namespace = f"tb{i}"
        slam_node = Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            namespace=namespace,
            parameters=[
                slam_params_file,
                {
                    "use_sim_time": use_sim_time,
                    "odom_frame": f"{namespace}/odom",
                    "map_frame": f"{namespace}/map",
                    "base_frame": f"{namespace}/base_link",
                    "scan_topic": "scan",
                    # Explicitly set laser range parameters to match LDS-01 sensor
                    "minimum_laser_range": 0.1,  # LDS-01 minimum range
                    "max_laser_range": 3.5,      # LDS-01 maximum range
                    # Map update rate - very conservative to prevent queue overflow
                    "map_update_interval": 2.0,  # Update every 2.0 seconds (very conservative)
                    "transform_timeout": 10.0,  # Very long timeout to prevent queue overflow
                    # Set consistent map size for all robots to enable proper merging
                    "map_start_size": 1000,  # Large initial size (1000x1000 cells) for consistent maps
                    "resolution": 0.03,  # 3cm resolution - sharper maps, consistent across all robots
                },
            ],
            remappings=[
                ("/scan", f"/{namespace}/scan"),  # Explicitly remap scan topic
                ("/odom", f"/{namespace}/odom"),  # CRITICAL: Remap odom topic so SLAM can track robot position
                ("/map", f"/{namespace}/map"),
                ("/map_metadata", f"/{namespace}/map_metadata"),
            ],
            output="screen",
            condition=IfCondition(enable_slam),
        )
        slam_nodes.append(slam_node)
    
    # Delay SLAM launch to ensure robots and controllers are fully initialized
    # Increased delay to allow TF tree to stabilize and prevent message queue overflow
    delayed_slam = TimerAction(
        period=15.0,  # Launch SLAM 15 seconds after simulation starts (increased from 12.0)
        actions=slam_nodes,
    )
    
    # ============================================================================
    # Map Merging Node (launched after SLAM to merge individual maps)
    # ============================================================================
    
    # Map merge node - stitches N local maps into one global map
    # Note: Setting known_init_poses parameters in robot namespaces is complex
    # because the map_merge node doesn't exist when we need to set them.
    # Using known_init_poses: False allows automatic pose estimation, which works well.
    # The warnings are harmless - map_merge will estimate poses automatically.
    map_merge_node = Node(
        package="multirobot_map_merge",
        executable="map_merge",
        name="map_merge",
        output="screen",
        parameters=[{
            "robot_map_topic": "map",
            "robot_map_updates_topic": "map_updates",
            "robot_namespace": "tb",  # Discovers tb1, tb2, ..., tbN
            "merged_map_topic": "map_merged",
            "world_frame": "world",  # Use 'world' as the common frame (we publish transforms to it)
            "known_init_poses": False,  # Let map_merge estimate poses automatically (works well)
            "merging_rate": 1.0,  # Faster merging for sharper maps (was 0.5 for less flickering)
            "discovery_rate": 1.0,
            "estimation_rate": 2.0,  # Increased from 1.0 for faster pose estimation and better alignment
            "estimation_confidence": 0.3,  # Lower threshold to accept matches (was 1.0, but actual confidence is 0.37-0.51)
            "use_sim_time": use_sim_time,
            "compositor_rate": 2.0,  # Faster compositor for sharper merged maps (was 1.0)
        }],
        remappings=[
            ("/map_merged", "/map_merged"),
        ],
        condition=IfCondition(enable_map_merge),
    )
    
    # CRITICAL: Create 'world' frame first (it doesn't exist by default)
    # Instead of anchoring to odom (which SLAM controls), anchor to a fixed frame
    # We'll publish world -> map directly, since map frames are static relative to world
    # The world frame is created implicitly when we publish world -> tb1/map
    # No separate world_frame_creator needed - the first world -> tb1/map transform creates it
    
    # Static transforms to anchor each robot's map frame to world frame
    # CRITICAL: Webots and ROS use the same coordinate system (X=forward, Y=left, Z=up)
    # BUT: SLAM initializes map origin when mapping starts, which should be at spawn.
    # We use spawn positions to set the initial transform, but map_merge will refine it.
    # NOTE: If maps appear misaligned, the issue is that map origin != spawn position.
    # Solution: Set transforms to spawn positions, but let map_merge refine via feature matching.
    world_to_map_transforms = []
    for i in range(1, num_robots + 1):
        namespace = f"tb{i}"
        spawn_pos = SPAWN_POSITIONS[(i - 1) % len(SPAWN_POSITIONS)]
        x, y, z = spawn_pos
        
        # Publish static transform from 'world' to '{namespace}/map'
        # COORDINATE SYSTEM FIX: Based on user observation:
        #   - Map filling on LEFT in RViz, robots on RIGHT in Webots
        #   - This suggests coordinate system mismatch
        #   - Try inverting X coordinate: if robots spawn at +X, map should appear at -X in RViz
        #   - OR: The issue might be that map origin != spawn position (SLAM starts mapping later)
        # Solution: Invert X to fix the left/right mismatch
        # If this doesn't work, we may need to remove static transforms and let map_merge auto-align
        world_to_map_tf = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=f"{namespace}_world_to_map_tf",
            arguments=[
                str(-x), str(y), str(z),  # -x y z (invert X to fix left/right mismatch in RViz)
                "0", "0", "0", "1",  # qx qy qz qw (identity quaternion - no rotation)
                "world",  # parent frame (created automatically by first transform)
                f"{namespace}/map",  # child frame (robot's map frame)
            ],
            output="screen",
            condition=IfCondition(enable_map_merge),
        )
        world_to_map_transforms.append(world_to_map_tf)
    
    # Static transform publisher for the merged map frame (map_merge publishes to 'map')
    static_map_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_map_tf",
        arguments=[
            "0", "0", "0",  # x y z
            "0", "0", "0", "1",  # qx qy qz qw (identity quaternion)
            "world",  # parent frame (use world as common frame)
            "map",  # child frame (merged map frame - map_merge publishes here)
        ],
        output="screen",
        condition=IfCondition(enable_map_merge),
    )
    
    # CRITICAL FIX: Launch world-to-map transforms EARLY (before SLAM starts)
    # These transforms tell map_merge where each robot's map origin is in world coordinates
    # Without these, maps will stack on top of each other because they don't know their relative positions
    # The first transform (world -> tb1/map) automatically creates the 'world' frame
    delayed_world_to_map_tfs = TimerAction(
        period=5.0,  # Launch transforms 5 seconds after simulation starts (before SLAM at 15s)
        actions=world_to_map_transforms,  # First transform creates world frame automatically
    )
    
    # Delay map merge launch to ensure SLAM nodes have started and maps are being published
    # Increased delay to allow SLAM to build initial maps before merging
    # Launch map_merge node after transforms are already published
    delayed_map_merge = TimerAction(
        period=20.0,  # Launch map merge 20 seconds after simulation starts (increased from 15.0)
        actions=[map_merge_node, static_map_tf],
    )

    return [
        # Launch Webots once
        webots,
        webots._supervisor,
        # Launch all robot controllers
        *robot_nodes,
        # Launch all controller spawners with delay
        delayed_spawners,
        # Launch world-to-map transforms EARLY (before SLAM starts)
        delayed_world_to_map_tfs,
        # Launch SLAM nodes with delay (after robots are ready)
        delayed_slam,
        # Launch map merge node with delay (after SLAM nodes are ready)
        delayed_map_merge,
        # Shutdown when Webots exits
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
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            "world",
            default_value="warehouse.wbt",
            description=(
                "Webots world file (must contain robots: "
                "TurtleBot3Burger_1, TurtleBot3Burger_2, etc.)"
            ),
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time",
        ),
        DeclareLaunchArgument(
            "mode",
            default_value="realtime",
            description="Webots startup mode (realtime, pause, fast)",
        ),
        DeclareLaunchArgument(
            "enable_slam",
            default_value="true",
            description="Enable SLAM for all robots",
        ),
        DeclareLaunchArgument(
            "enable_map_merge",
            default_value="true",
            description="Enable map merging (requires multirobot-map-merge package)",
        ),
        # New argument for dynamic scalability
        DeclareLaunchArgument(
            "num_robots",
            default_value="3",
            description="Number of robots to spawn (1-10). Robots must exist in world file.",
        ),
        # Use OpaqueFunction for dynamic node generation
        OpaqueFunction(function=launch_setup),
    ])
