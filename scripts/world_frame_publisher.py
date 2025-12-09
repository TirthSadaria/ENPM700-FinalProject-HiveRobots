#!/usr/bin/env python3
"""
World Frame Publisher.

Publishes static transforms to establish the world frame and connect it to
robot map frames. This ensures world frame exists and connects properly when
map frames appear from SLAM.
"""

import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import sys


# Spawn positions (must match launch file)
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


class WorldFramePublisher(Node):
    def __init__(self):
        super().__init__('world_frame_publisher')

        # Get number of robots from parameter (default to 3)
        self.declare_parameter('num_robots', 3)
        num_robots = self.get_parameter('num_robots').get_parameter_value().integer_value
        
        # Create static transform broadcaster
        self.static_broadcaster = StaticTransformBroadcaster(self)
        
        # Publish world->map transforms for all robots
        # These will create the world frame and connect when map frames appear
        transforms = []
        for i in range(1, min(num_robots + 1, len(SPAWN_POSITIONS) + 1)):
            if i <= len(SPAWN_POSITIONS):
                x, y, z = SPAWN_POSITIONS[i - 1]
                robot_ns = f'tb{i}'

                transform = TransformStamped()
                transform.header.stamp = self.get_clock().now().to_msg()
                transform.header.frame_id = 'world'
                transform.child_frame_id = f'{robot_ns}/map'
                
                # Use spawn position
                transform.transform.translation.x = float(x)
                transform.transform.translation.y = float(y)
                transform.transform.translation.z = float(z)
                transform.transform.rotation.x = 0.0
                transform.transform.rotation.y = 0.0
                transform.transform.rotation.z = 0.0
                transform.transform.rotation.w = 1.0

                transforms.append(transform)
                self.get_logger().info(
                    f'Publishing world -> {robot_ns}/map at ({x}, {y}, {z})')

        # Store transforms for later publishing
        self.world_map_transforms = transforms

        # Establish world frame by publishing transform to base_link
        # In ROS2, static transforms can create parent frames, but we need
        # to publish FROM world TO an existing frame. Since base_link appears
        # from robot_state_publisher, we use a timer to retry until base_link
        # exists, then publish world->base_link to create the world frame
        # and connect it to the robot TF tree.
        self.world_frame_established = False
        self.create_timer(0.5, self.establish_world_frame)

        # Publish world->map transforms immediately
        if transforms:
            self.static_broadcaster.sendTransform(transforms)
            msg = (f'Published {len(transforms)} world->map transforms '
                   f'(will activate when map frames appear)')
            self.get_logger().info(msg)

    def establish_world_frame(self):
        """
        Establish world frame by publishing transform to base_link.

        Attempts to publish a static transform from world to tb1/base_link.
        This creates the world frame and connects it to the robot TF tree.
        Retries periodically until base_link frame becomes available.
        """
        if self.world_frame_established:
            return  # World frame already established

        try:
            # Create identity transform from world to tb1/base_link
            # This establishes world as the root frame of the TF tree
            world_base_transform = TransformStamped()
            world_base_transform.header.stamp = self.get_clock().now().to_msg()
            world_base_transform.header.frame_id = 'world'
            world_base_transform.child_frame_id = 'tb1/base_link'
            world_base_transform.transform.translation.x = 0.0
            world_base_transform.transform.translation.y = 0.0
            world_base_transform.transform.translation.z = 0.0
            world_base_transform.transform.rotation.x = 0.0
            world_base_transform.transform.rotation.y = 0.0
            world_base_transform.transform.rotation.z = 0.0
            world_base_transform.transform.rotation.w = 1.0

            # Publish transform to establish world frame
            self.static_broadcaster.sendTransform([world_base_transform])
            self.get_logger().info(
                'Published world -> tb1/base_link (world frame established)')
            self.world_frame_established = True
        except Exception:
            # base_link not available yet, will retry on next timer callback
            pass


def main(args=None):
    rclpy.init(args=args)
    node = WorldFramePublisher()

    # Keep node alive to maintain static transforms
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
