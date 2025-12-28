#!/usr/bin/env python3
"""
Lab 1: Hello ROS 2 - Solution
Create your first ROS 2 node
"""

import rclpy
from rclpy.node import Node


class HelloNode(Node):
    """A simple ROS 2 node that logs messages."""

    def __init__(self):
        # Initialize the node with name 'hello_ros2_node'
        super().__init__('hello_ros2_node')

        # Log startup message
        self.get_logger().info('Hello, ROS 2!')

        # Create a timer that calls timer_callback every 2 seconds
        self.timer = self.create_timer(2.0, self.timer_callback)

    def timer_callback(self):
        """Called every 2 seconds by the timer."""
        self.get_logger().info('Still running...')


def main(args=None):
    """Main entry point."""
    # Initialize rclpy
    rclpy.init(args=args)

    # Create an instance of HelloNode
    node = HelloNode()

    try:
        # Spin the node to keep it running
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
