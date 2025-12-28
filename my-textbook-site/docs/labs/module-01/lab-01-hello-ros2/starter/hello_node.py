#!/usr/bin/env python3
"""
Lab 1: Hello ROS 2 - Starter Code
Create your first ROS 2 node

TODO: Complete the implementation following the instructions below
"""

import rclpy
from rclpy.node import Node


class HelloNode(Node):
    """A simple ROS 2 node that logs messages."""

    def __init__(self):
        # TODO: Initialize the node with name 'hello_ros2_node'
        # Hint: Call super().__init__('node_name')
        super().__init__('hello_ros2_node')

        # TODO: Log "Hello, ROS 2!" using self.get_logger().info()


        # TODO: Create a timer that calls timer_callback every 2 seconds
        # Hint: self.create_timer(period_seconds, callback)


    def timer_callback(self):
        """Called every 2 seconds by the timer."""
        # TODO: Log "Still running..." using self.get_logger().info()
        pass


def main(args=None):
    """Main entry point."""
    # TODO: Initialize rclpy
    # Hint: rclpy.init(args=args)

    # TODO: Create an instance of HelloNode

    # TODO: Spin the node to keep it running
    # Hint: rclpy.spin(node)

    # TODO: Clean up
    # Hint: node.destroy_node() and rclpy.shutdown()
    pass


if __name__ == '__main__':
    main()
