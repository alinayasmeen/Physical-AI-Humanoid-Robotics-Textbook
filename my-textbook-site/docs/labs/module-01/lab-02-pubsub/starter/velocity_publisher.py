#!/usr/bin/env python3
"""
Lab 2: Publisher Node - Starter Code
Publishes velocity commands to /cmd_vel

TODO: Complete the implementation following the instructions below
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class VelocityPublisher(Node):
    """Publishes velocity commands at 10 Hz."""

    def __init__(self):
        # TODO: Initialize the node with name 'velocity_publisher'
        super().__init__('velocity_publisher')

        # TODO: Create a publisher for Twist messages on topic '/cmd_vel'
        # Hint: self.create_publisher(Twist, '/cmd_vel', 10)
        self.publisher = None  # Replace with actual publisher

        # TODO: Create a timer that calls publish_velocity every 0.1 seconds
        # Hint: self.create_timer(0.1, self.publish_velocity)

        self.get_logger().info('Velocity publisher started')

    def publish_velocity(self):
        """Publish velocity command."""
        # TODO: Create a Twist message
        msg = Twist()

        # TODO: Set linear.x to 0.5 (forward velocity)

        # TODO: Set angular.z to 0.1 (rotation velocity)

        # TODO: Publish the message
        # Hint: self.publisher.publish(msg)

        # Log the published values
        self.get_logger().info(
            f'Publishing: linear={msg.linear.x:.2f}, angular={msg.angular.z:.2f}'
        )


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = VelocityPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
