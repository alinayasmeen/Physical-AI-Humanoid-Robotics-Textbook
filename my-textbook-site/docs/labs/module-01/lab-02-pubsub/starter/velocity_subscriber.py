#!/usr/bin/env python3
"""
Lab 2: Subscriber Node - Starter Code
Subscribes to velocity commands from /cmd_vel

TODO: Complete the implementation following the instructions below
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class VelocitySubscriber(Node):
    """Subscribes to velocity commands and logs them."""

    def __init__(self):
        # TODO: Initialize the node with name 'velocity_subscriber'
        super().__init__('velocity_subscriber')

        # TODO: Create a subscription to '/cmd_vel' topic
        # Hint: self.create_subscription(Twist, '/cmd_vel', self.velocity_callback, 10)
        self.subscription = None  # Replace with actual subscription

        self.get_logger().info('Velocity subscriber started')

    def velocity_callback(self, msg):
        """Called when a velocity message is received."""
        # TODO: Log the received linear and angular velocities
        # Hint: Use msg.linear.x and msg.angular.z
        # self.get_logger().info(f'Received: linear={...}, angular={...}')
        pass


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = VelocitySubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
