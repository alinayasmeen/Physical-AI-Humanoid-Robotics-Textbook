#!/usr/bin/env python3
"""
Lab 2: Subscriber Node - Solution
Subscribes to velocity commands from /cmd_vel
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class VelocitySubscriber(Node):
    """Subscribes to velocity commands and logs them."""

    def __init__(self):
        # Initialize the node
        super().__init__('velocity_subscriber')

        # Create subscription
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.velocity_callback,
            10
        )

        self.get_logger().info('Velocity subscriber started')

    def velocity_callback(self, msg):
        """Called when a velocity message is received."""
        self.get_logger().info(
            f'Received: linear={msg.linear.x:.2f}, angular={msg.angular.z:.2f}'
        )


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
