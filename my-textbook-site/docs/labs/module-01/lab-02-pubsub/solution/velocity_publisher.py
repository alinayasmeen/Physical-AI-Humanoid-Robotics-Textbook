#!/usr/bin/env python3
"""
Lab 2: Publisher Node - Solution
Publishes velocity commands to /cmd_vel
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class VelocityPublisher(Node):
    """Publishes velocity commands at 10 Hz."""

    def __init__(self):
        # Initialize the node
        super().__init__('velocity_publisher')

        # Create publisher for Twist messages on /cmd_vel
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Create timer at 10 Hz (0.1 second period)
        self.timer = self.create_timer(0.1, self.publish_velocity)

        self.get_logger().info('Velocity publisher started')

    def publish_velocity(self):
        """Publish velocity command."""
        # Create message
        msg = Twist()

        # Set velocities
        msg.linear.x = 0.5   # Forward at 0.5 m/s
        msg.angular.z = 0.1  # Rotate at 0.1 rad/s

        # Publish
        self.publisher.publish(msg)

        # Log
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
