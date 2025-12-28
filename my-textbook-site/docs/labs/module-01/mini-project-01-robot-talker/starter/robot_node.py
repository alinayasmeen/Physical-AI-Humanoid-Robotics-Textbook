#!/usr/bin/env python3
"""
Mini-Project 1: Robot Status Node - Starter Code

This node publishes robot status at 1 Hz.
TODO: Complete the implementation
"""

import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RobotStatusNode(Node):
    """Publishes robot status information."""

    def __init__(self):
        super().__init__('robot_status_node')

        # TODO: Declare 'robot_id' parameter with default value 'robot_1'
        # Hint: self.declare_parameter('robot_id', 'robot_1')
        self.robot_id = 'robot_1'  # Replace with parameter

        # TODO: Create publisher for status topic
        # Topic name should be: /robot_<id>/status
        # Hint: f'/{self.robot_id}/status'
        self.publisher = None  # Replace

        # TODO: Create timer for 1 Hz publishing
        # Hint: self.create_timer(1.0, self.publish_status)

        # Internal state
        self.battery = 100
        self.status = 'active'

        self.get_logger().info(f'{self.robot_id} status node started')

    def publish_status(self):
        """Publish current robot status."""
        # TODO: Simulate battery drain (1% every 10 calls)
        # Hint: self.battery = max(0, self.battery - 0.1)

        # TODO: Set status to 'error' if battery < 20
        # Hint: self.status = 'error' if self.battery < 20 else 'active'

        # TODO: Create status dictionary
        status_data = {
            'robot_id': self.robot_id,
            'status': self.status,
            'battery': round(self.battery, 1),
            'position': {'x': 0.0, 'y': 0.0},  # Simulate position
            'timestamp': time.strftime('%Y-%m-%dT%H:%M:%S')
        }

        # TODO: Publish as JSON string
        msg = String()
        msg.data = json.dumps(status_data)
        # self.publisher.publish(msg)

        self.get_logger().info(f'Published: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = RobotStatusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
