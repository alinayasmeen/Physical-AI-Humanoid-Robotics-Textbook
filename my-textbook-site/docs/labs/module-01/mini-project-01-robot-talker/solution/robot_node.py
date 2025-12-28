#!/usr/bin/env python3
"""
Mini-Project 1: Robot Status Node - Solution
Publishes robot status at 1 Hz with simulated battery drain.
"""

import json
import time
import random
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RobotStatusNode(Node):
    """Publishes robot status information."""

    def __init__(self):
        super().__init__('robot_status_node')

        # Declare and get robot_id parameter
        self.declare_parameter('robot_id', 'robot_1')
        self.robot_id = self.get_parameter('robot_id').value

        # Create publisher
        topic_name = f'/{self.robot_id}/status'
        self.publisher = self.create_publisher(String, topic_name, 10)

        # Create 1 Hz timer
        self.timer = self.create_timer(1.0, self.publish_status)

        # Internal state
        self.battery = 100.0
        self.status = 'active'
        self.position = {
            'x': random.uniform(-5, 5),
            'y': random.uniform(-5, 5)
        }

        self.get_logger().info(f'{self.robot_id} status node started')
        self.get_logger().info(f'Publishing to {topic_name}')

    def publish_status(self):
        """Publish current robot status."""
        # Simulate battery drain
        self.battery = max(0, self.battery - 0.1)

        # Update status based on battery
        if self.battery < 10:
            self.status = 'error'
        elif self.battery < 20:
            self.status = 'low_battery'
        else:
            self.status = 'active'

        # Simulate slight position movement
        self.position['x'] += random.uniform(-0.1, 0.1)
        self.position['y'] += random.uniform(-0.1, 0.1)

        # Create status dictionary
        status_data = {
            'robot_id': self.robot_id,
            'status': self.status,
            'battery': round(self.battery, 1),
            'position': {
                'x': round(self.position['x'], 2),
                'y': round(self.position['y'], 2)
            },
            'timestamp': time.strftime('%Y-%m-%dT%H:%M:%S')
        }

        # Publish as JSON string
        msg = String()
        msg.data = json.dumps(status_data)
        self.publisher.publish(msg)

        # Log at debug level to avoid spam
        self.get_logger().debug(f'Published: battery={self.battery:.1f}%')


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
