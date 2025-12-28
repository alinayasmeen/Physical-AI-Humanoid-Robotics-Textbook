#!/usr/bin/env python3
"""
Mini-Project 1: Dashboard Node - Starter Code

This node aggregates status from all robots.
TODO: Complete the implementation
"""

import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class DashboardNode(Node):
    """Aggregates and displays robot status information."""

    def __init__(self):
        super().__init__('dashboard_node')

        # Store latest status for each robot
        self.robot_statuses = {}

        # TODO: Create subscriptions for known robots
        # For simplicity, subscribe to robot_1, robot_2, robot_3
        # Hint: self.create_subscription(String, '/robot_1/status',
        #           lambda msg: self.status_callback(msg, 'robot_1'), 10)

        # TODO: Create timer for summary logging (every 5 seconds)
        # Hint: self.create_timer(5.0, self.log_summary)

        self.get_logger().info('Dashboard node started')

    def status_callback(self, msg, robot_id):
        """Handle incoming status message."""
        try:
            # TODO: Parse JSON and store with timestamp
            status = json.loads(msg.data)
            status['last_seen'] = time.time()
            self.robot_statuses[robot_id] = status

            self.get_logger().debug(f'Updated {robot_id} status')

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid JSON from {robot_id}: {e}')

    def log_summary(self):
        """Log summary of all robot statuses."""
        current_time = time.time()

        self.get_logger().info('=' * 50)
        self.get_logger().info('ROBOT STATUS SUMMARY')
        self.get_logger().info('=' * 50)

        for robot_id, status in self.robot_statuses.items():
            # TODO: Calculate time since last update
            time_since = current_time - status.get('last_seen', 0)

            # TODO: Mark as 'offline' if no update in 10+ seconds
            current_status = status.get('status', 'unknown')
            if time_since > 10:
                current_status = 'OFFLINE'

            # TODO: Log robot status
            battery = status.get('battery', 0)
            self.get_logger().info(
                f'{robot_id}: {current_status} | Battery: {battery}%'
            )

        if not self.robot_statuses:
            self.get_logger().info('No robots connected')

        self.get_logger().info('=' * 50)


def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
