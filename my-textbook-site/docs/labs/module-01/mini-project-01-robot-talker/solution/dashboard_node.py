#!/usr/bin/env python3
"""
Mini-Project 1: Dashboard Node - Solution
Aggregates status from all robots and provides query service.
"""

import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


class DashboardNode(Node):
    """Aggregates and displays robot status information."""

    def __init__(self):
        super().__init__('dashboard_node')

        # Store latest status for each robot
        self.robot_statuses = {}

        # Known robot IDs (could also discover dynamically)
        self.known_robots = ['robot_1', 'robot_2', 'robot_3']

        # Create subscriptions for each robot
        for robot_id in self.known_robots:
            topic = f'/{robot_id}/status'
            self.create_subscription(
                String,
                topic,
                lambda msg, rid=robot_id: self.status_callback(msg, rid),
                10
            )
            self.get_logger().info(f'Subscribed to {topic}')

        # Create summary timer (every 5 seconds)
        self.summary_timer = self.create_timer(5.0, self.log_summary)

        # Create query service
        self.query_srv = self.create_service(
            Trigger,
            '/query_all_status',
            self.query_callback
        )

        self.get_logger().info('Dashboard node started')

    def status_callback(self, msg, robot_id):
        """Handle incoming status message."""
        try:
            status = json.loads(msg.data)
            status['last_seen'] = time.time()
            self.robot_statuses[robot_id] = status

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid JSON from {robot_id}: {e}')

    def log_summary(self):
        """Log summary of all robot statuses."""
        current_time = time.time()

        self.get_logger().info('')
        self.get_logger().info('=' * 60)
        self.get_logger().info('ROBOT FLEET STATUS DASHBOARD')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'{"Robot":<12} {"Status":<12} {"Battery":<10} {"Position":<20}')
        self.get_logger().info('-' * 60)

        active_count = 0
        offline_count = 0

        for robot_id in self.known_robots:
            if robot_id in self.robot_statuses:
                status = self.robot_statuses[robot_id]
                time_since = current_time - status.get('last_seen', 0)

                # Determine display status
                if time_since > 10:
                    display_status = 'OFFLINE'
                    offline_count += 1
                else:
                    display_status = status.get('status', 'unknown')
                    active_count += 1

                battery = status.get('battery', 0)
                pos = status.get('position', {'x': 0, 'y': 0})
                pos_str = f"({pos['x']:.1f}, {pos['y']:.1f})"

                self.get_logger().info(
                    f'{robot_id:<12} {display_status:<12} {battery:>6.1f}%    {pos_str:<20}'
                )
            else:
                self.get_logger().info(
                    f'{robot_id:<12} {"NOT SEEN":<12} {"---":<10} {"---":<20}'
                )
                offline_count += 1

        self.get_logger().info('-' * 60)
        self.get_logger().info(f'Active: {active_count} | Offline: {offline_count}')
        self.get_logger().info('=' * 60)

    def query_callback(self, request, response):
        """Handle status query service calls."""
        summary = {
            'timestamp': time.strftime('%Y-%m-%dT%H:%M:%S'),
            'robots': {}
        }

        current_time = time.time()
        for robot_id, status in self.robot_statuses.items():
            time_since = current_time - status.get('last_seen', 0)
            summary['robots'][robot_id] = {
                'status': 'offline' if time_since > 10 else status.get('status'),
                'battery': status.get('battery'),
                'last_seen_seconds_ago': round(time_since, 1)
            }

        response.success = True
        response.message = json.dumps(summary, indent=2)
        return response


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
