#!/usr/bin/env python3
"""
Lab 3: Service Server - Solution
Provides a service for querying robot state
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


class RobotStateServer(Node):
    """Service server that manages robot state."""

    def __init__(self):
        # Initialize the node
        super().__init__('robot_state_server')

        # Create service
        self.srv = self.create_service(
            SetBool,
            '/get_state',
            self.handle_state_request
        )

        # Internal state tracking
        self.is_running = False

        self.get_logger().info('Robot state server ready')

    def handle_state_request(self, request, response):
        """Handle incoming service requests."""
        self.get_logger().info(f'Received request: data={request.data}')

        # Update internal state
        self.is_running = request.data

        # Set response
        response.success = True
        if self.is_running:
            response.message = 'Robot is running'
        else:
            response.message = 'Robot is stopped'

        self.get_logger().info(
            f"Responding: success={response.success}, message='{response.message}'"
        )

        return response


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = RobotStateServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
