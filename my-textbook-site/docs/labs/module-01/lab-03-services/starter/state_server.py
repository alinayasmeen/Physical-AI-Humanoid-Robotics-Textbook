#!/usr/bin/env python3
"""
Lab 3: Service Server - Starter Code
Provides a service for querying robot state

TODO: Complete the implementation following the instructions below
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


class RobotStateServer(Node):
    """Service server that manages robot state."""

    def __init__(self):
        # TODO: Initialize the node with name 'robot_state_server'
        super().__init__('robot_state_server')

        # TODO: Create a service '/get_state' with type SetBool
        # Hint: self.create_service(SetBool, '/get_state', self.handle_state_request)
        self.srv = None  # Replace with actual service

        # Internal state tracking
        self.is_running = False

        self.get_logger().info('Robot state server ready')

    def handle_state_request(self, request, response):
        """Handle incoming service requests."""
        # TODO: Log the received request
        # Hint: self.get_logger().info(f'Received request: data={request.data}')

        # TODO: Update internal state based on request
        # If request.data is True, set self.is_running to True
        # If request.data is False, set self.is_running to False

        # TODO: Set response.success to True

        # TODO: Set response.message based on state
        # Hint: 'Robot is running' or 'Robot is stopped'

        # TODO: Return the response
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
