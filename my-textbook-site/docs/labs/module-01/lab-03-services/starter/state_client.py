#!/usr/bin/env python3
"""
Lab 3: Service Client - Starter Code
Calls the robot state service

TODO: Complete the implementation following the instructions below
"""

import sys
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


class RobotStateClient(Node):
    """Service client that queries robot state."""

    def __init__(self):
        # TODO: Initialize the node with name 'robot_state_client'
        super().__init__('robot_state_client')

        # TODO: Create a service client for '/get_state'
        # Hint: self.create_client(SetBool, '/get_state')
        self.client = None  # Replace with actual client

        # TODO: Wait for service to be available
        # Hint: while not self.client.wait_for_service(timeout_sec=1.0):
        #           self.get_logger().info('Waiting for service...')

        self.get_logger().info('Connected to state service')

    def send_request(self, request_data):
        """Send a request to the service."""
        # TODO: Create a SetBool.Request
        request = SetBool.Request()

        # TODO: Set request.data to request_data

        # TODO: Call the service asynchronously
        # Hint: self.future = self.client.call_async(request)

        self.get_logger().info(f'Sending request: data={request_data}')


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = RobotStateClient()

    # Get request value from command line (default: True)
    request_value = True
    if len(sys.argv) > 1:
        request_value = sys.argv[1].lower() == 'true'

    # Send request
    node.send_request(request_value)

    # TODO: Spin until response received
    # Hint: while rclpy.ok():
    #           rclpy.spin_once(node)
    #           if node.future.done():
    #               response = node.future.result()
    #               node.get_logger().info(f'Response: {response}')
    #               break

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
