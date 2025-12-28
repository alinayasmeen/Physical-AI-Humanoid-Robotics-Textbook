#!/usr/bin/env python3
"""
Lab 3: Service Client - Solution
Calls the robot state service
"""

import sys
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


class RobotStateClient(Node):
    """Service client that queries robot state."""

    def __init__(self):
        # Initialize the node
        super().__init__('robot_state_client')

        # Create service client
        self.client = self.create_client(SetBool, '/get_state')

        # Wait for service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')

        self.get_logger().info('Connected to state service')
        self.future = None

    def send_request(self, request_data):
        """Send a request to the service."""
        # Create request
        request = SetBool.Request()
        request.data = request_data

        # Call service asynchronously
        self.future = self.client.call_async(request)

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

    # Spin until response received
    while rclpy.ok():
        rclpy.spin_once(node)
        if node.future.done():
            try:
                response = node.future.result()
                node.get_logger().info(
                    f"Response: success={response.success}, message='{response.message}'"
                )
            except Exception as e:
                node.get_logger().error(f'Service call failed: {e}')
            break

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
