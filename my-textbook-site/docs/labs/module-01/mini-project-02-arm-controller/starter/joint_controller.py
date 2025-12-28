#!/usr/bin/env python3
"""
Mini-Project 2: Joint Controller Node - Starter Code

Receives joint commands and publishes joint states.
TODO: Complete the implementation
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
import math


class JointController(Node):
    """Controls arm joints and publishes state."""

    def __init__(self):
        super().__init__('joint_controller')

        # Joint configuration
        self.joint_names = ['shoulder', 'elbow']

        # TODO: Set joint limits (from URDF)
        self.joint_limits = {
            'shoulder': {'lower': -1.57, 'upper': 1.57},
            'elbow': {'lower': 0.0, 'upper': 2.35}
        }

        # Current and target positions
        self.current_positions = [0.0, 0.0]
        self.target_positions = [0.0, 0.0]

        # Emergency stop flag
        self.stopped = False

        # TODO: Create subscriber for joint commands
        # Topic: /joint_commands
        # Type: Float64MultiArray
        # Hint: self.create_subscription(...)

        # TODO: Create publisher for joint states
        # Topic: /joint_states
        # Type: JointState
        # Hint: self.create_publisher(...)

        # TODO: Create emergency stop service
        # Service: /emergency_stop
        # Type: Trigger
        # Hint: self.create_service(...)

        # TODO: Create timer for state publishing (30 Hz)
        # Hint: self.create_timer(1.0/30.0, self.publish_state)

        self.get_logger().info('Joint controller started')

    def command_callback(self, msg):
        """Handle incoming joint commands."""
        if self.stopped:
            self.get_logger().warn('Emergency stop active - ignoring command')
            return

        if len(msg.data) != len(self.joint_names):
            self.get_logger().error('Invalid command length')
            return

        # TODO: Set target positions with limit enforcement
        for i, name in enumerate(self.joint_names):
            limits = self.joint_limits[name]
            # Clamp value to limits
            # self.target_positions[i] = ...
            pass

    def publish_state(self):
        """Publish current joint states."""
        # TODO: Interpolate current toward target (smooth motion)
        # Use step size of 0.05 radians
        for i in range(len(self.current_positions)):
            if not self.stopped:
                # Interpolate toward target
                pass

        # TODO: Create and publish JointState message
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.current_positions
        msg.velocity = [0.0, 0.0]
        msg.effort = [0.0, 0.0]

        # TODO: Publish message
        # self.state_publisher.publish(msg)

    def emergency_stop_callback(self, request, response):
        """Handle emergency stop service call."""
        # TODO: Set stopped flag and zero targets
        self.stopped = True
        self.target_positions = list(self.current_positions)

        response.success = True
        response.message = 'Emergency stop activated'
        self.get_logger().warn('EMERGENCY STOP ACTIVATED')

        return response


def main(args=None):
    rclpy.init(args=args)
    node = JointController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
