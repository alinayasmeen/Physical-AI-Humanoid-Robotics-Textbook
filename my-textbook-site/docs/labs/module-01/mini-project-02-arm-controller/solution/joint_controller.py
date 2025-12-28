#!/usr/bin/env python3
"""
Mini-Project 2: Joint Controller Node - Solution
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger


class JointController(Node):
    """Controls arm joints and publishes state."""

    def __init__(self):
        super().__init__('joint_controller')

        # Joint configuration
        self.joint_names = ['shoulder', 'elbow']

        # Joint limits (from URDF)
        self.joint_limits = {
            'shoulder': {'lower': -1.57, 'upper': 1.57},
            'elbow': {'lower': 0.0, 'upper': 2.35}
        }

        # Current and target positions
        self.current_positions = [0.0, 0.0]
        self.target_positions = [0.0, 0.0]

        # Emergency stop flag
        self.stopped = False

        # Interpolation step (radians per update)
        self.step = 0.05

        # Subscriber for joint commands
        self.command_sub = self.create_subscription(
            Float64MultiArray,
            '/joint_commands',
            self.command_callback,
            10
        )

        # Publisher for joint states
        self.state_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Emergency stop service
        self.estop_srv = self.create_service(
            Trigger,
            '/emergency_stop',
            self.emergency_stop_callback
        )

        # Reset service
        self.reset_srv = self.create_service(
            Trigger,
            '/reset_estop',
            self.reset_callback
        )

        # State publishing timer (30 Hz)
        self.timer = self.create_timer(1.0 / 30.0, self.publish_state)

        self.get_logger().info('Joint controller started')
        self.get_logger().info('Listening for commands on /joint_commands')

    def clamp(self, value, lower, upper):
        """Clamp value between lower and upper bounds."""
        return max(lower, min(upper, value))

    def interpolate(self, current, target):
        """Move current toward target by step amount."""
        if abs(target - current) < self.step:
            return target
        elif target > current:
            return current + self.step
        else:
            return current - self.step

    def command_callback(self, msg):
        """Handle incoming joint commands."""
        if self.stopped:
            self.get_logger().warn('Emergency stop active - ignoring command')
            return

        if len(msg.data) != len(self.joint_names):
            self.get_logger().error(
                f'Invalid command length: expected {len(self.joint_names)}, got {len(msg.data)}'
            )
            return

        # Set target positions with limit enforcement
        for i, name in enumerate(self.joint_names):
            limits = self.joint_limits[name]
            clamped = self.clamp(msg.data[i], limits['lower'], limits['upper'])

            if clamped != msg.data[i]:
                self.get_logger().warn(
                    f'{name} command {msg.data[i]:.2f} clamped to {clamped:.2f}'
                )

            self.target_positions[i] = clamped

    def publish_state(self):
        """Publish current joint states."""
        # Interpolate current toward target (smooth motion)
        for i in range(len(self.current_positions)):
            if not self.stopped:
                self.current_positions[i] = self.interpolate(
                    self.current_positions[i],
                    self.target_positions[i]
                )

        # Create and publish JointState message
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = list(self.current_positions)
        msg.velocity = [0.0, 0.0]
        msg.effort = [0.0, 0.0]

        self.state_pub.publish(msg)

    def emergency_stop_callback(self, request, response):
        """Handle emergency stop service call."""
        self.stopped = True
        self.target_positions = list(self.current_positions)

        response.success = True
        response.message = 'Emergency stop activated - arm motion halted'
        self.get_logger().error('EMERGENCY STOP ACTIVATED')

        return response

    def reset_callback(self, request, response):
        """Reset emergency stop."""
        self.stopped = False
        response.success = True
        response.message = 'Emergency stop reset - arm control enabled'
        self.get_logger().info('Emergency stop reset')

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
