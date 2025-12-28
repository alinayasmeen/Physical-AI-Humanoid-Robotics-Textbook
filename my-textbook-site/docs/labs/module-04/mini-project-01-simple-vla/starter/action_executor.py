#!/usr/bin/env python3
"""
Action Executor Module for Simple VLA System
Handles robot action execution via ROS 2.

TODO: Implement the missing functionality as described below.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class ActionExecutor(Node):
    """
    Action executor that sends velocity commands to the robot.
    Publishes Twist messages to /cmd_vel topic.
    """

    def __init__(self):
        super().__init__('action_executor')
        self.get_logger().info('Initializing Action Executor...')

        # TODO: Create a publisher for velocity commands
        # Topic: /cmd_vel
        # Message type: Twist
        self.cmd_vel_pub = None

        # Movement parameters
        self.linear_speed = 0.5  # m/s
        self.angular_speed = 0.5  # rad/s

        # Current state
        self.is_moving = False

    def move_forward(self, duration=1.0):
        """
        Move the robot forward.

        Args:
            duration: How long to move (seconds)

        Returns:
            bool: True if action executed successfully
        """
        # TODO: Implement forward movement
        # 1. Create a Twist message
        # 2. Set linear.x to self.linear_speed
        # 3. Publish the message
        # 4. Log the action
        pass

    def move_backward(self, duration=1.0):
        """
        Move the robot backward.

        Args:
            duration: How long to move (seconds)

        Returns:
            bool: True if action executed successfully
        """
        # TODO: Implement backward movement
        # Similar to forward, but with negative linear.x
        pass

    def turn_left(self, duration=1.0):
        """
        Turn the robot left.

        Args:
            duration: How long to turn (seconds)

        Returns:
            bool: True if action executed successfully
        """
        # TODO: Implement left turn
        # Set angular.z to positive value for left turn
        pass

    def turn_right(self, duration=1.0):
        """
        Turn the robot right.

        Args:
            duration: How long to turn (seconds)

        Returns:
            bool: True if action executed successfully
        """
        # TODO: Implement right turn
        # Set angular.z to negative value for right turn
        pass

    def stop(self):
        """
        Stop all robot movement.

        Returns:
            bool: True if action executed successfully
        """
        # TODO: Implement stop
        # Create Twist message with all zeros and publish
        pass

    def execute_action(self, action_name, **kwargs):
        """
        Execute an action by name.

        Args:
            action_name: Name of the action to execute
            **kwargs: Additional arguments for the action

        Returns:
            bool: True if action executed successfully
        """
        actions = {
            'forward': self.move_forward,
            'backward': self.move_backward,
            'left': self.turn_left,
            'right': self.turn_right,
            'stop': self.stop,
        }

        # TODO: Implement action execution
        # Look up action in actions dict and call it with kwargs
        pass


def main(args=None):
    rclpy.init(args=args)
    executor = ActionExecutor()

    try:
        rclpy.spin(executor)
    except KeyboardInterrupt:
        pass
    finally:
        executor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
