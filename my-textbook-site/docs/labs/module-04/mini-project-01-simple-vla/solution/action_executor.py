#!/usr/bin/env python3
"""
Action Executor Module for Simple VLA System
Handles robot action execution via ROS 2.

SOLUTION: Complete implementation with all functionality.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class ActionExecutor(Node):
    """
    Action executor that sends velocity commands to the robot.
    Publishes Twist messages to /cmd_vel topic.
    """

    def __init__(self):
        super().__init__('action_executor')
        self.get_logger().info('Initializing Action Executor...')

        # Create a publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Movement parameters
        self.linear_speed = 0.5  # m/s
        self.angular_speed = 0.5  # rad/s

        # Current state
        self.is_moving = False

        self.get_logger().info('Action Executor initialized successfully')

    def move_forward(self, duration=1.0):
        """
        Move the robot forward.

        Args:
            duration: How long to move (seconds)

        Returns:
            bool: True if action executed successfully
        """
        try:
            twist = Twist()
            twist.linear.x = self.linear_speed
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0

            self.cmd_vel_pub.publish(twist)
            self.is_moving = True
            self.get_logger().info(f'Moving forward at {self.linear_speed} m/s')

            return True

        except Exception as e:
            self.get_logger().error(f'Error moving forward: {e}')
            return False

    def move_backward(self, duration=1.0):
        """
        Move the robot backward.

        Args:
            duration: How long to move (seconds)

        Returns:
            bool: True if action executed successfully
        """
        try:
            twist = Twist()
            twist.linear.x = -self.linear_speed
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0

            self.cmd_vel_pub.publish(twist)
            self.is_moving = True
            self.get_logger().info(f'Moving backward at {self.linear_speed} m/s')

            return True

        except Exception as e:
            self.get_logger().error(f'Error moving backward: {e}')
            return False

    def turn_left(self, duration=1.0):
        """
        Turn the robot left.

        Args:
            duration: How long to turn (seconds)

        Returns:
            bool: True if action executed successfully
        """
        try:
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = self.angular_speed  # Positive for left turn

            self.cmd_vel_pub.publish(twist)
            self.is_moving = True
            self.get_logger().info(f'Turning left at {self.angular_speed} rad/s')

            return True

        except Exception as e:
            self.get_logger().error(f'Error turning left: {e}')
            return False

    def turn_right(self, duration=1.0):
        """
        Turn the robot right.

        Args:
            duration: How long to turn (seconds)

        Returns:
            bool: True if action executed successfully
        """
        try:
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = -self.angular_speed  # Negative for right turn

            self.cmd_vel_pub.publish(twist)
            self.is_moving = True
            self.get_logger().info(f'Turning right at {self.angular_speed} rad/s')

            return True

        except Exception as e:
            self.get_logger().error(f'Error turning right: {e}')
            return False

    def stop(self):
        """
        Stop all robot movement.

        Returns:
            bool: True if action executed successfully
        """
        try:
            twist = Twist()
            # All values default to 0.0
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0

            self.cmd_vel_pub.publish(twist)
            self.is_moving = False
            self.get_logger().info('Robot stopped')

            return True

        except Exception as e:
            self.get_logger().error(f'Error stopping: {e}')
            return False

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

        if action_name not in actions:
            self.get_logger().warning(f'Unknown action: {action_name}')
            return False

        action_func = actions[action_name]
        return action_func(**kwargs)


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
