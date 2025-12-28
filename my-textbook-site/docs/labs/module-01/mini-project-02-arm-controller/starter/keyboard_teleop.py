#!/usr/bin/env python3
"""
Mini-Project 2: Keyboard Teleop Node - Starter Code

Reads keyboard input and sends joint commands.
TODO: Complete the implementation
"""

import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger


# Key bindings
KEY_BINDINGS = """
Keyboard Controls:
------------------
  w : Shoulder up (+0.1 rad)
  s : Shoulder down (-0.1 rad)
  a : Elbow up (+0.1 rad)
  d : Elbow down (-0.1 rad)
  h : Home position (all zeros)
  e : Emergency stop
  q : Quit
------------------
"""


class KeyboardTeleop(Node):
    """Keyboard control for arm joints."""

    def __init__(self):
        super().__init__('keyboard_teleop')

        # Current target positions
        self.positions = [0.0, 0.0]  # [shoulder, elbow]

        # Step size for each key press
        self.step = 0.1

        # TODO: Create publisher for joint commands
        # Topic: /joint_commands
        # Type: Float64MultiArray
        self.publisher = None  # Replace

        # TODO: Create client for emergency stop service
        # Service: /emergency_stop
        # Type: Trigger
        self.estop_client = None  # Replace

        self.get_logger().info('Keyboard teleop started')
        print(KEY_BINDINGS)

    def get_key(self):
        """Read single key from terminal (non-blocking)."""
        settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin.fileno())
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist:
                key = sys.stdin.read(1)
            else:
                key = ''
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def process_key(self, key):
        """Process key press and update positions."""
        if key == 'w':
            # TODO: Increase shoulder position
            self.positions[0] += self.step
            self.get_logger().info(f'Shoulder: {self.positions[0]:.2f}')

        elif key == 's':
            # TODO: Decrease shoulder position
            self.positions[0] -= self.step
            self.get_logger().info(f'Shoulder: {self.positions[0]:.2f}')

        elif key == 'a':
            # TODO: Increase elbow position
            self.positions[1] += self.step
            self.get_logger().info(f'Elbow: {self.positions[1]:.2f}')

        elif key == 'd':
            # TODO: Decrease elbow position
            self.positions[1] -= self.step
            self.get_logger().info(f'Elbow: {self.positions[1]:.2f}')

        elif key == 'h':
            # Home position
            self.positions = [0.0, 0.0]
            self.get_logger().info('Home position')

        elif key == 'e':
            # TODO: Call emergency stop service
            self.get_logger().warn('Emergency stop requested')
            # self.call_emergency_stop()

        elif key == 'q':
            return False  # Signal to quit

        # TODO: Publish updated command
        # self.publish_command()

        return True  # Continue running

    def publish_command(self):
        """Publish current positions as command."""
        msg = Float64MultiArray()
        msg.data = self.positions
        # TODO: Publish message
        # self.publisher.publish(msg)

    def call_emergency_stop(self):
        """Call emergency stop service."""
        if self.estop_client is None:
            return

        request = Trigger.Request()
        # TODO: Call service asynchronously
        # future = self.estop_client.call_async(request)

    def run(self):
        """Main teleop loop."""
        try:
            while rclpy.ok():
                key = self.get_key()
                if key:
                    if not self.process_key(key):
                        break
                rclpy.spin_once(self, timeout_sec=0.01)
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
        finally:
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN,
                            termios.tcgetattr(sys.stdin))


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
