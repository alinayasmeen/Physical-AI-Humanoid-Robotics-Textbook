#!/usr/bin/env python3
"""
Mini-Project 2: Keyboard Teleop Node - Solution
"""

import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger


KEY_BINDINGS = """
╔══════════════════════════════════════╗
║      ARM KEYBOARD TELEOP             ║
╠══════════════════════════════════════╣
║  w : Shoulder up   (+0.1 rad)        ║
║  s : Shoulder down (-0.1 rad)        ║
║  a : Elbow up      (+0.1 rad)        ║
║  d : Elbow down    (-0.1 rad)        ║
║  h : Home position (all zeros)       ║
║  e : Emergency stop                  ║
║  r : Reset emergency stop            ║
║  q : Quit                            ║
╚══════════════════════════════════════╝
"""


class KeyboardTeleop(Node):
    """Keyboard control for arm joints."""

    def __init__(self):
        super().__init__('keyboard_teleop')

        # Current target positions [shoulder, elbow]
        self.positions = [0.0, 0.0]

        # Step size for each key press (radians)
        self.step = 0.1

        # Publisher for joint commands
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/joint_commands',
            10
        )

        # Client for emergency stop service
        self.estop_client = self.create_client(Trigger, '/emergency_stop')

        # Client for reset service
        self.reset_client = self.create_client(Trigger, '/reset_estop')

        # Store original terminal settings
        self.settings = termios.tcgetattr(sys.stdin)

        self.get_logger().info('Keyboard teleop started')

    def get_key(self):
        """Read single key from terminal (non-blocking)."""
        try:
            tty.setraw(sys.stdin.fileno())
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist:
                key = sys.stdin.read(1)
            else:
                key = ''
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def process_key(self, key):
        """Process key press and update positions."""
        changed = False

        if key == 'w':
            self.positions[0] += self.step
            changed = True
            print(f'\rShoulder: {self.positions[0]:+.2f} rad    ', end='')

        elif key == 's':
            self.positions[0] -= self.step
            changed = True
            print(f'\rShoulder: {self.positions[0]:+.2f} rad    ', end='')

        elif key == 'a':
            self.positions[1] += self.step
            changed = True
            print(f'\rElbow: {self.positions[1]:+.2f} rad    ', end='')

        elif key == 'd':
            self.positions[1] -= self.step
            changed = True
            print(f'\rElbow: {self.positions[1]:+.2f} rad    ', end='')

        elif key == 'h':
            self.positions = [0.0, 0.0]
            changed = True
            print('\rHome position                ', end='')

        elif key == 'e':
            self.call_emergency_stop()
            print('\r*** EMERGENCY STOP ***       ', end='')

        elif key == 'r':
            self.call_reset()
            print('\rEmergency stop reset         ', end='')

        elif key == 'q':
            print('\nQuitting...')
            return False

        if changed:
            self.publish_command()

        return True

    def publish_command(self):
        """Publish current positions as command."""
        msg = Float64MultiArray()
        msg.data = list(self.positions)
        self.publisher.publish(msg)

    def call_emergency_stop(self):
        """Call emergency stop service."""
        if not self.estop_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Emergency stop service not available')
            return

        request = Trigger.Request()
        future = self.estop_client.call_async(request)

    def call_reset(self):
        """Call reset service."""
        if not self.reset_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Reset service not available')
            return

        request = Trigger.Request()
        future = self.reset_client.call_async(request)

    def run(self):
        """Main teleop loop."""
        print(KEY_BINDINGS)
        print('Ready for input...\n')

        try:
            while rclpy.ok():
                key = self.get_key()
                if key:
                    if not self.process_key(key):
                        break
                rclpy.spin_once(self, timeout_sec=0.01)
        finally:
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()

    try:
        node.run()
    except KeyboardInterrupt:
        print('\nInterrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
