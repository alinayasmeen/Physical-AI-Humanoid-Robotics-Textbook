#!/usr/bin/env python3
"""
Mini-Project 2: Visual Servoing Robot - Starter Code
Main node implementing visual servo control with state machine.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
from enum import Enum


class ServoState(Enum):
    """Visual servoing state machine states."""
    SEARCHING = "searching"
    TRACKING = "tracking"
    APPROACHING = "approaching"
    HOLDING = "holding"
    LOST = "lost"


class VisualServoNode(Node):
    """Visual servoing robot controller."""

    def __init__(self):
        super().__init__('visual_servo_node')

        self.bridge = CvBridge()

        # Image parameters
        self.image_width = 640
        self.image_height = 480
        self.image_center_x = self.image_width / 2
        self.image_center_y = self.image_height / 2

        # Control gains
        self.K_angular = 0.002
        self.K_linear = 0.0001

        # Target parameters
        self.desired_area = 25000  # Target bounding box area
        self.min_area = 500       # Minimum detection area

        # State machine
        self.state = ServoState.SEARCHING
        self.frames_lost = 0
        self.lost_threshold = 30

        # Velocity smoothing
        self.alpha = 0.3
        self.prev_linear = 0.0
        self.prev_angular = 0.0

        # Safety limits
        self.max_linear = 0.5
        self.max_angular = 1.0

        # TODO 1: Create image subscriber
        # Subscribe to /camera/image_raw

        # TODO 2: Create velocity publisher
        # Publish to /cmd_vel

        # Control loop timer
        self.create_timer(0.1, self.control_loop)

        self.current_target = None

        self.get_logger().info('Visual servo node started')

    def image_callback(self, msg):
        """Process incoming camera images."""
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # TODO 3: Detect target in image
        # Returns dict with 'center_x', 'center_y', 'area' or None
        self.current_target = None  # Replace with detection

    def detect_target(self, image):
        """Detect target object in image."""
        # TODO 4: Implement target detection
        # Use color-based or shape-based detection
        # Return {'center_x': x, 'center_y': y, 'area': area} or None
        pass

    def control_loop(self):
        """Main control loop with state machine."""
        cmd = Twist()

        # TODO 5: Implement state machine
        # Handle each state: SEARCHING, TRACKING, APPROACHING, HOLDING, LOST

        if self.state == ServoState.SEARCHING:
            # Rotate to search for target
            cmd.angular.z = 0.3
            # Transition to TRACKING if target found

        elif self.state == ServoState.TRACKING:
            # Follow moving target
            if self.current_target is not None:
                # TODO 6: Calculate control commands
                pass
            else:
                self.frames_lost += 1
                # Transition to LOST if too many frames

        elif self.state == ServoState.APPROACHING:
            # Move toward stationary target
            pass

        elif self.state == ServoState.HOLDING:
            # Maintain position
            pass

        elif self.state == ServoState.LOST:
            # Target lost
            pass

        # TODO 7: Apply velocity smoothing
        # Use exponential moving average

        # TODO 8: Apply safety limits
        # Clamp velocities to max values

        # Publish command
        # self.cmd_pub.publish(cmd)  # Uncomment after creating publisher

    def compute_angular_velocity(self, target_x):
        """Compute angular velocity to center target."""
        error = target_x - self.image_center_x
        return -self.K_angular * error

    def compute_linear_velocity(self, target_area):
        """Compute linear velocity to maintain distance."""
        error = self.desired_area - target_area
        return self.K_linear * error

    def smooth_velocity(self, current, previous):
        """Apply exponential smoothing to velocity."""
        return self.alpha * current + (1 - self.alpha) * previous

    def apply_limits(self, cmd):
        """Apply safety velocity limits."""
        cmd.linear.x = max(-self.max_linear, min(self.max_linear, cmd.linear.x))
        cmd.angular.z = max(-self.max_angular, min(self.max_angular, cmd.angular.z))
        return cmd


def main(args=None):
    rclpy.init(args=args)
    node = VisualServoNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Stop robot on shutdown
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
