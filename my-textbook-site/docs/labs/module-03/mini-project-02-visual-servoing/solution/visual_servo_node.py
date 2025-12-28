#!/usr/bin/env python3
"""
Mini-Project 2: Visual Servoing Robot - Solution
Complete implementation of visual servo control with state machine.
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
        self.desired_area = 25000
        self.approach_area = 35000
        self.hold_area = 40000
        self.min_area = 500

        # State machine
        self.state = ServoState.SEARCHING
        self.prev_state = None
        self.frames_lost = 0
        self.lost_threshold = 30
        self.search_timeout = 100

        # Velocity smoothing
        self.alpha = 0.3
        self.prev_linear = 0.0
        self.prev_angular = 0.0

        # Safety limits
        self.max_linear = 0.5
        self.max_angular = 1.0

        # SOLUTION 1: Create image subscriber
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw',
            self.image_callback, 10)

        # SOLUTION 2: Create velocity publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Control loop timer
        self.create_timer(0.1, self.control_loop)

        self.current_target = None

        self.get_logger().info('Visual servo node started')

    def image_callback(self, msg):
        """Process incoming camera images."""
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # SOLUTION 3: Detect target in image
        self.current_target = self.detect_target(cv_image)

    def detect_target(self, image):
        """Detect target object in image using color."""
        # SOLUTION 4: Color-based detection
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Detect red objects
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        # Find largest contour
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return None

        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)

        if area < self.min_area:
            return None

        M = cv2.moments(largest)
        if M['m00'] == 0:
            return None

        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])

        return {'center_x': cx, 'center_y': cy, 'area': area}

    def control_loop(self):
        """Main control loop with state machine."""
        cmd = Twist()

        # SOLUTION 5: State machine implementation
        target = self.current_target

        if self.state == ServoState.SEARCHING:
            cmd.angular.z = 0.3
            if target is not None:
                self.transition_to(ServoState.TRACKING)

        elif self.state == ServoState.TRACKING:
            if target is not None:
                self.frames_lost = 0
                # SOLUTION 6: Calculate control commands
                cmd.angular.z = self.compute_angular_velocity(target['center_x'])
                cmd.linear.x = self.compute_linear_velocity(target['area'])

                # Check for state transitions
                if target['area'] > self.approach_area:
                    self.transition_to(ServoState.APPROACHING)
            else:
                self.frames_lost += 1
                if self.frames_lost > self.lost_threshold:
                    self.transition_to(ServoState.LOST)

        elif self.state == ServoState.APPROACHING:
            if target is not None:
                self.frames_lost = 0
                cmd.angular.z = self.compute_angular_velocity(target['center_x'])
                cmd.linear.x = self.compute_linear_velocity(target['area']) * 0.5

                if target['area'] > self.hold_area:
                    self.transition_to(ServoState.HOLDING)
                elif target['area'] < self.approach_area:
                    self.transition_to(ServoState.TRACKING)
            else:
                self.frames_lost += 1
                if self.frames_lost > self.lost_threshold:
                    self.transition_to(ServoState.LOST)

        elif self.state == ServoState.HOLDING:
            if target is not None:
                self.frames_lost = 0
                cmd.angular.z = self.compute_angular_velocity(target['center_x']) * 0.5

                if target['area'] < self.approach_area:
                    self.transition_to(ServoState.TRACKING)
            else:
                self.frames_lost += 1
                if self.frames_lost > self.lost_threshold:
                    self.transition_to(ServoState.LOST)

        elif self.state == ServoState.LOST:
            self.frames_lost += 1
            cmd.angular.z = 0.2  # Slow search

            if target is not None:
                self.transition_to(ServoState.TRACKING)
            elif self.frames_lost > self.search_timeout:
                self.transition_to(ServoState.SEARCHING)

        # SOLUTION 7: Apply velocity smoothing
        cmd.linear.x = self.smooth_velocity(cmd.linear.x, self.prev_linear)
        cmd.angular.z = self.smooth_velocity(cmd.angular.z, self.prev_angular)
        self.prev_linear = cmd.linear.x
        self.prev_angular = cmd.angular.z

        # SOLUTION 8: Apply safety limits
        cmd = self.apply_limits(cmd)

        # Publish command
        self.cmd_pub.publish(cmd)

    def transition_to(self, new_state):
        """Transition to new state with logging."""
        if new_state != self.state:
            self.get_logger().info(f'State: {self.state.value} -> {new_state.value}')
            self.prev_state = self.state
            self.state = new_state
            self.frames_lost = 0

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
        stop_cmd = Twist()
        node.cmd_pub.publish(stop_cmd)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
