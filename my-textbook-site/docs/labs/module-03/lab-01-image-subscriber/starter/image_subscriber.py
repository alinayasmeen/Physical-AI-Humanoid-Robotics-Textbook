#!/usr/bin/env python3
"""
Lab 1: Image Subscriber - Starter Code
Module 3: Perception & Sensors

Complete the TODOs to build a working image processing node.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class ImageSubscriber(Node):
    """ROS 2 node that subscribes to images and processes them."""

    def __init__(self):
        super().__init__('image_subscriber')

        # Initialize cv_bridge for ROS <-> OpenCV conversion
        self.bridge = CvBridge()

        # Create subscriber for raw camera images
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)

        # Create publisher for processed images
        self.processed_pub = self.create_publisher(
            Image,
            '/camera/image_processed',
            10)

        self.get_logger().info('Image subscriber node started')
        self.frame_count = 0

    def image_callback(self, msg):
        """Process incoming camera images."""
        self.frame_count += 1

        # TODO 1: Convert ROS Image message to OpenCV format
        # Hint: Use self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        cv_image = None  # Replace this line

        if cv_image is None:
            self.get_logger().warn('TODO 1: Implement image conversion')
            return

        # TODO 2: Process the image
        # Step 2a: Convert to grayscale using cv2.cvtColor()
        # Step 2b: Apply Gaussian blur using cv2.GaussianBlur()
        # Step 2c: Apply Canny edge detection using cv2.Canny()
        gray = None
        blurred = None
        edges = None  # Replace these lines

        if edges is None:
            self.get_logger().warn('TODO 2: Implement image processing')
            return

        # TODO 3: Convert processed image back to ROS format and publish
        # Step 3a: Convert edges (grayscale) to BGR using cv2.cvtColor()
        # Step 3b: Convert to ROS Image using self.bridge.cv2_to_imgmsg()
        # Step 3c: Copy the header from original message
        # Step 3d: Publish using self.processed_pub.publish()

        # Your code here

        # Log progress every 30 frames
        if self.frame_count % 30 == 0:
            self.get_logger().info(
                f'Processed {self.frame_count} frames, '
                f'image shape: {cv_image.shape}'
            )


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)

    node = ImageSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
