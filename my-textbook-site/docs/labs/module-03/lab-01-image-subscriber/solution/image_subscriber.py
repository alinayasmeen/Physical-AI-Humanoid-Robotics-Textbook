#!/usr/bin/env python3
"""
Lab 1: Image Subscriber - Solution
Module 3: Perception & Sensors

Complete implementation of image processing node.
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

        # SOLUTION 1: Convert ROS Image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # SOLUTION 2: Process the image
        # Step 2a: Convert to grayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Step 2b: Apply Gaussian blur (5x5 kernel)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Step 2c: Apply Canny edge detection
        edges = cv2.Canny(blurred, 50, 150)

        # SOLUTION 3: Convert processed image back to ROS format and publish
        # Step 3a: Convert edges (grayscale) to BGR for publishing
        edges_bgr = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)

        # Step 3b: Convert to ROS Image message
        out_msg = self.bridge.cv2_to_imgmsg(edges_bgr, 'bgr8')

        # Step 3c: Copy the header from original message (preserves timestamp)
        out_msg.header = msg.header

        # Step 3d: Publish the processed image
        self.processed_pub.publish(out_msg)

        # Log progress every 30 frames
        if self.frame_count % 30 == 0:
            self.get_logger().info(
                f'Processed {self.frame_count} frames, '
                f'image shape: {cv_image.shape}, '
                f'edges detected'
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
