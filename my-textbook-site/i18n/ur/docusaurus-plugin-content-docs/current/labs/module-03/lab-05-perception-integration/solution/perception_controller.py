#!/usr/bin/env python3
"""
لیب 5: ادراک سے کنٹرول کا انضمام - حل
ماڈیول 3: ادراک اور سینسرز

ٹارگٹ فالو کرنے والے روبوٹ کا مکمل نفاذ۔
"""

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Twist
import math


class ControlState:
    """اسٹیٹ مشین اسٹیٹس۔"
    SEARCHING = 'searching'
    TRACKING = 'tracking'
    APPROACHING = 'approaching'
    STOPPED = 'stopped'


class SimpleTracker:
    """سادہ سنگل ٹارگٹ ٹریکر۔"

    def __init__(self, max_distance=100):
        self.last_bbox = None
        self.max_distance = max_distance
        self.frames_lost = 0

    def update(self, detections, target_class):
        """ٹارگٹ اوبجیکٹ تلاش کریں اور ٹریک کریں۔"
        # حل 2: ٹارگٹ کلاس کے ذریعے فلٹر کریں
        candidates = []
        for det in detections:
            if len(det.results) > 0:
                if det.results[0].hypothesis.class_id == target_class:
                    candidates.append(det)

        if not candidates:
            self.frames_lost += 1
            return None

        if self.last_bbox is None:
            # سب سے زیادہ یقین چنیں
            best = max(candidates,
                      key=lambda d: d.results[0].hypothesis.score)
            self.last_bbox = best.bbox
            self.frames_lost = 0
            return best

        # آخری پوزیشن سے قریب ترین تلاش کریں
        best_match = None
        best_dist = self.max_distance

        for det in candidates:
            dist = self.bbox_distance(self.last_bbox, det.bbox)
            if dist < best_dist:
                best_dist = dist
                best_match = det

        if best_match:
            self.last_bbox = best_match.bbox
            self.frames_lost = 0
            return best_match

        self.frames_lost += 1
        return None

    def bbox_distance(self, bbox1, bbox2):
        """سینٹر سے سینٹر کا فاصلہ حساب لگائیں۔"
        c1x = bbox1.center.position.x
        c1y = bbox1.center.position.y
        c2x = bbox2.center.position.x
        c2y = bbox2.center.position.y
        return math.sqrt((c1x - c2x)**2 + (c1y - c2y)**2)


class PerceptionController(Node):
    """وژول سروو کنٹرولر۔"

    def __init__(self):
        super().__init__('perception_controller')

        # کنفیگریشن
        self.target_class = 'person'
        self.image_width = 640
        self.image_height = 480

        # کنٹرول گینز
        self.angular_gain = 0.003
        self.linear_gain = 0.0001
        self.desired_area = 25000

        # اسٹیٹ
        self.state = ControlState.SEARCHING
        self.current_target = None
        self.tracker = SimpleTracker()

        # حل 1: سبسکرائیبرز اور پبلشرز تخلیق کریں
        self.det_sub = self.create_subscription(
            Detection2DArray, '/detections',
            self.detection_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # کنٹرول لوپ ٹائمر
        self.create_timer(0.1, self.control_loop)

        self.get_logger().info('ادراک کنٹرولر شروع ہوا')
        self.get_logger().info(f' فالو کر رہا ہے: {self.target_class}')

    def detection_callback(self, msg):
        """ڈیٹیکشنز کو پروسیس کریں اور ٹریکنگ اپ ڈیٹ کریں۔"
        self.current_target = self.tracker.update(
            msg.detections, self.target_class)
        self.update_state()

    def update_state(self):
        """اسٹیٹ مشین اپ ڈیٹ کریں۔"
        # حل 4: اسٹیٹ مشین لااجک
        if self.tracker.frames_lost > 30:
            self.state = ControlState.SEARCHING
            self.tracker.last_bbox = None  # ٹریکر ری سیٹ کریں
        elif self.current_target is not None:
            bbox = self.current_target.bbox
            area = bbox.size_x * bbox.size_y

            if area > 40000:
                self.state = ControlState.STOPPED
            elif area > 20000:
                self.state = ControlState.APPROACHING
            else:
                self.state = ControlState.TRACKING

    def control_loop(self):
        """ویلوسٹی کمانڈز جنریٹ کریں۔"
        # حل 3: اسٹیٹ بیسڈ کنٹرول
        cmd = Twist()

        if self.state == ControlState.SEARCHING:
            cmd.angular.z = 0.3  # تلاش کرنے کے لیے گھومیں
        elif self.state == ControlState.STOPPED:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        elif self.current_target is not None:
            cmd = self.compute_velocity(self.current_target.bbox)

        self.cmd_pub.publish(cmd)

        # اسٹیٹ تبدیلیاں لاگ کریں
        if hasattr(self, '_last_state') and self._last_state != self.state:
            self.get_logger().info(f'اسٹیٹ: {self.state}')
        self._last_state = self.state

    def compute_velocity(self, bbox):
        """وژول سروو کا حساب۔"
        cmd = Twist()

        if bbox is None:
            return cmd

        cx = bbox.center.position.x
        area = bbox.size_x * bbox.size_y

        # اینگولر: ٹارگٹ کی طرف گھومیں
        x_error = cx - self.image_width / 2
        cmd.angular.z = -self.angular_gain * x_error

        # لینیئر: فاصلہ برقرار رکھیں
        area_error = self.desired_area - area
        cmd.linear.x = self.linear_gain * area_error

        # محدود کریں
        cmd.linear.x = max(-0.3, min(0.5, cmd.linear.x))
        cmd.angular.z = max(-0.5, min(0.5, cmd.angular.z))

        return cmd


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # روبوٹ کو روکیں
        stop_cmd = Twist()
        node.cmd_pub.publish(stop_cmd)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()