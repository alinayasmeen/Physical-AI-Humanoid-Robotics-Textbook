#!/usr/bin/env python3
"""
لیب 5: ادراک سے کنٹرول کا انضمام - ابتدائی کوڈ
ماڈیول 3: ادراک اور سینسرز

ٹارگٹ فالو کرنے والے روبوٹ کو بنانے کے لیے TODOs مکمل کریں۔
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
        # TODO 2: ٹریکنگ لااجک نافذ کریں
        # 1. ٹارگٹ_کلاس کے ذریعے ڈیٹیکشنز کو فلٹر کریں
        # 2. اگر کوئی last_bbox نہیں ہے، سب سے زیادہ یقین چنیں
        # 3. بصورت دیگر، last_bbox کے سب سے قریب ترین میچ تلاش کریں
        # 4. frames_lost کاؤنٹر اپ ڈیٹ کریں

        return None  # ٹریکڈ ڈیٹیکشن کے ساتھ تبدیل کریں

    def bbox_distance(self, bbox1, bbox2):
        """بکسز کے درمیان سینٹر سے سینٹر کا فاصلہ حساب لگائیں۔"
        c1x = bbox1.center.position.x
        c1y = bbox1.center.position.y
        c2x = bbox2.center.position.x
        c2y = bbox2.center.position.y
        return math.sqrt((c1x - c2x)**2 + (c1y - c2y)**2)


class PerceptionController(Node):
    """وژول سروو کنٹرولر جو ڈیٹیکٹڈ ٹارگٹس کو فالو کرتا ہے۔"

    def __init__(self):
        super().__init__('perception_controller')

        # کنفیگریشن
        self.target_class = 'person'  # فالو کرنے کے لیے کلاس
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

        # TODO 1: سبسکرائیبرز اور پبلشرز تخلیق کریں
        # /detections (Detection2DArray) کو سبسکرائیب کریں
        # /cmd_vel (Twist) کو پبلش کریں

        # کنٹرول لوپ کے لیے ٹائمر تخلیق کریں
        self.create_timer(0.1, self.control_loop)

        self.get_logger().info('ادراک کنٹرولر شروع ہوا')
        self.get_logger().info(f'ٹارگٹ کلاس کو فالو کر رہا ہے: {self.target_class}')

    def detection_callback(self, msg):
        """ڈیٹیکشنز کو پروسیس کریں اور ٹریکنگ اپ ڈیٹ کریں۔"
        # نئی ڈیٹیکشنز کے ساتھ ٹریکر اپ ڈیٹ کریں
        self.current_target = self.tracker.update(
            msg.detections, self.target_class)

        # اسٹیٹ مشین اپ ڈیٹ کریں
        self.update_state()

    def update_state(self):
        """ٹریکنگ کی حیثیت کے مطابق اسٹیٹ اپ ڈیٹ کریں۔"
        # TODO 4: اسٹیٹ مشین نافذ کریں
        # - SEARCHING: tracker.frames_lost > 30
        # - STOPPED: ٹارگٹ بہت قریب (area > 40000)
        # - APPROACHING: ٹارگٹ قریب (area > 20000)
        # - TRACKING: ٹارگٹ قابل دید ہے لیکن دور
        pass

    def control_loop(self):
        """ویلوسٹی کمانڈز جنریٹ اور پبلش کریں۔"
        # TODO 3: اسٹیٹ اور ٹارگٹ کے مطابق ویلوسٹی کا حساب لگائیں
        cmd = Twist()

        if self.state == ControlState.SEARCHING:
            # ٹارگٹ تلاش کرنے کے لیے گھومیں
            cmd.angular.z = 0.3
        elif self.state == ControlState.STOPPED:
            # جب کافی قریب ہو تو رکیں
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        elif self.current_target is not None:
            # ٹارگٹ کی طرف وژول سروو
            cmd = self.compute_velocity(self.current_target.bbox)

        # ویلوسٹی کمانڈ پبلش کریں
        # self.cmd_pub.publish(cmd)  # پبلشر تخلیق کرنے کے بعد انکمنٹ کریں

        # اسٹیٹ لاگ کریں
        if hasattr(self, '_last_state') and self._last_state != self.state:
            self.get_logger().info(f'اسٹیٹ: {self.state}')
        self._last_state = self.state

    def compute_velocity(self, bbox):
        """ٹارگٹ کو فالو کرنے کے لیے ویلوسٹی کا حساب لگائیں۔"
        cmd = Twist()

        if bbox is None:
            return cmd

        # ٹارگٹ سینٹر اور سائز حاصل کریں
        cx = bbox.center.position.x
        area = bbox.size_x * bbox.size_y

        # اینگولر: ٹارگٹ کی طرف گھومیں
        x_error = cx - self.image_width / 2
        cmd.angular.z = -self.angular_gain * x_error

        # لینیئر: فاصلہ برقرار رکھیں (دور ہونے پر قریب آئیں، قریب ہونے پر پیچھے)
        area_error = self.desired_area - area
        cmd.linear.x = self.linear_gain * area_error

        # ویلوسٹیز کو محدود کریں
        cmd.linear.x = max(-0.3, min(0.5, cmd.linear.x))
        cmd.angular.z = max(-0.5, min(0.5, cmd.angular.z))

        return cmd


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # شٹ ڈاؤن پر روبوٹ کو روکیں
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()