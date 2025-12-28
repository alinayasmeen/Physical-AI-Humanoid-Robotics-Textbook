# لیب 5: ادراک سے کنٹرول کا انضمام

**ماڈیول**: 3 - ادراک اور سینسرز
**دورانیہ**: 150 منٹ
**مشکل**: اعلی درجہ

## مقصد

اوبجیکٹس کو ڈیٹیکٹ، ٹریک، اور روبوٹ کنٹرول کمانڈز جنریٹ کرنے والا مکمل ادراک سے ایکشن پائپ لائن بنائیں۔ یہ کیپ سٹون لیب ماڈیول 3 کے تمام تصورات کو کام کرتے ہوئے ٹارگٹ فالو سسٹم میں ضم کرتا ہے۔

## ضروریات

- لیبز 1-4 مکمل کر لی ہیں
- وژول سروو کے تصورات کی سمجھ
- روبوٹ ویلوسٹی کمانڈز کے لیے ROS 2 geometry_msgs

## سیکھنے کے نتائج

اس لیب کو مکمل کرنے کے بعد، آپ:
1. ایک ہی سسٹم میں ڈیٹیکشن، ٹریکنگ، اور کنٹرول کو ضم کریں گے
2. سادہ وژول سروو نافذ کریں گے
3. ایج کیسز کو سنبھالیں گے (کھویا ہوا ٹارگٹ، متعدد ٹارگٹس)
4. بند لوپ ادراک-کنٹرول برتاؤ کو ٹیسٹ کریں گے

---

## سیٹ اپ

### 1. ضروریات کی تصدیق کریں

```bash
# تمام ضروری پیکجز چیک کریں
python3 -c "import cv2, rclpy, numpy; print('کور پیکجز ٹھیک ہیں')"
ros2 pkg list | grep -E "(vision_msgs|geometry_msgs)"
```

### 2. لیب ڈائریکٹری میں جائیں

```bash
cd ~/ros2_ws/src/labs/module-03/lab-05-perception-integration
```

### 3. روبوٹ سیمولیشن لانچ کریں

```bash
# ٹرمنل 1: کیمرہ کے ساتھ روبوٹ لانچ کریں
ros2 launch gazebo_ros gazebo.launch.py

# ٹرمنل 2: ٹارگٹ اوبجیکٹس اسپون کریں
ros2 run gazebo_ros spawn_entity.py -entity target -file target.sdf
```

---

## لیب اسٹیپس

### اسٹیپ 1: ڈیٹیکشن انضمام (30 منٹ)

TODO 1 مکمل کریں - اوبجیکٹ ڈیٹیکٹر کو ضم کریں:

```python
class PerceptionController(Node):
    def __init__(self):
        super().__init__('perception_controller')

        # ڈیٹیکشن سبسکرائیب
        self.det_sub = self.create_subscription(
            Detection2DArray, '/detections',
            self.detection_callback, 10)

        # ویلوسٹی پبلشر
        self.cmd_pub = self.create_publisher(
            Twist, '/cmd_vel', 10)

        # فالو کرنے کے لیے ٹارگٹ کلاس
        self.target_class = 'person'
        self.current_target = None
```

### اسٹیپ 2: سادہ ٹریکر نافذ کریں (30 منٹ)

TODO 2 مکمل کریں - فریم کے ذریعے ٹارگٹ ٹریک کریں:

```python
class SimpleTracker:
    def __init__(self, max_distance=100):
        self.track_id = None
        self.last_bbox = None
        self.max_distance = max_distance
        self.frames_lost = 0

    def update(self, detections, target_class):
        """ٹارگٹ اوبجیکٹ تلاش کریں اور ٹریک کریں۔"""
        candidates = [d for d in detections
                     if d.class_id == target_class]

        if not candidates:
            self.frames_lost += 1
            return None

        if self.last_bbox is None:
            # سب سے زیادہ یقین کے ساتھ نیا ٹریک شروع کریں
            best = max(candidates, key=lambda d: d.confidence)
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
```

### اسٹیپ 3: وژول سروو کنٹرولر (40 منٹ)

TODO 3 مکمل کریں - ویلوسٹی کمانڈز جنریٹ کریں:

```python
def compute_velocity(self, target_bbox, image_width=640, image_height=480):
    """ٹارگٹ کو فالو کرنے کے لیے ویلوسٹی کمانڈز کا حساب لگائیں۔"""
    cmd = Twist()

    if target_bbox is None:
        # ٹارگٹ کھونے پر تلاش کا برتاؤ
        cmd.angular.z = 0.3  # تلاش کرنے کے لیے گھومیں
        return cmd

    # ٹارگٹ سینٹر حاصل کریں
    cx = target_bbox.center.x
    cy = target_bbox.center.y
    area = target_bbox.size_x * target_bbox.size_y

    # اینگولر ویلوسٹی: ٹارگٹ کی طرف گھومیں
    # مثبت خامی = ٹارگٹ دائیں = دائیں گھومیں (منفی z)
    x_error = cx - image_width / 2
    cmd.angular.z = -0.003 * x_error

    # لینیئر ویلوسٹی: بی بکس ایریا کے ذریعے فاصلہ برقرار رکھیں
    desired_area = 25000  # مطلوبہ فاصلے پر ہونے پر ٹارگٹ ایریا
    area_error = desired_area - area
    cmd.linear.x = 0.0001 * area_error

    # ویلوسٹیز کو محدود کریں
    cmd.linear.x = max(-0.3, min(0.5, cmd.linear.x))
    cmd.angular.z = max(-0.5, min(0.5, cmd.angular.z))

    return cmd
```

### اسٹیپ 4: اسٹیٹ مشین (30 منٹ)

TODO 4 مکمل کریں - مختلف اسٹیٹس کو سنبھالیں:

```python
class ControlState:
    SEARCHING = 'searching'
    TRACKING = 'tracking'
    APPROACHING = 'approaching'
    STOPPED = 'stopped'

def update_state(self):
    """ٹریکنگ کی حیثیت کے مطابق اسٹیٹ اپ ڈیٹ کریں۔"
    if self.tracker.frames_lost > 30:
        self.state = ControlState.SEARCHING
    elif self.current_target is not None:
        area = (self.current_target.bbox.size_x *
                self.current_target.bbox.size_y)
        if area > 40000:  # بہت قریب
            self.state = ControlState.STOPPED
        elif area > 20000:  # کافی قریب
            self.state = ControlState.APPROACHING
        else:
            self.state = ControlState.TRACKING
```

### اسٹیپ 5: ٹیسٹ اور ٹیون (20 منٹ)

```bash
# اپنی ایجاد چلائیں
python3 starter/perception_controller.py

# دوسری ٹرمنل میں، ٹارگٹ کو منتقل کریں
ros2 topic pub /target/cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}}"

# روبوٹ فالو کرنے کا برتاؤ دیکھیں
```

---

## فراہم کردہ چیزیں

1. **کام کرتا کوڈ**: مکمل `starter/perception_controller.py`
2. **ڈیمو ویڈیو**: روبوٹ کو ٹارگٹ کو فالو کرتے ہوئے ریکارڈ کریں
3. **پیرامیٹر ٹیوننگ**: بہترین گین ویلیوز دستاویز کریں
4. **ریفلیکشن جوابات**: ذیل کے سوالات مکمل کریں

---

## ریفلیکشن سوالات

1. **جب ٹارگٹ ویو سے باہر چلا جاتا ہے تو کیا ہوتا ہے؟**
   - تلاش کا برتاؤ دیکھیں اور تفصیل دیں

2. **کنٹرول گینز فالو کرنے کے برتاؤ کو کیسے متاثر کرتے ہیں؟**
   - 2x اور 0.5x گینز کے ساتھ ٹیسٹ کریں، فرق کی تفصیل دیں

3. **فالو کرتے وقت اوسیلیشن کی وجہ کیا ہے؟**
   - غور کریں: کنٹرول فریکوینسی، سینسر لیٹنسی، گین ٹیوننگ

4. **آپ اسے متعدد ٹارگٹس کے لیے کیسے بڑھائیں گے؟**
   - غور کریں: ترجیح، سوئچنگ لااجک، ٹارگٹ سلیکشن

5. **ایک حقیقی سسٹم میں کون سی سیفٹی خصوصیات ہونی چاہئیں؟**
   - غور کریں: کولیژن ایوائڈنس، ویلوسٹی حدود، ایمرجنسی سٹاپ

---

## تصدیق

```bash
python3 validate.py
```

متوقع آؤٹ پٹ:
```
[PASS] perception_controller.py موجود ہے
[PASS] درست Python سینٹیکس
[PASS] ڈیٹیکشن سبسکرپشن موجود ہے
[PASS] ویلوسٹی پبلشر موجود ہے
[PASS] وژول سروو لاجک نافذ کیا گیا
[PASS] اسٹیٹ مشین نافذ کی گئی
لیب 5 مکمل! ماڈیول 3 ختم!
```

---

## توسیع چیلنجز

1. **PID کنٹرول**: سMOOTH موشن کے لیے P کنٹرولر کو PID کے ساتھ تبدیل کریں
2. **آبست ایوائڈنس**: فالو کرتے وقت رکاوٹوں سے بچنے کے لیے LiDAR استعمال کریں
3. **متعدد ٹارگٹ**: متعدد ٹارگٹس کو ٹریک کریں اور ان کے درمیان سوئچ کریں
4. **جیسچر کنٹرول**: ڈیٹیکٹڈ ہینڈ جیسچر کے جواب میں

---

## مبارکباد!

آپ نے ماڈیول 3: ادراک اور سینسرز مکمل کر لیا ہے۔ اب آپ کر سکتے ہیں:
- OpenCV کے ساتھ کیمرہ ایمیجز کو پروسیس
- LiDAR پوائنٹ کلاؤڈ کے ساتھ کام
- متعدد سینسر سٹریمز فیوژن
- اوبجیکٹس کو ڈیٹیکٹ اور ٹریک
- ادراک کو روبوٹ کنٹرول سے جوڑیں

**اگلا**: ماڈیول 4 - موشن پلاننگ اور کنٹرول