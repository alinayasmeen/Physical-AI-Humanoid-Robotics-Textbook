# لیب 3: ملٹی سینسر فیوژن

**موڈیول**: 3 - پرچیپشن اور سینسر
**مدت**: 120 منٹ
**پریشانی**: متوسط

## اہداف

کیمرہ اور لیڈار سینسرز سے ڈیٹا کو ہم وقت کر کے اور ضم کر کے سینسر فیوژن نافذ کریں. آپ ٹائم ہم وقت سازی کے لیے ROS 2 میسج فلٹرز کا استعمال کریں گے اور 3D پوائنٹس کو 2D امیجز پر پروجیکٹ کرنا سیکھیں گے.

## ضروریات

- لیب 1 (امیج سبسکرائبر) مکمل
- لیب 2 (لیڈار پروسیسنگ) مکمل
- کوآرڈینیٹ فریم اور TF2 کی سمجھ
- ROS 2 ہمبل `message_filters` پیکج کے ساتھ

## سیکھنے کے نتائج

اس لیب کو مکمل کرنے کے بعد، آپ:

1. وقتی ہم وقت سازی کے لیے message_filters استعمال کریں گے
2. کوآرڈینیٹ فریم ٹرانسفارمیشنز کو سمجھیں گے
3. لیڈار پوائنٹس کو کیمرہ امیجز پر پروجیکٹ کریں گے
4. بنیادی سینسر فیوژن پائپ لائن نافذ کریں گے

---

## سیٹ اپ

### 1. انسٹالیشن کی تصدیق کریں

```bash
# message_filters چیک کریں
python3 -c "from message_filters import Subscriber, ApproximateTimeSynchronizer; print('OK')"

# TF2 چیک کریں
ros2 pkg list | grep tf2
```

### 2. لیب ڈائریکٹری میں جائیں

```bash
cd ~/ros2_ws/src
cp -r /path/to/labs/module-03/lab-03-sensor-fusion .
cd lab-03-sensor-fusion
```

### 3. ملٹی سینسر سیمولیشن لانچ کریں

```bash
# ٹرمنل 1: کیمرہ + لیڈار روبوٹ کے ساتھ گیزبو لانچ کریں
ros2 launch gazebo_ros gazebo.launch.py

# ٹرمنل 2: دونوں ٹاپکس کی تصدیق کریں
ros2 topic list | grep -E "(camera|lidar)"
```

---

## لیب اسٹیپس

### اسٹیپ 1: ہم وقت سبسکرائبرز سیٹ اپ کریں (25 منٹ)

TODO 1 مکمل کریں - ہم وقت سبسکرائبرز بنائیں:

```python
from message_filters import Subscriber, ApproximateTimeSynchronizer

# سبسکرائبرز بنائیں
self.image_sub = Subscriber(self, Image, '/camera/image_raw')
self.lidar_sub = Subscriber(self, PointCloud2, '/lidar/points')

# 0.1s ٹولرنس کے ساتھ ہم وقت ساز بنائیں
self.sync = ApproximateTimeSynchronizer(
    [self.image_sub, self.lidar_sub],
    queue_size=10,
    slop=0.1)
self.sync.registerCallback(self.sync_callback)
```

### اسٹیپ 2: ہم وقت ڈیٹا کو عمل کریں (25 منٹ)

TODO 2 مکمل کریں - ہم وقت کال بیک کو ہینڈل کریں:

```python
def sync_callback(self, image_msg, lidar_msg):
    # امیج تبدیل کریں
    cv_image = self.bridge.imgmsg_to_cv2(image_msg, 'bgr8')

    # پوائنٹ کلاؤڈ تبدیل کریں
    points = self.pointcloud_to_numpy(lidar_msg)

    # ہم وقت سازی کو لاگ کریں
    self.get_logger().info(
        f'Synced: {len(points)} points with {cv_image.shape} image'
    )
```

### اسٹیپ 3: امیج پر پوائنٹس پروجیکٹ کریں (30 منٹ)

TODO 3 مکمل کریں - 3D لیڈار پوائنٹس کو 2D کیمرہ امیج پر پروجیکٹ کریں:

```python
def project_points_to_image(self, points_3d, camera_matrix):
    """3D پوائنٹس کو 2D امیج کوآرڈینیٹس پر پروجیکٹ کریں."""
    # کیمرہ کے سامنے کے پوائنٹس فلٹر کریں (z > 0)
    mask = points_3d[:, 2] > 0
    valid_points = points_3d[mask]

    # پروجیکٹ: [u, v, w] = K @ [x, y, z]
    projected = camera_matrix @ valid_points.T
    projected = projected.T

    # نارملائز: u = u/w, v = v/w
    u = projected[:, 0] / projected[:, 2]
    v = projected[:, 1] / projected[:, 2]

    return u, v, valid_points[:, 2]  # x, y, گہرائی
```

### اسٹیپ 4: لیڈار کو امیج پر اوور لے کریں (25 منٹ)

TODO 4 مکمل کریں - فیوژن کو وژولائز کریں:

```python
def overlay_points_on_image(self, image, u, v, depths):
    """گہرائی کے رنگ کے ساتھ کیمرہ امیج پر لیڈار پوائنٹس ڈرائیں."""
    # رنگ کے لیے گہرائی نارملائز کریں
    max_depth = 10.0
    for i in range(len(u)):
        x, y = int(u[i]), int(v[i])
        if 0 <= x < image.shape[1] and 0 <= y < image.shape[0]:
            # گہرائی کے ذریعے رنگ (red=قریب، blue=دور)
            intensity = int(255 * (1 - min(depths[i], max_depth) / max_depth))
            color = (intensity, 0, 255 - intensity)
            cv2.circle(image, (x, y), 2, color, -1)
    return image
```

### اسٹیپ 5: ٹیسٹ اور تصدیق کریں (15 منٹ)

```bash
# اپنے نفاذ چلائیں
python3 starter/sensor_fusion.py

# حل کے ساتھ موازنہ کریں
python3 solution/sensor_fusion.py

# تصدیق کریں
python3 validate.py
```

---

## ڈلیوریبلز

1. **کام کرتا ہوا کوڈ**: مکمل `starter/sensor_fusion.py`
2. **فیوژن وژولائزیشن**: لیڈار پوائنٹس کو امیج پر اوور لے کرتے ہوئے دکھانے والا اسکرین شاٹ
3. **ریفلیکشن جوابات**: ذیل کے سوالات مکمل کریں

---

## ریفلیکشن سوالات

1. **سینسر فیوژن کے لیے ٹائم ہم وقت سازی کیوں اہم ہے؟**
   - غور کریں: روبوٹ موشن، ٹیمپورل مطابقت

2. **اگر کیمرہ اور لیڈار کے مختلف فریم ریٹس ہوں تو کیا ہوتا ہے؟**
   - سوچیں: کیو سائز، سلپ ٹولرنس

3. **پروجیکشن سے پہلے ہمیں z کم یا برابر 0 والے پوائنٹس کو کیوں فلٹر کرنا چاہیے؟**
   - غور کریں: کیمرہ جیومیٹری، کیمرہ کے پیچھے کے پوائنٹس

4. **آپ متحرک اشیاء کے لیے اس فیوژن کو کیسے بہتر بنا سکتے ہیں؟**
   - سوچیں: موشن کمپن سیشن، پیش گوئی

---

## تصدیق

```bash
python3 validate.py
```

متوقع آؤٹ پٹ:

```
[PASS] sensor_fusion.py موجود ہے
[PASS] درست پائی تھن نحو
[PASS] ApproximateTimeSynchronizer استعمال کیا گیا
[PASS] sync_callback نافذ کیا گیا
[PASS] پوائنٹ پروجیکشن منطق موجود ہے
[PASS] امیج اوور لے نافذ کیا گیا
لیب 3 مکمل!
```

---

## توسیع چیلنجز

1. **گہرائی کے بیدار ڈیٹیکشن**: کیمرہ ڈیٹیکشنز کو فلٹر کرنے کے لیے لیڈار گہرائی استعمال کریں
2. **پوائنٹ رنگ**: لیڈار پوائنٹس کو مطابق امیج پکسلز کے ساتھ رنگ دیں
3. **رکاوٹ گہرائی**: ڈیٹیکٹ کی گئی اشیاء تک گہرائی کا حساب کریں
4. **ریل ٹائم ڈسپلے**: اوپن سی وی ونڈو میں فیوژن نتائج دکھائیں

---

## اگلے اقدامات

- **لیب 4**: اوبجیکٹ ڈیٹیکشن - ڈیٹیکشن پائپ لائنز نافذ کریں
- **لیسن 3**: پرچیپشن پائپ لائنز - مکمل پرچیپشن سسٹم