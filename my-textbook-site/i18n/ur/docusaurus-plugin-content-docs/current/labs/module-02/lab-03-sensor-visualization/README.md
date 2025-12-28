# لیب 3: سینسر کنفیگریشن اور وژولائزیشن

**موڈیول**: 2 - ڈیجیٹل ٹوئن
**مدت**: 90 منٹ
**پریشانی**: متوسط

## اہداف

گیزبو میں سیمولیٹڈ سینسرز (کیمرہ، لیڈار، IMU) کنفیگر کریں اور ROS 2 ٹولز کا استعمال کرتے ہوئے سینسر ڈیٹا وژولائز کریں. اس لیب کے اختتام تک، آپ سیمولیشن سے ROS 2 ٹاپکس تک مکمل سینسر ڈیٹا پائپ لائن کو سمجھیں گے.

## ضروریات

- لیب 2 (گیزبو ورلڈز) مکمل
- موڈیول 2 لیسن 3 (سینسرز اور یونٹی) مکمل
- ROS 2 ہمبل کے ساتھ اوبنٹو 22.04
- ros_gz پیکجز کے ساتھ گیزبو فورٹریس

## سیکھنے کے نتائج

اس لیب کو مکمل کرنے کے بعد، آپ:

1. SDF/URDF میں کیمرہ سینسرز کنفیگر کریں گے
2. قابل کسٹمائز پیرامیٹر کے ساتھ لیڈار سینسرز کنفیگر کریں گے
3. نوائز ماڈل کے ساتھ IMU سینسرز کنفیگر کریں گے
4. گیزبو سے ROS 2 تک سینسر ٹاپکس برج کریں گے
5. RViz2 میں سینسر ڈیٹا وژولائز کریں گے
6. ایک پائی تھن نوڈ میں سینسر میسجز کو عمل کریں گے

---

## سیٹ اپ

### 1. ضروری پیکجز انسٹال کریں

```bash
# یقینی بنائیں کہ ros_gz پیکجز انسٹال ہیں
sudo apt install ros-humble-ros-gz-bridge ros-humble-ros-gz-sim ros-humble-ros-gz-image

# اگر موجود نہ ہو تو RViz2 انسٹال کریں
sudo apt install ros-humble-rviz2
```

### 2. لیب ڈائریکٹری میں جائیں

```bash
cd ~/ros2_ws/src/labs/module-02/lab-03-sensor-visualization
```

---

## لیب اسٹیپس

### اسٹیپ 1: سینسر کنفیگریشن کا جائزہ لیں (15 منٹ)

اسٹارٹر سینسر کنفیگریشن فائل کا جائزہ لیں:

```bash
cat starter/sensor_config.yaml
```

یہ YAML فائل سینسر پیرامیٹر کی وضاحت کرتی ہے جو آپ SDF روبوٹ ماڈل میں استعمال کریں گے.

### اسٹیپ 2: کیمرہ سینسر شامل کریں (20 منٹ)

روبوٹ ماڈل میں ایک کیمرہ سینسر شامل کریں. ایک SDF فائل بنائیں یا ایڈٹ کریں:

```xml
<sensor name="camera" type="camera">
  <pose>0.1 0 0.1 0 0 0</pose>
  <always_on>true</always_on>
  <update_rate>30</update_rate>
  <camera name="front_camera">
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>100</far>
    </clip>
  </camera>
  <topic>camera/image_raw</topic>
</sensor>
```

**کیمرہ ٹاپک برج کریں:**

```bash
ros2 run ros_gz_bridge parameter_bridge \
  /camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image
```

### اسٹیپ 3: لیڈار سینسر شامل کریں (20 منٹ)

ایک 2D لیڈار سینسر شامل کریں:

```xml
<sensor name="lidar" type="gpu_lidar">
  <pose>0 0 0.2 0 0 0</pose>
  <always_on>true</always_on>
  <update_rate>10</update_rate>
  <lidar>
    <scan>
      <horizontal>
        <samples>360</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>
      <max>30.0</max>
      <resolution>0.01</resolution>
    </range>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.01</stddev>
    </noise>
  </lidar>
  <topic>scan</topic>
</sensor>
```

**لیڈار ٹاپک برج کریں:**

```bash
ros2 run ros_gz_bridge parameter_bridge \
  /scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan
```

### اسٹیپ 4: IMU سینسر شامل کریں (15 منٹ)

نوائز کے ساتھ ایک IMU سینسر شامل کریں:

```xml
<sensor name="imu" type="imu">
  <pose>0 0 0.05 0 0 0</pose>
  <always_on>true</always_on>
  <update_rate>100</update_rate>
  <imu>
    <angular_velocity>
      <x><noise type="gaussian"><mean>0</mean><stddev>0.01</stddev></noise></x>
      <y><noise type="gaussian"><mean>0</mean><stddev>0.01</stddev></noise></y>
      <z><noise type="gaussian"><mean>0</mean><stddev>0.01</stddev></noise></z>
    </angular_velocity>
    <linear_acceleration>
      <x><noise type="gaussian"><mean>0</mean><stddev>0.1</stddev></noise></x>
      <y><noise type="gaussian"><mean>0</mean><stddev>0.1</stddev></noise></y>
      <z><noise type="gaussian"><mean>0</mean><stddev>0.1</stddev></noise></z>
    </linear_acceleration>
  </imu>
  <topic>imu/data</topic>
</sensor>
```

**IMU ٹاپک برج کریں:**

```bash
ros2 run ros_gz_bridge parameter_bridge \
  /imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU
```

### اسٹیپ 5: لانچ اور ٹاپکس کی تصدیق کریں (10 منٹ)

اپنے سینسر والے روبوٹ کے ساتھ گیزبو لانچ کریں:

```bash
# ٹرمنل 1: سیمولیشن لانچ کریں
gz sim -r your_world_with_robot.sdf

# ٹرمنل 2: تمام برج چلائیں
ros2 run ros_gz_bridge parameter_bridge \
  /camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image \
  /scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan \
  /imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU

# ٹرمنل 3: ٹاپکس کو فہرست کریں
ros2 topic list
```

متوقع ٹاپکس:

- `/camera/image_raw`
- `/scan`
- `/imu/data`

### اسٹیپ 6: RViz2 میں وژولائز کریں (10 منٹ)

RViz2 لانچ کریں اور ڈسپلیز شامل کریں:

```bash
ros2 run rviz2 rviz2
```

**ڈسپلیز شامل کریں:**

1. شامل کریں → ٹاپک کے ذریعے → `/camera/image_raw` → امیج
2. شامل کریں → ٹاپک کے ذریعے → `/scan` → لیزر اسکین
3. فکسڈ فریم کو `base_link` یا مناسب فریم پر سیٹ کریں

**مستقبل کے استعمال کے لیے اپنی RViz کنفیگ محفوظ کریں.**

### اسٹیپ 7: پائی تھن کے ساتھ سینسرز کو عمل کریں (15 منٹ)

سینسر اینالائزر اسکرپٹ مکمل کریں:

```bash
# متوقع رویہ دیکھنے کے لیے حل چلائیں
python3 solution/sensor_analyzer.py

# پھر اسٹارٹر میں اپنا ورژن نافذ کریں
python3 starter/sensor_analyzer.py
```

اسکرپٹ کو چاہیے:

- تینوں سینسر ٹاپکس کو سبسکرائب کریں
- سینسر ڈیٹا کے اعداد و شمار لاگ کریں
- جب سینسرز شائع کر رہے ہوں تو ڈیٹیکٹ کریں

---

## ڈلیوریبلز

1. **کام کرتی ہوئی سینسر کنفیگریشن**: کیمرہ، لیڈار، اور IMU کے ساتھ روبوٹ ماڈل
2. **RViz کنفیگریشن**: تمام سینسرز دکھانے والی محفوظ شدہ `.rviz` فائل
3. **سینسر اینالائزر اسکرپٹ**: تمام سینسر ڈیٹا کو عمل کرنے والا پائی تھن نوڈ

---

## توثیق

توثیق اسکرپٹ چلائیں:

```bash
python3 validate.py
```

متوقع آؤٹ پٹ:

```
[PASS] سینسر کنفیگریشن فائل موجود ہے
[PASS] حل اسکرپٹ کا نحو درست ہے
[PASS] حل امیج میسجز کو ہینڈل کرتا ہے
[PASS] حل لیزر اسکین میسجز کو ہینڈل کرتا ہے
[PASS] حل Imu میسجز کو ہینڈل کرتا ہے
لیب 3 مکمل!
```

---

## ریفلیکشن سوالات

1. **سیمولیٹڈ سینسرز میں نوائز کیوں شامل کریں؟**
   - نوائز سیم-ٹو-ریل ٹرانسفر میں کیسے مدد کرتا ہے؟

2. **اگر کیمرہ اور لیڈار فریم غلط طور پر ایلائن ہوں تو کیا ہوتا ہے؟**
   - سینسر فیوژن کے منظر کو مدنظر رکھیں

3. **آپ لیڈار ریزولوشن بمقابلہ رینج کو کیسے بڑھا سکتے ہیں؟**
   - سینسر کنفیگریشن میں ٹریڈ آف

---

## بونس چیلنجز

1. **ڈیپتھ کیمرہ شامل کریں**: پوائنٹ کلاؤڈ آؤٹ پٹ کے ساتھ ڈیپتھ کیمرہ کنفیگر کریں
2. **سینسر فیوژن**: ایک ہی نوڈ میں کیمرہ اور لیڈار ڈیٹا کو ضم کریں
3. **کسٹم نوائز ماڈل**: IMU کے لیے بائیس ڈرائیف نافذ کریں

---

## مسئلہ حل

### ٹاپکس نظر نہیں آ رہے

```bash
# چیک کریں کہ گیزبو سینسر پلگ ان لوڈ ہوئے
gz topic -l

# یقینی بنائیں کہ برج چل رہا ہے
ros2 node list | grep bridge
```

### RViz میں کوئی ڈیٹا نہیں دکھ رہا

- چیک کریں کہ فکسڈ فریم سینسر فریم سے مماثل ہے
- ڈسپلیز میں ٹاپک نام چیک کریں
- چیک کریں کہ سیمولیشن چل رہا ہے (موقوف نہیں)

### سینسر ڈیٹا غلط لگ رہا ہے

- SDF میں سینسر پوز چیک کریں
- کوآرڈینیٹ فریم کنونشن چیک کریں
- یقینی بنائیں کہ فزکس اسٹیپنگ ہو رہی ہے (ریل ٹائم فیکٹر > 0)

---

## اگلے اقدامات

اس لیب کو مکمل کرنے کے بعد:

- لیسن 3 میں موڈیول 2 کوئز کا جائزہ لیں
- موڈیول 3 پر جاری رکھیں: پرچیپشن الگورتھم
- نیویگیشن ٹیسٹنگ کے لیے اپنا سینسر والے ورلڈ استعمال کریں