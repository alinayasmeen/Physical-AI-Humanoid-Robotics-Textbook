# لیب 2: پبلشر-سبسکرائبر کمیونیکیشن

**مدت**: 90 منٹ
**درس**: [Lesson 3: rclpy کے ساتھ پائی تھن ایجنٹس](/docs/module-01-ros2/lesson-03-rclpy-agents)
**ضروریات**: لیب 1 مکمل، ROS 2 ورک سپیس تشکیل دیا گیا

## اہداف

دو نوڈز کے درمیان ٹاپک ہیڈ کمیونیکیشن نافذ کریں. ایک پبلشر بنائیں جو رفتار کمانڈز بھیجتا ہے اور ایک سبسکرائبر جو وصول کرتا ہے اور انہیں لاگ کرتا ہے.

## سیکھنے کے نتائج

اس لیب کو مکمل کرنے کے بعد، آپ کر پائیں گے:
- `create_publisher()` کا استعمال کرتے ہوئے پبلشر نوڈز بنانا
- `create_subscription()` کا استعمال کرتے ہوئے سبسکرائبر نوڈز بنانا
- رفتار کمانڈز کے لیے `geometry_msgs/Twist` استعمال کرنا
- ROS 2 CLI ٹولز کا استعمال کرتے ہوئے پیغام کے تبادلے کی تصدیق کرنا

## معماری

```
┌─────────────────┐    /cmd_vel    ┌─────────────────┐
│ velocity_pub    │ ──────────────▶│ velocity_sub    │
│ (Publisher)     │   Twist msg    │ (Subscriber)    │
└─────────────────┘                └─────────────────┘
```

## سیٹ اپ

### 1. پیکج بنائیں

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python pubsub_demo \
    --dependencies rclpy geometry_msgs
cd pubsub_demo/pubsub_demo
```

## کام

### کام 1: پبلشر بنائیں (30 منٹ)

`velocity_publisher.py` بنائیں جو:
- `'velocity_publisher'` نام کا نوڈ بناتا ہے
- `/cmd_vel` ٹاپک پر `Twist` پیغام کی قسم کے ساتھ شائع کرتا ہے
- 10 Hz ٹائمر (0.1 سیکنڈ کی مدت) استعمال کرتا ہے
- `linear.x = 0.5` اور `angular.z = 0.1` بھیجتا ہے

**اسٹارٹر کوڈ `starter/velocity_publisher.py` میں فراہم کیا گیا ہے**

### کام 2: سبسکرائبر بنائیں (30 منٹ)

`velocity_subscriber.py` بنائیں جو:
- `'velocity_subscriber'` نام کا نوڈ بناتا ہے
- `/cmd_vel` ٹاپک کو سبسکرائب کرتا ہے
- موصول شدہ لینیئر اور اینگولر رفتار کو لاگ کرتا ہے

**اسٹارٹر کوڈ `starter/velocity_subscriber.py` میں فراہم کیا گیا ہے**

### کام 3: پیکج کی ترتیب (10 منٹ)

`setup.py` اینٹری پوائنٹس میں ترمیم کریں:

```python
entry_points={
    'console_scripts': [
        'velocity_pub = pubsub_demo.velocity_publisher:main',
        'velocity_sub = pubsub_demo.velocity_subscriber:main',
    ],
},
```

### کام 4: تعمیر اور ٹیسٹ (20 منٹ)

```bash
cd ~/ros2_ws
colcon build --packages-select pubsub_demo
source install/setup.bash

# ٹرمنل 1: پبلشر چلائیں
ros2 run pubsub_demo velocity_pub

# ٹرمنل 2: سبسکرائبر چلائیں
ros2 run pubsub_demo velocity_sub

# ٹرمنل 3: تصدیق کریں
ros2 topic list
ros2 topic echo /cmd_vel
ros2 topic hz /cmd_vel
```

## توثیق

توثیق اسکرپٹ چلائیں:

```bash
python3 validate.py
```

متوقع آؤٹ پٹ:
```
✅ پبلشر نوڈ کا نحو درست ہے
✅ سبسکرائبر نوڈ کا نحو درست ہے
✅ ضروری ROS 2 پیٹرنز مل گئے
✅ تمام چیکس پاس!
```

## متوقع نتائج

1. پبلشر آؤٹ پٹ: "Publishing: linear=0.50, angular=0.10"
2. سبسکرائبر آؤٹ پٹ: "Received: linear=0.50, angular=0.10"
3. `ros2 topic hz /cmd_vel` ~10 Hz دکھاتا ہے
4. `ros2 topic echo /cmd_vel` ٹوسٹ پیغامات ڈسپلے کرتا ہے

## کوڈ کو سمجھنا

### پیغام کی اقسام

`geometry_msgs` کا `Twist` پیغام پر مشتمل ہے:
```
Vector3 linear   # x, y, z لینیئر رفتار
Vector3 angular  # x, y, z اینگولر رفتار
```

زمینی روبوٹس کے لیے:
- `linear.x` = آگے/پیچھے کی رفتار
- `angular.z` = گردش کی رفتار (yaw)

### QoS (کوالٹی آف سروس)

`create_publisher()` میں `10` پیرامیٹر کیو گہرائی سیٹ کرتا ہے. یہ اثر ڈالتا ہے:
- **قابل اعتمادی**: کتنے پیغامات بفر کریں
- **دیری**: ریل ٹائم کارکردگی کے ساتھ تجارتی کمپرومائز

## مسئلہ حل

| مسئلہ | حل |
|-------|----------|
| سبسکرائبر وصول نہیں کرتا | چیک کریں کہ ٹاپک کے نام بالکل مماثل ہیں |
| امپورٹ کی غلطی | geometry_msgs انسٹال کریں: `sudo apt install ros-humble-geometry-msgs` |
| پیغامات تاخیر شدہ | کیو گہرائی کم کریں یا نیٹ ورک چیک کریں |

## توسیعات (اختیاری)

1. رفتار کی اقدار کو کنٹرول کرنے کے لیے ایک پیرامیٹر شامل کریں
2. ایک رفتار ریمپ ( gradual acceleration) نافذ کریں
3. رفتار کو تبدیل کرنے والے تیسرے نوڈ کو شامل کریں

## اگلے اقدامات

ریکویسٹ-ریسپانس پیٹرنز سیکھنے کے لیے [لیب 3: سروسز](/labs/module-01/lab-03-services) پر جاری رکھیں.