# لیب 3: ROS 2 سروسز

**مدت**: 90 منٹ
**درس**: [Lesson 3: rclpy کے ساتھ پائی تھن ایجنٹس](/docs/module-01-ros2/lesson-03-rclpy-agents)
**ضروریات**: لیب 1 اور لیب 2 مکمل

## اہداف

ROS 2 سروسز کا استعمال کرتے ہوئے ریکویسٹ-ریسپانس کمیونیکیشن نافذ کریں. ایک سروس سرور بنائیں جو روبوٹ اسٹیٹ کوائف کو ہینڈل کرتا ہے اور ایک کلائنٹ جو درخواستیں بھیجتا ہے.

## سیکھنے کے نتائج

اس لیب کو مکمل کرنے کے بعد، آپ کر پائیں گے:
- `create_service()` کا استعمال کرتے ہوئے سروس سرورز بنانا
- `create_client()` کا استعمال کرتے ہوئے سروس کلائنٹس بنانا
- معیاری سروس کی اقسام (`std_srvs`) استعمال کرنا
- ہم وقتی ریکویسٹ-ریسپانس پیٹرنز کو ہینڈل کرنا

## معماری

```
┌─────────────────┐  request   ┌─────────────────┐
│ state_client    │ ─────────▶│ state_server    │
│ (Client)        │            │ (Server)        │
│                 │ ◀───────── │                 │
└─────────────────┘  response  └─────────────────┘
         │                              │
         └──────── /get_state ──────────┘
```

## سیٹ اپ

### 1. پیکج بنائیں

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python services_demo \
    --dependencies rclpy std_srvs
cd services_demo/services_demo
```

## کام

### کام 1: سروس سرور بنائیں (35 منٹ)

`state_server.py` بنائیں جو:
- `'robot_state_server'` نام کا نوڈ بناتا ہے
- `SetBool` قسم کا `/get_state` سروس فراہم کرتا ہے
- جب `data=True`: روبوٹ کا "چل رہا ہے" اسٹیٹ لوٹاتا ہے
- جب `data=False`: روبوٹ کا "روکا ہوا ہے" اسٹیٹ لوٹاتا ہے
- اندرونی اسٹیٹ کو ٹریک کرتا ہے اور ٹرانزیشنز کو لاگ کرتا ہے

**اسٹارٹر کوڈ `starter/state_server.py` میں فراہم کیا گیا ہے**

### کام 2: سروس کلائنٹ بنائیں (35 منٹ)

`state_client.py` بنائیں جو:
- `'robot_state_client'` نام کا نوڈ بناتا ہے
- `/get_state` سروس کو کال کرتا ہے
- درخواست کی قیمت کے لیے کمانڈ لائن آرگومنٹ قبول کرتا ہے
- ریسپانس کو لاگ کرتا ہے

**اسٹارٹر کوڈ `starter/state_client.py` میں فراہم کیا گیا ہے**

### کام 3: تشکیل اور ٹیسٹ (20 منٹ)

`setup.py` میں ترمیم کریں:

```python
entry_points={
    'console_scripts': [
        'state_server = services_demo.state_server:main',
        'state_client = services_demo.state_client:main',
    ],
},
```

تعمیر اور ٹیسٹ:

```bash
cd ~/ros2_ws
colcon build --packages-select services_demo
source install/setup.bash

# ٹرمنل 1: سرور چلائیں
ros2 run services_demo state_server

# ٹرمنل 2: CLI کے ذریعے سروس کال کریں
ros2 service call /get_state std_srvs/srv/SetBool "{data: true}"

# ٹرمنل 3: کلائنٹ چلائیں
ros2 run services_demo state_client --ros-args -p request:=true
```

## توثیق

توثیق اسکرپٹ چلائیں:

```bash
python3 validate.py
```

متوقع آؤٹ پٹ:
```
✅ سرور نوڈ کا نحو درست ہے
✅ کلائنٹ نوڈ کا نحو درست ہے
✅ ضروری سروس پیٹرنز مل گئے
✅ تمام چیکس پاس!
```

## متوقع نتائج

### سرور آؤٹ پٹ
```
[INFO] روبوٹ اسٹیٹ سرور تیار
[INFO] درخواست موصول ہوئی: data=True
[INFO] جواب دیا جا رہا ہے: success=True, message='روبوٹ چل رہا ہے'
```

### کلائنٹ آؤٹ پٹ
```
[INFO] درخواست بھیجی جا رہی ہے: data=True
[INFO] ریسپانس: success=True, message='روبوٹ چل رہا ہے'
```

### CLI تصدیق
```bash
$ ros2 service list
/get_state

$ ros2 service type /get_state
std_srvs/srv/SetBool
```

## کوڈ کو سمجھنا

### SetBool سروس قسم

`std_srvs/srv/SetBool` سروس میں ہے:

**ریکویسٹ:**
```
bool data
```

**ریسپانس:**
```
bool success
string message
```

### ہم وقتی بمقابلہ غیر ہم وقتی کلائنٹس

یہ لیب غیر ہم وقتی کلائنٹ کالز (`call_async`) استعمال کرتی ہے. سادہ اسکرپٹس کے لیے، آپ ہم وقتی کالز استعمال کر سکتے ہیں، لیکن نوڈ کال بیکس میں بلاک ہونے سے بچنے کے لیے غیر ہم وقتی ترجیح دی جاتی ہے.

## مسئلہ حل

| مسئلہ | حل |
|-------|----------|
| سروس نہیں ملا | یقینی بنائیں کہ سرور پہلے چل رہا ہے |
| ٹائم آؤٹ کی غلطی | چیک کریں کہ سروس کا نام بالکل مماثل ہے |
| امپورٹ کی غلطی | std_srvs انسٹال کریں: `sudo apt install ros-humble-std-srvs` |

## توسیعات (اختیاری)

1. متعدد سروسز شامل کریں (شروع، روکیں، ری سیٹ)
2. کسٹم پیغام کی قسم کے ساتھ سروس نافذ کریں
3. کلائنٹ میں ٹائم آؤٹ ہینڈلنگ شامل کریں
4. موازنہ کے لیے ایکشن بیسڈ متبادل بنائیں

## اگلے اقدامات

روبوٹ ماڈلنگ سیکھنے کے لیے [لیب 4: URDF بیسکس](/labs/module-01/lab-04-urdf-basics) پر جاری رکھیں.