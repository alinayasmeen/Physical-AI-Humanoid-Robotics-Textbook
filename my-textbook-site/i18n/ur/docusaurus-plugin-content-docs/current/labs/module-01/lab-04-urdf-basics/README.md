# لیب 4: URDF بیسکس

**مدت**: 120 منٹ
**درس**: [Lesson 4: ہیومنوائڈ URDF فنڈامینٹلز](/docs/module-01-ros2/lesson-04-urdf-fundamentals)
**ضروریات**: لیبز 1-3 مکمل، ROS 2 ورک سپیس تشکیل دیا گیا

## اہداف

2-DOF روبوٹک ارم URDF ماڈل بنائیں اور اسے RViz میں وژولائز کریں. یہ لیب روبوٹ ماڈلنگ کے بنیادیات سکھاتی ہے جو پیچیدہ ہیومنوائڈ سسٹم تک پھیل سکتے ہیں.

## سیکھنے کے نتائج

اس لیب کو مکمل کرنے کے بعد، آپ کر پائیں گے:

- درست URDF XML فائلیں لکھنا
- وژول اور کولیژن جیومیٹری کے ساتھ لنکس کی وضاحت کرنا
- مناسب حدود اور ایکسز کے ساتھ جوائنٹس بنانا
- RViz میں روبوٹ ماڈلز کو وژولائز کرنا
- URDF پارسنگ کی غلطیوں کو ڈیبگ کرنا

## معماری

```
2-DOF ارم کی ساخت:

    [base_link]
         |
    (shoulder) - revolute joint, Y-axis
         |
    [upper_arm]
         |
     (elbow) - revolute joint, Y-axis
         |
    [lower_arm]
         |
    [end_effector]
```

## سیٹ اپ

### 1. پیکج بنائیں

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python urdf_demo \
    --dependencies rclpy urdf launch launch_ros
cd urdf_demo
mkdir -p urdf launch rviz
```

### 2. وژولائزیشن ڈیپینڈنسیز انسٹال کریں

```bash
sudo apt install ros-humble-rviz2 ros-humble-joint-state-publisher-gui
```

## کام

### کام 1: بیس URDF بنائیں (30 منٹ)

`urdf/simple_arm.urdf` بنائیں جس میں:

- `base_link`: ایک باکس (0.1 x 0.1 x 0.05 m)
- مواد کا رنگ: گرے (rgba: 0.5 0.5 0.5 1.0)

**اسٹارٹر کوڈ `starter/simple_arm.urdf` میں فراہم کیا گیا ہے**

### کام 2: شولڈر جوائنٹ اور اپر ارم شامل کریں (30 منٹ)

اپنے URDF میں شامل کریں:

- `shoulder` جوائنٹ: ریوولوٹ، Y-axis، حدود ±90°
- `upper_arm` لنک: سلنڈر (ریڈیس 0.02m، لمبائی 0.3m)

### کام 3: البو جوائنٹ اور لوور ارم شامل کریں (30 منٹ)

اپنے URDF میں شامل کریں:

- `elbow` جوائنٹ: ریوولوٹ، Y-axis، حدود 0° سے 135°
- `lower_arm` لنک: سلنڈر (ریڈیس 0.015m، لمبائی 0.2m)
- `end_effector` لنک: سپیئر (ریڈیس 0.02m) - فکسڈ جوائنٹ

### کام 4: لانچ فائل بنائیں اور وژولائز کریں (30 منٹ)

`launch/display.launch.py` بنائیں:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('urdf_demo')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'simple_arm.urdf')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg_dir, 'rviz', 'urdf.rviz')]
        ),
    ])
```

`setup.py` کو ڈیٹا فائلز شامل کرنے کے لیے اپ ڈیٹ کریں:

```python
data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/urdf', ['urdf/simple_arm.urdf']),
    ('share/' + package_name + '/launch', ['launch/display.launch.py']),
    ('share/' + package_name + '/rviz', ['rviz/urdf.rviz']),
],
```

تعمیر اور لانچ:

```bash
cd ~/ros2_ws
colcon build --packages-select urdf_demo
source install/setup.bash
ros2 launch urdf_demo display.launch.py
```

## توثیق

توثیق اسکرپٹ چلائیں:

```bash
python3 validate.py urdf/simple_arm.urdf
```

متوقع آؤٹ پٹ:

```
✅ URDF فائل کامیابی سے پارس ہوتی ہے
✅ 4 لنکس مل گئے: base_link, upper_arm, lower_arm, end_effector
✅ 3 جوائنٹس مل گئے: shoulder, elbow, wrist
✅ جوائنٹ کی حدود درست ہیں
✅ تمام چیکس پاس!
```

## متوقع نتائج

1. RViz ارم ماڈل ڈسپلے کرتا ہے
2. جوائنٹ اسٹیٹ پبلشر GUI شولڈر اور البو کے لیے سلائیڈر دکھاتا ہے
3. سلائیڈر ملانے سے ارم اینیمیٹ ہوتا ہے
4. TF ٹری صحیح والد-بچہ تعلقات دکھاتی ہے

### RViz کنفیگریشن

RViz میں:

1. `RobotModel` ڈسپلے شامل کریں
2. فکسڈ فریم کو `base_link` پر سیٹ کریں
3. فریم دیکھنے کے لیے `TF` ڈسپلے شامل کریں

## مسئلہ حل

| مسئلہ | حل |
|-------|----------|
| "Invalid URDF" | XML نحو اور جوائنٹ والد/بچہ حوالہ جات چیک کریں |
| ماڈل نظر نہیں آتا | یقینی بنائیں کہ فکسڈ فریم `base_link` پر سیٹ ہے |
| جوائنٹس نہیں چلتے | تصدیق کریں کہ جوائنٹ کی قسم `revolute` ہے، `fixed` نہیں |
| لانچ ناکام | setup.py میں تمام data_files پاتھ چیک کریں |

## ماڈل کو سمجھنا

### کوآرڈینیٹ فریم

ہر جوائنٹ ایک نیا کوآرڈینیٹ فریم بناتا ہے:

- `base_link` → روبوٹ کا اصل
- `shoulder` → Y کے ارد گرد اپر ارم کو گھوماتا ہے
- `elbow` → Y کے ارد گرد لوور ارم کو گھوماتا ہے

### TF کے ساتھ چیک کرنا

```bash
ros2 run tf2_tools view_frames
```

یہ ٹرینسفر ٹری دکھانے والی PDF جنریٹ کرتا ہے.

## توسیعات (اختیاری)

1. ارم سیگمنٹس کو الگ کرنے کے لیے رنگ شامل کریں
2. سیمولیشن کے لیے انرشل پراپرٹیز شامل کریں
3. 6-DOF ارم بنائیں (شولڈر پچ/رول، wrist شامل کریں)
4. کولیژن جیومیٹری شامل کریں

## مکمل URDF حوالہ

کام کرتے URDF کے لیے `solution/simple_arm.urdf` دیکھیں.

## اگلے اقدامات

URDF کو سیمولیشن سے جوڑنے کے طریقے سیکھنے کے لیے [Lesson 5: سیمولیشن، مشق اور جائزہ](/docs/module-01-ros2/lesson-05-simulation-review) پر جاری رکھیں.
