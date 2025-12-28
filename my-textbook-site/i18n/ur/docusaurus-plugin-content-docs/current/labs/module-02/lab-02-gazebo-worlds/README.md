# لیب 2: سیمولیشن ورلڈز تیار کرنا

**موڈیول**: 2 - ڈیجیٹل ٹوئن
**مدت**: 90 منٹ
**پریشانی**: متوسط

## اہداف

رکاوٹوں، زمینی سرزمین، اور لائٹنگ کے ساتھ ایک کسٹم گیزبو سیمولیشن ورلڈ بنائیں. اس لیب کے اختتام تک، آپ کے پاس روبوٹ نیویگیشن کی ٹیسٹنگ کے لیے ایک دوبارہ استعمال کے قابل سیمولیشن ماحول ہوگا.

## ضروریات

- لیب 1 (فزکس تجربات) مکمل
- موڈیول 2 لیسن 2 (گیزبو ورک فلو) مکمل
- ROS 2 ہمبل کے ساتھ اوبنٹو 22.04
- گیزبو فورٹریس (`ros-humble-ros-gz`)

## سیکھنے کے نتائج

اس لیب کو مکمل کرنے کے بعد، آپ:

1. صفر سے SDF ورلڈ فائلز بنائیں گے
2. فزکس انجن پیرامیٹر کنفیگر کریں گے
3. سٹیٹک رکاوٹیں شامل کریں گے (باکسز، سلنڈرز، سپیئرز)
4. حقیقی وژولائزیشن کے لیے لائٹنگ سیٹ اپ کریں گے
5. اسپون کیے گئے روبوٹ کے ساتھ ورلڈ ٹیسٹ کریں گے

---

## سیٹ اپ

### 1. لیب ڈائریکٹری میں جائیں

```bash
cd ~/ros2_ws/src/labs/module-02/lab-02-gazebo-worlds
```

### 2. اسٹارٹر فائلز کا جائزہ لیں

```bash
ls starter/worlds/
# اس میں شامل ہونا چاہیے: empty_world.sdf
```

---

## لیب اسٹیپس

### اسٹیپ 1: خالی ورلڈ کو سمجھیں (10 منٹ)

خالی ورلڈ ٹیمپلیٹ کو کھولیں اور دیکھیں:

```bash
cat starter/worlds/empty_world.sdf
```

پہچاننے کے لیے کلیدی سیکشنز:

- `<world>` - روٹ عنصر
- `<physics>` - انجن کنفیگریشن
- `<light>` - لائٹنگ
- `<model name="ground_plane">` - فرش

### اسٹیپ 2: فزکس کنفیگریشن شامل کریں (15 منٹ)

`starter/worlds/empty_world.sdf` میں فزکس کنفیگر کرنے کے لیے ایڈٹ کریں:

```xml
<physics name="custom_physics" type="ode">
  <!-- اسٹیپ سائز: چھوٹا = زیادہ درست، سست -->
  <max_step_size>0.001</max_step_size>

  <!-- ریل ٹائم فیکٹر: 1.0 = ریل ٹائم، >1.0 = تیز -->
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>

  <!-- ODE مخصوص ترتیبات -->
  <ode>
    <solver>
      <type>quick</type>
      <iters>50</iters>
      <sor>1.3</sor>
    </solver>
    <constraints>
      <cfm>0.0</cfm>
      <erp>0.2</erp>
    </constraints>
  </ode>
</physics>
```

### اسٹیپ 3: رکاوٹیں شامل کریں (25 منٹ)

مختلف اقسام کی کم از کم 3 رکاوٹیں شامل کریں:

**باکس رکاوٹ:**

```xml
<model name="obstacle_box">
  <static>true</static>
  <pose>3 0 0.5 0 0 0</pose>
  <link name="link">
    <collision name="collision">
      <geometry>
        <box><size>1 1 1</size></box>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box><size>1 1 1</size></box>
      </geometry>
      <material>
        <ambient>0.8 0.2 0.2 1</ambient>
        <diffuse>0.8 0.2 0.2 1</diffuse>
      </material>
    </visual>
  </link>
</model>
```

**سلنڈر رکاوٹ:**

```xml
<model name="obstacle_cylinder">
  <static>true</static>
  <pose>0 3 0.75 0 0 0</pose>
  <link name="link">
    <collision name="collision">
      <geometry>
        <cylinder>
          <radius>0.5</radius>
          <length>1.5</length>
        </cylinder>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <cylinder>
          <radius>0.5</radius>
          <length>1.5</length>
        </cylinder>
      </geometry>
      <material>
        <ambient>0.2 0.8 0.2 1</ambient>
        <diffuse>0.2 0.8 0.2 1</diffuse>
      </material>
    </visual>
  </link>
</model>
```

**سپیئر رکاوٹ:**

```xml
<model name="obstacle_sphere">
  <static>true</static>
  <pose>-2 2 0.5 0 0 0</pose>
  <link name="link">
    <collision name="collision">
      <geometry>
        <sphere><radius>0.5</radius></sphere>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <sphere><radius>0.5</radius></sphere>
      </geometry>
      <material>
        <ambient>0.2 0.2 0.8 1</ambient>
        <diffuse>0.2 0.2 0.8 1</diffuse>
      </material>
    </visual>
  </link>
</model>
```

### اسٹیپ 4: لائٹنگ کنفیگر کریں (15 منٹ)

سایوں کے ساتھ حقیقی لائٹنگ شامل کریں:

```xml
<!-- سورج کی لائٹ -->
<light type="directional" name="sun">
  <cast_shadows>true</cast_shadows>
  <pose>0 0 10 0 0 0</pose>
  <diffuse>0.8 0.8 0.8 1</diffuse>
  <specular>0.2 0.2 0.2 1</specular>
  <direction>-0.5 0.5 -1</direction>
</light>

<!-- فل کے لیے ایمبیئنٹ لائٹ -->
<light type="point" name="ambient_light">
  <cast_shadows>false</cast_shadows>
  <pose>0 0 5 0 0 0</pose>
  <diffuse>0.3 0.3 0.3 1</diffuse>
  <specular>0.0 0.0 0.0 1</specular>
  <attenuation>
    <range>20</range>
    <constant>0.5</constant>
    <linear>0.01</linear>
    <quadratic>0.001</quadratic>
  </attenuation>
</light>
```

### اسٹیپ 5: لانچ اور ٹیسٹ (15 منٹ)

اپنا کسٹم ورلڈ لانچ کریں:

```bash
gz sim -r starter/worlds/empty_world.sdf
```

**تصدیق کریں:**

- [ ] ورلڈ غلطیوں کے بغیر لوڈ ہوتا ہے
- [ ] تمام رکاوٹیں نظر آتی ہیں
- [ ] لائٹنگ سایے بناتی ہے
- [ ] فزکس کا رویہ مستحکم ہے

### اسٹیپ 6: ایک ٹیسٹ روبوٹ اسپون کریں (10 منٹ)

اپنے ورلڈ کو ایک سادہ روبوٹ کے ساتھ ٹیسٹ کریں:

```bash
# ٹرمنل 1: ورلڈ لانچ کریں
gz sim -r starter/worlds/empty_world.sdf

# ٹرمنل 2: ماڈل اسپون کریں (بیلٹ ان شکلیں استعمال کریں)
gz service -s /world/custom_world/create \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 1000 \
  --req 'sdf: "<sdf version=\"1.8\"><model name=\"test_box\"><pose>0 0 2 0 0 0</pose><link name=\"link\"><collision name=\"collision\"><geometry><box><size>0.5 0.5 0.5</size></box></geometry></collision><visual name=\"visual\"><geometry><box><size>0.5 0.5 0.5</size></box></geometry></visual></link></model></sdf>"'
```

باکس گرتے ہوئے اور اپنے ورلڈ کے ساتھ بات چیت کرتے ہوئے دیکھیں!

---

## ڈلیوریبلز

1. **کسٹم ورلڈ فائل**: `starter/worlds/empty_world.sdf` جس میں:

   - کسٹم فزکس کنفیگریشن
   - کم از کم 3 مختلف رکاوٹ کی اقسام
   - سایوں کے ساتھ مناسب لائٹنگ
   - مناسب طریقے سے نامزد (نام `custom_world.sdf` پر تبدیل کریں)

2. **اسکرین شاٹ**: گیزبو میں اپنے ورلڈ کا ایک اسکرین شاٹ محفوظ کریں

---

## توثیق

توثیق اسکرپٹ چلائیں:

```bash
python3 validate.py
```

متوقع آؤٹ پٹ:

```
[PASS] SDF فائل درست XML ہے
[PASS] فزکس کنفیگریشن موجود ہے
[PASS] کم از کم 3 رکاوٹیں تعریف شدہ ہیں
[PASS] لائٹنگ کنفیگریشن موجود ہے
[PASS] گراؤنڈ پلین موجود ہے
لیب 2 مکمل!
```

---

## بونس چیلنجز

1. **ایک دیوار شامل کریں**: ماحول کے ارد گرد ایک باؤنڈری دیوار بنائیں
2. **اونچائی کی تبدیلی**: مختلف اونچائیوں پر پلیٹ فارم یا ریمپس شامل کریں
3. **ٹیکسچر والی زمین**: زمینی سطح پر ایک ٹیکسچر شامل کریں
4. **حرکت پذیر رکاوٹ**: ایک غیر سٹیٹک رکاوٹ بنائیں جسے دھکیلا جا سکے

---

## مسئلہ حل

### ورلڈ لوڈ ہونے میں ناکام
- XML نحو چیک کریں (استعمال کریں `xmllint --noout file.sdf`)
- یقینی بنائیں کہ SDF ورژن گیزبو ورژن سے مماثل ہے
- ٹیگ کے ناموں میں ٹائپوں کی تلاش کریں

### رکاوٹیں نظر نہیں آتیں
- `<pose>` قیمتیں چیک کریں (x y z roll pitch yaw)
- یقینی بنائیں کہ `<visual>` سیکشن تعریف شدہ ہے
- یقینی بنائیں کہ ماڈل `<world>` ٹیگس کے اندر ہے

### فزکس غیر مستحکم
- حل کرنے والے اقدامات میں اضافہ کریں
- اسٹیپ سائز کم کریں
- متداخل اشیاء کی جانچ کریں

---

## اگلے اقدامات

اس لیب کو مکمل کرنے کے بعد، آگے بڑھیں:

- **لیب 3**: سینسر وژولائزیشن - اپنے ورلڈ میں سینسرز شامل کریں
- **لیسن 3**: سینسرز اور یونٹی - سینسر کنفیگریشن کے بارے میں سیکھیں