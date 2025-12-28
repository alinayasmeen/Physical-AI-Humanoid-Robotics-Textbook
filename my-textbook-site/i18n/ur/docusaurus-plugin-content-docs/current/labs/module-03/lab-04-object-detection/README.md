# لیب 4: اوبجیکٹ ڈیٹیکشن پائپ لائن

**موڈیول**: 3 - پرچیپشن اور سینسر
**مدت**: 120 منٹ
**پریشانی**: متوسط

## اہداف

OpenCV DNN کے ساتھ YOLO کا استعمال کرتے ہوئے ایک مکمل اوبجیکٹ ڈیٹیکشن پائپ لائن نافذ کریں، ڈیٹیکشنز کو ROS 2 vision_msgs کے طور پر شائع کریں. آپ کیمرہ امیجز کو عمل کریں گے، انفرینس چلائیں گے، نتائج کو یقین کے ذریعے فلٹر کریں گے، اور ڈیٹیکشنز کو وژولائز کریں گے.

## ضروریات

- لیبز 1-3 مکمل
- OpenCV 4.x DNN ماڈیول کے ساتھ
- YOLO ماڈل فائلز (فراہم یا ڈاؤن لوڈ ہدایات)
- `ros-humble-vision-msgs` پیکج

## سیکھنے کے نتائج

اس لیب کو مکمل کرنے کے بعد، آپ:

1. OpenCV DNN کا استعمال کرتے ہوئے YOLO ماڈلز لوڈ اور چلائیں گے
2. ماڈل آؤٹ پٹس کو پروسیس کر کے باؤنڈنگ باکسز نکالیں گے
3. غیر زیادہ سے زیادہ دباؤ (NMS) لاگو کریں گے
4. vision_msgs کا استعمال کرتے ہوئے ڈیٹیکشنز شائع کریں گے

---

## سیٹ اپ

### 1. انحصار انسٹال کریں

```bash
# vision_msgs انسٹال کریں
sudo apt install ros-humble-vision-msgs

# OpenCV DNN کی تصدیق کریں
python3 -c "import cv2; print(cv2.dnn)"
```

### 2. ماڈل فائلز ڈاؤن لوڈ کریں

```bash
cd starter/models

# YOLOv5s ONNX ڈاؤن لوڈ کریں (یا فراہم کردہ استعمال کریں)
wget https://github.com/ultralytics/yolov5/releases/download/v7.0/yolov5s.onnx

# COCO کلاس نام ڈاؤن لوڈ کریں
wget https://raw.githubusercontent.com/pjreddie/darknet/master/data/coco.names
```

### 3. لیب ڈائریکٹری میں جائیں

```bash
cd ~/ros2_ws/src/labs/module-03/lab-04-object-detection
```

---

## لیب اسٹیپس

### اسٹیپ 1: YOLO ماڈل لوڈ کریں (20 منٹ)

TODO 1 مکمل کریں - YOLO ماڈل لوڈ کریں:

```python
# ONNX ماڈل لوڈ کریں
self.net = cv2.dnn.readNetFromONNX('models/yolov5s.onnx')

# بیک اینڈ سیٹ کریں (CPU یا CUDA)
self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

# کلاس نام لوڈ کریں
with open('models/coco.names', 'r') as f:
    self.classes = [line.strip() for line in f]
```

### اسٹیپ 2: امیج پری پروسیس کریں (20 منٹ)

TODO 2 مکمل کریں - YOLO کے لیے امیج تیار کریں:

```python
def preprocess(self, image):
    # ری سائز اور نارملائز کریں
    blob = cv2.dnn.blobFromImage(
        image,
        scalefactor=1/255.0,
        size=(640, 640),
        swapRB=True,
        crop=False
    )
    return blob
```

### اسٹیپ 3: ماڈل آؤٹ پٹ پروسیس کریں (30 منٹ)

TODO 3 مکمل کریں - YOLO آؤٹ پٹس کو پارس کریں:

```python
def postprocess(self, outputs, image_shape, conf_threshold=0.5):
    h, w = image_shape[:2]
    boxes, confidences, class_ids = [], [], []

    for detection in outputs[0]:
        scores = detection[5:]
        class_id = np.argmax(scores)
        confidence = scores[class_id]

        if confidence > conf_threshold:
            # YOLO آؤٹ پٹس سینٹر_x، سینٹر_y، چوڑائی، اونچائی
            cx, cy, bw, bh = detection[0:4]

            # کونر فارمیٹ میں تبدیل کریں
            x = int((cx - bw/2) * w)
            y = int((cy - bh/2) * h)
            width = int(bw * w)
            height = int(bh * h)

            boxes.append([x, y, width, height])
            confidences.append(float(confidence))
            class_ids.append(class_id)

    return boxes, confidences, class_ids
```

### اسٹیپ 4: غیر زیادہ سے زیادہ دباؤ لاگو کریں (20 منٹ)

TODO 4 مکمل کریں - ڈوپلیکیٹ ڈیٹیکشنز ہٹائیں:

```python
def apply_nms(self, boxes, confidences, nms_threshold=0.4):
    indices = cv2.dnn.NMSBoxes(
        boxes, confidences,
        score_threshold=0.5,
        nms_threshold=nms_threshold
    )
    return indices.flatten() if len(indices) > 0 else []
```

### اسٹیپ 5: ڈیٹیکشنز شائع کریں (20 منٹ)

TODO 5 مکمل کریں - ROS 2 ڈیٹیکشن میسجز بنائیں:

```python
from vision_msgs.msg import Detection2DArray, Detection2D
from vision_msgs.msg import ObjectHypothesisWithPose

def create_detection_msg(self, boxes, confidences, class_ids, indices, header):
    det_array = Detection2DArray()
    det_array.header = header

    for i in indices:
        det = Detection2D()
        det.header = header

        # باؤنڈنگ باکس سیٹ کریں
        det.bbox.center.position.x = boxes[i][0] + boxes[i][2]/2
        det.bbox.center.position.y = boxes[i][1] + boxes[i][3]/2
        det.bbox.size_x = float(boxes[i][2])
        det.bbox.size_y = float(boxes[i][3])

        # کلاسیفکیشن نتیجہ سیٹ کریں
        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = self.classes[class_ids[i]]
        hyp.hypothesis.score = confidences[i]
        det.results.append(hyp)

        det_array.detections.append(det)

    return det_array
```

### اسٹیپ 6: تصدیق (10 منٹ)

```bash
python3 starter/object_detector.py
python3 validate.py
```

---

## ڈلیوریبلز

1. **کام کرتا ہوا کوڈ**: مکمل `starter/object_detector.py`
2. **ڈیٹیکشن آؤٹ پٹ**: امیج پر باؤنڈنگ باکسز دکھانے والا اسکرین شاٹ
3. **ریفلیکشن جوابات**: ذیل کے سوالات مکمل کریں

---

## ریفلیکشن سوالات

1. **ہم غیر زیادہ سے زیادہ دباؤ کیوں استعمال کرتے ہیں؟**
   - غور کریں: اوور لیپنگ پریڈکشنز، ڈوپلیکیٹ ڈیٹیکشنز

2. **یقین کی حد اور یاد دہانی کے درمیان کیا توازن ہے؟**
   - سوچیں: جھوٹے مثبت، چھوٹی ڈیٹیکشنز

3. **آپ ایج ڈیپلائمنٹ کے لیے کیسے بہتر کریں گے؟**
   - غور کریں: ماڈل سائز، کوانٹائزیشن، ہارڈ ویئر ایکسلریشن

4. **YOLO کورنرز کے بجائے سینٹر کوآرڈینیٹس کیوں آؤٹ پٹ کرتا ہے؟**
   - سوچیں: اینکر باکسز، پریڈکشن استحکام

---

## تصدیق

```bash
python3 validate.py
```

متوقع آؤٹ پٹ:

```
[PASS] object_detector.py موجود ہے
[PASS] درست پائی تھن نحو
[PASS] YOLO ماڈل لوڈنگ موجود ہے
[PASS] پری پروسیسنگ نافذ کی گئی
[PASS] NMS لاگو کیا گیا
[PASS] Detection2DArray استعمال کیا گیا
لیب 4 مکمل!
```

---

## اگلے اقدامات

- **لیب 5**: پرچیپشن انضمام - ڈیٹیکشن کو روبوٹ کنٹرول سے جوڑیں
- **موڈیول 4**: موشن پلاننگ اور کنٹرول