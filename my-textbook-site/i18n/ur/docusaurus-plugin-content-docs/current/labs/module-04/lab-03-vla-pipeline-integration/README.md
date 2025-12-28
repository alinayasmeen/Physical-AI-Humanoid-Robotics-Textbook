# لیب 3: VLA پائپ لائن انضمام - ایک اینڈ-ٹو-اینڈ سسٹم بنانا

**موڈیول**: 4 - وژن-زبان-ایکشن (VLA) سسٹم
**مدت**: 120 منٹ
**پریشانی**: اعلی

## اہداف

تمام VLA اجزاء (وژن، زبان، ایکشن) کو ایک مکمل اینڈ-ٹو-اینڈ پائپ لائن میں ضم کریں. آپ سپیچ ریکوگنیشن، کوگنیٹو پلاننگ، وژن پروسیسنگ، اور ROS 2 ایکشن ایکسیکشن کو جوڑیں گے تاکہ ایک مکمل طور پر کام کرنے والا VLA سسٹم بن سکے جو بولی گئی کمانڈز وصول کر سکے اور روبوٹ پر انہیں انجام دے سکے.

## ضروریات

- لیب 1 (سپیچ سے کمانڈ) مکمل
- لیب 2 (LLM کوگنیٹو پلاننگ) مکمل
- موڈیول 3 (پرچیپشن اور سینسر) مکمل
- ROS 2 ہمبل کے ساتھ اوبنٹو 22.04 انسٹال
- پائی تھن 3.10+
- گیزبو سیمولیشن ماحول کنفیگر کیا گیا

## سیکھنے کے نتائج

اس لیب کو مکمل کرنے کے بعد، آپ:

1. متعدد VLA اجزاء کو ایک متحدہ پائپ لائن میں ضم کریں گے
2. پیچیدہ روبوٹک ورک فلو کے لیے اسٹیٹ مینجمنٹ نافذ کریں گے
3. جزو کی ناکامیوں کو ہینڈل کریں گے اور باوقار ڈیگریڈیشن نافذ کریں گے
4. ملٹی موڈل سسٹم کے لیے ڈیبگنگ کے لیے وژولائزیشنز بنائیں گے
5. سیمولیشن میں اینڈ-ٹو-اینڈ VLA فعالیت ٹیسٹ کریں گے

---

## سیٹ اپ

### 1. انحصار انسٹال کریں

```bash
# یقینی بنائیں کہ تمام پچھلی لیب کے انحصار انسٹال ہیں
pip install openai-whisper openai sounddevice numpy scipy

# وژولائزیشن ٹولز انسٹال کریں
pip install matplotlib plotly

# ROS 2 ایکشن انحصار انسٹال کریں
sudo apt install ros-humble-action-tutorials-interfaces

# سیٹ اپ کی تصدیق کریں
python3 -c "import whisper, openai; print('VLA dependencies ready')"
```

### 2. لیب ڈائریکٹری میں جائیں

```bash
cd ~/ros2_ws/src
cp -r /path/to/labs/module-04/lab-03-vla-pipeline-integration .
cd lab-03-vla-pipeline-integration
```

### 3. سیمولیشن ماحول لانچ کریں

```bash
# ٹرمنل 1: روبوٹ کے ساتھ گیزبو لانچ کریں
ros2 launch gazebo_ros gazebo.launch.py world:=/path/to/manipulation_world.sdf

# ٹرمنل 2: بازو اور کیمرہ والے روبوٹ کو اسپون کریں
ros2 run gazebo_ros spawn_entity.py -entity vla_robot -file robot.urdf
```

---

## لیب اسٹیپس

### اسٹیپ 1: VLA پائپ لائن معماری کو سمجھیں (15 منٹ)

`starter/vla_pipeline.py` کھولیں اور مکمل معماری کا جائزہ لیں:

```bash
cat starter/vla_pipeline.py
```

VLA پائپ لائن میں ہے:

```
آڈیو ان پٹ → سپیچ ریکوگنیشن → کمانڈ پارسر → کوگنیٹو پلینر
                                                          ↓
                                                    پلان والیڈیٹر
                                                          ↓
وژن سسٹم → منظر کی سمجھ → ایکشن ایکزیکیوٹر → روبوٹ ایکشنز
```

**آپ کا کام**: تمام اجزاء کو مربوط کرنے والے پائپ لائن آرکیسٹریٹر نافذ کریں.

### اسٹیپ 2: پائپ لائن اسٹیٹ مشین نافذ کریں (25 منٹ)

پائپ لائن ایکسیکشن کے انتظام کے لیے اسٹیٹ مشین بنائیں:

```python
# TODO 1: VLA پائپ لائن اسٹیٹ مشین نافذ کریں
from enum import Enum, auto

class PipelineState(Enum):
    IDLE = auto()
    LISTENING = auto()
    TRANSCRIBING = auto()
    PARSING = auto()
    PLANNING = auto()
    VALIDATING = auto()
    PERCEIVING = auto()
    EXECUTING = auto()
    RECOVERING = auto()
    COMPLETED = auto()
    FAILED = auto()

class VLAPipeline:
    def __init__(self):
        self.state = PipelineState.IDLE
        self.speech_recognizer = SpeechRecognizer()
        self.cognitive_planner = CognitivePlanner()
        self.vision_system = VisionSystem()
        self.action_executor = ActionExecutor()
        self.current_plan = None
        self.current_step = 0

    def transition(self, new_state: PipelineState):
        """نئی حالت میں منتقل ہوں لاگ کے ساتھ."""
        self.get_logger().info(f'State: {self.state.name} -> {new_state.name}')
        self.state = new_state
        self._publish_state()

    async def run(self):
        """مرکزی پائپ لائن ایکسیکشن لوپ."""
        while True:
            if self.state == PipelineState.IDLE:
                await self._wait_for_trigger()
            elif self.state == PipelineState.LISTENING:
                await self._listen_for_command()
            elif self.state == PipelineState.TRANSCRIBING:
                await self._transcribe_audio()
            elif self.state == PipelineState.PARSING:
                await self._parse_command()
            elif self.state == PipelineState.PLANNING:
                await self._generate_plan()
            elif self.state == PipelineState.VALIDATING:
                await self._validate_plan()
            elif self.state == PipelineState.PERCEIVING:
                await self._update_perception()
            elif self.state == PipelineState.EXECUTING:
                await self._execute_step()
            elif self.state == PipelineState.RECOVERING:
                await self._handle_recovery()
            elif self.state == PipelineState.COMPLETED:
                await self._complete_task()
            elif self.state == PipelineState.FAILED:
                await self._handle_failure()
```

اسٹیٹ مشین ٹیسٹ کریں:

```bash
python3 starter/vla_pipeline.py --test-states
```

### اسٹیپ 3: سپیچ ریکوگنیشن جزو ضم کریں (15 منٹ)

لیب 1 سے سپیچ ریکوگنیشن کو جوڑیں:

```python
# TODO 2: سپیچ ریکوگنیشن ضم کریں
async def _listen_for_command(self):
    """کمانڈ کے لیے سنیں."""
    self.get_logger().info('Listening for command...')

    try:
        audio = await self.speech_recognizer.record_async(duration=5.0)
        self.audio_buffer = audio
        self.transition(PipelineState.TRANSCRIBING)
    except AudioCaptureError as e:
        self.get_logger().error(f'Audio capture failed: {e}')
        self.transition(PipelineState.FAILED)

async def _transcribe_audio(self):
    """ریکارڈ کردہ آڈیو کو ٹرانسکرائیب کریں."""
    self.get_logger().info('Transcribing audio...')

    try:
        text = await self.speech_recognizer.transcribe_async(self.audio_buffer)
        self.transcribed_text = text
        self.get_logger().info(f'Transcribed: "{text}"')
        self.transition(PipelineState.PARSING)
    except TranscriptionError as e:
        self.get_logger().error(f'Transcription failed: {e}')
        self.transition(PipelineState.FAILED)
```

### اسٹیپ 4: کوگنیٹو پلاننگ جزو ضم کریں (15 منٹ)

لیب 2 سے کوگنیٹو پلینر کو جوڑیں:

```python
# TODO 3: کوگنیٹو پلاننگ ضم کریں
async def _generate_plan(self):
    """LLM کا استعمال کرتے ہوئے ایکشن پلان جنریٹ کریں."""
    self.get_logger().info('Generating plan...')

    # وژن سے موجودہ ماحول کی حالت حاصل کریں
    environment = await self.vision_system.get_scene_state()
    robot_state = await self.get_robot_state()

    try:
        self.current_plan = await self.cognitive_planner.generate_plan_async(
            task=self.parsed_command,
            environment=environment,
            robot_state=robot_state
        )
        self.current_step = 0
        self.get_logger().info(f'Plan generated with {len(self.current_plan["steps"])} steps')
        self.transition(PipelineState.VALIDATING)
    except PlanningError as e:
        self.get_logger().error(f'Planning failed: {e}')
        self.transition(PipelineState.FAILED)
```

### اسٹیپ 5: وژن سسٹم ضم کریں (15 منٹ)

ماحول کی آگاہی کے لیے وژن پروسیسنگ کو جوڑیں:

```python
# TODO 4: وژن سسٹم ضم کریں
async def _update_perception(self):
    """ایکشن ایکسیکشن سے پہلے ادراک کو اپ ڈیٹ کریں."""
    self.get_logger().info('Updating perception...')

    try:
        # تازہ ترین کیمرہ فریم حاصل کریں
        image = await self.vision_system.get_latest_frame()

        # منظر میں اشیاء ڈیٹیکٹ کریں
        detections = await self.vision_system.detect_objects(image)

        # ماحول کا ماڈل اپ ڈیٹ کریں
        self.environment_state = {
            'objects': [
                {
                    'label': det.label,
                    'position': det.position.tolist(),
                    'confidence': det.confidence
                }
                for det in detections
            ],
            'timestamp': time.time()
        }

        # چیک کریں کہ کیا موجودہ ایکشن ٹارگٹ نظر آ رہا ہے
        current_step = self.current_plan['steps'][self.current_step]
        if 'object' in current_step.get('parameters', {}):
            target = current_step['parameters']['object']
            if not self._is_object_visible(target):
                self.get_logger().warning(f'Target object "{target}" not visible')
                self.transition(PipelineState.RECOVERING)
                return

        self.transition(PipelineState.EXECUTING)
    except VisionError as e:
        self.get_logger().error(f'Vision update failed: {e}')
        self.transition(PipelineState.RECOVERING)
```

### اسٹیپ 6: ایکشن ایکسیکشن نافذ کریں (20 منٹ)

ROS 2 کے ذریعے منصوبہ بند ایکشنز انجام دیں:

```python
# TODO 5: ایکشن ایکسیکشن نافذ کریں
async def _execute_step(self):
    """موجودہ پلان اسٹیپ انجام دیں."""
    step = self.current_plan['steps'][self.current_step]
    action = step['action']
    params = step.get('parameters', {})

    self.get_logger().info(f'Executing step {self.current_step + 1}: {action}')

    try:
        # ایکشن کو ROS 2 ایکشن کلائنٹ میں میپ کریں
        if action == 'navigate':
            result = await self.action_executor.navigate(params['location'])
        elif action == 'pick':
            result = await self.action_executor.pick(params['object'])
        elif action == 'place':
            result = await self.action_executor.place(params['object'], params['location'])
        elif action == 'look':
            result = await self.action_executor.look(params['direction'])
        elif action == 'speak':
            result = await self.action_executor.speak(params['message'])
        elif action == 'wait':
            await asyncio.sleep(params['seconds'])
            result = True
        else:
            self.get_logger().warning(f'Unknown action: {action}')
            result = False

        if result:
            self.current_step += 1
            if self.current_step >= len(self.current_plan['steps']):
                self.transition(PipelineState.COMPLETED)
            else:
                # اگلے اسٹیپ سے پہلے ادراک کو اپ ڈیٹ کریں
                self.transition(PipelineState.PERCEIVING)
        else:
            self.get_logger().error(f'Action {action} failed')
            self.transition(PipelineState.RECOVERING)

    except ActionExecutionError as e:
        self.get_logger().error(f'Execution error: {e}')
        self.transition(PipelineState.RECOVERING)
```

### اسٹیپ 7: خامی کی بازیابی نافذ کریں (10 منٹ)

ناکامیوں کو ہینڈل کریں اور بازیابی کی حکمت عملیاں نافذ کریں:

```python
# TODO 6: خامی کی بازیابی نافذ کریں
async def _handle_recovery(self):
    """ناکامیوں سے بازیابی کی کوشش کریں."""
    self.get_logger().info('Attempting recovery...')

    # موجودہ ناکامی کا سیاق و سباق حاصل کریں
    failed_step = self.current_step
    failure_reason = self.last_error

    # موجودہ حالت سے دوبارہ پلاننگ کی کوشش کریں
    try:
        new_plan = await self.cognitive_planner.replan_on_failure(
            original_task=self.parsed_command,
            failed_step=failed_step,
            failure_reason=failure_reason,
            environment=self.environment_state
        )

        if new_plan and len(new_plan['steps']) > 0:
            self.current_plan = new_plan
            self.current_step = 0
            self.get_logger().info('Re-planning successful, resuming execution')
            self.transition(PipelineState.PERCEIVING)
        else:
            self.get_logger().error('Re-planning failed, no viable plan')
            self.transition(PipelineState.FAILED)

    except Exception as e:
        self.get_logger().error(f'Recovery failed: {e}')
        self.transition(PipelineState.FAILED)
```

### اسٹیپ 8: وژولائزیشن اور ڈیبگنگ شامل کریں (5 منٹ)

پائپ لائن کی حالت کے لیے وژولائزیشن بنائیں:

```python
# TODO 7: وژولائزیشن نافذ کریں
def visualize_pipeline_state(self):
    """ڈیبگنگ کے لیے وژولائزیشن مارکرز شائع کریں."""
    # /vla/state ٹاپک پر حالت شائع کریں
    state_msg = String()
    state_msg.data = json.dumps({
        'state': self.state.name,
        'current_step': self.current_step,
        'total_steps': len(self.current_plan['steps']) if self.current_plan else 0,
        'transcribed_text': self.transcribed_text,
        'environment': self.environment_state
    })
    self.state_pub.publish(state_msg)

    # ڈیٹیکٹ کردہ اشیاء کے لیے وژول مارکرز شائع کریں
    for obj in self.environment_state.get('objects', []):
        marker = self._create_object_marker(obj)
        self.marker_pub.publish(marker)
```

---

## ڈلیوریبلز

1. **کام کرتا ہوا پائپ لائن**: `starter/vla_pipeline.py` جو:

   - بولی گئی کمانڈز وصول کرتا ہے
   - LLM کا استعمال کرتے ہوئے پلانز جنریٹ کرتا ہے
   - ایکشنز سے پہلے ادراک کو اپ ڈیٹ کرتا ہے
   - روبوٹ پر ایکشنز انجام دیتا ہے
   - ناکامیوں سے بازیابی کرتا ہے

2. **ڈیمو ویڈیو**: کمانڈ کے اینڈ-ٹو-اینڈ ایکسیکشن کو دکھانے والی ریکارڈنگ

3. **سسٹم ڈائیاگرام**: آپ کے نفاذ کے ساتھ اپ ڈیٹ کردہ ڈیٹا فلو ڈائیاگرام

4. **جوابات**: ذیل کے ریفلیکشن سوالات مکمل کریں

---

## ریفلیکشن سوالات

1. **ہر ایکشن سے پہلے ادراک کی اپ ڈیٹ کیوں اہم ہے؟**
   - غور کریں: ماحولیاتی تبدیلیاں، چیز کی حرکت، عدم یقینی

2. **آپ کون سی جزو کی ناکامیوں کو بازیابی کے لیے ترجیح دیں گے؟**
   - سوچیں: حفاظت، کام کی اہمیت، بازیابی کی لاگت

3. **آپ کے پائپ لائن میں رکاوٹیں کیا ہیں؟ آپ انہیں کیسے بہتر کریں گے؟**
   - غور کریں: لیٹنسی پیمائش، متوازی کارروائی کے مواقع

4. **آپ مستقل آپریشن (ہمیشہ سننے والا) کے لیے اس سسٹم کو کیسے وسعت دیں گے؟**
   - سوچیں: جاگنے کا لفظ ڈیٹیکشن، وسائل کا انتظام، رازداری

---

## تصدیق

توثیق اسکرپٹ چلائیں:

```bash
python3 validate.py
```

متوقع آؤٹ پٹ:

```
[PASS] starter/ میں vla_pipeline.py موجود ہے
[PASS] درست پائی تھن نحو
[PASS] اسٹیٹ مشین درست طریقے سے نافذ کی گئی
[PASS] سپیچ ریکوگنیشن ضم کیا گیا
[PASS] کوگنیٹو پلاننگ ضم کی گئی
[PASS] وژن سسٹم ضم کیا گیا
[PASS] ایکشن ایکسیکشن نافذ کیا گیا
[PASS] خامی کی بازیابی نافذ کی گئی
[PASS] ROS 2 نوڈ ساخت درست ہے
لیب 3 مکمل!
```

---

## مسئلہ حل

### پائپ لائن LISTENING اسٹیٹ میں پھنس جاتا ہے

```bash
# مائیکروفون اجازتوں کی تصدیق کریں
pactl list sources

# آڈیو ان پٹ کی تصدیق کریں
python3 -c "import sounddevice as sd; print(sd.query_devices())"
```

### ایکشنز انجام نہیں دیتے

```bash
# چیک کریں کہ ROS 2 ایکشن سرور چل رہے ہیں
ros2 action list

# یقینی بنائیں کہ روبوٹ کنٹرولر فعال ہے
ros2 node list | grep controller
```

### وژن ڈیٹیکشنز غلط ہیں

- کیمرہ کیلیبریشن چیک کریں
- سیمولیشن میں لائٹنگ کی حالت کی تصدیق کریں
- ڈیٹیکشن یقین کے تھریشولڈز ایڈجسٹ کریں

### LLM ریسپانس سست ہیں

- چھوٹے/تیز ماڈلز استعمال کریں (GPT-3.5-turbo یا مقامی Llama2)
- دہرائے گئے سوالات کے لیے ریسپانس کیش نافذ کریں
- ایج ڈیپلائمنٹ کے اختیارات پر غور کریں

---

## توسیع چیلنجز

1. **ملٹی-موڈل تصدیق**: اہم ایکشنز سے پہلے وژول تصدیق شامل کریں
2. **وائس فیڈ بیک**: ایکسیکشن سے پہلے روبوٹ کو اس کا پلان بولنے دیں
3. **اصلاحات سے سیکھنا**: صارف کی اصلاحات سے آن لائن سیکھنا نافذ کریں
4. **ملٹی-روبوٹ ہم آہنگی**: متعدد تعاون کرنے والے روبوٹس کے لیے پائپ لائن کو وسعت دیں

---

## اگلے اقدامات

اس لیب کو مکمل کرنے کے بعد، آگے بڑھیں:

- **مینی-پروجیکٹ 1**: سادہ VLA سسٹم - اپنا VLA ایپلیکیشن بنائیں
- **مینی-پروجیکٹ 2**: اعلی VLA سسٹم - پیچیدہ رویے نافذ کریں
- **موڈیول جائزہ**: تکمیل کا جائزہ اور سرٹیفکیشن