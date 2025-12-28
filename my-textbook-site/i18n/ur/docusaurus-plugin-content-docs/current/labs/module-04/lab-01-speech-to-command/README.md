# لیب 1: سpeech سے کمانڈ - بولی گئی زبان کو روبوٹ کاموں میں تبدیل کرنا

**موڈیول**: 4 - وژن-لینگویج-ایکشن (VLA) سسٹم
**مدت**: 90 منٹ
**پریشانی**: متوسط

## اہداف

ایک سpeech ریکوگنیشن پائپ لائن بنائیں جو بولی گئی انسانی کمانڈز کو منظم روبوٹ کاموں میں تبدیل کرتی ہے. آپ سpeech-to-text تبدیلی کے لیے OpenAI کا وِسپر استعمال کریں گے اور کارروائی کے قابل روبوٹ ہدایات نکالنے کے لیے کمانڈ پارسنگ نافذ کریں گے.

## ضروریات

- موڈیول 1-3 مکمل (ROS 2 بنیادیات، سیمولیشن، پرچیپشن)
- ROS 2 ہمبل کے ساتھ اوبنٹو 22.04 انسٹال
- پائی تھن 3.10+
- مائیکروفون یا آڈیو ان پٹ ڈیوائس (یا پہلے سے ریکارڈ کردہ آڈیو فائلز)

## سیکھنے کے نتائج

اس لیب کو مکمل کرنے کے بعد، آپ:

1. VLA سسٹم میں سpeech ریکوگنیشن کے کردار کو سمجھیں گے
2. سpeech-to-text تبدیلی کے لیے وِسپر ASR (آٹومیٹک سpeech ریکوگنیشن) استعمال کریں گے
3. منظم روبوٹ کاموں میں قدرتی زبان کی کمانڈز کو پارس کریں گے
4. غیر واضح یا نامعلوم کمانڈز کو مناسب طریقے سے ہینڈل کریں گے
5. ROS 2 میسج کے ساتھ سpeech ان پٹ کو انضمام کریں گے

---

## سیٹ اپ

### 1. انحصار انسٹال کریں

```bash
# وِسپر انسٹال کریں
pip install openai-whisper

# آڈیو پروسیسنگ لائبریریز انسٹال کریں
pip install sounddevice numpy scipy

# سpeech ریکوگنیشن یوٹیلیٹیز انسٹال کریں
pip install SpeechRecognition pyaudio

# انسٹالیشن کی تصدیق کریں
python3 -c "import whisper; print('Whisper installed successfully')"
```

### 2. لیب ڈائریکٹری میں جائیں

```bash
cd ~/ros2_ws/src
cp -r /path/to/labs/module-04/lab-01-speech-to-command .
cd lab-01-speech-to-command
```

### 3. وِسپر ماڈل ڈاؤن لوڈ کریں

```bash
# بیس ماڈل ڈاؤن لوڈ کریں (پہلی چلائش خود بخود ڈاؤن لوڈ کرے گی)
python3 -c "import whisper; whisper.load_model('base')"
```

---

## لیب اسٹیپس

### اسٹیپ 1: سpeech-to-کمانڈ پائپ لائن کو سمجھیں (15 منٹ)

`starter/speech_to_command.py` کھولیں اور اس کی ساخت کا جائزہ لیں:

```bash
cat starter/speech_to_command.py
```

پائپ لائن تین اسٹیجز پر مشتمل ہے:

1. **آڈیو کیپچر**: آڈیو ان پٹ ریکارڈ یا لوڈ کریں
2. **سpeech ریکوگنیشن**: وِسپر کا استعمال کرتے ہوئے آڈیو کو متن میں تبدیل کریں
3. **کمانڈ پارسنگ**: متن سے منظم کمانڈز نکالیں

**آپ کا کام**: پائپ لائن کو کام کرتا بنانے کے لیے ٹوڈو مکمل کریں.

### اسٹیپ 2: آڈیو کیپچر نافذ کریں (15 منٹ)

مائیکروفون سے ریکارڈ کرنے کے لیے آڈیو کیپچر فنکشن مکمل کریں:

```python
# TODO 1: آڈیو ریکارڈنگ نافذ کریں
def record_audio(duration: float = 5.0, sample_rate: int = 16000) -> np.ndarray:
    """مخصوص مدت کے لیے مائیکروفون سے آڈیو ریکارڈ کریں."""
    print(f"Recording for {duration} seconds...")
    audio = sd.rec(int(duration * sample_rate), samplerate=sample_rate, channels=1)
    sd.wait()  # ریکارڈنگ ختم ہونے تک انتظار کریں
    return audio.flatten()
```

اپنی ریکارڈنگ ٹیسٹ کریں:

```bash
python3 starter/speech_to_command.py --test-audio
```

### اسٹیپ 3: وِسپر ٹرانسکرپشن نافذ کریں (20 منٹ)

آڈیو کو متن میں تبدیل کرنے کے لیے وِسپر استعمال کریں:

```python
# TODO 2: سpeech-to-text تبدیلی نافذ کریں
def transcribe_audio(audio: np.ndarray, model: whisper.Whisper) -> str:
    """وِسپر ماڈل کا استعمال کرتے ہوئے آڈیو کو ٹرانسکرائیب کریں."""
    # وِسپر [-1, 1] تک نارملائزڈ کیے گئے فلوٹ32 آڈیو کی توقع کرتا ہے
    audio = audio.astype(np.float32)
    if audio.max() > 1.0:
        audio = audio / 32768.0  # اگر int16 ہو تو نارملائز کریں
    result = model.transcribe(audio, language='en')
    return result['text'].strip()
```

ٹرانسکرپشن ٹیسٹ کریں:

```bash
# ریکارڈ اور ٹرانسکرائیب کریں
python3 starter/speech_to_command.py --test-transcribe
```

### اسٹیپ 4: کمانڈ پارسنگ نافذ کریں (25 منٹ)

قدرتی زبان کو منظم روبوٹ کمانڈز میں پارس کریں:

```python
# TODO 3: کمانڈ پارسنگ نافذ کریں
class RobotCommand:
    def __init__(self, action: str, target: str = None, location: str = None, parameters: dict = None):
        self.action = action
        self.target = target
        self.location = location
        self.parameters = parameters or {}

def parse_command(text: str) -> RobotCommand:
    """ٹرانسکرائیبڈ متن کو منظم روبوٹ کمانڈ میں پارس کریں."""
    text = text.lower().strip()

    # ایکشن کی ورڈز کی وضاحت کریں
    action_patterns = {
        'pick': ['pick up', 'pick', 'grab', 'grasp', 'take'],
        'place': ['place', 'put', 'set', 'drop'],
        'move': ['move', 'go', 'navigate', 'drive'],
        'look': ['look', 'find', 'search', 'locate'],
        'stop': ['stop', 'halt', 'pause', 'wait']
    }

    # ایکشن نکالیں
    action = 'unknown'
    for cmd, patterns in action_patterns.items():
        if any(p in text for p in patterns):
            action = cmd
            break

    # ہدف آبجیکٹ نکالیں (مثلاً "the red ball")
    target = extract_object(text)

    # جگہ نکالیں (مثلاً "on the table")
    location = extract_location(text)

    return RobotCommand(action=action, target=target, location=location)
```

کمانڈ پارسنگ ٹیسٹ کریں:

```bash
python3 starter/speech_to_command.py --test-parse "pick up the red ball from the table"
```

### اسٹیپ 5: ROS 2 انضمام بنائیں (10 منٹ)

ROS 2 میں پارسڈ کمانڈز شائع کریں:

```python
# TODO 4: ROS 2 پبلشر نافذ کریں
class SpeechCommandNode(Node):
    def __init__(self):
        super().__init__('speech_command_node')
        self.command_pub = self.create_publisher(String, '/robot/command', 10)
        self.model = whisper.load_model('base')
        self.timer = self.create_timer(5.0, self.listen_callback)

    def listen_callback(self):
        """کمانڈز کے لیے مسلسل سنیں."""
        audio = record_audio(duration=3.0)
        text = transcribe_audio(audio, self.model)

        if text:
            command = parse_command(text)
            msg = String()
            msg.data = json.dumps({
                'action': command.action,
                'target': command.target,
                'location': command.location
            })
            self.command_pub.publish(msg)
            self.get_logger().info(f'Published command: {command.action}')
```

### اسٹیپ 6: اینڈ-ٹو-اینڈ پائپ لائن ٹیسٹ کریں (5 منٹ)

مکمل پائپ لائن چلائیں:

```bash
# ٹرمنل 1: سpeech کمانڈ نوڈ چلائیں
ros2 run speech_to_command speech_command_node

# ٹرمنل 2: شائع شدہ کمانڈز مانیٹر کریں
ros2 topic echo /robot/command
```

---

## ڈلیوریبلز

1. **کام کرتی ہوئی پائپ لائن**: `starter/speech_to_command.py` جو:

   - مائیکروفون سے آڈیو ریکارڈ کرتا ہے
   - وِسپر کا استعمال کرتے ہوئے سpeech کو ٹرانسکرائیب کرتا ہے
   - کمانڈز کو منظم فارمیٹ میں پارس کرتا ہے
   - ROS 2 ٹاپک پر شائع کرتا ہے

2. **ٹیسٹ کے نتائج**: کامیاب کمانڈ پارسنگ دکھانے والی اسکرین شاٹ

3. **جوابات**: ذیل کے ریفلیکشن سوالات مکمل کریں

---

## ریفلیکشن سوالات

1. **روبوٹکس کے لیے روایتی سpeech ریکوگنیشن کے مقابلے میں وِسپر کیوں ترجیح دی جاتی ہے؟**
   - غور کریں: نوائز کے خلاف مضبوطی، زبان کی معاونت، درستی

2. **آپ "put that over there" جیسے غیر واضح کمانڈز کو کیسے ہینڈل کریں گے؟**
   - سوچیں: سیاق و سباق ٹریکنگ، وضاحت کی درخواستیں، ڈیفالٹ رویے

3. **ریل ٹائم روبوٹکس میں وِسپر استعمال کرنے کے لیٹنسی کے نتائج کیا ہیں؟**
   - غور کریں: ماڈل سائز بمقابلہ سپیڈ ٹریڈ آف، اسٹریمنگ متبادل

4. **آپ کئی زبانوں کی معاونت کے لیے اس سسٹم کو کیسے بڑھائیں گے؟**
   - سوچیں: ماڈل کی صلاحیتیں، کمانڈ پارسنگ میں تبدیلیاں

---

## توثیق

اپنی لیب کے مکمل ہونے کی تصدیق کے لیے توثیق اسکرپٹ چلائیں:

```bash
python3 validate.py
```

متوقع آؤٹ پٹ:

```
[PASS] starter/ میں speech_to_command.py موجود ہے
[PASS] درست پائی تھن نحو
[PASS] وِسپر امپورٹ موجود ہے
[PASS] آڈیو ریکارڈنگ فنکشن نافذ کیا گیا
[PASS] ٹرانسکرپشن فنکشن نافذ کیا گیا
[PASS] کمانڈ پارسنگ فنکشن نافذ کیا گیا
[PASS] ROS 2 نوڈ ساخت درست ہے
لیب 1 مکمل!
```

---

## مسئلہ حل

### کوئی آڈیو ان پٹ نہیں ملا

```bash
# آڈیو ڈیوائسز کو فہرست کریں
python3 -c "import sounddevice as sd; print(sd.query_devices())"

# ڈیفالٹ ان پٹ ڈیوائس سیٹ کریں
export SD_INPUT_DEVICE=0
```

### وِسپر ماڈل ڈاؤن لوڈ ناکام

```bash
# دستی ڈاؤن لوڈ
wget https://openaipublic.azureedge.net/main/whisper/models/base.pt -P ~/.cache/whisper/
```

### ٹرانسکرپشن کی درستی کم

- بہتر درستی کے لیے 'small' یا 'medium' ماڈل استعمال کریں
- کم بیک گراؤنڈ نوائز کے ساتھ صاف آڈیو کو یقینی بنائیں
- صاف اور معتدل رفتار سے بولیں

---

## توسیع چیلنجز

1. **کنفیڈنس سکورنگ**: پارسڈ کمانڈز میں کنفیڈنس اسکور شامل کریں
2. **سیاق و سباق کا ادراک**: ضمیر کے حل کے لیے گفتگو کا سیاق و سباق ٹریک کریں
3. **ملٹی-کمانڈ**: "pick up the ball and put it on the table" جیسے مرکب کمانڈز ہینڈل کریں
4. **اسٹریمنگ**: کم لیٹنسی کے لیے ریل ٹائم اسٹریمنگ ٹرانسکرپشن نافذ کریں

---

## اگلے اقدامات

اس لیب کو مکمل کرنے کے بعد، آگے بڑھیں:

- **لیب 2**: LLM کوگنیٹو پلاننگ - پیچیدہ کاموں کو تقسیم کرنے کے لیے LLMs استعمال کریں
- **لیسن 2**: VLA اجزاء - مکمل VLA آرکیٹیکچر کو سمجھیں