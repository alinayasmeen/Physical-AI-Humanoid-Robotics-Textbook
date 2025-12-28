# لیب 2: LLM کوگنیٹو پلاننگ - بڑے زبان کے ماڈلز کے ساتھ کام کی تقسیم

**موڈیول**: 4 - وژن-زبان-ایکشن (VLA) سسٹم
**مدت**: 90 منٹ
**پریشانی**: متوسط-اعلی

## اہداف

ایک کوگنیٹو پلاننگ سسٹم بنائیں جو پیچیدہ کاموں کو قابل عمل روبوٹ ایکشنز میں تقسیم کرنے کے لیے بڑے زبان کے ماڈلز (LLMs) کا استعمال کرتا ہے. آپ پروموٹ انجینئرنگ تکنیکس نافذ کریں گے اور اسٹرکچرڈ ایکشن سیکوئنس جنریٹ کرنے والی ایک پلاننگ پائپ لائن بنائیں گے.

## ضروریات

- لیب 1 (سپیچ سے کمانڈ) مکمل
- ROS 2 ہمبل کے ساتھ اوبنٹو 22.04 انسٹال
- پائی تھن 3.10+
- OpenAI API کلید یا مقامی LLM سیٹ اپ (Ollama تجویز کردہ)

## سیکھنے کے نتائج

اس لیب کو مکمل کرنے کے بعد، آپ:

1. روبوٹکس میں LLMs کا استعمال کرتے ہوئے کوگنیٹو پلاننگ کو سمجھیں گے
2. کام کی تقسیم کے لیے مؤثر پروموٹس ڈیزائن کریں گے
3. قدرتی زبان کے اہداف سے اسٹرکچرڈ ایکشن پلانز جنریٹ کریں گے
4. روبوٹ ایکسیکشن کے لیے جنریٹ کردہ پلانز کی توثیق اور تصدیق کریں گے
5. پلان کی ناکامیوں اور دوبارہ پلاننگ کے منظرناموں کو ہینڈل کریں گے

---

## سیٹ اپ

### 1. انحصار انسٹال کریں

```bash
# OpenAI کلائنٹ انسٹال کریں (API-بیسڈ LLMs کے لیے)
pip install openai

# یا مقامی LLMs کے لیے Ollama انسٹال کریں (آف لائن استعمال کے لیے تجویز کردہ)
curl -fsSL https://ollama.ai/install.sh | sh
ollama pull llama2

# اضافی انحصار انسٹال کریں
pip install langchain pydantic

# انسٹالیشن کی تصدیق کریں
python3 -c "import openai; print('OpenAI client installed')"
```

### 2. API رسائی کنفیگر کریں

```bash
# OpenAI API کے لیے
export OPENAI_API_KEY="your-api-key-here"

# یا مقامی Ollama کے لیے
ollama serve  # Ollama سرور کو پس منظر میں شروع کریں
```

### 3. لیب ڈائریکٹری میں جائیں

```bash
cd ~/ros2_ws/src
cp -r /path/to/labs/module-04/lab-02-llm-cognitive-planning .
cd lab-02-llm-cognitive-planning
```

---

## لیب اسٹیپس

### اسٹیپ 1: کوگنیٹو پلاننگ آرکیٹیکچر کو سمجھیں (15 منٹ)

`starter/cognitive_planner.py` کھولیں اور پلاننگ آرکیٹیکچر کا جائزہ لیں:

```bash
cat starter/cognitive_planner.py
```

کوگنیٹو پلینر میں ہے:

1. **کام کا تجزیہ**: ہائی لیول کا ہدف سمجھیں
2. **ماحول کا سیاق**: موجودہ روبوٹ کی حالت اور ماحول
3. **پلان جنریشن**: LLM-بیسڈ ایکشن سیکوئنس جنریشن
4. **پلان کی تصدیق**: قابل عمل ہونے اور سیفٹی کی تصدیق

**آپ کا کام**: کام کرتا ہوا کوگنیٹو پلینر بنانے کے لیے ٹوڈو مکمل کریں.

### اسٹیپ 2: پروموٹ ٹیمپلیٹ ڈیزائن کریں (20 منٹ)

کام کی تقسیم کے لیے ایک مؤثر پروموٹ بنائیں:

```python
# TODO 1: پلاننگ پروموٹ ٹیمپلیٹ ڈیزائن کریں
PLANNING_PROMPT = """
آپ ہیومنوائڈ روبوٹ کے لیے ایک کوگنیٹو پلینر ہیں. آپ کا کام ہائی لیول کمانڈز کو قابل عمل روبوٹ ایکشنز کی سیکوئنس میں تقسیم کرنا ہے.

## روبوٹ کی صلاحیتیں:
- navigate(location): مخصوص جگہ پر جائیں
- pick(object): گریپر کے ساتھ ایک چیز اٹھائیں
- place(object, location): ایک چیز کو ایک جگہ پر رکھیں
- look(direction): سر گھمائیں اور ایک سمت میں دیکھیں
- speak(message): اسپیکرز کے ذریعے کچھ کہیں
- wait(seconds): مخصوص مدت تک انتظار کریں

## موجودہ ماحول:
{environment_context}

## موجودہ روبوٹ کی حالت:
{robot_state}

## کام:
{task_description}

## ہدایات:
1. کام کو ایٹمک روبوٹ ایکشنز میں تقسیم کریں
2. موجودہ ماحول اور روبوٹ کی حالت پر غور کریں
3. مناسب جگہوں پر سیفٹی چیکس شامل کریں
4. ایکشنز کا JSON ارے آؤٹ پٹ کریں

## آؤٹ پٹ فارمیٹ:
```json
{{
    "plan_id": "unique_id",
    "goal": "original task",
    "steps": [
        {{"step": 1, "action": "action_name", "parameters": {{}}, "preconditions": [], "expected_outcome": ""}},
        ...
    ],
    "estimated_duration": "X seconds",
    "risk_assessment": "low/medium/high"
}}
```

پلان جنریٹ کریں:
"""
```

اپنے پروموٹ کو ٹیسٹ کریں:

```bash
python3 starter/cognitive_planner.py --test-prompt
```

### اسٹیپ 3: LLM انضمام نافذ کریں (20 منٹ)

پلان جنریشن کے لیے LLM سے منسلک ہوں:

```python
# TODO 2: LLM انضمام نافذ کریں
class CognitivePlanner:
    def __init__(self, model: str = "gpt-3.5-turbo"):
        self.client = openai.OpenAI()
        self.model = model

    def generate_plan(self, task: str, environment: dict, robot_state: dict) -> dict:
        """LLM کا استعمال کرتے ہوئے ایک ایکشن پلان جنریٹ کریں."""
        prompt = PLANNING_PROMPT.format(
            environment_context=json.dumps(environment, indent=2),
            robot_state=json.dumps(robot_state, indent=2),
            task_description=task
        )

        response = self.client.chat.completions.create(
            model=self.model,
            messages=[
                {"role": "system", "content": "You are a robot task planner."},
                {"role": "user", "content": prompt}
            ],
            temperature=0.2,  # مسلسل پلاننگ کے لیے کم درجہ حرارت
            max_tokens=1000
        )

        plan_text = response.choices[0].message.content
        return self._parse_plan(plan_text)

    def _parse_plan(self, plan_text: str) -> dict:
        """LLM ریسپانس کو اسٹرکچرڈ پلان میں پارس کریں."""
        # ریسپانس سے JSON نکالیں
        import re
        json_match = re.search(r'```json\s*(.*?)\s*```', plan_text, re.DOTALL)
        if json_match:
            return json.loads(json_match.group(1))
        return json.loads(plan_text)
```

پلان جنریشن ٹیسٹ کریں:

```bash
python3 starter/cognitive_planner.py --generate "Set the table for dinner"
```

### اسٹیپ 4: پلان کی تصدیق نافذ کریں (20 منٹ)

قابل عمل ہونے کے لیے جنریٹ کردہ پلانز کی تصدیق کریں:

```python
# TODO 3: پلان کی تصدیق نافذ کریں
class PlanValidator:
    def __init__(self, robot_capabilities: list, environment_objects: list):
        self.capabilities = robot_capabilities
        self.objects = environment_objects

    def validate_plan(self, plan: dict) -> tuple[bool, list]:
        """قابل عمل ہونے اور سیفٹی کے لیے ایک پلان کی تصدیق کریں."""
        errors = []

        for step in plan.get('steps', []):
            # چیک کریں کہ ایکشن روبوٹ کی صلاحیتوں میں ہے
            if step['action'] not in self.capabilities:
                errors.append(f"Unknown action: {step['action']}")

            # چیک کریں کہ چیز کے حوالے موجود ہیں
            params = step.get('parameters', {})
            if 'object' in params and params['object'] not in self.objects:
                errors.append(f"Unknown object: {params['object']}")

            # چیک کریں کہ پیش شرطیں پوری ہو سکتی ہیں
            for precond in step.get('preconditions', []):
                if not self._check_precondition(precond):
                    errors.append(f"Unsatisfiable precondition: {precond}")

        return len(errors) == 0, errors

    def _check_precondition(self, precondition: str) -> bool:
        """چیک کریں کہ ایک پیش شرط پوری کی جا سکتی ہے."""
        # سادہ چیک - عمل میں، علامتی پلاننگ استعمال کریں
        return True
```

تصدیق ٹیسٹ کریں:

```bash
python3 starter/cognitive_planner.py --validate-plan sample_plan.json
```

### اسٹیپ 5: دوبارہ پلاننگ نافذ کریں (10 منٹ)

ناکامیوں کے ساتھ دوبارہ پلاننگ کو ہینڈل کریں:

```python
# TODO 4: ناکامی پر دوبارہ پلاننگ نافذ کریں
def replan_on_failure(self, original_task: str, failed_step: int,
                       failure_reason: str, environment: dict) -> dict:
    """ایکسیکشن ناکام ہونے پر ایک نیا پلان جنریٹ کریں."""
    replan_prompt = f"""
    پچھلا پلان {failed_step} میں ناکام ہوا.
    ناکامی کی وجہ: {failure_reason}

    اصل کام: {original_task}

    براہ کرم ایک متبادل پلان جنریٹ کریں جو:
    1. ناکامی کے طریقہ کار سے بچے
    2. موجودہ ماحول کی حالت کو مدنظر رکھے
    3. اب بھی اصل ہدف کو حاصل کرے

    موجودہ ماحول: {json.dumps(environment)}
    """

    return self.generate_plan(replan_prompt, environment, self.get_robot_state())
```

### اسٹیپ 6: ROS 2 انضمام بنائیں (5 منٹ)

پلانز کو ROS 2 میسجز کے طور پر شائع کریں:

```python
# TODO 5: ROS 2 پلان پبلشر
class CognitivePlannerNode(Node):
    def __init__(self):
        super().__init__('cognitive_planner_node')
        self.planner = CognitivePlanner()
        self.plan_pub = self.create_publisher(String, '/robot/plan', 10)
        self.task_sub = self.create_subscription(
            String, '/robot/task', self.task_callback, 10)

    def task_callback(self, msg):
        """کام وصول کریں اور پلان جنریٹ کریں."""
        task = msg.data
        environment = self.get_environment_state()
        robot_state = self.get_robot_state()

        plan = self.planner.generate_plan(task, environment, robot_state)

        plan_msg = String()
        plan_msg.data = json.dumps(plan)
        self.plan_pub.publish(plan_msg)
        self.get_logger().info(f'Generated plan with {len(plan["steps"])} steps')
```

---

## ڈلیوریبلز

1. **کام کرتا ہوا پلینر**: `starter/cognitive_planner.py` جو:

   - قدرتی زبان کے کام کی تفصیلات لیتا ہے
   - LLM کا استعمال کرتے ہوئے اسٹرکچرڈ ایکشن پلانز جنریٹ کرتا ہے
   - قابل عمل ہونے کے لیے پلانز کی تصدیق کرتا ہے
   - ناکامی پر دوبارہ پلاننگ کو سپورٹ کرتا ہے

2. **نمونہ پلانز**: مختلف کاموں کے لیے کم از کم 3 جنریٹ کردہ پلانز

3. **جوابات**: ذیل کے ریفلیکشن سوالات مکمل کریں

---

## ریفلیکشن سوالات

1. **پلاننگ کے لیے کم درجہ حرارت (0.2) استعمال کریں اور تخلیقی کاموں کے مقابلے میں زیادہ کیوں؟**
   - غور کریں: مطابقت، دوبارہ حاصل کرنا، تعیناتی

2. **سیفٹی کے لحاظ سے اہم روبوٹ پلاننگ کے لیے LLMs استعمال کرنے کے کیا خطرات ہیں؟**
   - سوچیں: ہالوسینیشنز، غیر متوقع آؤٹ پٹس، تصدیق کی ضرورت

3. **آپ موجودہ روبوٹ کی صلاحیتوں کے دیئے گئے ناممکن کاموں کو کیسے ہینڈل کریں گے؟**
   - غور کریں: باوقار ناکامی، صارف فیڈ بیک، صلاحیت کی مذاکرات

4. **ملٹی-روبوٹ پلاننگ کے لیے کیا تبدیلیوں کی ضرورت ہوگی؟**
   - سوچیں: ہم آہنگی، وسائل کے تنازعات، رابطہ

---

## تصدیق

توثیق اسکرپٹ چلائیں:

```bash
python3 validate.py
```

متوقع آؤٹ پٹ:

```
[PASS] starter/ میں cognitive_planner.py موجود ہے
[PASS] درست پائی تھن نحو
[PASS] LLM کلائنٹ کنفیگر کیا گیا
[PASS] پروموٹ ٹیمپلیٹ تعریف شدہ
[PASS] پلان جنریشن فنکشن نافذ کیا گیا
[PASS] پلان کی تصدیق فنکشن نافذ کیا گیا
[PASS] دوبارہ پلاننگ فنکشن نافذ کیا گیا
لیب 2 مکمل!
```

---

## مسئلہ حل

### OpenAI API کی شرح کی حدیں

```bash
# مقامی Ollama کا استعمال کریں
export USE_LOCAL_LLM=true
ollama serve
```

### LLM ریسپانس میں غلط JSON

- پروموٹ میں JSON فارمیٹنگ کی ہدایات شامل کریں
- اگر دستیاب ہو تو JSON موڈ استعمال کریں (OpenAI GPT-4)
- فیل بیکس کے ساتھ مضبوط پارسنگ نافذ کریں

### پلانز بہت ورbose/سادہ ہیں

- پروموٹ کو تبدیل کریں تاکہ مطلوبہ گرانولیرٹی مخصوص کی جا سکے
- پروموٹ میں مزید مثالیں فراہم کریں
- درجہ حرارت اور زیادہ سے زیادہ ٹوکن فائن ٹیون کریں

---

## توسیع چیلنجز

1. **ہیئرآرکیکل پلاننگ**: ایبسٹریکٹ اور کنکریٹ ایکشنز کے ساتھ متعدد سطح کی پلاننگ نافذ کریں
2. **ٹیمپورل کنٹریکٹس**: پلانز میں وقت کی پابندیاں اور شیڈولنگ شامل کریں
3. **ملٹی-ایجنٹ پلاننگ**: متعدد روبوٹس کے ہم آہنگ بنانے تک توسیع دیں
4. **فیڈ بیک سے سیکھنا**: ایکسیکشن کے نتائج کی بنیاد پر پلاننگ میں بہتری

---

## اگلے اقدامات

اس لیب کو مکمل کرنے کے بعد، آگے بڑھیں:

- **لیب 3**: VLA پائپ لائن انضمام - تمام اجزاء کو مکمل سسٹم میں ضم کریں
- **مینی-پروجیکٹ 1**: سادہ VLA سسٹم - ایک بنیادی اینڈ-ٹو-اینڈ VLA ایپلیکیشن بنائیں