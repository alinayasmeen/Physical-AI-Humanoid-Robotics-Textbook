# Lab 1: Speech to Command - Converting Spoken Language to Robot Tasks

**Module**: 4 - Vision-Language-Action (VLA) Systems
**Duration**: 90 minutes
**Difficulty**: Intermediate

## Objective

Build a speech recognition pipeline that converts spoken human commands into structured robot tasks. You'll use OpenAI's Whisper for speech-to-text conversion and implement command parsing to extract actionable robot instructions.

## Prerequisites

- Completed Modules 1-3 (ROS 2 fundamentals, simulation, perception)
- Ubuntu 22.04 with ROS 2 Humble installed
- Python 3.10+
- Microphone or audio input device (or pre-recorded audio files)

## Learning Outcomes

By completing this lab, you will:
1. Understand the role of speech recognition in VLA systems
2. Use Whisper ASR (Automatic Speech Recognition) for speech-to-text conversion
3. Parse natural language commands into structured robot tasks
4. Handle ambiguous or unclear commands appropriately
5. Integrate speech input with ROS 2 messaging

---

## Setup

### 1. Install Dependencies

```bash
# Install Whisper
pip install openai-whisper

# Install audio processing libraries
pip install sounddevice numpy scipy

# Install speech recognition utilities
pip install SpeechRecognition pyaudio

# Verify installation
python3 -c "import whisper; print('Whisper installed successfully')"
```

### 2. Navigate to Lab Directory

```bash
cd ~/ros2_ws/src
cp -r /path/to/labs/module-04/lab-01-speech-to-command .
cd lab-01-speech-to-command
```

### 3. Download Whisper Model

```bash
# Download the base model (first run will download automatically)
python3 -c "import whisper; whisper.load_model('base')"
```

---

## Lab Steps

### Step 1: Understand the Speech-to-Command Pipeline (15 min)

Open `starter/speech_to_command.py` and examine its structure:

```bash
cat starter/speech_to_command.py
```

The pipeline consists of three stages:
1. **Audio Capture**: Record or load audio input
2. **Speech Recognition**: Convert audio to text using Whisper
3. **Command Parsing**: Extract structured commands from text

**Your task**: Complete the TODOs to make the pipeline functional.

### Step 2: Implement Audio Capture (15 min)

Complete the audio capture function to record from microphone:

```python
# TODO 1: Implement audio recording
def record_audio(duration: float = 5.0, sample_rate: int = 16000) -> np.ndarray:
    """Record audio from microphone for specified duration."""
    print(f"Recording for {duration} seconds...")
    audio = sd.rec(int(duration * sample_rate), samplerate=sample_rate, channels=1)
    sd.wait()  # Wait until recording is finished
    return audio.flatten()
```

Test your recording:
```bash
python3 starter/speech_to_command.py --test-audio
```

### Step 3: Implement Whisper Transcription (20 min)

Use Whisper to convert audio to text:

```python
# TODO 2: Implement speech-to-text conversion
def transcribe_audio(audio: np.ndarray, model: whisper.Whisper) -> str:
    """Transcribe audio using Whisper model."""
    # Whisper expects float32 audio normalized to [-1, 1]
    audio = audio.astype(np.float32)
    if audio.max() > 1.0:
        audio = audio / 32768.0  # Normalize if int16

    result = model.transcribe(audio, language='en')
    return result['text'].strip()
```

Test transcription:
```bash
# Record and transcribe
python3 starter/speech_to_command.py --test-transcribe
```

### Step 4: Implement Command Parsing (25 min)

Parse natural language into structured robot commands:

```python
# TODO 3: Implement command parsing
class RobotCommand:
    def __init__(self, action: str, target: str = None, location: str = None, parameters: dict = None):
        self.action = action
        self.target = target
        self.location = location
        self.parameters = parameters or {}

def parse_command(text: str) -> RobotCommand:
    """Parse transcribed text into structured robot command."""
    text = text.lower().strip()

    # Define action keywords
    action_patterns = {
        'pick': ['pick up', 'pick', 'grab', 'grasp', 'take'],
        'place': ['place', 'put', 'set', 'drop'],
        'move': ['move', 'go', 'navigate', 'drive'],
        'look': ['look', 'find', 'search', 'locate'],
        'stop': ['stop', 'halt', 'pause', 'wait']
    }

    # Extract action
    action = 'unknown'
    for cmd, patterns in action_patterns.items():
        if any(p in text for p in patterns):
            action = cmd
            break

    # Extract target object (e.g., "the red ball")
    target = extract_object(text)

    # Extract location (e.g., "on the table")
    location = extract_location(text)

    return RobotCommand(action=action, target=target, location=location)
```

Test command parsing:
```bash
python3 starter/speech_to_command.py --test-parse "pick up the red ball from the table"
```

### Step 5: Create ROS 2 Integration (10 min)

Publish parsed commands to ROS 2:

```python
# TODO 4: Implement ROS 2 publisher
class SpeechCommandNode(Node):
    def __init__(self):
        super().__init__('speech_command_node')
        self.command_pub = self.create_publisher(String, '/robot/command', 10)
        self.model = whisper.load_model('base')
        self.timer = self.create_timer(5.0, self.listen_callback)

    def listen_callback(self):
        """Periodically listen for commands."""
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

### Step 6: Test End-to-End Pipeline (5 min)

Run the complete pipeline:

```bash
# Terminal 1: Run the speech command node
ros2 run speech_to_command speech_command_node

# Terminal 2: Monitor published commands
ros2 topic echo /robot/command
```

---

## Deliverables

1. **Working Pipeline**: `starter/speech_to_command.py` that:
   - Records audio from microphone
   - Transcribes speech using Whisper
   - Parses commands into structured format
   - Publishes to ROS 2 topic

2. **Test Results**: Screenshot showing successful command parsing

3. **Answers**: Complete the reflection questions below

---

## Reflection Questions

1. **Why is Whisper preferred over traditional speech recognition for robotics?**
   - Consider: robustness to noise, language support, accuracy

2. **How would you handle ambiguous commands like "put that over there"?**
   - Think about: context tracking, clarification requests, default behaviors

3. **What are the latency implications of using Whisper in real-time robotics?**
   - Consider: model size vs speed tradeoffs, streaming alternatives

4. **How would you extend this system to support multiple languages?**
   - Think about: model capabilities, command parsing modifications

---

## Validation

Run the validation script to verify your lab completion:

```bash
python3 validate.py
```

Expected output:
```
[PASS] speech_to_command.py exists in starter/
[PASS] Valid Python syntax
[PASS] Whisper import present
[PASS] Audio recording function implemented
[PASS] Transcription function implemented
[PASS] Command parsing function implemented
[PASS] ROS 2 node structure correct
Lab 1 Complete!
```

---

## Troubleshooting

### No audio input detected

```bash
# List audio devices
python3 -c "import sounddevice as sd; print(sd.query_devices())"

# Set default input device
export SD_INPUT_DEVICE=0
```

### Whisper model download fails

```bash
# Manual download
wget https://openaipublic.azureedge.net/main/whisper/models/base.pt -P ~/.cache/whisper/
```

### Poor transcription accuracy

- Use the 'small' or 'medium' model for better accuracy
- Ensure clear audio with minimal background noise
- Speak clearly and at a moderate pace

---

## Extension Challenges

1. **Confidence Scoring**: Add confidence scores to parsed commands
2. **Context Awareness**: Track conversation context for pronoun resolution
3. **Multi-command**: Handle compound commands like "pick up the ball and put it on the table"
4. **Streaming**: Implement real-time streaming transcription for lower latency

---

## Next Steps

After completing this lab, proceed to:
- **Lab 2**: LLM Cognitive Planning - Use LLMs to decompose complex tasks
- **Lesson 2**: VLA Components - Understanding the complete VLA architecture
