# Lab 3: VLA Pipeline Integration - Building an End-to-End System

**Module**: 4 - Vision-Language-Action (VLA) Systems
**Duration**: 120 minutes
**Difficulty**: Advanced

## Objective

Integrate all VLA components (Vision, Language, Action) into a complete end-to-end pipeline. You'll connect speech recognition, cognitive planning, vision processing, and ROS 2 action execution to create a fully functional VLA system that can receive spoken commands and execute them on a robot.

## Prerequisites

- Completed Lab 1 (Speech to Command)
- Completed Lab 2 (LLM Cognitive Planning)
- Completed Module 3 (Perception & Sensors)
- Ubuntu 22.04 with ROS 2 Humble installed
- Python 3.10+
- Gazebo simulation environment configured

## Learning Outcomes

By completing this lab, you will:
1. Integrate multiple VLA components into a unified pipeline
2. Implement state management for complex robotic workflows
3. Handle component failures and implement graceful degradation
4. Create visualizations for debugging multimodal systems
5. Test end-to-end VLA functionality in simulation

---

## Setup

### 1. Install Dependencies

```bash
# Ensure all previous lab dependencies are installed
pip install openai-whisper openai sounddevice numpy scipy

# Install visualization tools
pip install matplotlib plotly

# Install ROS 2 action dependencies
sudo apt install ros-humble-action-tutorials-interfaces

# Verify setup
python3 -c "import whisper, openai; print('VLA dependencies ready')"
```

### 2. Navigate to Lab Directory

```bash
cd ~/ros2_ws/src
cp -r /path/to/labs/module-04/lab-03-vla-pipeline-integration .
cd lab-03-vla-pipeline-integration
```

### 3. Launch Simulation Environment

```bash
# Terminal 1: Launch Gazebo with robot
ros2 launch gazebo_ros gazebo.launch.py world:=/path/to/manipulation_world.sdf

# Terminal 2: Spawn robot with arm and camera
ros2 run gazebo_ros spawn_entity.py -entity vla_robot -file robot.urdf
```

---

## Lab Steps

### Step 1: Understand the VLA Pipeline Architecture (15 min)

Open `starter/vla_pipeline.py` and examine the complete architecture:

```bash
cat starter/vla_pipeline.py
```

The VLA pipeline consists of:

```
Audio Input → Speech Recognition → Command Parser → Cognitive Planner
                                                          ↓
                                                    Plan Validator
                                                          ↓
Vision System → Scene Understanding → Action Executor → Robot Actions
```

**Your task**: Implement the pipeline orchestrator that coordinates all components.

### Step 2: Implement the Pipeline State Machine (25 min)

Create a state machine to manage pipeline execution:

```python
# TODO 1: Implement VLA pipeline state machine
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
        """Transition to a new state with logging."""
        self.get_logger().info(f'State: {self.state.name} -> {new_state.name}')
        self.state = new_state
        self._publish_state()

    async def run(self):
        """Main pipeline execution loop."""
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

Test the state machine:
```bash
python3 starter/vla_pipeline.py --test-states
```

### Step 3: Integrate Speech Recognition Component (15 min)

Connect the speech recognition from Lab 1:

```python
# TODO 2: Integrate speech recognition
async def _listen_for_command(self):
    """Listen for voice command."""
    self.get_logger().info('Listening for command...')

    try:
        audio = await self.speech_recognizer.record_async(duration=5.0)
        self.audio_buffer = audio
        self.transition(PipelineState.TRANSCRIBING)
    except AudioCaptureError as e:
        self.get_logger().error(f'Audio capture failed: {e}')
        self.transition(PipelineState.FAILED)

async def _transcribe_audio(self):
    """Transcribe recorded audio."""
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

### Step 4: Integrate Cognitive Planning Component (15 min)

Connect the cognitive planner from Lab 2:

```python
# TODO 3: Integrate cognitive planning
async def _generate_plan(self):
    """Generate action plan using LLM."""
    self.get_logger().info('Generating plan...')

    # Get current environment state from vision
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

### Step 5: Integrate Vision System (15 min)

Connect vision processing for environment awareness:

```python
# TODO 4: Integrate vision system
async def _update_perception(self):
    """Update perception before action execution."""
    self.get_logger().info('Updating perception...')

    try:
        # Get latest camera frame
        image = await self.vision_system.get_latest_frame()

        # Detect objects in scene
        detections = await self.vision_system.detect_objects(image)

        # Update environment model
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

        # Check if current action target is visible
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

### Step 6: Implement Action Execution (20 min)

Execute planned actions via ROS 2:

```python
# TODO 5: Implement action execution
async def _execute_step(self):
    """Execute current plan step."""
    step = self.current_plan['steps'][self.current_step]
    action = step['action']
    params = step.get('parameters', {})

    self.get_logger().info(f'Executing step {self.current_step + 1}: {action}')

    try:
        # Map action to ROS 2 action client
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
                # Update perception before next step
                self.transition(PipelineState.PERCEIVING)
        else:
            self.get_logger().error(f'Action {action} failed')
            self.transition(PipelineState.RECOVERING)

    except ActionExecutionError as e:
        self.get_logger().error(f'Execution error: {e}')
        self.transition(PipelineState.RECOVERING)
```

### Step 7: Implement Error Recovery (10 min)

Handle failures and implement recovery strategies:

```python
# TODO 6: Implement error recovery
async def _handle_recovery(self):
    """Attempt to recover from failures."""
    self.get_logger().info('Attempting recovery...')

    # Get current failure context
    failed_step = self.current_step
    failure_reason = self.last_error

    # Try re-planning from current state
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

### Step 8: Add Visualization and Debugging (5 min)

Create a visualization for pipeline status:

```python
# TODO 7: Implement visualization
def visualize_pipeline_state(self):
    """Publish visualization markers for debugging."""
    # Publish state to /vla/state topic
    state_msg = String()
    state_msg.data = json.dumps({
        'state': self.state.name,
        'current_step': self.current_step,
        'total_steps': len(self.current_plan['steps']) if self.current_plan else 0,
        'transcribed_text': self.transcribed_text,
        'environment': self.environment_state
    })
    self.state_pub.publish(state_msg)

    # Publish visual markers for detected objects
    for obj in self.environment_state.get('objects', []):
        marker = self._create_object_marker(obj)
        self.marker_pub.publish(marker)
```

---

## Deliverables

1. **Working Pipeline**: `starter/vla_pipeline.py` that:
   - Receives spoken commands
   - Generates plans using LLM
   - Updates perception before actions
   - Executes actions on the robot
   - Recovers from failures

2. **Demo Video**: Recording showing end-to-end execution of a command

3. **System Diagram**: Updated data flow diagram with your implementation

4. **Answers**: Complete the reflection questions below

---

## Reflection Questions

1. **Why is the perception update step critical before each action?**
   - Consider: environmental changes, object movement, uncertainty

2. **How would you prioritize which component failures to recover from?**
   - Think about: safety, task criticality, recovery cost

3. **What are the bottlenecks in your pipeline? How would you optimize them?**
   - Consider: latency measurement, parallelization opportunities

4. **How would you extend this system for continuous operation (always listening)?**
   - Think about: wake word detection, resource management, privacy

---

## Validation

Run the validation script:

```bash
python3 validate.py
```

Expected output:
```
[PASS] vla_pipeline.py exists in starter/
[PASS] Valid Python syntax
[PASS] State machine implemented correctly
[PASS] Speech recognition integrated
[PASS] Cognitive planning integrated
[PASS] Vision system integrated
[PASS] Action execution implemented
[PASS] Error recovery implemented
[PASS] ROS 2 node structure correct
Lab 3 Complete!
```

---

## Troubleshooting

### Pipeline hangs in LISTENING state

```bash
# Check microphone permissions
pactl list sources

# Verify audio input
python3 -c "import sounddevice as sd; print(sd.query_devices())"
```

### Actions fail to execute

```bash
# Check ROS 2 action servers are running
ros2 action list

# Verify robot controller is active
ros2 node list | grep controller
```

### Vision detections are incorrect

- Check camera calibration
- Verify lighting conditions in simulation
- Adjust detection confidence thresholds

### LLM responses are slow

- Use smaller/faster models (GPT-3.5-turbo or local Llama2)
- Implement response caching for repeated queries
- Consider edge deployment options

---

## Extension Challenges

1. **Multi-modal Confirmation**: Add visual confirmation before critical actions
2. **Voice Feedback**: Have the robot speak its plan before execution
3. **Learning from Corrections**: Implement online learning from user corrections
4. **Multi-robot Coordination**: Extend pipeline for multiple cooperating robots

---

## Next Steps

After completing this lab, proceed to:
- **Mini-Project 1**: Simple VLA System - Build your own VLA application
- **Mini-Project 2**: Advanced VLA System - Implement complex behaviors
- **Module Review**: Complete assessment and certification
