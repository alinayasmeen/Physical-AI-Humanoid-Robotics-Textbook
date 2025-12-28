# Lab 2: LLM Cognitive Planning - Task Decomposition with Large Language Models

**Module**: 4 - Vision-Language-Action (VLA) Systems
**Duration**: 90 minutes
**Difficulty**: Intermediate-Advanced

## Objective

Build a cognitive planning system that uses Large Language Models (LLMs) to decompose complex tasks into executable robot actions. You'll implement prompt engineering techniques and create a planning pipeline that generates structured action sequences.

## Prerequisites

- Completed Lab 1 (Speech to Command)
- Ubuntu 22.04 with ROS 2 Humble installed
- Python 3.10+
- OpenAI API key or local LLM setup (Ollama recommended)

## Learning Outcomes

By completing this lab, you will:
1. Understand cognitive planning in robotics using LLMs
2. Design effective prompts for task decomposition
3. Generate structured action plans from natural language goals
4. Validate and verify generated plans for robot execution
5. Handle plan failures and re-planning scenarios

---

## Setup

### 1. Install Dependencies

```bash
# Install OpenAI client (for API-based LLMs)
pip install openai

# OR Install Ollama for local LLMs (recommended for offline use)
curl -fsSL https://ollama.ai/install.sh | sh
ollama pull llama2

# Install additional dependencies
pip install langchain pydantic

# Verify installation
python3 -c "import openai; print('OpenAI client installed')"
```

### 2. Configure API Access

```bash
# For OpenAI API
export OPENAI_API_KEY="your-api-key-here"

# OR for local Ollama
ollama serve  # Start Ollama server in background
```

### 3. Navigate to Lab Directory

```bash
cd ~/ros2_ws/src
cp -r /path/to/labs/module-04/lab-02-llm-cognitive-planning .
cd lab-02-llm-cognitive-planning
```

---

## Lab Steps

### Step 1: Understand Cognitive Planning Architecture (15 min)

Open `starter/cognitive_planner.py` and examine the planning architecture:

```bash
cat starter/cognitive_planner.py
```

The cognitive planner consists of:
1. **Task Analysis**: Understand the high-level goal
2. **Environment Context**: Current robot state and environment
3. **Plan Generation**: LLM-based action sequence generation
4. **Plan Validation**: Verify feasibility and safety

**Your task**: Complete the TODOs to create a functional cognitive planner.

### Step 2: Design the Prompt Template (20 min)

Create an effective prompt for task decomposition:

```python
# TODO 1: Design the planning prompt template
PLANNING_PROMPT = """
You are a cognitive planner for a humanoid robot. Your task is to decompose
high-level commands into a sequence of executable robot actions.

## Robot Capabilities:
- navigate(location): Move to a specified location
- pick(object): Pick up an object with the gripper
- place(object, location): Place an object at a location
- look(direction): Turn head to look in a direction
- speak(message): Say something through speakers
- wait(seconds): Wait for specified duration

## Current Environment:
{environment_context}

## Current Robot State:
{robot_state}

## Task:
{task_description}

## Instructions:
1. Break down the task into atomic robot actions
2. Consider the current environment and robot state
3. Include safety checks where appropriate
4. Output a JSON array of actions

## Output Format:
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

Generate the plan:
"""
```

Test your prompt:
```bash
python3 starter/cognitive_planner.py --test-prompt
```

### Step 3: Implement LLM Integration (20 min)

Connect to the LLM for plan generation:

```python
# TODO 2: Implement LLM integration
class CognitivePlanner:
    def __init__(self, model: str = "gpt-3.5-turbo"):
        self.client = openai.OpenAI()
        self.model = model

    def generate_plan(self, task: str, environment: dict, robot_state: dict) -> dict:
        """Generate an action plan using LLM."""
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
            temperature=0.2,  # Low temperature for consistent planning
            max_tokens=1000
        )

        plan_text = response.choices[0].message.content
        return self._parse_plan(plan_text)

    def _parse_plan(self, plan_text: str) -> dict:
        """Parse LLM response into structured plan."""
        # Extract JSON from response
        import re
        json_match = re.search(r'```json\s*(.*?)\s*```', plan_text, re.DOTALL)
        if json_match:
            return json.loads(json_match.group(1))
        return json.loads(plan_text)
```

Test plan generation:
```bash
python3 starter/cognitive_planner.py --generate "Set the table for dinner"
```

### Step 4: Implement Plan Validation (20 min)

Validate generated plans for feasibility:

```python
# TODO 3: Implement plan validation
class PlanValidator:
    def __init__(self, robot_capabilities: list, environment_objects: list):
        self.capabilities = robot_capabilities
        self.objects = environment_objects

    def validate_plan(self, plan: dict) -> tuple[bool, list]:
        """Validate a plan for feasibility and safety."""
        errors = []

        for step in plan.get('steps', []):
            # Check action is in robot capabilities
            if step['action'] not in self.capabilities:
                errors.append(f"Unknown action: {step['action']}")

            # Check object references exist
            params = step.get('parameters', {})
            if 'object' in params and params['object'] not in self.objects:
                errors.append(f"Unknown object: {params['object']}")

            # Check preconditions are satisfiable
            for precond in step.get('preconditions', []):
                if not self._check_precondition(precond):
                    errors.append(f"Unsatisfiable precondition: {precond}")

        return len(errors) == 0, errors

    def _check_precondition(self, precondition: str) -> bool:
        """Check if a precondition can be satisfied."""
        # Simplified check - in practice, use symbolic planning
        return True
```

Test validation:
```bash
python3 starter/cognitive_planner.py --validate-plan sample_plan.json
```

### Step 5: Implement Re-planning (10 min)

Handle plan failures with re-planning:

```python
# TODO 4: Implement re-planning on failure
def replan_on_failure(self, original_task: str, failed_step: int,
                       failure_reason: str, environment: dict) -> dict:
    """Generate a new plan when execution fails."""
    replan_prompt = f"""
    The previous plan failed at step {failed_step}.
    Failure reason: {failure_reason}

    Original task: {original_task}

    Please generate an alternative plan that:
    1. Avoids the failure mode
    2. Accounts for the current environment state
    3. Still achieves the original goal

    Current environment: {json.dumps(environment)}
    """

    return self.generate_plan(replan_prompt, environment, self.get_robot_state())
```

### Step 6: Create ROS 2 Integration (5 min)

Publish plans as ROS 2 messages:

```python
# TODO 5: ROS 2 plan publisher
class CognitivePlannerNode(Node):
    def __init__(self):
        super().__init__('cognitive_planner_node')
        self.planner = CognitivePlanner()
        self.plan_pub = self.create_publisher(String, '/robot/plan', 10)
        self.task_sub = self.create_subscription(
            String, '/robot/task', self.task_callback, 10)

    def task_callback(self, msg):
        """Receive task and generate plan."""
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

## Deliverables

1. **Working Planner**: `starter/cognitive_planner.py` that:
   - Takes natural language task descriptions
   - Generates structured action plans using LLM
   - Validates plans for feasibility
   - Supports re-planning on failure

2. **Sample Plans**: At least 3 generated plans for different tasks

3. **Answers**: Complete the reflection questions below

---

## Reflection Questions

1. **Why use low temperature (0.2) for planning vs higher for creative tasks?**
   - Consider: consistency, reproducibility, determinism

2. **What are the risks of using LLMs for safety-critical robot planning?**
   - Think about: hallucinations, unexpected outputs, validation needs

3. **How would you handle tasks that are impossible given current robot capabilities?**
   - Consider: graceful failure, user feedback, capability negotiation

4. **What modifications would be needed for multi-robot planning?**
   - Think about: coordination, resource conflicts, communication

---

## Validation

Run the validation script:

```bash
python3 validate.py
```

Expected output:
```
[PASS] cognitive_planner.py exists in starter/
[PASS] Valid Python syntax
[PASS] LLM client configured
[PASS] Prompt template defined
[PASS] Plan generation function implemented
[PASS] Plan validation function implemented
[PASS] Re-planning function implemented
Lab 2 Complete!
```

---

## Troubleshooting

### OpenAI API rate limits

```bash
# Use local Ollama instead
export USE_LOCAL_LLM=true
ollama serve
```

### Invalid JSON in LLM response

- Add explicit JSON formatting instructions to prompt
- Use JSON mode if available (OpenAI GPT-4)
- Implement robust parsing with fallbacks

### Plans are too verbose/simple

- Adjust the prompt to specify desired granularity
- Provide more examples in the prompt
- Fine-tune temperature and max_tokens

---

## Extension Challenges

1. **Hierarchical Planning**: Implement multi-level planning with abstract and concrete actions
2. **Temporal Constraints**: Add time constraints and scheduling to plans
3. **Multi-agent Planning**: Extend to coordinate multiple robots
4. **Learning from Feedback**: Improve planning based on execution outcomes

---

## Next Steps

After completing this lab, proceed to:
- **Lab 3**: VLA Pipeline Integration - Combine all components into a complete system
- **Mini-Project 1**: Simple VLA System - Build a basic end-to-end VLA application
