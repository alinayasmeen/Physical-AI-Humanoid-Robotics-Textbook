#!/usr/bin/env python3
"""
Lab 2: LLM Cognitive Planning
Module 4 - Vision-Language-Action (VLA) Systems

This module implements cognitive planning using Large Language Models
to decompose complex tasks into executable robot actions.

Complete the TODOs to implement the cognitive planning pipeline.
"""

import argparse
import json
import os
import re
from dataclasses import dataclass, field
from typing import List, Optional, Dict, Any

# TODO: Uncomment these imports after installing dependencies
# import openai
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String


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


@dataclass
class PlanStep:
    """Represents a single step in a robot plan."""
    step: int
    action: str
    parameters: Dict[str, Any] = field(default_factory=dict)
    preconditions: List[str] = field(default_factory=list)
    expected_outcome: str = ""


@dataclass
class CognitivePlan:
    """Represents a complete robot action plan."""
    plan_id: str
    goal: str
    steps: List[PlanStep]
    estimated_duration: str
    risk_assessment: str

    def to_dict(self) -> dict:
        return {
            'plan_id': self.plan_id,
            'goal': self.goal,
            'steps': [
                {
                    'step': s.step,
                    'action': s.action,
                    'parameters': s.parameters,
                    'preconditions': s.preconditions,
                    'expected_outcome': s.expected_outcome
                }
                for s in self.steps
            ],
            'estimated_duration': self.estimated_duration,
            'risk_assessment': self.risk_assessment
        }

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), indent=2)


class CognitivePlanner:
    """LLM-based cognitive planner for robot task decomposition."""

    # Available robot capabilities
    CAPABILITIES = ['navigate', 'pick', 'place', 'look', 'speak', 'wait']

    def __init__(self, model: str = "gpt-3.5-turbo", use_local: bool = False):
        """
        Initialize the cognitive planner.

        Args:
            model: LLM model to use
            use_local: Whether to use local LLM (Ollama) instead of OpenAI
        """
        self.model = model
        self.use_local = use_local

        # TODO 2: Initialize LLM client
        # YOUR CODE HERE
        # If use_local is True, configure for Ollama
        # Otherwise, configure OpenAI client
        self.client = None  # Replace with actual client initialization

    def generate_plan(self, task: str, environment: dict, robot_state: dict) -> CognitivePlan:
        """
        Generate an action plan using LLM.

        Args:
            task: Natural language task description
            environment: Current environment state
            robot_state: Current robot state

        Returns:
            CognitivePlan with decomposed actions
        """
        # TODO 2: Implement LLM integration
        # YOUR CODE HERE
        # 1. Format the prompt with environment and robot state
        # 2. Call the LLM API
        # 3. Parse the response into a CognitivePlan
        raise NotImplementedError("Implement plan generation")

    def _format_prompt(self, task: str, environment: dict, robot_state: dict) -> str:
        """Format the planning prompt with current context."""
        return PLANNING_PROMPT.format(
            environment_context=json.dumps(environment, indent=2),
            robot_state=json.dumps(robot_state, indent=2),
            task_description=task
        )

    def _parse_plan(self, plan_text: str) -> CognitivePlan:
        """
        Parse LLM response into structured plan.

        Args:
            plan_text: Raw LLM response text

        Returns:
            CognitivePlan object
        """
        # Extract JSON from response
        json_match = re.search(r'```json\s*(.*?)\s*```', plan_text, re.DOTALL)
        if json_match:
            plan_dict = json.loads(json_match.group(1))
        else:
            # Try parsing the entire response as JSON
            plan_dict = json.loads(plan_text)

        # Convert to CognitivePlan
        steps = [
            PlanStep(
                step=s['step'],
                action=s['action'],
                parameters=s.get('parameters', {}),
                preconditions=s.get('preconditions', []),
                expected_outcome=s.get('expected_outcome', '')
            )
            for s in plan_dict['steps']
        ]

        return CognitivePlan(
            plan_id=plan_dict['plan_id'],
            goal=plan_dict['goal'],
            steps=steps,
            estimated_duration=plan_dict['estimated_duration'],
            risk_assessment=plan_dict['risk_assessment']
        )


class PlanValidator:
    """Validates generated plans for feasibility and safety."""

    def __init__(self, robot_capabilities: List[str], environment_objects: List[str]):
        """
        Initialize the plan validator.

        Args:
            robot_capabilities: List of valid robot actions
            environment_objects: List of known objects in environment
        """
        self.capabilities = robot_capabilities
        self.objects = environment_objects

    def validate_plan(self, plan: CognitivePlan) -> tuple:
        """
        Validate a plan for feasibility and safety.

        Args:
            plan: CognitivePlan to validate

        Returns:
            Tuple of (is_valid, list of errors)
        """
        # TODO 3: Implement plan validation
        # YOUR CODE HERE
        # 1. Check each action is in robot capabilities
        # 2. Check object references exist in environment
        # 3. Check preconditions are satisfiable
        # 4. Return (True, []) if valid, (False, errors) otherwise
        raise NotImplementedError("Implement plan validation")

    def _check_precondition(self, precondition: str) -> bool:
        """Check if a precondition can be satisfied."""
        # Simplified check - in practice, use symbolic planning
        return True


# TODO 4: Implement re-planning on failure
def replan_on_failure(planner: CognitivePlanner, original_task: str,
                      failed_step: int, failure_reason: str,
                      environment: dict, robot_state: dict) -> Optional[CognitivePlan]:
    """
    Generate a new plan when execution fails.

    Args:
        planner: CognitivePlanner instance
        original_task: Original task description
        failed_step: Step number that failed
        failure_reason: Reason for failure
        environment: Current environment state
        robot_state: Current robot state

    Returns:
        New CognitivePlan or None if re-planning fails
    """
    # YOUR CODE HERE
    # 1. Create a modified task description that accounts for the failure
    # 2. Generate a new plan using the planner
    # 3. Return the new plan or None
    raise NotImplementedError("Implement re-planning")


# TODO 5: ROS 2 node (optional but recommended)
# Uncomment and complete this class for ROS 2 integration

# class CognitivePlannerNode(Node):
#     """ROS 2 node for cognitive planning."""
#
#     def __init__(self):
#         super().__init__('cognitive_planner_node')
#         # YOUR CODE HERE
#         raise NotImplementedError("Implement ROS 2 node")


def test_prompt():
    """Test prompt formatting."""
    print("Testing prompt formatting...")

    environment = {
        "objects": [
            {"name": "red_ball", "position": [1.0, 0.5, 0.1]},
            {"name": "table", "position": [1.5, 0.0, 0.0]}
        ],
        "room": "living_room"
    }

    robot_state = {
        "position": [0.0, 0.0, 0.0],
        "gripper": "empty",
        "battery": 85
    }

    planner = CognitivePlanner()
    prompt = planner._format_prompt("Pick up the red ball", environment, robot_state)
    print(prompt[:500] + "...")
    print("\nPrompt formatting test PASSED")


def test_generate(task: str):
    """Test plan generation."""
    print(f"Testing plan generation for: '{task}'")

    environment = {
        "objects": [
            {"name": "red_ball", "position": [1.0, 0.5, 0.1]},
            {"name": "table", "position": [1.5, 0.0, 0.0]}
        ],
        "room": "living_room"
    }

    robot_state = {
        "position": [0.0, 0.0, 0.0],
        "gripper": "empty",
        "battery": 85
    }

    try:
        planner = CognitivePlanner()
        plan = planner.generate_plan(task, environment, robot_state)
        print(f"\nGenerated plan:")
        print(plan.to_json())
        print("\nPlan generation test PASSED")
    except NotImplementedError:
        print("\nPlan generation not yet implemented")
    except Exception as e:
        print(f"\nPlan generation test FAILED: {e}")


def test_validate(plan_file: str):
    """Test plan validation."""
    print(f"Testing plan validation for: {plan_file}")

    try:
        with open(plan_file, 'r') as f:
            plan_dict = json.load(f)

        # Create plan object
        steps = [
            PlanStep(
                step=s['step'],
                action=s['action'],
                parameters=s.get('parameters', {}),
                preconditions=s.get('preconditions', []),
                expected_outcome=s.get('expected_outcome', '')
            )
            for s in plan_dict['steps']
        ]

        plan = CognitivePlan(
            plan_id=plan_dict['plan_id'],
            goal=plan_dict['goal'],
            steps=steps,
            estimated_duration=plan_dict['estimated_duration'],
            risk_assessment=plan_dict['risk_assessment']
        )

        # Validate
        validator = PlanValidator(
            robot_capabilities=CognitivePlanner.CAPABILITIES,
            environment_objects=['red_ball', 'table', 'chair']
        )

        is_valid, errors = validator.validate_plan(plan)

        if is_valid:
            print("Plan is VALID")
        else:
            print("Plan is INVALID:")
            for error in errors:
                print(f"  - {error}")

        print("\nPlan validation test PASSED")
    except NotImplementedError:
        print("\nPlan validation not yet implemented")
    except Exception as e:
        print(f"\nPlan validation test FAILED: {e}")


def main():
    """Main entry point for the cognitive planner module."""
    parser = argparse.ArgumentParser(description='LLM Cognitive Planning Lab')
    parser.add_argument('--test-prompt', action='store_true',
                       help='Test prompt formatting')
    parser.add_argument('--generate', type=str,
                       help='Generate plan for given task')
    parser.add_argument('--validate-plan', type=str,
                       help='Validate plan from JSON file')
    parser.add_argument('--ros2', action='store_true',
                       help='Run as ROS 2 node')

    args = parser.parse_args()

    if args.test_prompt:
        test_prompt()
    elif args.generate:
        test_generate(args.generate)
    elif args.validate_plan:
        test_validate(args.validate_plan)
    elif args.ros2:
        print("ROS 2 mode not yet implemented")
    else:
        parser.print_help()


if __name__ == '__main__':
    main()
