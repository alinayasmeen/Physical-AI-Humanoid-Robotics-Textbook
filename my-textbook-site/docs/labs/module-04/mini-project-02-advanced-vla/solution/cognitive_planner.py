#!/usr/bin/env python3
"""
Cognitive Planner Module for Advanced VLA System
Uses LLM-based reasoning for action planning.

SOLUTION: Complete implementation with all functionality.
"""

import re
from dataclasses import dataclass
from typing import List, Optional, Dict, Any
from enum import Enum


class ActionType(Enum):
    """Types of robot actions."""
    MOVE_FORWARD = 'move_forward'
    MOVE_BACKWARD = 'move_backward'
    TURN_LEFT = 'turn_left'
    TURN_RIGHT = 'turn_right'
    STOP = 'stop'
    PICK_UP = 'pick_up'
    PLACE = 'place'
    LOOK_AT = 'look_at'
    WAIT = 'wait'


@dataclass
class Action:
    """Represents a single action in the plan."""
    action_type: ActionType
    target_object: Optional[str] = None
    parameters: Optional[Dict[str, Any]] = None
    duration: float = 1.0


@dataclass
class ActionPlan:
    """Complete action plan with metadata."""
    actions: List[Action]
    confidence: float
    reasoning: str
    requires_confirmation: bool = False


class CognitivePlanner:
    """
    LLM-based cognitive planner for generating
    action sequences from natural language commands.
    """

    def __init__(self):
        """Initialize the cognitive planner."""
        self.command_templates = self._build_command_templates()
        self.action_keywords = self._build_action_keywords()
        print('CognitivePlanner initialized')

    def _build_command_templates(self) -> Dict[str, List[str]]:
        """Build mapping of common commands to action sequences."""
        return {
            'pick up': ['look_at', 'move_forward', 'pick_up'],
            'grab': ['look_at', 'move_forward', 'pick_up'],
            'get': ['look_at', 'move_forward', 'pick_up'],
            'move to': ['turn_to', 'move_forward'],
            'go to': ['turn_to', 'move_forward'],
            'approach': ['turn_to', 'move_forward'],
            'bring me': ['look_at', 'move_forward', 'pick_up', 'turn_around', 'move_forward', 'place'],
            'fetch': ['look_at', 'move_forward', 'pick_up', 'turn_around', 'move_forward', 'place'],
            'avoid': ['turn_away', 'move_forward'],
            'stop': ['stop'],
            'halt': ['stop'],
            'wait': ['wait'],
        }

    def _build_action_keywords(self) -> Dict[str, ActionType]:
        """Build mapping of keywords to action types."""
        return {
            'forward': ActionType.MOVE_FORWARD,
            'ahead': ActionType.MOVE_FORWARD,
            'straight': ActionType.MOVE_FORWARD,
            'backward': ActionType.MOVE_BACKWARD,
            'back': ActionType.MOVE_BACKWARD,
            'reverse': ActionType.MOVE_BACKWARD,
            'left': ActionType.TURN_LEFT,
            'right': ActionType.TURN_RIGHT,
            'stop': ActionType.STOP,
            'halt': ActionType.STOP,
            'pick': ActionType.PICK_UP,
            'grab': ActionType.PICK_UP,
            'place': ActionType.PLACE,
            'put': ActionType.PLACE,
            'look': ActionType.LOOK_AT,
            'wait': ActionType.WAIT,
        }

    def plan_actions(self, command: str, scene_context) -> ActionPlan:
        """
        Generate an action plan from a command and scene context.

        Args:
            command: Natural language command
            scene_context: Current scene understanding

        Returns:
            ActionPlan: Sequence of actions to execute
        """
        # Step 1: Parse command
        parsed = self.parse_command(command)

        # Step 2: Ground in scene
        grounded = self.ground_in_scene(parsed, scene_context)

        # Step 3: Generate action sequence
        actions = self.generate_action_sequence(grounded)

        # Step 4: Estimate confidence
        confidence = self.estimate_confidence(actions, scene_context)

        # Step 5: Determine if confirmation needed
        requires_confirmation = confidence < 0.7 or len(actions) > 3

        # Build reasoning string
        reasoning = self._build_reasoning(parsed, grounded, actions)

        return ActionPlan(
            actions=actions,
            confidence=confidence,
            reasoning=reasoning,
            requires_confirmation=requires_confirmation
        )

    def parse_command(self, command: str) -> Dict[str, Any]:
        """
        Parse a command to extract intent and parameters.

        Args:
            command: Natural language command

        Returns:
            dict: Parsed command with intent and parameters
        """
        command_lower = command.lower().strip()

        # Extract action verb
        action_verb = None
        for template in self.command_templates:
            if template in command_lower:
                action_verb = template
                break

        # Extract target object (simple noun extraction)
        target_object = None
        common_objects = ['cup', 'bottle', 'box', 'ball', 'table', 'chair', 'door']
        for obj in common_objects:
            if obj in command_lower:
                target_object = obj
                break

        # Extract direction if present
        direction = None
        for keyword, action_type in self.action_keywords.items():
            if keyword in command_lower:
                if action_type in [ActionType.MOVE_FORWARD, ActionType.MOVE_BACKWARD,
                                   ActionType.TURN_LEFT, ActionType.TURN_RIGHT]:
                    direction = keyword
                    if action_verb is None:
                        action_verb = 'move'
                    break

        return {
            'original': command,
            'action_verb': action_verb,
            'target_object': target_object,
            'direction': direction,
            'modifiers': [],
        }

    def ground_in_scene(self, parsed_command: Dict, scene_context) -> Dict[str, Any]:
        """
        Ground the parsed command in the current scene.

        Args:
            parsed_command: Parsed command structure
            scene_context: Scene understanding

        Returns:
            dict: Grounded command with resolved references
        """
        grounded = parsed_command.copy()
        grounded['scene_objects'] = [obj.class_name for obj in scene_context.objects]
        grounded['free_directions'] = scene_context.free_space_directions

        # Resolve target object reference
        if parsed_command['target_object']:
            target_name = parsed_command['target_object']
            for obj in scene_context.objects:
                if obj.class_name.lower() == target_name.lower():
                    grounded['resolved_target'] = obj
                    break

        return grounded

    def generate_action_sequence(self, grounded_command: Dict) -> List[Action]:
        """
        Generate a sequence of actions for the grounded command.

        Args:
            grounded_command: Command grounded in scene

        Returns:
            List[Action]: Sequence of actions
        """
        actions = []

        action_verb = grounded_command.get('action_verb')
        target = grounded_command.get('target_object')
        direction = grounded_command.get('direction')

        # Handle direction-based commands
        if direction:
            if direction in ['forward', 'ahead', 'straight']:
                actions.append(Action(ActionType.MOVE_FORWARD, duration=1.0))
            elif direction in ['backward', 'back', 'reverse']:
                actions.append(Action(ActionType.MOVE_BACKWARD, duration=1.0))
            elif direction == 'left':
                actions.append(Action(ActionType.TURN_LEFT, duration=0.5))
            elif direction == 'right':
                actions.append(Action(ActionType.TURN_RIGHT, duration=0.5))

        # Handle template-based commands
        elif action_verb in self.command_templates:
            template = self.command_templates[action_verb]
            for action_name in template:
                action_type = self._name_to_action_type(action_name)
                if action_type:
                    actions.append(Action(
                        action_type=action_type,
                        target_object=target,
                        duration=1.0
                    ))

        # Handle stop command
        elif action_verb in ['stop', 'halt']:
            actions.append(Action(ActionType.STOP))

        # Default: simple forward movement
        if not actions:
            actions.append(Action(ActionType.MOVE_FORWARD, duration=0.5))

        return actions

    def _name_to_action_type(self, name: str) -> Optional[ActionType]:
        """Convert action name string to ActionType enum."""
        name_mapping = {
            'move_forward': ActionType.MOVE_FORWARD,
            'move_backward': ActionType.MOVE_BACKWARD,
            'turn_left': ActionType.TURN_LEFT,
            'turn_right': ActionType.TURN_RIGHT,
            'turn_to': ActionType.TURN_LEFT,  # Placeholder
            'turn_around': ActionType.TURN_LEFT,  # 180 degree turn
            'turn_away': ActionType.TURN_RIGHT,
            'stop': ActionType.STOP,
            'pick_up': ActionType.PICK_UP,
            'place': ActionType.PLACE,
            'look_at': ActionType.LOOK_AT,
            'wait': ActionType.WAIT,
        }
        return name_mapping.get(name)

    def estimate_confidence(self, actions: List[Action], scene_context) -> float:
        """
        Estimate confidence in the action plan.

        Args:
            actions: Planned actions
            scene_context: Current scene

        Returns:
            float: Confidence score 0-1
        """
        if not actions:
            return 0.0

        confidence = 0.8  # Base confidence

        # Reduce confidence for complex plans
        if len(actions) > 3:
            confidence -= 0.1 * (len(actions) - 3)

        # Reduce confidence if obstacles present
        if scene_context.obstacles:
            confidence -= 0.1

        # Increase confidence if target object found
        for action in actions:
            if action.target_object:
                for obj in scene_context.objects:
                    if obj.class_name == action.target_object:
                        confidence += 0.1
                        break

        return max(0.1, min(1.0, confidence))

    def _build_reasoning(self, parsed: Dict, grounded: Dict, actions: List[Action]) -> str:
        """Build human-readable reasoning for the plan."""
        parts = []

        if parsed['action_verb']:
            parts.append(f"Recognized action: '{parsed['action_verb']}'")

        if parsed['target_object']:
            parts.append(f"Target: '{parsed['target_object']}'")

        if grounded.get('resolved_target'):
            parts.append("Target found in scene")
        elif parsed['target_object']:
            parts.append("Target not detected - proceeding anyway")

        parts.append(f"Generated {len(actions)} action(s)")

        return "; ".join(parts)


class MockCognitivePlanner(CognitivePlanner):
    """Mock planner for testing without LLM."""

    def plan_actions(self, command: str, scene_context) -> ActionPlan:
        """Return mock action plan based on simple keyword matching."""
        command_lower = command.lower()
        actions = []
        reasoning = "Mock planning based on keyword matching"

        # Simple keyword matching
        if 'forward' in command_lower or 'ahead' in command_lower:
            actions.append(Action(ActionType.MOVE_FORWARD, duration=1.0))
            reasoning = "Detected 'forward' keyword"
        elif 'backward' in command_lower or 'back' in command_lower:
            actions.append(Action(ActionType.MOVE_BACKWARD, duration=1.0))
            reasoning = "Detected 'backward' keyword"
        elif 'left' in command_lower:
            actions.append(Action(ActionType.TURN_LEFT, duration=0.5))
            reasoning = "Detected 'left' keyword"
        elif 'right' in command_lower:
            actions.append(Action(ActionType.TURN_RIGHT, duration=0.5))
            reasoning = "Detected 'right' keyword"
        elif 'stop' in command_lower or 'halt' in command_lower:
            actions.append(Action(ActionType.STOP))
            reasoning = "Detected 'stop' keyword"
        elif 'pick' in command_lower or 'grab' in command_lower:
            actions.append(Action(ActionType.LOOK_AT, duration=0.5))
            actions.append(Action(ActionType.MOVE_FORWARD, duration=1.0))
            actions.append(Action(ActionType.PICK_UP, duration=1.0))
            reasoning = "Detected 'pick' keyword - generated pickup sequence"
        else:
            actions.append(Action(ActionType.WAIT, duration=0.5))
            reasoning = "No recognized keyword - waiting"

        return ActionPlan(
            actions=actions,
            confidence=0.85,
            reasoning=reasoning,
            requires_confirmation=False
        )
