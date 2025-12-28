#!/usr/bin/env python3
"""
Safety Monitor Module for Advanced VLA System
Monitors and validates actions for safety compliance.

SOLUTION: Complete implementation with all functionality.
"""

from dataclasses import dataclass
from typing import List, Optional, Tuple
from enum import Enum


class SafetyLevel(Enum):
    """Safety violation severity levels."""
    SAFE = 'safe'
    WARNING = 'warning'
    DANGER = 'danger'
    CRITICAL = 'critical'


@dataclass
class SafetyViolation:
    """Represents a safety violation."""
    level: SafetyLevel
    description: str
    action_index: int
    mitigation: Optional[str] = None


@dataclass
class SafetyReport:
    """Complete safety assessment report."""
    is_safe: bool
    violations: List[SafetyViolation]
    overall_risk_score: float  # 0-1, lower is safer
    recommendations: List[str]


class SafetyMonitor:
    """
    Safety monitoring system for VLA actions.
    Validates actions against safety constraints.
    """

    def __init__(self):
        """Initialize the safety monitor."""
        # Workspace limits (meters)
        self.workspace_limits = {
            'x_min': -2.0, 'x_max': 2.0,
            'y_min': -2.0, 'y_max': 2.0,
            'z_min': 0.0, 'z_max': 1.5,
        }

        # Safety thresholds
        self.min_obstacle_distance = 0.3  # meters
        self.max_speed = 0.5  # m/s
        self.max_angular_speed = 1.0  # rad/s

        # Emergency stop flag
        self.emergency_stop_active = False

        print('SafetyMonitor initialized')

    def validate_action_plan(self, action_plan, scene_context) -> SafetyReport:
        """
        Validate a complete action plan for safety.

        Args:
            action_plan: Plan to validate
            scene_context: Current scene understanding

        Returns:
            SafetyReport: Complete safety assessment
        """
        all_violations = []

        # Check emergency stop
        if self.emergency_stop_active:
            all_violations.append(SafetyViolation(
                level=SafetyLevel.CRITICAL,
                description="Emergency stop is active",
                action_index=-1,
                mitigation="Release emergency stop before executing actions"
            ))

        # Check each action
        for i, action in enumerate(action_plan.actions):
            violations = self.check_action_safety(action, scene_context, i)
            all_violations.extend(violations)

        # Calculate risk score
        risk_score = self.calculate_risk_score(all_violations)

        # Generate recommendations
        recommendations = self.generate_recommendations(all_violations)

        # Determine overall safety
        is_safe = len([v for v in all_violations
                      if v.level in [SafetyLevel.CRITICAL, SafetyLevel.DANGER]]) == 0

        return SafetyReport(
            is_safe=is_safe,
            violations=all_violations,
            overall_risk_score=risk_score,
            recommendations=recommendations
        )

    def check_action_safety(self, action, scene_context, action_index) -> List[SafetyViolation]:
        """
        Check a single action for safety violations.

        Args:
            action: Action to check
            scene_context: Current scene
            action_index: Index in action sequence

        Returns:
            List[SafetyViolation]: Found violations
        """
        violations = []

        # Check speed limits
        speed_violation = self.check_speed_limits(action)
        if speed_violation:
            speed_violation.action_index = action_index
            violations.append(speed_violation)

        # Check obstacle clearance for movement actions
        if action.action_type.value in ['move_forward', 'move_backward']:
            obstacle_violation = self.check_obstacle_clearance(action, scene_context.obstacles)
            if obstacle_violation:
                obstacle_violation.action_index = action_index
                violations.append(obstacle_violation)

        return violations

    def check_workspace_limits(self, target_position: Tuple[float, float, float]) -> Optional[SafetyViolation]:
        """
        Check if target position is within workspace.

        Args:
            target_position: (x, y, z) position

        Returns:
            SafetyViolation if outside workspace, None otherwise
        """
        x, y, z = target_position

        if x < self.workspace_limits['x_min'] or x > self.workspace_limits['x_max']:
            return SafetyViolation(
                level=SafetyLevel.DANGER,
                description=f"X position {x:.2f}m outside workspace limits",
                action_index=0,
                mitigation="Adjust target to stay within workspace"
            )

        if y < self.workspace_limits['y_min'] or y > self.workspace_limits['y_max']:
            return SafetyViolation(
                level=SafetyLevel.DANGER,
                description=f"Y position {y:.2f}m outside workspace limits",
                action_index=0,
                mitigation="Adjust target to stay within workspace"
            )

        if z < self.workspace_limits['z_min'] or z > self.workspace_limits['z_max']:
            return SafetyViolation(
                level=SafetyLevel.DANGER,
                description=f"Z position {z:.2f}m outside workspace limits",
                action_index=0,
                mitigation="Adjust target to stay within height limits"
            )

        return None

    def check_obstacle_clearance(self, action, obstacles: List[Tuple[float, float]]) -> Optional[SafetyViolation]:
        """
        Check if action maintains safe obstacle clearance.

        Args:
            action: Action to check
            obstacles: List of obstacle positions (x, y)

        Returns:
            SafetyViolation if collision risk, None otherwise
        """
        if not obstacles:
            return None

        # Simple forward collision check
        for obstacle_pos in obstacles:
            distance = obstacle_pos[1]  # y is forward distance in simple model

            if distance < self.min_obstacle_distance:
                return SafetyViolation(
                    level=SafetyLevel.DANGER,
                    description=f"Obstacle at {distance:.2f}m - collision risk",
                    action_index=0,
                    mitigation="Navigate around obstacle or reduce movement distance"
                )

            if distance < self.min_obstacle_distance * 2:
                return SafetyViolation(
                    level=SafetyLevel.WARNING,
                    description=f"Obstacle at {distance:.2f}m - approaching limit",
                    action_index=0,
                    mitigation="Proceed with caution"
                )

        return None

    def check_speed_limits(self, action) -> Optional[SafetyViolation]:
        """
        Check if action speed is within limits.

        Args:
            action: Action to check

        Returns:
            SafetyViolation if speed exceeded, None otherwise
        """
        if action.parameters and 'speed' in action.parameters:
            speed = action.parameters['speed']
            if speed > self.max_speed:
                return SafetyViolation(
                    level=SafetyLevel.WARNING,
                    description=f"Speed {speed:.2f} m/s exceeds limit of {self.max_speed} m/s",
                    action_index=0,
                    mitigation=f"Reduce speed to {self.max_speed} m/s or below"
                )

        return None

    def trigger_emergency_stop(self, reason: str):
        """
        Trigger emergency stop.

        Args:
            reason: Reason for emergency stop
        """
        self.emergency_stop_active = True
        print(f"EMERGENCY STOP TRIGGERED: {reason}")

    def release_emergency_stop(self):
        """Release emergency stop."""
        self.emergency_stop_active = False
        print("Emergency stop released")

    def calculate_risk_score(self, violations: List[SafetyViolation]) -> float:
        """
        Calculate overall risk score from violations.

        Args:
            violations: List of safety violations

        Returns:
            float: Risk score 0-1 (lower is safer)
        """
        if not violations:
            return 0.0

        # Weight by severity
        weights = {
            SafetyLevel.SAFE: 0.0,
            SafetyLevel.WARNING: 0.2,
            SafetyLevel.DANGER: 0.5,
            SafetyLevel.CRITICAL: 1.0,
        }

        total_weight = sum(weights[v.level] for v in violations)
        # Normalize and cap at 1.0
        return min(1.0, total_weight / len(violations))

    def generate_recommendations(self, violations: List[SafetyViolation]) -> List[str]:
        """
        Generate safety recommendations based on violations.

        Args:
            violations: Found violations

        Returns:
            List[str]: Safety recommendations
        """
        recommendations = []

        # Add mitigation suggestions
        for v in violations:
            if v.mitigation:
                recommendations.append(v.mitigation)

        # Add general recommendations based on severity
        has_critical = any(v.level == SafetyLevel.CRITICAL for v in violations)
        has_danger = any(v.level == SafetyLevel.DANGER for v in violations)

        if has_critical:
            recommendations.insert(0, "CRITICAL: Resolve critical issues before proceeding")

        if has_danger:
            recommendations.append("Consider reducing movement speed near obstacles")

        if not violations:
            recommendations.append("No safety concerns detected - safe to proceed")

        return list(set(recommendations))  # Remove duplicates
