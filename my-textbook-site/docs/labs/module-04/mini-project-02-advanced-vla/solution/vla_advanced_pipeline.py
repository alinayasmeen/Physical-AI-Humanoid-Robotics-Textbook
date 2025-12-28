#!/usr/bin/env python3
"""
Advanced VLA Pipeline - Main orchestrator for the Advanced VLA System
Integrates vision, language, and action with safety monitoring.

SOLUTION: Complete implementation with all functionality.
"""

import time
import rclpy
from rclpy.node import Node
from vision_processor import VisionProcessor, MockVisionProcessor
from cognitive_planner import CognitivePlanner, MockCognitivePlanner
from safety_monitor import SafetyMonitor, SafetyLevel
from geometry_msgs.msg import Twist


class AdvancedVLAPipeline(Node):
    """
    Advanced VLA Pipeline with object-aware processing,
    LLM-based planning, and safety monitoring.
    """

    def __init__(self, use_mock=True):
        super().__init__('advanced_vla_pipeline')
        self.get_logger().info('Initializing Advanced VLA Pipeline...')

        # Initialize components (use mock versions for testing)
        if use_mock:
            self.vision_processor = MockVisionProcessor()
            self.cognitive_planner = MockCognitivePlanner()
        else:
            self.vision_processor = VisionProcessor()
            self.cognitive_planner = CognitivePlanner()

        self.safety_monitor = SafetyMonitor()

        # Create velocity publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Pipeline state
        self.current_state = 'IDLE'
        self.last_scene_context = None

        self.get_logger().info('Advanced VLA Pipeline initialized successfully')

    def process_multimodal_command(self, command_text, camera_image=None):
        """
        Process a multimodal command with vision context.

        Args:
            command_text: Text command (from speech recognition)
            camera_image: Optional camera image for context

        Returns:
            bool: True if command executed successfully
        """
        try:
            # Step 1: Update state to PROCESSING
            self.current_state = 'PROCESSING'
            self.get_logger().info(f'Processing command: "{command_text}"')

            # Step 2: Get scene context from vision processor
            scene_context = self.get_scene_context(camera_image)
            self.last_scene_context = scene_context
            self.get_logger().info(f'Scene analyzed: {len(scene_context.objects)} objects detected')

            # Step 3: Generate action plan using cognitive planner
            action_plan = self.generate_action_plan(command_text, scene_context)
            self.get_logger().info(f'Generated plan with {len(action_plan.actions)} actions')
            self.get_logger().info(f'Plan reasoning: {action_plan.reasoning}')

            # Step 4: Validate plan with safety monitor
            is_safe, violations = self.validate_plan(action_plan, scene_context)

            if not is_safe:
                self.current_state = 'SAFETY_VIOLATION'
                error_msg = self.handle_safety_violation(action_plan, violations)
                self.get_logger().warning(f'Safety violation: {error_msg}')
                return False

            # Step 5: Execute plan if safe
            self.current_state = 'EXECUTING'
            success = self.execute_plan(action_plan)

            # Step 6: Update state based on result
            if success:
                self.current_state = 'IDLE'
                self.get_logger().info('Command executed successfully')
            else:
                self.current_state = 'ERROR'
                self.get_logger().error('Command execution failed')

            return success

        except Exception as e:
            self.current_state = 'ERROR'
            self.get_logger().error(f'Error processing command: {e}')
            return False

    def get_scene_context(self, camera_image=None):
        """
        Analyze the environment and get scene context.

        Args:
            camera_image: Camera image to analyze

        Returns:
            SceneContext: Scene context with detected objects and spatial info
        """
        return self.vision_processor.analyze_scene(camera_image)

    def generate_action_plan(self, command_text, scene_context):
        """
        Generate an action plan based on command and context.

        Args:
            command_text: The command to execute
            scene_context: Environmental context

        Returns:
            ActionPlan: Sequence of actions to execute
        """
        return self.cognitive_planner.plan_actions(command_text, scene_context)

    def validate_plan(self, action_plan, scene_context):
        """
        Validate the action plan for safety.

        Args:
            action_plan: Sequence of actions
            scene_context: Current environmental context

        Returns:
            Tuple[bool, List]: (is_safe, violations)
        """
        safety_report = self.safety_monitor.validate_action_plan(action_plan, scene_context)
        return safety_report.is_safe, safety_report.violations

    def execute_plan(self, action_plan):
        """
        Execute a validated action plan.

        Args:
            action_plan: Sequence of actions to execute

        Returns:
            bool: True if plan executed successfully
        """
        for i, action in enumerate(action_plan.actions):
            self.get_logger().info(f'Executing action {i+1}/{len(action_plan.actions)}: {action.action_type.value}')

            success = self._execute_single_action(action)
            if not success:
                self.get_logger().error(f'Action {i+1} failed')
                # Stop robot on failure
                self._stop_robot()
                return False

            # Small delay between actions
            time.sleep(0.1)

        return True

    def _execute_single_action(self, action):
        """Execute a single action."""
        try:
            twist = Twist()

            if action.action_type.value == 'move_forward':
                twist.linear.x = 0.5
            elif action.action_type.value == 'move_backward':
                twist.linear.x = -0.5
            elif action.action_type.value == 'turn_left':
                twist.angular.z = 0.5
            elif action.action_type.value == 'turn_right':
                twist.angular.z = -0.5
            elif action.action_type.value == 'stop':
                pass  # All zeros
            else:
                self.get_logger().info(f'Simulating action: {action.action_type.value}')
                return True

            self.cmd_vel_pub.publish(twist)
            time.sleep(action.duration)
            self._stop_robot()

            return True

        except Exception as e:
            self.get_logger().error(f'Error executing action: {e}')
            return False

    def _stop_robot(self):
        """Send stop command to robot."""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    def handle_safety_violation(self, action_plan, violations):
        """
        Handle safety violations in the action plan.

        Args:
            action_plan: The violating plan
            violations: List of safety violations

        Returns:
            str: User-friendly error message
        """
        critical_violations = [v for v in violations if v.level == SafetyLevel.CRITICAL]
        danger_violations = [v for v in violations if v.level == SafetyLevel.DANGER]

        if critical_violations:
            return f"CRITICAL: {critical_violations[0].description}"
        elif danger_violations:
            return f"DANGER: {danger_violations[0].description}"
        else:
            return f"Safety check failed: {violations[0].description}"


def main(args=None):
    rclpy.init(args=args)
    pipeline = AdvancedVLAPipeline(use_mock=True)

    try:
        rclpy.spin(pipeline)
    except KeyboardInterrupt:
        pass
    finally:
        pipeline.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
