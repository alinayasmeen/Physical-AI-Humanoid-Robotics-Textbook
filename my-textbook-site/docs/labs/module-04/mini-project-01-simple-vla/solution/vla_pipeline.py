#!/usr/bin/env python3
"""
VLA Pipeline - Main orchestrator for the Simple VLA System
This module coordinates speech recognition and action execution.

SOLUTION: Complete implementation with all functionality.
"""

import rclpy
from rclpy.node import Node
from speech_recognizer import SpeechRecognizer
from action_executor import ActionExecutor


class VLAPipeline(Node):
    """
    Main VLA Pipeline orchestrator that coordinates:
    - Speech input processing
    - Command parsing
    - Action execution
    """

    def __init__(self):
        super().__init__('vla_pipeline')
        self.get_logger().info('Initializing VLA Pipeline...')

        # Initialize speech recognizer
        self.speech_recognizer = SpeechRecognizer()

        # Initialize action executor
        self.action_executor = ActionExecutor()

        # Supported commands mapping
        self.commands = {
            'move forward': self.move_forward,
            'move backward': self.move_backward,
            'turn left': self.turn_left,
            'turn right': self.turn_right,
            'stop': self.stop,
        }

        self.get_logger().info('VLA Pipeline initialized successfully')

    def process_audio_command(self, audio_input):
        """
        Process audio input and execute corresponding action.

        Args:
            audio_input: Audio data to process

        Returns:
            bool: True if command executed successfully, False otherwise
        """
        try:
            # Step 1: Convert audio to text
            recognized_text = self.speech_recognizer.process_audio(audio_input)
            if not recognized_text:
                self.get_logger().warning('No speech recognized')
                return False

            self.get_logger().info(f'Recognized: "{recognized_text}"')

            # Step 2: Parse the command
            matched_command = self.parse_command(recognized_text)
            if matched_command is None:
                self.get_logger().warning(f'Unknown command: "{recognized_text}"')
                return False

            # Step 3: Execute the corresponding action
            action_func = self.commands[matched_command]
            action_func()

            self.get_logger().info(f'Executed command: "{matched_command}"')
            return True

        except Exception as e:
            self.get_logger().error(f'Error processing command: {e}')
            return False

    def parse_command(self, text):
        """
        Parse recognized text to find matching command.

        Args:
            text: Recognized text from speech

        Returns:
            str: Matched command or None if not recognized
        """
        # Convert to lowercase for comparison
        text_lower = text.lower().strip()

        # Check for exact match
        if text_lower in self.commands:
            return text_lower

        # Check for partial match (command is contained in text)
        for command in self.commands:
            if command in text_lower:
                return command

        return None

    def move_forward(self):
        """Execute move forward action."""
        self.get_logger().info('Executing: Move Forward')
        return self.action_executor.move_forward()

    def move_backward(self):
        """Execute move backward action."""
        self.get_logger().info('Executing: Move Backward')
        return self.action_executor.move_backward()

    def turn_left(self):
        """Execute turn left action."""
        self.get_logger().info('Executing: Turn Left')
        return self.action_executor.turn_left()

    def turn_right(self):
        """Execute turn right action."""
        self.get_logger().info('Executing: Turn Right')
        return self.action_executor.turn_right()

    def stop(self):
        """Execute stop action."""
        self.get_logger().info('Executing: Stop')
        return self.action_executor.stop()


def main(args=None):
    rclpy.init(args=args)
    pipeline = VLAPipeline()

    try:
        rclpy.spin(pipeline)
    except KeyboardInterrupt:
        pass
    finally:
        pipeline.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
