#!/usr/bin/env python3
"""
Lab 1: Speech to Command
Module 4 - Vision-Language-Action (VLA) Systems

This module converts spoken human commands into structured robot tasks
using Whisper ASR and natural language parsing.

Complete the TODOs to implement the speech-to-command pipeline.
"""

import argparse
import json
from dataclasses import dataclass, asdict
from typing import Optional

import numpy as np

# TODO: Uncomment these imports after installing dependencies
# import whisper
# import sounddevice as sd
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String


@dataclass
class RobotCommand:
    """Structured representation of a robot command."""
    action: str
    target: Optional[str] = None
    location: Optional[str] = None
    parameters: dict = None

    def __post_init__(self):
        if self.parameters is None:
            self.parameters = {}

    def to_dict(self) -> dict:
        return asdict(self)

    def to_json(self) -> str:
        return json.dumps(self.to_dict())


# TODO 1: Implement audio recording
def record_audio(duration: float = 5.0, sample_rate: int = 16000) -> np.ndarray:
    """
    Record audio from microphone for specified duration.

    Args:
        duration: Recording duration in seconds
        sample_rate: Audio sample rate in Hz

    Returns:
        numpy array of audio samples
    """
    # YOUR CODE HERE
    # Hint: Use sounddevice.rec() to record audio
    # Don't forget to wait for recording to complete with sd.wait()
    raise NotImplementedError("Implement audio recording")


# TODO 2: Implement speech-to-text conversion
def transcribe_audio(audio: np.ndarray, model) -> str:
    """
    Transcribe audio using Whisper model.

    Args:
        audio: numpy array of audio samples
        model: Loaded Whisper model

    Returns:
        Transcribed text string
    """
    # YOUR CODE HERE
    # Hint: Normalize audio to float32 in range [-1, 1]
    # Use model.transcribe() with language='en'
    raise NotImplementedError("Implement audio transcription")


def extract_object(text: str) -> Optional[str]:
    """
    Extract target object from command text.

    Args:
        text: Command text to parse

    Returns:
        Extracted object name or None
    """
    # Common object patterns
    object_patterns = [
        'the (red|blue|green|yellow|white|black) (ball|box|cube|cylinder|bottle)',
        'the (ball|box|cube|cylinder|bottle)',
        'a (ball|box|cube|cylinder|bottle)',
    ]

    import re
    for pattern in object_patterns:
        match = re.search(pattern, text.lower())
        if match:
            return match.group(0)

    return None


def extract_location(text: str) -> Optional[str]:
    """
    Extract location from command text.

    Args:
        text: Command text to parse

    Returns:
        Extracted location or None
    """
    # Common location patterns
    location_patterns = [
        r'(on|to|from|at|near|beside) the (table|shelf|floor|counter|desk|box)',
        r'(on|to|from|at|near|beside) (table|shelf|floor|counter|desk|box)',
        r'(left|right|forward|backward|up|down)',
    ]

    import re
    for pattern in location_patterns:
        match = re.search(pattern, text.lower())
        if match:
            return match.group(0)

    return None


# TODO 3: Implement command parsing
def parse_command(text: str) -> RobotCommand:
    """
    Parse transcribed text into structured robot command.

    Args:
        text: Transcribed command text

    Returns:
        RobotCommand with parsed action, target, and location
    """
    text = text.lower().strip()

    # Define action keywords
    action_patterns = {
        'pick': ['pick up', 'pick', 'grab', 'grasp', 'take', 'get'],
        'place': ['place', 'put', 'set', 'drop', 'release'],
        'move': ['move', 'go', 'navigate', 'drive', 'walk'],
        'look': ['look', 'find', 'search', 'locate', 'see'],
        'stop': ['stop', 'halt', 'pause', 'wait', 'freeze']
    }

    # YOUR CODE HERE
    # 1. Extract the action by matching against action_patterns
    # 2. Call extract_object() to get the target
    # 3. Call extract_location() to get the location
    # 4. Return a RobotCommand with the extracted information
    raise NotImplementedError("Implement command parsing")


# TODO 4: Implement ROS 2 node (optional but recommended)
# Uncomment and complete this class for ROS 2 integration

# class SpeechCommandNode(Node):
#     """ROS 2 node for speech command processing."""
#
#     def __init__(self):
#         super().__init__('speech_command_node')
#         # YOUR CODE HERE
#         # 1. Create a publisher for /robot/command topic
#         # 2. Load the Whisper model
#         # 3. Create a timer that calls listen_callback periodically
#         raise NotImplementedError("Implement ROS 2 node initialization")
#
#     def listen_callback(self):
#         """Periodically listen for commands."""
#         # YOUR CODE HERE
#         # 1. Record audio
#         # 2. Transcribe audio
#         # 3. Parse command
#         # 4. Publish command to ROS 2 topic
#         raise NotImplementedError("Implement listen callback")


def test_audio():
    """Test audio recording functionality."""
    print("Testing audio recording...")
    try:
        audio = record_audio(duration=3.0)
        print(f"Recorded {len(audio)} samples")
        print(f"Audio shape: {audio.shape}")
        print(f"Audio dtype: {audio.dtype}")
        print("Audio recording test PASSED")
    except Exception as e:
        print(f"Audio recording test FAILED: {e}")


def test_transcribe():
    """Test transcription functionality."""
    print("Testing transcription...")
    try:
        import whisper
        model = whisper.load_model('base')
        audio = record_audio(duration=5.0)
        text = transcribe_audio(audio, model)
        print(f"Transcribed text: {text}")
        print("Transcription test PASSED")
    except Exception as e:
        print(f"Transcription test FAILED: {e}")


def test_parse(text: str):
    """Test command parsing functionality."""
    print(f"Testing command parsing for: '{text}'")
    try:
        command = parse_command(text)
        print(f"Parsed command:")
        print(f"  Action: {command.action}")
        print(f"  Target: {command.target}")
        print(f"  Location: {command.location}")
        print(f"  Parameters: {command.parameters}")
        print("Command parsing test PASSED")
    except Exception as e:
        print(f"Command parsing test FAILED: {e}")


def main():
    """Main entry point for the speech-to-command module."""
    parser = argparse.ArgumentParser(description='Speech to Command Lab')
    parser.add_argument('--test-audio', action='store_true',
                       help='Test audio recording')
    parser.add_argument('--test-transcribe', action='store_true',
                       help='Test audio transcription')
    parser.add_argument('--test-parse', type=str,
                       help='Test command parsing with given text')
    parser.add_argument('--ros2', action='store_true',
                       help='Run as ROS 2 node')

    args = parser.parse_args()

    if args.test_audio:
        test_audio()
    elif args.test_transcribe:
        test_transcribe()
    elif args.test_parse:
        test_parse(args.test_parse)
    elif args.ros2:
        print("ROS 2 mode not yet implemented")
        # Uncomment below when ROS 2 node is implemented
        # rclpy.init()
        # node = SpeechCommandNode()
        # rclpy.spin(node)
        # node.destroy_node()
        # rclpy.shutdown()
    else:
        parser.print_help()


if __name__ == '__main__':
    main()
