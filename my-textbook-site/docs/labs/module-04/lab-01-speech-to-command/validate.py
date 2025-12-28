#!/usr/bin/env python3
"""
Validation script for Lab 1: Speech to Command
Verifies that the lab implementation meets all requirements.
"""

import ast
import os
import sys

STARTER_FILE = "starter/speech_to_command.py"

def check_file_exists():
    """Check if starter file exists."""
    if os.path.exists(STARTER_FILE):
        print("[PASS] speech_to_command.py exists in starter/")
        return True
    else:
        print("[FAIL] speech_to_command.py not found in starter/")
        return False


def check_syntax():
    """Check if file has valid Python syntax."""
    try:
        with open(STARTER_FILE, 'r') as f:
            source = f.read()
        ast.parse(source)
        print("[PASS] Valid Python syntax")
        return True
    except SyntaxError as e:
        print(f"[FAIL] Syntax error: {e}")
        return False


def check_imports():
    """Check if required imports are present."""
    with open(STARTER_FILE, 'r') as f:
        source = f.read()

    # Check for whisper import (commented is OK for starter)
    if 'whisper' in source:
        print("[PASS] Whisper import present")
    else:
        print("[WARN] Whisper import not found (may be commented)")

    return True


def check_audio_function():
    """Check if audio recording function is implemented."""
    with open(STARTER_FILE, 'r') as f:
        source = f.read()

    if 'def record_audio' in source:
        if 'NotImplementedError' in source.split('def record_audio')[1].split('def ')[0]:
            print("[WARN] Audio recording function not yet implemented")
        else:
            print("[PASS] Audio recording function implemented")
        return True
    else:
        print("[FAIL] Audio recording function not found")
        return False


def check_transcription_function():
    """Check if transcription function is implemented."""
    with open(STARTER_FILE, 'r') as f:
        source = f.read()

    if 'def transcribe_audio' in source:
        if 'NotImplementedError' in source.split('def transcribe_audio')[1].split('def ')[0]:
            print("[WARN] Transcription function not yet implemented")
        else:
            print("[PASS] Transcription function implemented")
        return True
    else:
        print("[FAIL] Transcription function not found")
        return False


def check_parsing_function():
    """Check if command parsing function is implemented."""
    with open(STARTER_FILE, 'r') as f:
        source = f.read()

    if 'def parse_command' in source:
        if 'NotImplementedError' in source.split('def parse_command')[1].split('def ')[0]:
            print("[WARN] Command parsing function not yet implemented")
        else:
            print("[PASS] Command parsing function implemented")
        return True
    else:
        print("[FAIL] Command parsing function not found")
        return False


def check_ros2_structure():
    """Check if ROS 2 node structure is present."""
    with open(STARTER_FILE, 'r') as f:
        source = f.read()

    if 'SpeechCommandNode' in source:
        print("[PASS] ROS 2 node structure present")
        return True
    else:
        print("[WARN] ROS 2 node structure not found (optional)")
        return True


def check_robot_command_class():
    """Check if RobotCommand class is defined."""
    with open(STARTER_FILE, 'r') as f:
        source = f.read()

    if 'class RobotCommand' in source:
        print("[PASS] RobotCommand class defined")
        return True
    else:
        print("[FAIL] RobotCommand class not found")
        return False


def main():
    """Run all validation checks."""
    print("=" * 50)
    print("Lab 1: Speech to Command - Validation")
    print("=" * 50)
    print()

    checks = [
        check_file_exists,
        check_syntax,
        check_imports,
        check_robot_command_class,
        check_audio_function,
        check_transcription_function,
        check_parsing_function,
        check_ros2_structure,
    ]

    passed = 0
    failed = 0
    warned = 0

    for check in checks:
        result = check()
        if result:
            passed += 1
        else:
            failed += 1

    print()
    print("=" * 50)
    print(f"Results: {passed} passed, {failed} failed")
    print("=" * 50)

    if failed == 0:
        print("\nLab 1 Complete!")
        return 0
    else:
        print("\nPlease fix the failing checks before proceeding.")
        return 1


if __name__ == '__main__':
    sys.exit(main())
