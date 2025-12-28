#!/usr/bin/env python3
"""
Validation script for Lab 3: VLA Pipeline Integration
Verifies that the lab implementation meets all requirements.
"""

import ast
import os
import sys

STARTER_FILE = "starter/vla_pipeline.py"


def check_file_exists():
    """Check if starter file exists."""
    if os.path.exists(STARTER_FILE):
        print("[PASS] vla_pipeline.py exists in starter/")
        return True
    else:
        print("[FAIL] vla_pipeline.py not found in starter/")
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


def check_state_machine():
    """Check if state machine is implemented."""
    with open(STARTER_FILE, 'r') as f:
        source = f.read()

    if 'class PipelineState' in source and 'Enum' in source:
        states = ['IDLE', 'LISTENING', 'TRANSCRIBING', 'PARSING',
                  'PLANNING', 'VALIDATING', 'PERCEIVING', 'EXECUTING',
                  'RECOVERING', 'COMPLETED', 'FAILED']
        found = sum(1 for s in states if s in source)
        if found == len(states):
            print("[PASS] State machine implemented correctly")
            return True
        else:
            print(f"[WARN] Only {found}/{len(states)} states found")
            return True
    else:
        print("[FAIL] State machine not found")
        return False


def check_speech_recognition():
    """Check if speech recognition is integrated."""
    with open(STARTER_FILE, 'r') as f:
        source = f.read()

    if 'SpeechRecognizer' in source and '_listen_for_command' in source:
        print("[PASS] Speech recognition integrated")
        return True
    else:
        print("[FAIL] Speech recognition not integrated")
        return False


def check_cognitive_planning():
    """Check if cognitive planning is integrated."""
    with open(STARTER_FILE, 'r') as f:
        source = f.read()

    if 'CognitivePlanner' in source and '_generate_plan' in source:
        print("[PASS] Cognitive planning integrated")
        return True
    else:
        print("[FAIL] Cognitive planning not integrated")
        return False


def check_vision_system():
    """Check if vision system is integrated."""
    with open(STARTER_FILE, 'r') as f:
        source = f.read()

    if 'VisionSystem' in source and '_update_perception' in source:
        print("[PASS] Vision system integrated")
        return True
    else:
        print("[FAIL] Vision system not integrated")
        return False


def check_action_execution():
    """Check if action execution is implemented."""
    with open(STARTER_FILE, 'r') as f:
        source = f.read()

    if 'ActionExecutor' in source and '_execute_step' in source:
        print("[PASS] Action execution implemented")
        return True
    else:
        print("[FAIL] Action execution not implemented")
        return False


def check_error_recovery():
    """Check if error recovery is implemented."""
    with open(STARTER_FILE, 'r') as f:
        source = f.read()

    if '_handle_recovery' in source:
        print("[PASS] Error recovery implemented")
        return True
    else:
        print("[FAIL] Error recovery not implemented")
        return False


def check_ros2_structure():
    """Check if ROS 2 node structure is present."""
    with open(STARTER_FILE, 'r') as f:
        source = f.read()

    # ROS 2 integration is optional for this lab
    if 'rclpy' in source or 'Node' in source:
        print("[PASS] ROS 2 node structure present")
    else:
        print("[WARN] ROS 2 node structure not found (optional)")

    return True


def check_pipeline_class():
    """Check if VLAPipeline class is properly defined."""
    with open(STARTER_FILE, 'r') as f:
        source = f.read()

    required_methods = [
        'transition',
        'run',
        'start',
        '_listen_for_command',
        '_transcribe_audio',
        '_parse_command',
        '_generate_plan',
        '_validate_plan',
        '_update_perception',
        '_execute_step',
        '_handle_recovery'
    ]

    if 'class VLAPipeline' in source:
        found = sum(1 for m in required_methods if f'def {m}' in source)
        if found == len(required_methods):
            print(f"[PASS] VLAPipeline class has all {len(required_methods)} methods")
            return True
        else:
            print(f"[WARN] VLAPipeline has {found}/{len(required_methods)} methods")
            return True
    else:
        print("[FAIL] VLAPipeline class not found")
        return False


def main():
    """Run all validation checks."""
    print("=" * 50)
    print("Lab 3: VLA Pipeline Integration - Validation")
    print("=" * 50)
    print()

    checks = [
        check_file_exists,
        check_syntax,
        check_state_machine,
        check_speech_recognition,
        check_cognitive_planning,
        check_vision_system,
        check_action_execution,
        check_error_recovery,
        check_ros2_structure,
        check_pipeline_class,
    ]

    passed = 0
    failed = 0

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
        print("\nLab 3 Complete!")
        return 0
    else:
        print("\nPlease fix the failing checks before proceeding.")
        return 1


if __name__ == '__main__':
    sys.exit(main())
