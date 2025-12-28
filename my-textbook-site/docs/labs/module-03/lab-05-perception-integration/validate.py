#!/usr/bin/env python3
"""Lab 5 Validation Script - Perception Integration"""

import os
import sys
import ast
from pathlib import Path


def check_file_exists(filepath):
    if Path(filepath).exists():
        return True, "File found"
    return False, f"File not found: {filepath}"


def check_python_syntax(filepath):
    try:
        with open(filepath, 'r') as f:
            ast.parse(f.read())
        return True, "Valid Python syntax"
    except SyntaxError as e:
        return False, f"Syntax error at line {e.lineno}"
    except FileNotFoundError:
        return False, "File not found"


def check_pattern(filepath, pattern, desc):
    try:
        with open(filepath, 'r') as f:
            content = f.read()
        if pattern in content:
            return True, f"{desc} found"
        return False, f"{desc} not found"
    except FileNotFoundError:
        return False, "File not found"


def main():
    starter_file = "starter/perception_controller.py"

    checks = [
        ("File exists", lambda: check_file_exists(starter_file)),
        ("Valid syntax", lambda: check_python_syntax(starter_file)),
        ("Detection subscription", lambda: check_pattern(
            starter_file, "Detection2DArray", "Detection subscriber")),
        ("Velocity publisher", lambda: check_pattern(
            starter_file, "Twist", "Velocity publisher")),
        ("Visual servoing", lambda: check_pattern(
            starter_file, "compute_velocity", "Visual servoing")),
        ("State machine", lambda: check_pattern(
            starter_file, "ControlState", "State machine")),
        ("Tracker", lambda: check_pattern(
            starter_file, "SimpleTracker", "Object tracker")),
    ]

    print("=" * 50)
    print("Lab 5 Validation: Perception Integration")
    print("=" * 50)

    all_passed = True
    for name, check_func in checks:
        passed, msg = check_func()
        status = "[PASS]" if passed else "[FAIL]"
        print(f"{status} {name}: {msg}")
        if not passed:
            all_passed = False

    print("=" * 50)
    if all_passed:
        print("Lab 5 Complete! Module 3 Finished!")
        return 0
    else:
        print("Some checks failed. Complete the TODOs.")
        return 1


if __name__ == '__main__':
    os.chdir(Path(__file__).parent)
    sys.exit(main())
