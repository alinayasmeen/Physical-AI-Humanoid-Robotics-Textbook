#!/usr/bin/env python3
"""Lab 3 Validation Script - Sensor Fusion"""

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
    starter_file = "starter/sensor_fusion.py"

    checks = [
        ("File exists", lambda: check_file_exists(starter_file)),
        ("Valid syntax", lambda: check_python_syntax(starter_file)),
        ("ApproximateTimeSynchronizer", lambda: check_pattern(
            starter_file, "ApproximateTimeSynchronizer", "Time synchronizer")),
        ("sync_callback", lambda: check_pattern(
            starter_file, "sync_callback", "Sync callback method")),
        ("camera_matrix", lambda: check_pattern(
            starter_file, "camera_matrix", "Camera matrix")),
        ("cv_bridge", lambda: check_pattern(
            starter_file, "CvBridge", "CV Bridge")),
        ("Point projection", lambda: check_pattern(
            starter_file, "project_points", "Point projection function")),
    ]

    print("=" * 50)
    print("Lab 3 Validation: Sensor Fusion")
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
        print("Lab 3 Complete!")
        return 0
    else:
        print("Some checks failed. Complete the TODOs.")
        return 1


if __name__ == '__main__':
    os.chdir(Path(__file__).parent)
    sys.exit(main())
