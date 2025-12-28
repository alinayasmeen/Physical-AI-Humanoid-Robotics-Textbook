#!/usr/bin/env python3
"""
Lab 1 Validation Script
Checks that the image subscriber implementation is complete.
"""

import os
import sys
import ast
from pathlib import Path


def check_file_exists(filepath: str) -> tuple:
    """Check if a required file exists."""
    if Path(filepath).exists():
        return True, "File found"
    return False, f"File not found: {filepath}"


def check_python_syntax(filepath: str) -> tuple:
    """Validate Python file syntax."""
    try:
        with open(filepath, 'r') as f:
            ast.parse(f.read())
        return True, "Valid Python syntax"
    except SyntaxError as e:
        return False, f"Syntax error at line {e.lineno}: {e.msg}"
    except FileNotFoundError:
        return False, "File not found"


def check_import_exists(filepath: str, import_name: str) -> tuple:
    """Check if a specific import exists in the file."""
    try:
        with open(filepath, 'r') as f:
            content = f.read()
        if import_name in content:
            return True, f"'{import_name}' import present"
        return False, f"'{import_name}' import missing"
    except FileNotFoundError:
        return False, "File not found"


def check_pattern_exists(filepath: str, pattern: str, desc: str) -> tuple:
    """Check if a code pattern exists in the file."""
    try:
        with open(filepath, 'r') as f:
            content = f.read()
        if pattern in content:
            return True, f"{desc} found"
        return False, f"{desc} not found"
    except FileNotFoundError:
        return False, "File not found"


def main():
    """Run all validation checks."""
    starter_file = "starter/image_subscriber.py"

    checks = [
        ("File exists", lambda: check_file_exists(starter_file)),
        ("Valid syntax", lambda: check_python_syntax(starter_file)),
        ("cv_bridge import", lambda: check_import_exists(starter_file, "cv_bridge")),
        ("cv2 import", lambda: check_import_exists(starter_file, "cv2")),
        ("Image subscriber", lambda: check_pattern_exists(
            starter_file, "create_subscription", "Image subscriber")),
        ("Image publisher", lambda: check_pattern_exists(
            starter_file, "create_publisher", "Image publisher")),
        ("imgmsg_to_cv2", lambda: check_pattern_exists(
            starter_file, "imgmsg_to_cv2", "ROS to OpenCV conversion")),
        ("cv2_to_imgmsg", lambda: check_pattern_exists(
            starter_file, "cv2_to_imgmsg", "OpenCV to ROS conversion")),
        ("Grayscale conversion", lambda: check_pattern_exists(
            starter_file, "COLOR_BGR2GRAY", "Grayscale conversion")),
        ("Canny edge detection", lambda: check_pattern_exists(
            starter_file, "Canny", "Canny edge detection")),
    ]

    print("=" * 50)
    print("Lab 1 Validation: Image Subscriber")
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
        print("Lab 1 Complete!")
        return 0
    else:
        print("Some checks failed. Please complete the TODOs.")
        return 1


if __name__ == '__main__':
    os.chdir(Path(__file__).parent)
    sys.exit(main())
