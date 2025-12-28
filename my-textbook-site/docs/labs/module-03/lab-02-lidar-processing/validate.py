#!/usr/bin/env python3
"""Lab 2 Validation Script - LiDAR Processing"""

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
    starter_file = "starter/lidar_processor.py"

    checks = [
        ("File exists", lambda: check_file_exists(starter_file)),
        ("Valid syntax", lambda: check_python_syntax(starter_file)),
        ("PointCloud2 import", lambda: check_pattern(
            starter_file, "PointCloud2", "PointCloud2 import")),
        ("read_points used", lambda: check_pattern(
            starter_file, "read_points", "read_points function")),
        ("Ground removal", lambda: check_pattern(
            starter_file, "ground_height", "Ground height parameter")),
        ("Distance filter", lambda: check_pattern(
            starter_file, "min_distance", "Distance filtering")),
        ("NumPy arrays", lambda: check_pattern(
            starter_file, "np.array", "NumPy array conversion")),
    ]

    print("=" * 50)
    print("Lab 2 Validation: LiDAR Processing")
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
        print("Lab 2 Complete!")
        return 0
    else:
        print("Some checks failed. Complete the TODOs.")
        return 1


if __name__ == '__main__':
    os.chdir(Path(__file__).parent)
    sys.exit(main())
