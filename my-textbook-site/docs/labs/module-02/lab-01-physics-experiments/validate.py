#!/usr/bin/env python3
"""
Lab 1 Validation Script - Physics Experiments

Validates that Lab 1 has been completed successfully.

Usage:
    python3 validate.py
"""

import os
import sys
import ast
import subprocess


def check_file_exists(filepath, description):
    """Check if a file exists."""
    if os.path.exists(filepath):
        print(f"[PASS] {description}")
        return True
    else:
        print(f"[FAIL] {description} - File not found: {filepath}")
        return False


def check_python_syntax(filepath, description):
    """Check if Python file has valid syntax."""
    try:
        with open(filepath, 'r') as f:
            source = f.read()
        ast.parse(source)
        print(f"[PASS] {description}")
        return True
    except SyntaxError as e:
        print(f"[FAIL] {description} - Syntax error: {e}")
        return False
    except FileNotFoundError:
        print(f"[FAIL] {description} - File not found")
        return False


def check_todos_completed(filepath, description):
    """Check if TODO comments have been addressed in solution."""
    try:
        with open(filepath, 'r') as f:
            content = f.read()

        # Solution should have no TODOs remaining
        todo_count = content.lower().count('# todo')

        if todo_count == 0:
            print(f"[PASS] {description}")
            return True
        else:
            print(f"[WARN] {description} - {todo_count} TODOs remaining")
            return True  # Warning, not failure
    except FileNotFoundError:
        print(f"[FAIL] {description} - File not found")
        return False


def check_sdf_file(filepath, description):
    """Check if SDF file is valid XML."""
    try:
        import xml.etree.ElementTree as ET
        tree = ET.parse(filepath)
        root = tree.getroot()

        if root.tag == 'sdf':
            print(f"[PASS] {description}")
            return True
        else:
            print(f"[FAIL] {description} - Not a valid SDF file")
            return False
    except FileNotFoundError:
        print(f"[SKIP] {description} - SDF file optional")
        return True  # SDF file is optional for this lab
    except Exception as e:
        print(f"[FAIL] {description} - {e}")
        return False


def main():
    """Run all validation checks."""
    print("=" * 50)
    print("Lab 1 Validation: Physics Experiments")
    print("=" * 50)
    print()

    lab_dir = os.path.dirname(os.path.abspath(__file__))
    results = []

    # Check starter files
    print("Checking starter files...")
    results.append(check_file_exists(
        os.path.join(lab_dir, 'starter', 'physics_observer.py'),
        "Starter script exists"
    ))
    results.append(check_python_syntax(
        os.path.join(lab_dir, 'starter', 'physics_observer.py'),
        "Starter script has valid syntax"
    ))

    print()

    # Check solution files
    print("Checking solution files...")
    results.append(check_file_exists(
        os.path.join(lab_dir, 'solution', 'physics_observer.py'),
        "Solution script exists"
    ))
    results.append(check_python_syntax(
        os.path.join(lab_dir, 'solution', 'physics_observer.py'),
        "Solution script has valid syntax"
    ))
    results.append(check_todos_completed(
        os.path.join(lab_dir, 'solution', 'physics_observer.py'),
        "Solution has no remaining TODOs"
    ))

    print()

    # Check README
    print("Checking documentation...")
    results.append(check_file_exists(
        os.path.join(lab_dir, 'README.md'),
        "Lab README exists"
    ))

    print()

    # Summary
    print("=" * 50)
    passed = sum(results)
    total = len(results)

    if passed == total:
        print(f"Lab 1 Validation: PASSED ({passed}/{total})")
        print("All required files are present and valid.")
        return 0
    else:
        print(f"Lab 1 Validation: INCOMPLETE ({passed}/{total})")
        print("Some checks failed. Review the output above.")
        return 1


if __name__ == '__main__':
    sys.exit(main())
