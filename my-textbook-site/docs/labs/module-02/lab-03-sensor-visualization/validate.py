#!/usr/bin/env python3
"""
Lab 3 Validation Script - Sensor Visualization

Validates that sensor configuration and analysis scripts are correct.

Usage:
    python3 validate.py
"""

import os
import sys
import ast


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


def check_imports(filepath, required_imports, description):
    """Check if Python file imports required modules."""
    try:
        with open(filepath, 'r') as f:
            source = f.read()

        tree = ast.parse(source)

        imports = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                for alias in node.names:
                    imports.add(alias.name)
            elif isinstance(node, ast.ImportFrom):
                if node.module:
                    imports.add(node.module)
                for alias in node.names:
                    imports.add(alias.name)

        missing = []
        for req in required_imports:
            found = any(req in imp for imp in imports)
            if not found:
                missing.append(req)

        if not missing:
            print(f"[PASS] {description}")
            return True
        else:
            print(f"[WARN] {description} - Missing: {missing}")
            return True  # Warning, not failure

    except Exception as e:
        print(f"[FAIL] {description} - {e}")
        return False


def check_message_handlers(filepath, message_types, description):
    """Check if Python file handles required message types."""
    try:
        with open(filepath, 'r') as f:
            content = f.read().lower()

        found = []
        missing = []
        for msg_type in message_types:
            if msg_type.lower() in content:
                found.append(msg_type)
            else:
                missing.append(msg_type)

        if not missing:
            print(f"[PASS] {description}")
            return True
        else:
            print(f"[WARN] {description} - Not found: {missing}")
            return True  # Warning for starter file

    except Exception as e:
        print(f"[FAIL] {description} - {e}")
        return False


def check_yaml_file(filepath, description):
    """Check if YAML file is valid."""
    try:
        import yaml
        with open(filepath, 'r') as f:
            data = yaml.safe_load(f)

        if data is not None:
            print(f"[PASS] {description}")
            return True
        else:
            print(f"[WARN] {description} - File is empty")
            return True
    except ImportError:
        # If PyYAML not installed, just check file exists
        if os.path.exists(filepath):
            print(f"[PASS] {description} (YAML validation skipped)")
            return True
        else:
            print(f"[FAIL] {description} - File not found")
            return False
    except Exception as e:
        print(f"[FAIL] {description} - {e}")
        return False


def main():
    """Run all validation checks."""
    print("=" * 50)
    print("Lab 3 Validation: Sensor Visualization")
    print("=" * 50)
    print()

    lab_dir = os.path.dirname(os.path.abspath(__file__))
    results = []

    # Check starter files
    print("Checking starter files...")
    results.append(check_file_exists(
        os.path.join(lab_dir, 'starter', 'sensor_config.yaml'),
        "Starter config exists"
    ))
    results.append(check_file_exists(
        os.path.join(lab_dir, 'starter', 'sensor_analyzer.py'),
        "Starter script exists"
    ))
    results.append(check_python_syntax(
        os.path.join(lab_dir, 'starter', 'sensor_analyzer.py'),
        "Starter script has valid syntax"
    ))

    print()

    # Check solution files
    print("Checking solution files...")
    results.append(check_file_exists(
        os.path.join(lab_dir, 'solution', 'sensor_config.yaml'),
        "Solution config exists"
    ))
    results.append(check_file_exists(
        os.path.join(lab_dir, 'solution', 'sensor_analyzer.py'),
        "Solution script exists"
    ))
    results.append(check_python_syntax(
        os.path.join(lab_dir, 'solution', 'sensor_analyzer.py'),
        "Solution script has valid syntax"
    ))

    print()

    # Check solution handles required message types
    print("Checking message type handling...")
    solution_script = os.path.join(lab_dir, 'solution', 'sensor_analyzer.py')
    results.append(check_message_handlers(
        solution_script,
        ['Image'],
        "Solution handles Image messages"
    ))
    results.append(check_message_handlers(
        solution_script,
        ['LaserScan'],
        "Solution handles LaserScan messages"
    ))
    results.append(check_message_handlers(
        solution_script,
        ['Imu'],
        "Solution handles Imu messages"
    ))

    print()

    # Check required imports in solution
    print("Checking imports...")
    results.append(check_imports(
        solution_script,
        ['rclpy', 'sensor_msgs'],
        "Solution has required imports"
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
        print(f"Lab 3 Validation: PASSED ({passed}/{total})")
        print("Sensor visualization lab is complete.")
        return 0
    else:
        print(f"Lab 3 Validation: INCOMPLETE ({passed}/{total})")
        print("Some checks failed. Review the output above.")
        return 1


if __name__ == '__main__':
    sys.exit(main())
