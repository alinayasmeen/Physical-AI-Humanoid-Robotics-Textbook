#!/usr/bin/env python3
"""
Lab 1 Validation Script
Checks if the hello_ros2_node is implemented correctly
"""

import subprocess
import sys
import time


def check_node_running():
    """Check if the node appears in ros2 node list."""
    try:
        result = subprocess.run(
            ['ros2', 'node', 'list'],
            capture_output=True,
            text=True,
            timeout=5
        )
        return '/hello_ros2_node' in result.stdout
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return False


def check_syntax(file_path):
    """Check if the Python file has valid syntax."""
    try:
        with open(file_path, 'r') as f:
            code = f.read()
        compile(code, file_path, 'exec')
        return True, "Valid syntax"
    except SyntaxError as e:
        return False, f"Syntax error: {e}"
    except FileNotFoundError:
        return False, "File not found"


def check_required_elements(file_path):
    """Check if required code elements are present."""
    try:
        with open(file_path, 'r') as f:
            code = f.read()
    except FileNotFoundError:
        return False, "File not found"

    checks = [
        ("super().__init__", "Node initialization missing"),
        ("get_logger().info", "Logger call missing"),
        ("create_timer", "Timer creation missing"),
        ("rclpy.init", "rclpy.init() missing"),
        ("rclpy.spin", "rclpy.spin() missing"),
    ]

    for pattern, error in checks:
        if pattern not in code:
            return False, error

    return True, "All required elements present"


def main():
    """Run validation checks."""
    print("=" * 50)
    print("Lab 1: Hello ROS 2 - Validation")
    print("=" * 50)

    # Check starter file if provided as argument
    file_to_check = sys.argv[1] if len(sys.argv) > 1 else 'starter/hello_node.py'

    # Check 1: Syntax
    print("\n[1/3] Checking syntax...")
    valid, msg = check_syntax(file_to_check)
    if valid:
        print(f"  ✅ {msg}")
    else:
        print(f"  ❌ {msg}")
        sys.exit(1)

    # Check 2: Required elements
    print("\n[2/3] Checking required elements...")
    valid, msg = check_required_elements(file_to_check)
    if valid:
        print(f"  ✅ {msg}")
    else:
        print(f"  ❌ {msg}")
        sys.exit(1)

    # Check 3: Node running (only if ROS 2 is available)
    print("\n[3/3] Checking if node can be detected...")
    print("  ℹ️  Note: This check requires the node to be running")
    print("      Run: ros2 run hello_ros2 hello_node")

    if check_node_running():
        print("  ✅ Node 'hello_ros2_node' found")
    else:
        print("  ⚠️  Node not detected (may not be running)")

    print("\n" + "=" * 50)
    print("✅ Validation complete!")
    print("=" * 50)


if __name__ == '__main__':
    main()
