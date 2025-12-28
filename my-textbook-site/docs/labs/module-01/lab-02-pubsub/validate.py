#!/usr/bin/env python3
"""
Lab 2 Validation Script
Checks if publisher and subscriber are implemented correctly
"""

import sys


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


def check_publisher_elements(file_path):
    """Check if publisher has required elements."""
    try:
        with open(file_path, 'r') as f:
            code = f.read()
    except FileNotFoundError:
        return False, "File not found"

    checks = [
        ("create_publisher", "create_publisher() call missing"),
        ("Twist", "Twist message type missing"),
        ("/cmd_vel", "Topic '/cmd_vel' not found"),
        ("create_timer", "Timer for publishing missing"),
        ("publish(", "publish() call missing"),
    ]

    for pattern, error in checks:
        if pattern not in code:
            return False, error

    return True, "All publisher elements present"


def check_subscriber_elements(file_path):
    """Check if subscriber has required elements."""
    try:
        with open(file_path, 'r') as f:
            code = f.read()
    except FileNotFoundError:
        return False, "File not found"

    checks = [
        ("create_subscription", "create_subscription() call missing"),
        ("Twist", "Twist message type missing"),
        ("/cmd_vel", "Topic '/cmd_vel' not found"),
        ("def ", "Callback function missing"),
    ]

    for pattern, error in checks:
        if pattern not in code:
            return False, error

    return True, "All subscriber elements present"


def main():
    """Run validation checks."""
    print("=" * 50)
    print("Lab 2: Publisher-Subscriber - Validation")
    print("=" * 50)

    pub_file = sys.argv[1] if len(sys.argv) > 1 else 'starter/velocity_publisher.py'
    sub_file = sys.argv[2] if len(sys.argv) > 2 else 'starter/velocity_subscriber.py'

    all_passed = True

    # Check publisher
    print("\n[Publisher Checks]")

    print("  Checking syntax...")
    valid, msg = check_syntax(pub_file)
    if valid:
        print(f"    ✅ {msg}")
    else:
        print(f"    ❌ {msg}")
        all_passed = False

    print("  Checking required elements...")
    valid, msg = check_publisher_elements(pub_file)
    if valid:
        print(f"    ✅ {msg}")
    else:
        print(f"    ❌ {msg}")
        all_passed = False

    # Check subscriber
    print("\n[Subscriber Checks]")

    print("  Checking syntax...")
    valid, msg = check_syntax(sub_file)
    if valid:
        print(f"    ✅ {msg}")
    else:
        print(f"    ❌ {msg}")
        all_passed = False

    print("  Checking required elements...")
    valid, msg = check_subscriber_elements(sub_file)
    if valid:
        print(f"    ✅ {msg}")
    else:
        print(f"    ❌ {msg}")
        all_passed = False

    print("\n" + "=" * 50)
    if all_passed:
        print("✅ Validation complete!")
    else:
        print("❌ Some checks failed. Review the errors above.")
        sys.exit(1)
    print("=" * 50)


if __name__ == '__main__':
    main()
