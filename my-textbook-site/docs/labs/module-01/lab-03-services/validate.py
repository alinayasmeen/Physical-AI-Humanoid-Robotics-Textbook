#!/usr/bin/env python3
"""
Lab 3 Validation Script
Checks if service server and client are implemented correctly
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


def check_server_elements(file_path):
    """Check if server has required elements."""
    try:
        with open(file_path, 'r') as f:
            code = f.read()
    except FileNotFoundError:
        return False, "File not found"

    checks = [
        ("create_service", "create_service() call missing"),
        ("SetBool", "SetBool service type missing"),
        ("/get_state", "Service name '/get_state' not found"),
        ("response.success", "Response success field not set"),
        ("response.message", "Response message field not set"),
        ("return response", "Response not returned"),
    ]

    for pattern, error in checks:
        if pattern not in code:
            return False, error

    return True, "All server elements present"


def check_client_elements(file_path):
    """Check if client has required elements."""
    try:
        with open(file_path, 'r') as f:
            code = f.read()
    except FileNotFoundError:
        return False, "File not found"

    checks = [
        ("create_client", "create_client() call missing"),
        ("SetBool", "SetBool service type missing"),
        ("/get_state", "Service name '/get_state' not found"),
        ("Request()", "Request creation missing"),
        ("call_async", "call_async() call missing"),
    ]

    for pattern, error in checks:
        if pattern not in code:
            return False, error

    return True, "All client elements present"


def main():
    """Run validation checks."""
    print("=" * 50)
    print("Lab 3: Services - Validation")
    print("=" * 50)

    server_file = sys.argv[1] if len(sys.argv) > 1 else 'starter/state_server.py'
    client_file = sys.argv[2] if len(sys.argv) > 2 else 'starter/state_client.py'

    all_passed = True

    # Check server
    print("\n[Server Checks]")

    print("  Checking syntax...")
    valid, msg = check_syntax(server_file)
    if valid:
        print(f"    ✅ {msg}")
    else:
        print(f"    ❌ {msg}")
        all_passed = False

    print("  Checking required elements...")
    valid, msg = check_server_elements(server_file)
    if valid:
        print(f"    ✅ {msg}")
    else:
        print(f"    ❌ {msg}")
        all_passed = False

    # Check client
    print("\n[Client Checks]")

    print("  Checking syntax...")
    valid, msg = check_syntax(client_file)
    if valid:
        print(f"    ✅ {msg}")
    else:
        print(f"    ❌ {msg}")
        all_passed = False

    print("  Checking required elements...")
    valid, msg = check_client_elements(client_file)
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
