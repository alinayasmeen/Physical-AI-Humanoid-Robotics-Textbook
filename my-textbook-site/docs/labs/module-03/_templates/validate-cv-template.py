#!/usr/bin/env python3
"""
Lab Validation Script Template for Module 3 - Perception & Sensors

This template provides a framework for validating student lab completion.
Customize the checks for each specific lab exercise.
"""

import os
import sys
import ast
from pathlib import Path
from typing import Tuple, List

def check_file_exists(filepath: str, description: str) -> Tuple[bool, str]:
    """Check if a required file exists"""
    if Path(filepath).exists():
        return True, f"{description} found"
    return False, f"{description} not found at {filepath}"

def check_python_syntax(filepath: str) -> Tuple[bool, str]:
    """Validate Python file syntax"""
    try:
        with open(filepath, 'r') as f:
            ast.parse(f.read())
        return True, "Valid Python syntax"
    except SyntaxError as e:
        return False, f"Syntax error: {e.msg} at line {e.lineno}"
    except FileNotFoundError:
        return False, f"File not found: {filepath}"

def check_imports(filepath: str, required_imports: List[str]) -> Tuple[bool, str]:
    """Check if required imports are present"""
    try:
        with open(filepath, 'r') as f:
            content = f.read()

        missing = [imp for imp in required_imports if imp not in content]
        if missing:
            return False, f"Missing imports: {', '.join(missing)}"
        return True, "All required imports present"
    except FileNotFoundError:
        return False, f"File not found: {filepath}"

def check_class_exists(filepath: str, class_name: str) -> Tuple[bool, str]:
    """Check if a specific class is defined"""
    try:
        with open(filepath, 'r') as f:
            tree = ast.parse(f.read())

        for node in ast.walk(tree):
            if isinstance(node, ast.ClassDef) and node.name == class_name:
                return True, f"Class {class_name} found"
        return False, f"Class {class_name} not found"
    except Exception as e:
        return False, f"Error checking class: {str(e)}"

def check_function_exists(filepath: str, func_name: str) -> Tuple[bool, str]:
    """Check if a specific function is defined"""
    try:
        with open(filepath, 'r') as f:
            tree = ast.parse(f.read())

        for node in ast.walk(tree):
            if isinstance(node, ast.FunctionDef) and node.name == func_name:
                return True, f"Function {func_name} found"
        return False, f"Function {func_name} not found"
    except Exception as e:
        return False, f"Error checking function: {str(e)}"

def run_checks(checks: List[Tuple[callable, tuple, str]]) -> bool:
    """Run a list of validation checks"""
    all_passed = True

    print("=" * 50)
    print("Lab Validation Results")
    print("=" * 50)

    for check_func, args, description in checks:
        passed, message = check_func(*args)
        status = "[PASS]" if passed else "[FAIL]"
        print(f"{status} {description}: {message}")

        if not passed:
            all_passed = False

    print("=" * 50)

    if all_passed:
        print("Lab Complete! All checks passed.")
    else:
        print("Some checks failed. Please review and try again.")

    return all_passed

# Example usage - customize for each lab
if __name__ == '__main__':
    # Define checks for this lab
    checks = [
        # (function, (args,), "Description")
        (check_file_exists, ("starter/main.py", "Main script"), "Main script exists"),
        (check_python_syntax, ("starter/main.py",), "Valid Python syntax"),
        (check_imports, ("starter/main.py", ["cv2", "rclpy"]), "Required imports"),
        # Add more checks as needed
    ]

    success = run_checks(checks)
    sys.exit(0 if success else 1)
