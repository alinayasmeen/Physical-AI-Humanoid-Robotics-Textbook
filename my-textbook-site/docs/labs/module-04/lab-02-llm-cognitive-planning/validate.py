#!/usr/bin/env python3
"""
Validation script for Lab 2: LLM Cognitive Planning
Verifies that the lab implementation meets all requirements.
"""

import ast
import os
import sys

STARTER_FILE = "starter/cognitive_planner.py"


def check_file_exists():
    """Check if starter file exists."""
    if os.path.exists(STARTER_FILE):
        print("[PASS] cognitive_planner.py exists in starter/")
        return True
    else:
        print("[FAIL] cognitive_planner.py not found in starter/")
        return False


def check_syntax():
    """Check if file has valid Python syntax."""
    try:
        with open(STARTER_FILE, 'r') as f:
            source = f.read()
        ast.parse(source)
        print("[PASS] Valid Python syntax")
        return True
    except SyntaxError as e:
        print(f"[FAIL] Syntax error: {e}")
        return False


def check_llm_client():
    """Check if LLM client is configured."""
    with open(STARTER_FILE, 'r') as f:
        source = f.read()

    if 'openai' in source.lower() or 'ollama' in source.lower():
        print("[PASS] LLM client configured")
        return True
    else:
        print("[WARN] LLM client configuration not found")
        return True


def check_prompt_template():
    """Check if prompt template is defined."""
    with open(STARTER_FILE, 'r') as f:
        source = f.read()

    if 'PLANNING_PROMPT' in source:
        print("[PASS] Prompt template defined")
        return True
    else:
        print("[FAIL] Prompt template not found")
        return False


def check_generate_plan():
    """Check if plan generation function is implemented."""
    with open(STARTER_FILE, 'r') as f:
        source = f.read()

    if 'def generate_plan' in source:
        if 'NotImplementedError' in source.split('def generate_plan')[1].split('def ')[0]:
            print("[WARN] Plan generation function not yet implemented")
        else:
            print("[PASS] Plan generation function implemented")
        return True
    else:
        print("[FAIL] Plan generation function not found")
        return False


def check_validate_plan():
    """Check if plan validation function is implemented."""
    with open(STARTER_FILE, 'r') as f:
        source = f.read()

    if 'def validate_plan' in source:
        if 'NotImplementedError' in source.split('def validate_plan')[1].split('def ')[0]:
            print("[WARN] Plan validation function not yet implemented")
        else:
            print("[PASS] Plan validation function implemented")
        return True
    else:
        print("[FAIL] Plan validation function not found")
        return False


def check_replan_function():
    """Check if re-planning function is implemented."""
    with open(STARTER_FILE, 'r') as f:
        source = f.read()

    if 'def replan_on_failure' in source:
        if 'NotImplementedError' in source.split('def replan_on_failure')[1].split('def ')[0]:
            print("[WARN] Re-planning function not yet implemented")
        else:
            print("[PASS] Re-planning function implemented")
        return True
    else:
        print("[FAIL] Re-planning function not found")
        return False


def check_data_classes():
    """Check if data classes are defined."""
    with open(STARTER_FILE, 'r') as f:
        source = f.read()

    classes = ['PlanStep', 'CognitivePlan']
    found = 0

    for cls in classes:
        if f'class {cls}' in source:
            found += 1

    if found == len(classes):
        print("[PASS] Data classes defined")
        return True
    else:
        print(f"[WARN] Only {found}/{len(classes)} data classes found")
        return True


def main():
    """Run all validation checks."""
    print("=" * 50)
    print("Lab 2: LLM Cognitive Planning - Validation")
    print("=" * 50)
    print()

    checks = [
        check_file_exists,
        check_syntax,
        check_llm_client,
        check_prompt_template,
        check_data_classes,
        check_generate_plan,
        check_validate_plan,
        check_replan_function,
    ]

    passed = 0
    failed = 0

    for check in checks:
        result = check()
        if result:
            passed += 1
        else:
            failed += 1

    print()
    print("=" * 50)
    print(f"Results: {passed} passed, {failed} failed")
    print("=" * 50)

    if failed == 0:
        print("\nLab 2 Complete!")
        return 0
    else:
        print("\nPlease fix the failing checks before proceeding.")
        return 1


if __name__ == '__main__':
    sys.exit(main())
