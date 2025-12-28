#!/usr/bin/env python3
"""
Validation script for Mini-Project 2: Advanced VLA System
This script validates that the advanced VLA system implementation meets the requirements.
"""

import os
import sys
import importlib.util
from pathlib import Path

def validate_file_exists(filepath):
    """Check if a file exists and is readable."""
    path = Path(filepath)
    if path.exists() and path.is_file():
        print(f"✓ {filepath} exists")
        return True
    else:
        print(f"✗ {filepath} does not exist")
        return False

def validate_module_importable(filepath):
    """Check if a Python file can be imported as a module."""
    try:
        spec = importlib.util.spec_from_file_location("module", filepath)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        print(f"✓ {filepath} is importable")
        return True
    except Exception as e:
        print(f"✗ {filepath} import failed: {e}")
        return False

def run_validation():
    """Run all validation checks."""
    print("Validating Mini-Project 2: Advanced VLA System")
    print("=" * 50)

    # Check required files exist
    required_files = [
        "starter/vla_advanced_pipeline.py",
        "starter/vision_processor.py",
        "starter/cognitive_planner.py",
        "starter/safety_monitor.py",
        "solution/vla_advanced_pipeline.py",
        "solution/vision_processor.py",
        "solution/cognitive_planner.py",
        "solution/safety_monitor.py"
    ]

    all_files_exist = True
    for file in required_files:
        full_path = f"labs/module-04/mini-project-02-advanced-vla/{file}"
        if not validate_file_exists(full_path):
            all_files_exist = False

    print()

    # Check if Python files are importable
    python_files = [
        "labs/module-04/mini-project-02-advanced-vla/starter/vla_advanced_pipeline.py",
        "labs/module-04/mini-project-02-advanced-vla/starter/vision_processor.py",
        "labs/module-04/mini-project-02-advanced-vla/starter/cognitive_planner.py",
        "labs/module-04/mini-project-02-advanced-vla/starter/safety_monitor.py",
        "labs/module-04/mini-project-02-advanced-vla/solution/vla_advanced_pipeline.py",
        "labs/module-04/mini-project-02-advanced-vla/solution/vision_processor.py",
        "labs/module-04/mini-project-02-advanced-vla/solution/cognitive_planner.py",
        "labs/module-04/mini-project-02-advanced-vla/solution/safety_monitor.py"
    ]

    all_importable = True
    for file in python_files:
        if not validate_module_importable(file):
            all_importable = False

    print()

    # Overall result
    if all_files_exist and all_importable:
        print("🎉 All validations passed! Mini-Project 2 is complete.")
        return True
    else:
        print("❌ Some validations failed. Please check the requirements.")
        return False

if __name__ == "__main__":
    success = run_validation()
    sys.exit(0 if success else 1)