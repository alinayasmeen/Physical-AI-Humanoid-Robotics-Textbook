#!/usr/bin/env python3
"""لیب 5 تصدیق اسکرپٹ - ادراک کا انضمام"""

import os
import sys
import ast
from pathlib import Path


def check_file_exists(filepath):
    if Path(filepath).exists():
        return True, "فائل مل گئی"
    return False, f"فائل نہیں ملی: {filepath}"


def check_python_syntax(filepath):
    try:
        with open(filepath, 'r') as f:
            ast.parse(f.read())
        return True, "درست Python سینٹیکس"
    except SyntaxError as e:
        return False, f"سینٹیکس خامی لائن {e.lineno} پر"
    except FileNotFoundError:
        return False, "فائل نہیں ملی"


def check_pattern(filepath, pattern, desc):
    try:
        with open(filepath, 'r') as f:
            content = f.read()
        if pattern in content:
            return True, f"{desc} مل گیا"
        return False, f"{desc} نہیں ملی"
    except FileNotFoundError:
        return False, "فائل نہیں ملی"


def main():
    starter_file = "starter/perception_controller.py"

    checks = [
        ("فائل موجود ہے", lambda: check_file_exists(starter_file)),
        ("درست سینٹیکس", lambda: check_python_syntax(starter_file)),
        ("ڈیٹیکشن سبسکرپشن", lambda: check_pattern(
            starter_file, "Detection2DArray", "ڈیٹیکشن سبسکرائیب")),
        ("ویلوسٹی پبلشر", lambda: check_pattern(
            starter_file, "Twist", "ویلوسٹی پبلشر")),
        ("وژول سروو", lambda: check_pattern(
            starter_file, "compute_velocity", "وژول سروو")),
        ("اسٹیٹ مشین", lambda: check_pattern(
            starter_file, "ControlState", "اسٹیٹ مشین")),
        ("ٹریکر", lambda: check_pattern(
            starter_file, "SimpleTracker", "اوبجیکٹ ٹریکر")),
    ]

    print("=" * 50)
    print("لیب 5 تصدیق: ادراک کا انضمام")
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
        print("لیب 5 مکمل! ماڈیول 3 ختم!")
        return 0
    else:
        print("کچھ چیکس ناکام ہو گئیں۔ TODOs مکمل کریں۔")
        return 1


if __name__ == '__main__':
    os.chdir(Path(__file__).parent)
    sys.exit(main())