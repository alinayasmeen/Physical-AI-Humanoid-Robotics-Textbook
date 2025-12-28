#!/usr/bin/env python3
"""
لیب تصدیق اسکرپٹ ٹیمپلیٹ برائے ماڈیول 3 - ادراک اور سینسرز

یہ ٹیمپلیٹ طالب علم کی لیب مکمل ہونے کی تصدیق کے لیے ایک فریم ورک فراہم کرتا ہے۔
ہر مخصوص لیب مشق کے لیے چیکس کو ایڈجسٹ کریں۔
"""

import os
import sys
import ast
from pathlib import Path
from typing import Tuple, List

def check_file_exists(filepath: str, description: str) -> Tuple[bool, str]:
    """چیک کریں کہ کیا ایک ضروری فائل موجود ہے"
    if Path(filepath).exists():
        return True, f"{description} مل گئی"
    return False, f"{description} نہیں ملی {filepath} پر"

def check_python_syntax(filepath: str) -> Tuple[bool, str]:
    """Python فائل سینٹیکس کی تصدیق کریں"
    try:
        with open(filepath, 'r') as f:
            ast.parse(f.read())
        return True, "درست Python سینٹیکس"
    except SyntaxError as e:
        return False, f"سینٹیکس خامی: {e.msg} لائن {e.lineno} پر"
    except FileNotFoundError:
        return False, f"فائل نہیں ملی: {filepath}"

def check_imports(filepath: str, required_imports: List[str]) -> Tuple[bool, str]:
    """چیک کریں کہ کیا ضروری امپورٹس موجود ہیں"
    try:
        with open(filepath, 'r') as f:
            content = f.read()

        missing = [imp for imp in required_imports if imp not in content]
        if missing:
            return False, f"امپورٹس غائب: {', '.join(missing)}"
        return True, "تمام ضروری امپورٹس موجود ہیں"
    except FileNotFoundError:
        return False, f"فائل نہیں ملی: {filepath}"

def check_class_exists(filepath: str, class_name: str) -> Tuple[bool, str]:
    """چیک کریں کہ کیا ایک مخصوص کلاس تعریف شدہ ہے"
    try:
        with open(filepath, 'r') as f:
            tree = ast.parse(f.read())

        for node in ast.walk(tree):
            if isinstance(node, ast.ClassDef) and node.name == class_name:
                return True, f"کلاس {class_name} مل گئی"
        return False, f"کلاس {class_name} نہیں ملی"
    except Exception as e:
        return False, f"کلاس چیک کرتے وقت خامی: {str(e)}"

def check_function_exists(filepath: str, func_name: str) -> Tuple[bool, str]:
    """چیک کریں کہ کیا ایک مخصوص فنکشن تعریف شدہ ہے"
    try:
        with open(filepath, 'r') as f:
            tree = ast.parse(f.read())

        for node in ast.walk(tree):
            if isinstance(node, ast.FunctionDef) and node.name == func_name:
                return True, f"فنکشن {func_name} مل گیا"
        return False, f"فنکشن {func_name} نہیں ملا"
    except Exception as e:
        return False, f"فنکشن چیک کرتے وقت خامی: {str(e)}"

def run_checks(checks: List[Tuple[callable, tuple, str]]) -> bool:
    """توثیق چیکس کی ایک فہرست چلائیں"
    all_passed = True

    print("=" * 50)
    print("لیب توثیق کے نتائج")
    print("=" * 50)

    for check_func, args, description in checks:
        passed, message = check_func(*args)
        status = "[PASS]" if passed else "[FAIL]"
        print(f"{status} {description}: {message}")

        if not passed:
            all_passed = False

    print("=" * 50)

    if all_passed:
        print("لیب مکمل! تمام چیکس پاس ہو گئے۔")
    else:
        print("کچھ چیکس ناکام ہو گئے۔ براہ کرم جائزہ لیں اور دوبارہ کوشش کریں۔")

    return all_passed

# مثال استعمال - ہر لیب کے لیے ایڈجسٹ کریں
if __name__ == '__main__':
    # اس لیب کے لیے چیکس کی وضاحت کریں
    checks = [
        # (function, (args,), "Description")
        (check_file_exists, ("starter/main.py", "مرکزی اسکرپٹ"), "مرکزی اسکرپٹ موجود ہے"),
        (check_python_syntax, ("starter/main.py",), "درست Python سینٹیکس"),
        (check_imports, ("starter/main.py", ["cv2", "rclpy"]), "ضروری امپورٹس"),
        # ضرورت کے مطابق مزید چیکس شامل کریں
    ]

    success = run_checks(checks)
    sys.exit(0 if success else 1)