#!/usr/bin/env python3
"""
Lab 2 Validation Script - Gazebo Worlds

Validates that the custom world meets lab requirements.

Usage:
    python3 validate.py
"""

import os
import sys
import xml.etree.ElementTree as ET


def check_file_exists(filepath, description):
    """Check if a file exists."""
    if os.path.exists(filepath):
        print(f"[PASS] {description}")
        return True
    else:
        print(f"[FAIL] {description} - File not found: {filepath}")
        return False


def validate_sdf_structure(filepath):
    """Validate SDF file structure and content."""
    results = []

    try:
        tree = ET.parse(filepath)
        root = tree.getroot()

        # Check SDF root element
        if root.tag == 'sdf':
            print("[PASS] Valid SDF root element")
            results.append(True)
        else:
            print(f"[FAIL] Invalid root element: {root.tag}")
            results.append(False)
            return results

        # Find world element
        world = root.find('world')
        if world is not None:
            print("[PASS] World element present")
            results.append(True)
        else:
            print("[FAIL] No world element found")
            results.append(False)
            return results

        # Check physics configuration
        physics = world.find('physics')
        if physics is not None:
            max_step = physics.find('max_step_size')
            rtf = physics.find('real_time_factor')
            if max_step is not None and rtf is not None:
                print("[PASS] Physics configuration present")
                results.append(True)
            else:
                print("[WARN] Physics configuration incomplete")
                results.append(True)  # Warning, not failure
        else:
            print("[FAIL] No physics configuration")
            results.append(False)

        # Check lighting
        lights = world.findall('light')
        if len(lights) >= 1:
            print(f"[PASS] Lighting configured ({len(lights)} light(s))")
            results.append(True)
        else:
            print("[FAIL] No lighting configured")
            results.append(False)

        # Check for shadows
        has_shadows = False
        for light in lights:
            shadows = light.find('cast_shadows')
            if shadows is not None and shadows.text == 'true':
                has_shadows = True
                break
        if has_shadows:
            print("[PASS] Shadow casting enabled")
            results.append(True)
        else:
            print("[WARN] No shadow casting lights")
            results.append(True)

        # Check ground plane
        models = world.findall('model')
        has_ground = any('ground' in m.get('name', '').lower() for m in models)
        if has_ground:
            print("[PASS] Ground plane present")
            results.append(True)
        else:
            print("[FAIL] No ground plane found")
            results.append(False)

        # Count obstacles (non-ground, non-plugin models)
        obstacle_count = 0
        obstacle_types = set()
        for model in models:
            name = model.get('name', '')
            if 'ground' not in name.lower() and 'wall' not in name.lower():
                obstacle_count += 1
                # Check geometry type
                for link in model.findall('.//link'):
                    for collision in link.findall('.//collision'):
                        geom = collision.find('geometry')
                        if geom is not None:
                            for child in geom:
                                obstacle_types.add(child.tag)

        if obstacle_count >= 3:
            print(f"[PASS] At least 3 obstacles ({obstacle_count} found)")
            results.append(True)
        else:
            print(f"[FAIL] Need at least 3 obstacles ({obstacle_count} found)")
            results.append(False)

        if len(obstacle_types) >= 2:
            print(f"[PASS] Multiple obstacle types: {', '.join(obstacle_types)}")
            results.append(True)
        else:
            print(f"[WARN] Only {len(obstacle_types)} obstacle type(s)")
            results.append(True)

    except ET.ParseError as e:
        print(f"[FAIL] XML parse error: {e}")
        results.append(False)
    except Exception as e:
        print(f"[FAIL] Validation error: {e}")
        results.append(False)

    return results


def main():
    """Run all validation checks."""
    print("=" * 50)
    print("Lab 2 Validation: Gazebo Worlds")
    print("=" * 50)
    print()

    lab_dir = os.path.dirname(os.path.abspath(__file__))
    results = []

    # Check starter file
    print("Checking starter files...")
    starter_file = os.path.join(lab_dir, 'starter', 'worlds', 'empty_world.sdf')
    results.append(check_file_exists(starter_file, "Starter world template exists"))

    print()

    # Check solution file
    print("Checking solution files...")
    solution_file = os.path.join(lab_dir, 'solution', 'worlds', 'custom_world.sdf')
    results.append(check_file_exists(solution_file, "Solution world exists"))

    print()

    # Validate solution structure
    print("Validating solution SDF structure...")
    if os.path.exists(solution_file):
        sdf_results = validate_sdf_structure(solution_file)
        results.extend(sdf_results)

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
        print(f"Lab 2 Validation: PASSED ({passed}/{total})")
        print("Custom world meets all requirements.")
        return 0
    else:
        print(f"Lab 2 Validation: INCOMPLETE ({passed}/{total})")
        print("Some checks failed. Review the output above.")
        return 1


if __name__ == '__main__':
    sys.exit(main())
