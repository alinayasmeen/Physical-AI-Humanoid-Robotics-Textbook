#!/usr/bin/env python3
"""
Lab 4 Validation Script
Checks if URDF file is valid and complete
"""

import sys
import xml.etree.ElementTree as ET


def parse_urdf(file_path):
    """Parse URDF file and return root element."""
    try:
        tree = ET.parse(file_path)
        return tree.getroot(), None
    except ET.ParseError as e:
        return None, f"XML parse error: {e}"
    except FileNotFoundError:
        return None, "File not found"


def check_links(root, expected_links):
    """Check if required links exist."""
    found_links = [link.get('name') for link in root.findall('link')]

    missing = [link for link in expected_links if link not in found_links]
    if missing:
        return False, f"Missing links: {missing}"

    return True, f"Found {len(found_links)} links: {', '.join(found_links)}"


def check_joints(root, expected_joints):
    """Check if required joints exist."""
    found_joints = [joint.get('name') for joint in root.findall('joint')]

    missing = [joint for joint in expected_joints if joint not in found_joints]
    if missing:
        return False, f"Missing joints: {missing}"

    return True, f"Found {len(found_joints)} joints: {', '.join(found_joints)}"


def check_joint_limits(root):
    """Check if revolute joints have valid limits."""
    for joint in root.findall('joint'):
        joint_type = joint.get('type')
        joint_name = joint.get('name')

        if joint_type == 'revolute':
            limit = joint.find('limit')
            if limit is None:
                return False, f"Joint '{joint_name}' missing <limit> element"

            lower = limit.get('lower')
            upper = limit.get('upper')

            if lower is None or upper is None:
                return False, f"Joint '{joint_name}' missing limit bounds"

            try:
                lower_val = float(lower)
                upper_val = float(upper)
                if lower_val >= upper_val:
                    return False, f"Joint '{joint_name}' has invalid limits (lower >= upper)"
            except ValueError:
                return False, f"Joint '{joint_name}' has non-numeric limits"

    return True, "Joint limits are valid"


def check_parent_child(root):
    """Check if all joint parent/child references are valid."""
    link_names = {link.get('name') for link in root.findall('link')}

    for joint in root.findall('joint'):
        joint_name = joint.get('name')
        parent = joint.find('parent')
        child = joint.find('child')

        if parent is None or child is None:
            return False, f"Joint '{joint_name}' missing parent or child"

        parent_link = parent.get('link')
        child_link = child.get('link')

        if parent_link not in link_names:
            return False, f"Joint '{joint_name}' references unknown parent '{parent_link}'"
        if child_link not in link_names:
            return False, f"Joint '{joint_name}' references unknown child '{child_link}'"

    return True, "All parent/child references are valid"


def main():
    """Run validation checks."""
    print("=" * 50)
    print("Lab 4: URDF Basics - Validation")
    print("=" * 50)

    file_path = sys.argv[1] if len(sys.argv) > 1 else 'starter/simple_arm.urdf'

    # Expected structure for complete arm
    expected_links = ['base_link', 'upper_arm', 'lower_arm', 'end_effector']
    expected_joints = ['shoulder', 'elbow', 'wrist']

    all_passed = True

    # Check 1: Parse URDF
    print("\n[1/5] Parsing URDF file...")
    root, error = parse_urdf(file_path)
    if root is not None:
        print(f"  ✅ URDF file parses successfully")
    else:
        print(f"  ❌ {error}")
        sys.exit(1)

    # Check 2: Verify links
    print("\n[2/5] Checking links...")
    valid, msg = check_links(root, expected_links)
    if valid:
        print(f"  ✅ {msg}")
    else:
        print(f"  ❌ {msg}")
        all_passed = False

    # Check 3: Verify joints
    print("\n[3/5] Checking joints...")
    valid, msg = check_joints(root, expected_joints)
    if valid:
        print(f"  ✅ {msg}")
    else:
        print(f"  ❌ {msg}")
        all_passed = False

    # Check 4: Verify joint limits
    print("\n[4/5] Checking joint limits...")
    valid, msg = check_joint_limits(root)
    if valid:
        print(f"  ✅ {msg}")
    else:
        print(f"  ❌ {msg}")
        all_passed = False

    # Check 5: Verify parent/child references
    print("\n[5/5] Checking parent/child references...")
    valid, msg = check_parent_child(root)
    if valid:
        print(f"  ✅ {msg}")
    else:
        print(f"  ❌ {msg}")
        all_passed = False

    print("\n" + "=" * 50)
    if all_passed:
        print("✅ All validation checks passed!")
    else:
        print("❌ Some checks failed. Review the errors above.")
        sys.exit(1)
    print("=" * 50)


if __name__ == '__main__':
    main()
