#!/usr/bin/env python3
"""
Test the specific error handling for bcrypt __about__ attribute issue
"""
import sys
import os

# Add the current directory to the path so we can import our modules
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Import the functions from main.py
import main

def test_error_handling():
    print("Testing error handling for password hashing...")

    # Test with a normal password
    try:
        password = "test_password_123"
        hashed = main.get_password_hash(password)
        print(f"✓ Normal password hashing works")
    except Exception as e:
        print(f"✗ Normal password hashing failed: {e}")
        return False

    # Test with a long password
    try:
        long_password = "this_is_a_very_long_password_that_exceeds_the_72_character_limit_of_bcrypt_for_testing_purposes_and_should_be_truncated"
        hashed = main.get_password_hash(long_password)
        print(f"✓ Long password hashing works with truncation")
    except Exception as e:
        print(f"✗ Long password hashing failed: {e}")
        return False

    # Test verification
    try:
        is_valid = main.verify_password("test_password_123", hashed)
        print(f"✓ Password verification works")
    except Exception as e:
        print(f"✗ Password verification failed: {e}")
        return False

    print("\nAll error handling tests passed!")
    return True

if __name__ == "__main__":
    success = test_error_handling()
    if success:
        print("\n✓ Error handling test completed successfully")
    else:
        print("\n✗ Error handling test failed")
        sys.exit(1)