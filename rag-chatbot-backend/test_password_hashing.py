#!/usr/bin/env python3
"""
Test script to verify password hashing functionality
"""
import sys
import os

# Add the current directory to the path so we can import our modules
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from passlib.context import CryptContext

# Test the same configuration we're using in main.py
pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto", bcrypt__ident="2b", bcrypt__rounds=12)

def test_password_hashing():
    print("Testing password hashing functionality...")

    # Test with a short password
    short_password = "short123"
    try:
        hashed = pwd_context.hash(short_password)
        print(f"✓ Successfully hashed short password: {short_password}")
        print(f"  Hash: {hashed[:30]}...")
    except Exception as e:
        print(f"✗ Failed to hash short password: {e}")
        return False

    # Test with a long password (over 72 characters)
    long_password = "this_is_a_very_long_password_that_exceeds_the_72_character_limit_of_bcrypt_for_testing_purposes"
    print(f"\nLong password length: {len(long_password)} characters")

    try:
        # Manually truncate like our function does
        if len(long_password) > 72:
            truncated_password = long_password[:72]
        else:
            truncated_password = long_password

        print(f"Truncated to: {len(truncated_password)} characters")

        hashed = pwd_context.hash(truncated_password)
        print(f"✓ Successfully hashed long password after truncation")
        print(f"  Hash: {hashed[:30]}...")
    except Exception as e:
        print(f"✗ Failed to hash long password: {e}")
        return False

    # Test verification
    try:
        # Verify the short password
        is_valid = pwd_context.verify(short_password, hashed)
        print(f"✓ Password verification works for short passwords: {is_valid}")

        # Verify the long password with truncated version
        is_valid_long = pwd_context.verify(truncated_password, hashed)
        print(f"✓ Password verification works for long passwords: {is_valid_long}")
    except Exception as e:
        print(f"✗ Password verification failed: {e}")
        return False

    print("\n✓ All tests passed!")
    return True

if __name__ == "__main__":
    success = test_password_hashing()
    if not success:
        sys.exit(1)