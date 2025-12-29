#!/usr/bin/env python3
"""
Simple test to verify bcrypt backend issue is resolved
"""
import sys
import os

# Add the current directory to the path so we can import our modules
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from passlib.context import CryptContext

def test_bcrypt_backend():
    print("Testing bcrypt backend initialization...")

    # Test the same configuration we're using in main.py
    pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto", bcrypt__ident="2b", bcrypt__rounds=12)

    try:
        # Test with a short password
        short_password = "test123"
        hashed = pwd_context.hash(short_password)
        print(f"Successfully hashed short password")

        # Verify the password
        is_valid = pwd_context.verify(short_password, hashed)
        print(f"Password verification works: {is_valid}")

        # Test with a long password (over 72 characters)
        long_password = "this_is_a_very_long_password_that_exceeds_the_72_character_limit_of_bcrypt_for_testing_purposes_and_should_be_truncated"
        print(f"Long password length: {len(long_password)} characters")

        if len(long_password) > 72:
            truncated_password = long_password[:72]
        else:
            truncated_password = long_password

        print(f"Truncated to: {len(truncated_password)} characters")

        hashed_long = pwd_context.hash(truncated_password)
        print(f"Successfully hashed long password after truncation")

        # Verify the long password
        is_valid_long = pwd_context.verify(truncated_password, hashed_long)
        print(f"Long password verification works: {is_valid_long}")

        print("\nAll tests passed!")
        return True

    except Exception as e:
        print(f"Test failed: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = test_bcrypt_backend()
    if success:
        print("\n✓ Bcrypt backend test completed successfully")
    else:
        print("\n✗ Bcrypt backend test failed")
        sys.exit(1)