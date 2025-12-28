"""
Authentication Test Script
This script tests the authentication endpoints to help diagnose issues
"""
import requests
import json

def test_authentication():
    """
    Test the authentication endpoints to help diagnose issues
    """
    print("Testing authentication endpoints...")

    # Use the same URL as in your frontend
    base_url = "https://physical-ai-humanoid-robotics-textbook-fcve.onrender.com"

    print(f"Testing against: {base_url}")

    # Test the health endpoint first
    try:
        health_response = requests.get(f"{base_url}/health")
        print(f"Health check status: {health_response.status_code}")
        print(f"Health check response: {health_response.json() if health_response.content else 'No content'}")
    except Exception as e:
        print(f"Health check failed: {e}")
        return

    # Test registration (try to create a test user)
    print("\nTesting registration...")
    try:
        registration_data = {
            "email": "testuser@example.com",
            "password": "testpassword123",
            "full_name": "Test User"
        }

        # Set headers for registration
        headers = {"Content-Type": "application/json"}

        registration_response = requests.post(
            f"{base_url}/register",
            json=registration_data,
            headers=headers
        )

        print(f"Registration status: {registration_response.status_code}")
        try:
            print(f"Registration response: {registration_response.json()}")
        except:
            print(f"Registration response text: {registration_response.text}")

    except Exception as e:
        print(f"Registration test failed: {e}")

    # Test login with the same credentials
    print("\nTesting login...")
    try:
        # Prepare form data for login (as the backend expects form data)
        login_data = {
            "username": "testuser@example.com",  # This is actually the email
            "password": "testpassword123"
        }

        # Login request uses form data, not JSON
        login_response = requests.post(
            f"{base_url}/token",
            data=login_data  # This sends form data, not JSON
        )

        print(f"Login status: {login_response.status_code}")
        try:
            print(f"Login response: {login_response.json()}")
        except:
            print(f"Login response text: {login_response.text}")

    except Exception as e:
        print(f"Login test failed: {e}")

if __name__ == "__main__":
    test_authentication()