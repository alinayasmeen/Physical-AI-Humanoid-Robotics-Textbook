"""
CORS Test Script
This script tests if the backend properly handles CORS requests from localhost:3000
"""
import requests
import sys

def test_cors():
    """
    Test if the backend properly handles CORS requests
    """
    print("Testing CORS configuration...")

    # Test URL - replace with your actual backend URL
    backend_url = "https://physical-ai-humanoid-robotics-textbook-fcve.onrender.com"

    try:
        # Test the health endpoint first
        health_url = f"{backend_url}/health"
        print(f"Testing health endpoint: {health_url}")

        # Make a request to test if the server is accessible
        response = requests.get(health_url)
        print(f"Health check response status: {response.status_code}")
        print(f"Response headers: {dict(response.headers)}")

        # Check for CORS headers in the response
        cors_headers = {k: v for k, v in response.headers.items() if 'cors' in k.lower() or 'access-control' in k.lower()}
        print(f"CORS-related headers: {cors_headers}")

        # Test preflight request (OPTIONS) to /token endpoint
        token_url = f"{backend_url}/token"
        print(f"\nTesting preflight request to: {token_url}")

        options_response = requests.options(
            token_url,
            headers={
                "Origin": "http://localhost:3000",
                "Access-Control-Request-Method": "POST",
                "Access-Control-Request-Headers": "Content-Type",
            }
        )

        print(f"OPTIONS response status: {options_response.status_code}")
        print(f"OPTIONS response headers: {dict(options_response.headers)}")

        # Check for CORS headers in OPTIONS response
        cors_headers_options = {k: v for k, v in options_response.headers.items() if 'cors' in k.lower() or 'access-control' in k.lower()}
        print(f"CORS-related headers in OPTIONS: {cors_headers_options}")

        if 'Access-Control-Allow-Origin' in options_response.headers:
            allowed_origin = options_response.headers.get('Access-Control-Allow-Origin')
            print(f"Allowed origin: {allowed_origin}")

            if allowed_origin == "http://localhost:3000" or allowed_origin == "*":
                print("✅ CORS is properly configured for localhost:3000")
                return True
            else:
                print(f"❌ CORS is not configured for localhost:3000. Allowed origin: {allowed_origin}")
                return False
        else:
            print("❌ No CORS headers found in response")
            return False

    except requests.exceptions.ConnectionError:
        print("❌ Cannot connect to the backend server")
        print("The backend might not be accessible or running")
        return False
    except requests.exceptions.RequestException as e:
        print(f"❌ Request failed: {e}")
        return False
    except Exception as e:
        print(f"❌ An error occurred: {e}")
        return False

if __name__ == "__main__":
    success = test_cors()
    if success:
        print("\n✅ CORS test passed!")
        sys.exit(0)
    else:
        print("\n❌ CORS test failed!")
        sys.exit(1)