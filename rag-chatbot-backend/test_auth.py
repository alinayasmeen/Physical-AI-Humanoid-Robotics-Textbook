import asyncio
import aiohttp
import json

async def test_auth_endpoints():
    """
    Test script to verify the authentication endpoints are working
    """
    base_url = "http://localhost:8000"
    
    # Test health check first
    print("Testing health check endpoint...")
    try:
        async with aiohttp.ClientSession() as session:
            async with session.get(f"{base_url}/health") as response:
                if response.status == 200:
                    result = await response.json()
                    print(f"✅ Health check successful: {result}")
                else:
                    print(f"❌ Health check failed with status {response.status}")
                    print(f"Response: {await response.text()}")
    except Exception as e:
        print(f"❌ Error during health check: {str(e)}")

    # Test registration
    print("\nTesting registration endpoint...")
    try:
        async with aiohttp.ClientSession() as session:
            # Register a test user
            register_payload = {
                "email": "test@example.com",
                "password": "testpassword123",
                "full_name": "Test User"
            }

            async with session.post(f"{base_url}/register", json=register_payload) as response:
                if response.status in [200, 400]:  # 400 means user already exists
                    result = await response.json()
                    print(f"✅ Registration response: {response.status} - {result}")
                    if response.status == 200:
                        token = result.get('access_token')
                        print(f"Received token: {token[:20]}..." if token else "No token received")
                else:
                    print(f"❌ Registration failed with status {response.status}")
                    print(f"Response: {await response.text()}")
    except Exception as e:
        print(f"❌ Error during registration: {str(e)}")

    # Test login
    print("\nTesting login endpoint...")
    try:
        async with aiohttp.ClientSession() as session:
            # Login with the test user
            login_data = aiohttp.FormData()
            login_data.add_field('username', 'test@example.com')
            login_data.add_field('password', 'testpassword123')

            async with session.post(f"{base_url}/token", data=login_data) as response:
                if response.status == 200:
                    result = await response.json()
                    print(f"✅ Login successful: {result}")
                    token = result.get('access_token')
                    print(f"Received token: {token[:20]}..." if token else "No token received")
                    
                    # Test protected endpoint with the token
                    print("\nTesting protected endpoint with token...")
                    headers = {"Authorization": f"Bearer {token}"}
                    async with session.get(f"{base_url}/users/me", headers=headers) as protected_response:
                        if protected_response.status == 200:
                            user_data = await protected_response.json()
                            print(f"✅ Protected endpoint access successful: {user_data}")
                        else:
                            print(f"❌ Protected endpoint failed with status {protected_response.status}")
                            print(f"Response: {await protected_response.text()}")
                else:
                    print(f"❌ Login failed with status {response.status}")
                    print(f"Response: {await response.text()}")
    except Exception as e:
        print(f"❌ Error during login: {str(e)}")

if __name__ == "__main__":
    asyncio.run(test_auth_endpoints())