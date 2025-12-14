import asyncio
import aiohttp
import os

async def test_translation_api():
    """
    Test script to verify the translation API endpoints are working
    """
    # Get the token from environment or user input
    token = os.getenv("TEST_USER_TOKEN") or input("Enter your auth token: ")

    if not token:
        print("No token provided. Please provide a valid JWT token for testing.")
        return

    base_url = "http://localhost:8000"

    # Test the text translation endpoint
    print("Testing text translation endpoint...")
    try:
        async with aiohttp.ClientSession() as session:
            # Test text translation
            text_payload = {
                "text": "Hello, how are you today?",
                "target_language": "ur"
            }

            headers = {
                "Authorization": f"Bearer {token}",
                "Content-Type": "application/json"
            }

            async with session.post(f"{base_url}/translate", json=text_payload, headers=headers) as response:
                if response.status == 200:
                    result = await response.json()
                    print("✅ Text translation successful!")
                    print(f"Original: {result['original_text']}")
                    print(f"Translated: {result['translated_text']}")
                else:
                    print(f"❌ Text translation failed with status {response.status}")
                    print(f"Response: {await response.text()}")

            print("\nTesting markdown translation endpoint...")
            # Test markdown translation
            markdown_payload = {
                "markdown_content": "# Hello World\n\nThis is a **test** of the translation system.",
                "target_language": "ur"
            }

            async with session.post(f"{base_url}/translate-markdown", json=markdown_payload, headers=headers) as response:
                if response.status == 200:
                    result = await response.json()
                    print("✅ Markdown translation successful!")
                    print(f"Original: {result['original_content']}")
                    print(f"Translated: {result['translated_content']}")
                else:
                    print(f"❌ Markdown translation failed with status {response.status}")
                    print(f"Response: {await response.text()}")

    except Exception as e:
        print(f"❌ Error during testing: {str(e)}")

if __name__ == "__main__":
    asyncio.run(test_translation_api())