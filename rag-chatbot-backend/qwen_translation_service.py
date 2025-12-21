from typing import Dict, Optional
import os
from dotenv import load_dotenv
import json
import asyncio
from pathlib import Path
import httpx

load_dotenv()

class QwenTranslationService:
    def __init__(self):
        # Use the Qwen API key
        self.qwen_api_key = os.getenv("QWEN_API_KEY")
        self.base_url = os.getenv("QWEN_BASE_URL", "https://dashscope.aliyuncs.com/api/v1/services/aigc/text-generation/generation")

        if not self.qwen_api_key:
            raise ValueError("QWEN_API_KEY environment variable is required")

    async def translate_text(self, text: str, target_language: str = "ur") -> str:
        """
        Translate text to the target language using Qwen API with caching
        """
        # First, try to get from cache
        cached_translation = await self.get_cached_translation(text, target_language)
        if cached_translation:
            return cached_translation

        try:
            # Create a prompt for translation
            prompt = f"Translate the following text to {target_language} (Urdu). Only respond with the translated text, nothing else:\n\n{text}"

            headers = {
                "Authorization": f"Bearer {self.qwen_api_key}",
                "Content-Type": "application/json",
                "X-DashScope-SSE": "disable"  # Disable streaming for simpler response handling
            }

            payload = {
                "model": "qwen-max",  # or "qwen-plus", "qwen-turbo" depending on your needs
                "input": {
                    "messages": [
                        {
                            "role": "user",
                            "content": prompt
                        }
                    ]
                },
                "parameters": {
                    "temperature": 0.1,  # Low temperature for more consistent translations
                    "max_tokens": len(text) * 3,  # Allow for longer Urdu text
                }
            }

            async with httpx.AsyncClient(timeout=30.0) as client:
                response = await client.post(
                    self.base_url,
                    headers=headers,
                    json=payload
                )

            if response.status_code != 200:
                raise Exception(f"Qwen API request failed with status {response.status_code}: {response.text}")

            result = response.json()

            # Extract the translated text from the response
            # Qwen API typically returns structured response with 'output' field
            if "output" in result and "choices" in result["output"]:
                choices = result["output"]["choices"]
                if choices:
                    # Get the content from the first choice
                    translated_text = choices[0].get("message", {}).get("content", "")
                    if translated_text:
                        translated_text = translated_text.strip()
                        # Cache the translation for future use
                        await self.cache_translation(text, translated_text, target_language)
                        return translated_text

            # Fallback: try different possible response structures
            translated_text = result.get("output", {}).get("text", "")
            if translated_text:
                translated_text = translated_text.strip()
                # Cache the translation for future use
                await self.cache_translation(text, translated_text, target_language)
                return translated_text

            # If still no result, check if it's in the 'text' field directly
            if "text" in result:
                translated_text = result["text"].strip()
                # Cache the translation for future use
                await self.cache_translation(text, translated_text, target_language)
                return translated_text

            # If all attempts fail, return original text
            print(f"Warning: Could not extract translated text from Qwen response: {result}")
            return text

        except Exception as e:
            print(f"Error translating text with Qwen: {str(e)}")
            # Return original text if translation fails
            return text

    async def translate_markdown_content(self, markdown_content: str) -> str:
        """
        Translate markdown content while preserving markdown structure
        """
        # First, try to get from cache
        cached_translation = await self.get_cached_translation(markdown_content, "ur")
        if cached_translation:
            return cached_translation

        try:
            # Split content into lines to preserve structure
            lines = markdown_content.split('\n')
            translated_lines = []

            for line in lines:
                # Don't translate markdown headers, code blocks, or links
                if line.strip().startswith('#') or \
                   line.strip().startswith('```') or \
                   line.strip().startswith('![') or \
                   line.strip().startswith('[') and '](http' in line:
                    # Preserve markdown structure
                    translated_lines.append(line)
                elif line.strip() != '':
                    # Translate regular text content
                    translated_line = await self.translate_text(line)
                    translated_lines.append(translated_line)
                else:
                    # Preserve empty lines
                    translated_lines.append(line)

            result = '\n'.join(translated_lines)

            # Cache the full markdown translation for future use
            await self.cache_translation(markdown_content, result, "ur")

            return result

        except Exception as e:
            print(f"Error translating markdown content: {str(e)}")
            return markdown_content

    async def translate_file(self, file_path: str, output_path: Optional[str] = None) -> str:
        """
        Translate a markdown file to Urdu
        """
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()

        translated_content = await self.translate_markdown_content(content)

        if output_path:
            with open(output_path, 'w', encoding='utf-8') as f:
                f.write(translated_content)

        return translated_content

    async def get_cached_translation(self, content: str, target_language: str = "ur"):
        """Get cached translation from database if it exists"""
        # Import the function from main.py
        from main import get_cached_translation
        return await get_cached_translation(content, target_language)

    async def cache_translation(self, original_content: str, translated_content: str, target_language: str = "ur"):
        """Cache the translation in the database"""
        # Import the function from main.py
        from main import cache_translation
        await cache_translation(original_content, translated_content, target_language)

# Singleton instance
qwen_translation_service = QwenTranslationService()