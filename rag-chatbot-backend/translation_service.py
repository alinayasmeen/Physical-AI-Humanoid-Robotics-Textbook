from google.cloud import translate
from typing import Dict, Optional
import os
from dotenv import load_dotenv
import json
import asyncio
from pathlib import Path

load_dotenv()

class UrduTranslationService:
    def __init__(self):
        # Use the Gemini API key for translation if available, otherwise look for a translation key
        self.gemini_api_key = os.getenv("GEMINI_API_KEY")
        self.neon_db_url = os.getenv("NEON_DATABASE_URL")

        # For now, we'll use the Gemini API for translation since you have it set up
        # We can use Gemini's capabilities to translate text
        from openai import OpenAI
        self.client = OpenAI(
            api_key=self.gemini_api_key,
            base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
        )

    async def translate_text(self, text: str, target_language: str = "ur") -> str:
        """
        Translate text to Urdu using Gemini API with caching
        """
        # First, try to get from cache
        cached_translation = await self.get_cached_translation(text, target_language)
        if cached_translation:
            return cached_translation

        try:
            # Create a prompt for translation
            prompt = f"Translate the following text to {target_language} (Urdu). Only respond with the translated text, nothing else:\n\n{text}"

            response = self.client.chat.completions.create(
                model="gemini-2.0-flash",
                messages=[{"role": "user", "content": prompt}],
                temperature=0.1,  # Low temperature for more consistent translations
                max_tokens=len(text) * 3,  # Allow for longer Urdu text
            )

            translated_text = response.choices[0].message.content.strip()

            # Cache the translation for future use
            await self.cache_translation(text, translated_text, target_language)

            return translated_text

        except Exception as e:
            print(f"Error translating text: {str(e)}")
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
translation_service = UrduTranslationService()