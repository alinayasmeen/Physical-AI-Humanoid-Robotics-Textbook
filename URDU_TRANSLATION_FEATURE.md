# Urdu Translation Feature

## Overview
This feature allows users to translate textbook content into Urdu, making the educational material accessible to Urdu-speaking students.

## How It Works

### Backend
- The backend provides two API endpoints for translation:
  1. `POST /translate` - Translates plain text to Urdu
  2. `POST /translate-markdown` - Translates markdown content while preserving structure

- Translation can be powered by either:
  - Gemini API through the `UrduTranslationService` class, OR
  - Qwen API through the `QwenTranslationService` class
- The service intelligently preserves markdown formatting (headers, code blocks, links) while translating regular text content

### Frontend
- A reusable `<TranslateToUrdu>` component can be added to any textbook page
- The component adds a "Translate to Urdu" button to the page
- When clicked, the button sends the page content to the backend for translation
- Users can toggle between original and translated content

### Implementation
1. The translation services are integrated in `translation_service.py` (Gemini) and `qwen_translation_service.py` (Qwen)
2. API endpoints are added to `main.py`
3. Frontend components are in `src/chatbot/TranslationButton.tsx` and `src/components/TranslateToUrdu/`
4. The component can be used in any markdown file by adding:
   ```
   import TranslateToUrdu from '@site/src/components/TranslateToUrdu';

   <TranslateToUrdu>
   <!-- Your content here -->
   </TranslateToUrdu>
   ```

## Configuration
- To use Qwen API: Set the `QWEN_API_KEY` environment variable
- To use Gemini API: Set the `GEMINI_API_KEY` environment variable (and don't set QWEN_API_KEY)
- The system will automatically use Qwen if `QWEN_API_KEY` is present, otherwise it defaults to Gemini

## Usage
1. Users must be logged in to use the translation feature
2. The translation button appears at the top of pages with translation support
3. Click the button to translate the content to Urdu
4. Use the "Restore Original" button to switch back to English

## Technical Details
- Translation uses either Qwen (qwen-max model) or Gemini (2.0 Flash model) for fast, accurate translations
- Markdown structure is preserved during translation
- Authentication is required via JWT tokens
- The feature is integrated with the existing chatbot authentication system