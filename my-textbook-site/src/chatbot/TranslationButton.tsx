import React, { useState, useEffect } from 'react';
import styles from './styles.module.css';

interface TranslationButtonProps {
  content?: string; // Optional content to translate, if not provided, will translate current page
  onTranslationComplete?: (translatedContent: string) => void;
  containerSelector?: string; // Selector for the content container, defaults to main content area
}

const TranslationButton: React.FC<TranslationButtonProps> = ({
  content,
  onTranslationComplete,
  containerSelector = 'main .container .row .col'
}) => {
  const [isTranslating, setIsTranslating] = useState(false);
  const [isTranslated, setIsTranslated] = useState(false);
  const [originalContent, setOriginalContent] = useState<string | null>(null);

  // Get current page content if no content is provided
  useEffect(() => {
    if (!content) {
      // Try to get the main content of the page
      const mainContent = document.querySelector(containerSelector);
      if (mainContent) {
        setOriginalContent(mainContent.innerHTML);
      }
    } else {
      setOriginalContent(content);
    }
  }, [content, containerSelector]);

  const translateToUrdu = async () => {
    if (isTranslating) return;

    setIsTranslating(true);

    try {
      const token = localStorage.getItem('token');
      if (!token) {
        alert('Please log in to use translation feature');
        setIsTranslating(false);
        return;
      }

      let contentToTranslate = content || originalContent;
      if (!contentToTranslate) {
        // If no content provided and we couldn't get it from the DOM, try to extract text content
        const mainContent = document.querySelector(containerSelector);
        if (mainContent) {
          contentToTranslate = mainContent.textContent || '';
        }
      }

      if (!contentToTranslate) {
        alert('No content to translate');
        setIsTranslating(false);
        return;
      }

      // Call the backend translation API
      const response = await fetch('http://localhost:8000/translate-markdown', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${token}`
        },
        body: JSON.stringify({
          markdown_content: contentToTranslate,
          target_language: 'ur'
        })
      });

      if (!response.ok) {
        throw new Error(`Translation failed: ${response.status}`);
      }

      const data = await response.json();

      if (onTranslationComplete) {
        onTranslationComplete(data.translated_content);
      } else {
        // If no callback provided, try to replace the content on the page
        const mainContent = document.querySelector(containerSelector);
        if (mainContent) {
          // Note: Using innerHTML with translated content could be dangerous if content is not trusted
          // For markdown content from our own system it should be safe
          mainContent.innerHTML = data.translated_content;
        }
      }

      setIsTranslated(true);
    } catch (error) {
      console.error('Translation error:', error);
      alert('Translation failed. Please make sure the backend server is running and you are logged in.');
    } finally {
      setIsTranslating(false);
    }
  };

  const restoreOriginalContent = () => {
    if (onTranslationComplete) {
      // If there's a callback, let the parent handle restoring content
      onTranslationComplete(originalContent || '');
    } else {
      // If no callback, try to restore original content
      const mainContent = document.querySelector(containerSelector);
      if (mainContent && originalContent) {
        mainContent.innerHTML = originalContent;
      }
    }
    setIsTranslated(false);
  };

  if (isTranslated) {
    return (
      <button
        className={styles.translationButton}
        onClick={restoreOriginalContent}
        disabled={isTranslating}
      >
        {isTranslating ? 'Restoring...' : 'Restore Original'}
      </button>
    );
  }

  return (
    <button
      className={styles.translationButton}
      onClick={translateToUrdu}
      disabled={isTranslating}
    >
      {isTranslating ? 'Translating...' : '.Translate to Urdu'}
    </button>
  );
};

export default TranslationButton;