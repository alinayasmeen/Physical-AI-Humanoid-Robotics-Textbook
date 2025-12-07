import React, { useState, useEffect, useRef } from 'react';
import styles from './styles.module.css';
import { Message } from './types';

interface ChatWindowProps {
  onClose: () => void;
}

const ChatWindow: React.FC<ChatWindowProps> = ({ onClose }) => {
  const [messages, setMessages] = useState<Message[]>([]);
  const [inputValue, setInputValue] = useState('');
  const [selectedText, setSelectedText] = useState<string>('');
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Auto-scroll when new messages arrive
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  // Capture selected text from the page
  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      const text = selection?.toString().trim();
      if (text && text.length > 0) setSelectedText(text);
    };

    document.addEventListener('mouseup', handleSelection);
    return () => document.removeEventListener('mouseup', handleSelection);
  }, []);

  const handleSendMessage = async () => {
    if (!inputValue.trim() && !selectedText) return;

    setIsLoading(true);

    // Prepare user message for UI
    const userMessage: Message = { 
      text: inputValue || 'Question about selected text',
      sender: 'user',
      selectedText: selectedText || undefined
    };
    setMessages(prev => [...prev, userMessage]);
    setInputValue('');

    try {
      // Ensure payload matches FastAPI ChatRequest
      const payload = {
        query: inputValue.trim() || 'Question about selected text',
        selected_text: selectedText || null
      };

      const response = await fetch('http://localhost:8000/chat', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();
      const botMessage: Message = { text: data.response, sender: 'bot' };
      setMessages(prev => [...prev, botMessage]);

      setSelectedText('');
    } catch (error) {
      console.error('Error sending message:', error);
      const errorMessage: Message = {
        text: 'Sorry, something went wrong. Please make sure the backend server is running.',
        sender: 'bot'
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e: React.KeyboardEvent<HTMLInputElement>) => {
    if (e.key === 'Enter' && !isLoading) handleSendMessage();
  };

  const clearSelection = () => setSelectedText('');

  return (
    <div className={styles.chatWindow}>
      <div className={styles.chatHeader}>
        <h2>📚 Book Assistant</h2>
        <button onClick={onClose}>✕</button>
      </div>

      <div className={styles.chatMessages}>
        {messages.length === 0 && (
          <div className={styles.emptyState}>
            <p>Ask me anything about the book!</p>
            <p style={{ fontSize: '0.8rem', color: '#888' }}>
              💡 Tip: Select text from the page to ask specific questions
            </p>
          </div>
        )}

        {messages.map((message, index) => (
          <div key={index} className={`${styles.message} ${styles[message.sender]}`}>
            {message.selectedText && (
              <div className={styles.selectedTextPreview}>
                <em>Selected: "{message.selectedText.substring(0, 50)}..."</em>
              </div>
            )}
            <div>{message.text}</div>
          </div>
        ))}

        {isLoading && (
          <div className={`${styles.message} ${styles.bot}`}>
            <div className={styles.loadingDots}>
              <span>.</span>
              <span>.</span>
              <span>.</span>
            </div>
          </div>
        )}

        <div ref={messagesEndRef} />
      </div>

      {selectedText && (
        <div className={styles.selectedIndicator}>
          <div className={styles.selectedTextContent}>
            <strong>Selected:</strong> "{selectedText.substring(0, 60)}..."
          </div>
          <button 
            onClick={clearSelection}
            className={styles.clearButton}
            aria-label="Clear selection"
          >
            ✕
          </button>
        </div>
      )}

      <div className={styles.chatInput}>
        <input
          type="text"
          value={inputValue}
          onChange={(e) => setInputValue(e.target.value)}
          onKeyPress={handleKeyPress}
          placeholder={selectedText ? "Ask about selected text..." : "Ask a question..."}
          disabled={isLoading}
        />
        <button 
          onClick={handleSendMessage}
          disabled={isLoading || (!inputValue.trim() && !selectedText)}
        >
          {isLoading ? '...' : 'Send'}
        </button>
      </div>
    </div>
  );
};

export default ChatWindow;
