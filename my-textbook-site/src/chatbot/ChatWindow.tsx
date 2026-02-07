import React, { useState, useEffect, useRef } from 'react';
import styles from './styles.module.css';
import { Message } from './types';
import { useAuth } from '../contexts/AuthContext';
import AuthWrapper from './AuthWrapper';

interface ChatWindowProps {
  onClose: () => void;
}

const ChatWindow: React.FC<ChatWindowProps> = ({ onClose }) => {
  const { user, isAuthenticated, logout, token } = useAuth(); // ✅ SINGLE hook call

  const [messages, setMessages] = useState<Message[]>([]);
  const [inputValue, setInputValue] = useState('');
  const [selectedText, setSelectedText] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Auto-scroll
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  // Capture selected text (CLIENT ONLY)
  useEffect(() => {
    if (typeof window === 'undefined') return;

    const handleSelection = () => {
      const selection = window.getSelection();
      const text = selection?.toString().trim();
      if (text) setSelectedText(text);
    };

    document.addEventListener('mouseup', handleSelection);
    return () => document.removeEventListener('mouseup', handleSelection);
  }, []);

  const handleSendMessage = async () => {
    if ((!inputValue.trim() && !selectedText) || !token) return;

    setIsLoading(true);

    setMessages(prev => [
      ...prev,
      {
        text: inputValue || 'Question about selected text',
        sender: 'user',
        selectedText: selectedText || undefined,
      },
    ]);

    setInputValue('');

    try {
      const response = await fetch(
        'https://physical-ai-humanoid-robotics-textbook-fcve.onrender.com/chat',
        {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
            Authorization: `Bearer ${token}`,
          },
          body: JSON.stringify({
            query: inputValue || 'Question about selected text',
            selected_text: selectedText || null,
          }),
        }
      );

      if (!response.ok) {
        if (response.status === 401 && typeof window !== 'undefined') {
          window.location.reload();
        }
        throw new Error(`HTTP ${response.status}`);
      }

      const data = await response.json();
      setMessages(prev => [...prev, { text: data.response, sender: 'bot' }]);
      setSelectedText('');
    } catch {
      setMessages(prev => [
        ...prev,
        {
          text: '⚠️ Something went wrong. Please ensure you are logged in and the server is running.',
          sender: 'bot',
        },
      ]);
    } finally {
      setIsLoading(false);
    }
  };

  if (!isAuthenticated) {
    return (
      <div className={styles.chatWindow}>
        <div className={styles.chatHeader}>
          <h2>📚 Book Assistant</h2>
          <button onClick={onClose}>✕</button>
        </div>
        <AuthWrapper onAuthSuccess={() => {}} />
      </div>
    );
  }

  return (
    <div className={styles.chatWindow}>
      <div className={styles.chatHeader}>
        <h2>📚 Book Assistant</h2>
        <div>
          <button 
            className="font-md"
            onClick={() => {
              logout();
              if (typeof window !== 'undefined') window.location.reload();
            }}
          >
            Logout
          </button>
          <button onClick={onClose}>✕</button>
        </div>
      </div>

      <div className={styles.chatMessages}>
        {messages.map((m, i) => (
          <div key={i} className={`${styles.message} ${styles[m.sender]}`}>
            {m.selectedText && (
              <div className={styles.selectedTextPreview}>
                <em>{m.selectedText.slice(0, 50)}…</em>
              </div>
            )}
            {m.text}
          </div>
        ))}
        {isLoading && <div className={styles.bot}>...</div>}
        <div ref={messagesEndRef} />
      </div>

      <div className={styles.chatInput}>
        <input
          value={inputValue}
          onChange={e => setInputValue(e.target.value)}
          onKeyDown={e => e.key === 'Enter' && handleSendMessage()}
          placeholder="Ask a question…"
        />
        <button onClick={handleSendMessage} disabled={isLoading}>
          Send
        </button>
      </div>
    </div>
  );
};

export default ChatWindow;
