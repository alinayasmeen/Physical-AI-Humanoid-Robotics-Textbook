import React from 'react';
import styles from './styles.module.css';

interface ChatbotButtonProps {
  onClick: () => void;
}

const ChatbotButton: React.FC<ChatbotButtonProps> = ({ onClick }) => {
  return (
    <button className={styles.chatbotButton} onClick={onClick}>
      Chat
    </button>
  );
};

export default ChatbotButton;
