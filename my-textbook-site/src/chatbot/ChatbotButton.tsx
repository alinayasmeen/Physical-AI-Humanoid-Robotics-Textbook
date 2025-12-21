import React from 'react';
import { useAuth } from '../contexts/AuthContext';
import styles from './styles.module.css';

interface ChatbotButtonProps {
  onClick: () => void;
}

const ChatbotButton: React.FC<ChatbotButtonProps> = ({ onClick }) => {
  const { user, isAuthenticated } = useAuth();

  return (
    <button className={styles.chatbotButton} onClick={onClick}>
      {/* User Avatar or Chat Icon */}
      {isAuthenticated && user ? (
        <div className={styles.userAvatar}>
          {user.full_name.charAt(0).toUpperCase()}
        </div>
      ) : (
        <>
          {/* Animated Chat Icon */}
          <svg
            width="32"
            height="32"
            viewBox="0 0 24 24"
            fill="none"
            xmlns="http://www.w3.org/2000/svg"
            className={styles.chatIcon}
          >
            {/* Chat bubble */}
            <path
              d="M20 2H4C2.9 2 2 2.9 2 4V22L6 18H20C21.1 18 22 17.1 22 16V4C22 2.9 21.1 2 20 2Z"
              stroke="currentColor"
              strokeWidth="2"
              strokeLinecap="round"
              strokeLinejoin="round"
              fill="none"
            />

            {/* Animated dots */}
            <circle
              cx="8"
              cy="10"
              r="1.5"
              fill="currentColor"
              className={styles.dot1}
            />
            <circle
              cx="12"
              cy="10"
              r="1.5"
              fill="currentColor"
              className={styles.dot2}
            />
            <circle
              cx="16"
              cy="10"
              r="1.5"
              fill="currentColor"
              className={styles.dot3}
            />
          </svg>
        </>
      )}
      <span className={styles.buttonText}>
        {isAuthenticated ? user?.full_name : 'Chat'}
      </span>
    </button>
  );
};

export default ChatbotButton;