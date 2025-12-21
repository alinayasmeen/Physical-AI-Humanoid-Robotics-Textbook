import React, { useState, useEffect } from 'react';
import ChatbotButton from './ChatbotButton';
import ChatWindow from './ChatWindow';

const Chatbot = () => {
  const [isOpen, setIsOpen] = useState(false);

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  useEffect(() => {
    // Listen for the custom event to open the chatbot from other components
    const handleOpenChatbot = () => {
      setIsOpen(true);
    };

    window.addEventListener('openChatbot', handleOpenChatbot);

    // Cleanup event listener on component unmount
    return () => {
      window.removeEventListener('openChatbot', handleOpenChatbot);
    };
  }, []);

  return (
    <>
      <ChatbotButton onClick={toggleChat} />
      {isOpen && <ChatWindow onClose={toggleChat} />}
    </>
  );
};

export default Chatbot;
