import React from 'react';
import { FaRobot } from 'react-icons/fa';
import { useChat } from '../../context/ChatContext';
import styles from './FloatingButton.module.css';

const FloatingButton: React.FC = () => {
  const { isChatOpen, openChat, closeChat } = useChat();

  const handleClick = () => {
    if (isChatOpen) {
      closeChat();
    } else {
      openChat();
    }
  };

  return (
    <button
      className={`${styles.floatingButton} ${isChatOpen ? styles.hidden : ''}`}
      onClick={handleClick}
      aria-label={isChatOpen ? 'Close AI Assistant' : 'Open AI Assistant'}
      title="Ask AI Assistant"
    >
      <FaRobot className={styles.icon} />
    </button>
  );
};

export default FloatingButton;