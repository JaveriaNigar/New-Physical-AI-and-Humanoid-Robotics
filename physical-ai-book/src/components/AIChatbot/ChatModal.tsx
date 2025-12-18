import React, { useEffect } from 'react';
import { FaTimes, FaPaperPlane } from 'react-icons/fa';
import { motion, AnimatePresence } from 'framer-motion';
import { useChat } from '../../context/ChatContext';
import styles from './ChatModal.module.css';

const ChatModal: React.FC = () => {
  const { isChatOpen, closeChat, messages, addMessage, isLoading } = useChat();
  const [inputValue, setInputValue] = React.useState('');

  const handleSend = () => {
    if (inputValue.trim() !== '') {
      addMessage({ text: inputValue, sender: 'user' });
      setInputValue('');
      
      // Simulate bot response after a delay
      setTimeout(() => {
        addMessage({ text: "Thanks for your message! I'm an AI assistant for the Physical AI & Humanoid Robotics textbook. I'm currently in development mode and can't provide actual answers yet, but I'll be ready soon!", sender: 'bot' });
      }, 1000);
    }
  };

  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSend();
    }
  };

  // Close chat when pressing Escape key
  useEffect(() => {
    const handleEsc = (e: KeyboardEvent) => {
      if (e.key === 'Escape') {
        closeChat();
      }
    };

    if (isChatOpen) {
      window.addEventListener('keydown', handleEsc);
    }

    return () => {
      window.removeEventListener('keydown', handleEsc);
    };
  }, [isChatOpen, closeChat]);

  return (
    <AnimatePresence>
      {isChatOpen && (
        <motion.div
          className={styles.overlay}
          initial={{ opacity: 0 }}
          animate={{ opacity: 1 }}
          exit={{ opacity: 0 }}
          onClick={closeChat}
        >
          <motion.div
            className={styles.chatModal}
            initial={{ y: 50, opacity: 0 }}
            animate={{ y: 0, opacity: 1 }}
            exit={{ y: 50, opacity: 0 }}
            transition={{ type: 'spring', damping: 25, stiffness: 300 }}
            onClick={(e) => e.stopPropagation()}
          >
            {/* Chat Header */}
            <div className={styles.header}>
              <div className={styles.headerContent}>
                <h3 className={styles.headerTitle}>AI Assistant</h3>
                <button 
                  className={styles.closeButton} 
                  onClick={closeChat}
                  aria-label="Close chat"
                >
                  <FaTimes />
                </button>
              </div>
            </div>

            {/* Chat Messages Area */}
            <div className={styles.messagesContainer}>
              {messages.length === 0 ? (
                <div className={styles.welcomeMessage}>
                  <p>Hello! I'm your AI Assistant for the Physical AI & Humanoid Robotics textbook.</p>
                  <p>Ask me anything about the content, and I'll do my best to help!</p>
                </div>
              ) : (
                <div className={styles.messagesList}>
                  {messages.map((message) => (
                    <div
                      key={message.id}
                      className={`${styles.message} ${styles[message.sender]}`}
                    >
                      <div className={styles.messageText}>{message.text}</div>
                      <div className={styles.messageTime}>
                        {message.timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
                      </div>
                    </div>
                  ))}
                  {isLoading && (
                    <div className={`${styles.message} ${styles.bot}`}>
                      <div className={styles.typingIndicator}>
                        <div className={styles.dot}></div>
                        <div className={styles.dot}></div>
                        <div className={styles.dot}></div>
                      </div>
                    </div>
                  )}
                </div>
              )}
            </div>

            {/* Input Area */}
            <div className={styles.inputContainer}>
              <textarea
                className={styles.textInput}
                value={inputValue}
                onChange={(e) => setInputValue(e.target.value)}
                onKeyPress={handleKeyPress}
                placeholder="Tell me your query"
                rows={1}
                aria-label="Type your message"
              />
              <button
                className={styles.sendButton}
                onClick={handleSend}
                disabled={inputValue.trim() === ''}
                aria-label="Send message"
              >
                <FaPaperPlane />
              </button>
            </div>
          </motion.div>
        </motion.div>
      )}
    </AnimatePresence>
  );
};

export default ChatModal;