import React, { useEffect, useRef } from 'react';
import { FaTimes, FaPaperPlane } from 'react-icons/fa';
import { motion, AnimatePresence } from 'framer-motion';
import { useChat } from '../../context/ChatContext';
import styles from './ChatModal.module.css';
import RAGChatbotIntegration from './RAGChatbot'; // Import the RAGChatbotIntegration class

const ChatModal: React.FC = () => {
  const { isChatOpen, closeChat, messages, addMessage, isLoading, setIsLoading } = useChat();
  const [inputValue, setInputValue] = React.useState('');
  const textAreaRef = useRef<HTMLTextAreaElement>(null);
  const chatbotIntegrationRef = useRef<RAGChatbotIntegration | null>(null);

  useEffect(() => {
    // Initialize with the correct backend URL for the new backend
    chatbotIntegrationRef.current = new RAGChatbotIntegration('https://huggingface.co/spaces/javeria-nigar/chatbot?logs=container');
  }, []);

  const handleSend = async () => {
    if (inputValue.trim() === '') return;

    addMessage({ text: inputValue, sender: 'user' });
    setIsLoading(true);

    try {
      if (chatbotIntegrationRef.current) {
        const answer = await chatbotIntegrationRef.current.sendQuestionToBackend(inputValue);
        addMessage({ text: answer, sender: "bot" });
      } else {
        throw new Error("Chatbot integration not initialized.");
      }
    } catch (err) {
      console.error('ChatModal send error:', err);
      addMessage({
        text: "Sorry, I encountered an error processing your request. Please try again.",
        sender: "bot",
      });
    }

    setIsLoading(false);
    setInputValue("");
  };

  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSend();
    }
  };

  useEffect(() => {
    const handleEsc = (e: KeyboardEvent) => {
      if (e.key === 'Escape') closeChat();
    };
    if (isChatOpen) window.addEventListener('keydown', handleEsc);
    return () => window.removeEventListener('keydown', handleEsc);
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
            <div className={styles.header}>
              <div className={styles.headerContent}>
                <h3 className={styles.headerTitle}>AI Assistant</h3>
                <button className={styles.closeButton} onClick={closeChat} aria-label="Close chat">
                  <FaTimes />
                </button>
              </div>
            </div>

            <div className={styles.messagesContainer}>
              {messages.length === 0 ? (
                <div className={styles.welcomeMessage}>
                  <p>I'm your AI Assistant for the Physical AI & Humanoid Robotics textbook.</p>
                </div>
              ) : (
                <div className={styles.messagesList}>
                  {messages.map((message) => (
                    <div key={message.id} className={`${styles.message} ${styles[message.sender]}`}>
                      <div className={styles.messageText}>{message.text}</div>
                      <div className={styles.messageTime}>
                        {message.timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
                      </div>
                    </div>
                  ))}
                  {isLoading && (
                    <div className={`${styles.message} ${styles.bot}`}>
                      <div className={`${styles.messageText} ${styles.typingIndicator}`}>
                        <div className={styles.dot}></div>
                        <div className={styles.dot}></div>
                        <div className={styles.dot}></div>
                      </div>
                    </div>
                  )}
                </div>
              )}
            </div>

            <div className={styles.inputContainer}>
              <textarea
                ref={textAreaRef}
                className={styles.textInput}
                value={inputValue}
                onChange={(e) => setInputValue(e.target.value)}
                onKeyPress={handleKeyPress}
                placeholder="Ask Anything..."
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
