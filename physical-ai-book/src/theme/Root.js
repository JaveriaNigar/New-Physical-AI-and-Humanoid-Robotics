import React, { useEffect } from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import { ChatProvider } from '../context/ChatContext';
import AskAIButton from '../components/AskAIButton';
import RAGChatbotIntegration from '../components/AIChatbot/RAGChatbot'; // Import the RAGChatbot

// Root component that wraps the entire app
const Root = ({ children }) => {
  useEffect(() => {
    // Initialize the RAGChatbotIntegration
    // This will now use the environment variable set in .env
    const chatbotIntegration = new RAGChatbotIntegration();
    
    // Make it globally accessible if needed for debugging or other parts of the app
    window.ragChatbot = chatbotIntegration;
  }, []); // Run once on component mount

  return (
    <ChatProvider>
      {children}
      <BrowserOnly>
        {() => <AskAIButton />}
      </BrowserOnly>
    </ChatProvider>
  );
};

export default Root;