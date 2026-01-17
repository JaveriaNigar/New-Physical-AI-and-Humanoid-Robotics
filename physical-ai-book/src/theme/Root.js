import React, { useEffect } from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext'; // Import useDocusaurusContext
import { ChatProvider } from '../context/ChatContext';
import AskAIButton from '../components/AskAIButton';
import RAGChatbotIntegration from '../components/AIChatbot/RAGChatbot'; // Import the RAGChatbot

// Root component that wraps the entire app
const Root = ({ children }) => {
  const { siteConfig } = useDocusaurusContext(); // Get siteConfig
  const { apiBaseUrl } = siteConfig.customFields; // Extract apiBaseUrl from customFields

  useEffect(() => {
    // Initialize the RAGChatbotIntegration
    const chatbotIntegration = new RAGChatbotIntegration(apiBaseUrl); // Pass apiBaseUrl
    
    // Make it globally accessible if needed for debugging or other parts of the app
    window.ragChatbot = chatbotIntegration;
  }, [apiBaseUrl]); // Re-run if apiBaseUrl changes

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