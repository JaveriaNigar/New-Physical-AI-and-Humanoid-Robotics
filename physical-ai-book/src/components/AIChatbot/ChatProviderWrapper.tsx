import React from 'react';
import { ChatProvider } from '../../context/ChatContext';
import AIChatbot from './index';

const ChatProviderWrapper: React.FC = () => {
  return (
    <ChatProvider>
      <AIChatbot />
    </ChatProvider>
  );
};

export default ChatProviderWrapper;