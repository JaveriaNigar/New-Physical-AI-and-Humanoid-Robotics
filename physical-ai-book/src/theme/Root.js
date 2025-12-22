import React from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import { ChatProvider } from '../context/ChatContext';
import AskAIButton from '../components/AskAIButton';

// Root component that wraps the entire app
const Root = ({ children }) => {
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