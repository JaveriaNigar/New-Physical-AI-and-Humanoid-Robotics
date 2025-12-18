import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import { ChatProvider } from '../context/ChatContext';
import AIChatbot from '../components/AIChatbot';

type LayoutProps = {
  children: React.ReactNode;
};

export default function Layout(props: LayoutProps) {
  return (
    <ChatProvider>
      <OriginalLayout {...props} />
      <AIChatbot />
    </ChatProvider>
  );
}