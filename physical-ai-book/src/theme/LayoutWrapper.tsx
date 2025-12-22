import React from 'react';
import Layout from '@theme-original/Layout';
import ChatProviderWrapper from '../components/AIChatbot/ChatProviderWrapper';

export default function LayoutWrapper(props) {
  return (
    <Layout {...props}>
      {props.children}
      <ChatProviderWrapper />
    </Layout>
  );
}