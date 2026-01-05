import React from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import Chatbot from './Chatbot';

const ChatbotWidget = () => {
  return (
    <BrowserOnly>
      {() => {
        // Get API URL from a global variable that can be set via Docusaurus config
        // Default to localhost if not provided
        const apiUrl = (window as any).CHATBOT_API_URL || 'http://localhost:8000';
        return <Chatbot apiUrl={apiUrl} position="bottom-right" />;
      }}
    </BrowserOnly>
  );
};

export default ChatbotWidget;