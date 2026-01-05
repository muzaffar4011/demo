import React from 'react';
import ChatbotWidget from '../components/Chatbot/ChatbotWidget';

const Root = ({ children }) => {
  return (
    <>
      {children}
      <ChatbotWidget />
    </>
  );
};

export default Root;