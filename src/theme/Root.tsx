import React from 'react';
import ChatbotWidget from '../components/Chatbot/ChatbotWidget';
import TextSelectionHandler from '../components/TextSelection/TextSelectionHandler';

export default function Root({ children }): JSX.Element {
  return (
    <>
      {children}
      <TextSelectionHandler />
      <ChatbotWidget />
    </>
  );
}
