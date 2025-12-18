import React, { useEffect, useState } from 'react';

export default function Root({ children }): JSX.Element {
  const [Components, setComponents] = useState<{
    ChatbotWidget: React.ComponentType<any> | null;
    TextSelectionHandler: React.ComponentType<any> | null;
  }>({ ChatbotWidget: null, TextSelectionHandler: null });

  useEffect(() => {
    // Dynamically import components only on client side
    Promise.all([
      import('../components/Chatbot/ChatbotWidget'),
      import('../components/TextSelection/TextSelectionHandler'),
    ]).then(([chatbot, textSelection]) => {
      setComponents({
        ChatbotWidget: chatbot.default,
        TextSelectionHandler: textSelection.default,
      });
    });
  }, []);

  const { ChatbotWidget, TextSelectionHandler } = Components;

  return (
    <>
      {children}
      {ChatbotWidget && <ChatbotWidget />}
      {TextSelectionHandler && <TextSelectionHandler />}
    </>
  );
}
