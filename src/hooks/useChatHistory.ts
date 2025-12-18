/**
 * useChatHistory Hook
 * Manages chat message history with size limit
 *
 * @feature 003-chatkit
 */

import { useState, useCallback } from 'react';
import { ChatMessage, VALIDATION } from '../types/chatbot';

export interface UseChatHistoryReturn {
  messages: ChatMessage[];
  addMessage: (message: ChatMessage) => void;
  clearHistory: () => void;
}

/**
 * Hook for managing chat message history
 * Enforces maximum message limit per FR-036
 */
export function useChatHistory(): UseChatHistoryReturn {
  const [messages, setMessages] = useState<ChatMessage[]>([]);

  const addMessage = useCallback((message: ChatMessage) => {
    setMessages((prevMessages) => {
      const newMessages = [...prevMessages, message];

      // Enforce max messages limit (FR-036)
      if (newMessages.length > VALIDATION.MAX_MESSAGES) {
        // Remove oldest messages to stay within limit
        return newMessages.slice(newMessages.length - VALIDATION.MAX_MESSAGES);
      }

      return newMessages;
    });
  }, []);

  const clearHistory = useCallback(() => {
    setMessages([]);
  }, []);

  return {
    messages,
    addMessage,
    clearHistory,
  };
}
