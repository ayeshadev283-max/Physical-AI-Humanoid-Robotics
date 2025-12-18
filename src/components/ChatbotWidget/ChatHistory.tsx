/**
 * ChatHistory Component
 * Displays chronological list of chat messages
 *
 * @feature 003-chatkit
 */

import React, { useEffect, useRef } from 'react';
import { ChatMessage } from '../../types/chatbot';
import { ResponseDisplay } from './ResponseDisplay';
import styles from './styles.module.css';

export interface ChatHistoryProps {
  messages: ChatMessage[];
  welcomeMessage?: string;
  onFeedback?: (responseId: string, rating: 'helpful' | 'not_helpful') => void;
}

/**
 * Scrollable chat history with ARIA live region for screen readers
 * Displays messages in chronological order (oldest first)
 * Auto-scrolls to newest message
 */
export function ChatHistory({
  messages,
  welcomeMessage = 'Ask me anything about this chapter!',
  onFeedback,
}: ChatHistoryProps): JSX.Element {
  const scrollRef = useRef<HTMLDivElement>(null);

  // Auto-scroll to bottom when new messages arrive
  useEffect(() => {
    if (scrollRef.current) {
      scrollRef.current.scrollTop = scrollRef.current.scrollHeight;
    }
  }, [messages]);

  return (
    <div
      ref={scrollRef}
      className={styles.chatHistoryContainer}
      role="log"
      aria-live="polite"
      aria-label="Chat message history"
    >
      {messages.length === 0 && (
        <div className={styles.welcomeMessage} role="status">
          <p>{welcomeMessage}</p>
        </div>
      )}

      {messages.map((message) => (
        <div
          key={message.id}
          className={`${styles.messageContainer} ${
            message.type === 'user'
              ? styles.messageUser
              : styles.messageAssistant
          }`}
        >
          {message.type === 'user' ? (
            <div className={styles.userMessage}>
              <p className={styles.userMessageText}>{message.content}</p>
            </div>
          ) : (
            <ResponseDisplay message={message} onFeedback={onFeedback} />
          )}

          {message.error && (
            <div className={styles.messageError} role="alert">
              <span className={styles.messageErrorIcon} aria-hidden="true">
                ⚠️
              </span>
              <span>{message.error}</span>
            </div>
          )}

          <time className={styles.messageTimestamp} dateTime={message.timestamp.toISOString()}>
            {message.timestamp.toLocaleTimeString([], {
              hour: '2-digit',
              minute: '2-digit',
            })}
          </time>
        </div>
      ))}
    </div>
  );
}
