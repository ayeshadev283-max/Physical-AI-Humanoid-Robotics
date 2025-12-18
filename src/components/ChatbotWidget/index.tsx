/**
 * ChatbotWidget - Main chatbot component
 * Composes all subcomponents and manages state via useChatbot hook
 *
 * @feature 003-chatkit
 */

import React, { useCallback } from 'react';
import { ChatbotProps, Theme, Position } from '../../types/chatbot';
import { useChatbot } from '../../hooks/useChatbot';
import { QueryInput } from './QueryInput';
import { ChatHistory } from './ChatHistory';
import { LoadingIndicator } from './LoadingIndicator';
import { ErrorMessage } from './ErrorMessage';
import { chatbotApi } from '../../services/chatbotApi';
import styles from './styles.module.css';

/**
 * Main chatbot widget component
 * Provides complete chat interface with history, input, and error handling
 */
export function ChatbotWidget({
  bookId,
  chapterNumber,
  theme = 'light',
  position = 'bottom-right',
  welcomeMessage = 'Ask me anything about this chapter!',
  onQuerySubmit,
  onError,
}: ChatbotProps): JSX.Element {
  const { messages, loading, error, sendQuery, retryLast, clearHistory } = useChatbot({
    bookId,
    chapterNumber,
    onQuerySubmit,
    onError,
  });

  // Handle feedback submission (User Story 3)
  const handleFeedback = useCallback(
    async (responseId: string, rating: 'helpful' | 'not_helpful') => {
      try {
        const { userId } = await import('../../hooks/useUserId').then((m) => {
          const hook = m.useUserId();
          return { userId: hook.userId };
        });

        await chatbotApi.submitFeedback({
          response_id: responseId,
          rating,
          comment: null,
          user_id: userId,
        });
      } catch (err) {
        console.error('Failed to submit feedback:', err);
      }
    },
    []
  );

  const handleRetry = useCallback(() => {
    retryLast();
  }, [retryLast]);

  const handleDismissError = useCallback(() => {
    // Error is already added to chat history, no need to do anything
  }, []);

  // Position class mapping
  const positionClass = {
    'bottom-right': styles.positionBottomRight,
    'bottom-left': styles.positionBottomLeft,
    'top-right': styles.positionTopRight,
    'top-left': styles.positionTopLeft,
  }[position];

  // Theme class
  const themeClass = theme === 'dark' ? styles.themeDark : styles.themeLight;

  return (
    <div className={`${styles.chatbotWidget} ${positionClass} ${themeClass}`}>
      <div className={styles.chatbotHeader}>
        <h3 className={styles.chatbotTitle}>AI Assistant</h3>
        <button
          className={styles.clearButton}
          onClick={clearHistory}
          aria-label="Clear chat history"
          disabled={messages.length === 0}
        >
          Clear
        </button>
      </div>

      <ChatHistory
        messages={messages}
        welcomeMessage={welcomeMessage}
        onFeedback={handleFeedback}
      />

      {loading && <LoadingIndicator />}

      {error && !loading && (
        <ErrorMessage
          error={error}
          onRetry={handleRetry}
          onDismiss={handleDismissError}
        />
      )}

      <QueryInput onSubmit={sendQuery} disabled={loading} />
    </div>
  );
}

export default ChatbotWidget;
