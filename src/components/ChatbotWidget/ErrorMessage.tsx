/**
 * ErrorMessage Component
 * Displays error messages with retry and dismiss actions
 *
 * @feature 003-chatkit
 */

import React, { useCallback } from 'react';
import { ChatbotError } from '../../types/chatbot';
import styles from './styles.module.css';

export interface ErrorMessageProps {
  error: Error | ChatbotError;
  onRetry?: () => void;
  onDismiss?: () => void;
}

/**
 * Error message component with ARIA alert role for accessibility
 * Shows retry button only for retryable errors
 */
export function ErrorMessage({
  error,
  onRetry,
  onDismiss,
}: ErrorMessageProps): JSX.Element {
  const isRetryable =
    error instanceof ChatbotError ? error.retryable : true;

  const handleRetry = useCallback(() => {
    if (onRetry) {
      onRetry();
    }
  }, [onRetry]);

  const handleDismiss = useCallback(() => {
    if (onDismiss) {
      onDismiss();
    }
  }, [onDismiss]);

  return (
    <div className={styles.errorContainer} role="alert" aria-live="assertive">
      <div className={styles.errorContent}>
        <svg
          className={styles.errorIcon}
          width="20"
          height="20"
          viewBox="0 0 20 20"
          fill="none"
          xmlns="http://www.w3.org/2000/svg"
          aria-hidden="true"
        >
          <path
            d="M10 18C14.4183 18 18 14.4183 18 10C18 5.58172 14.4183 2 10 2C5.58172 2 2 5.58172 2 10C2 14.4183 5.58172 18 10 18Z"
            stroke="currentColor"
            strokeWidth="2"
          />
          <path
            d="M10 6V10M10 14H10.01"
            stroke="currentColor"
            strokeWidth="2"
            strokeLinecap="round"
          />
        </svg>
        <p className={styles.errorText}>{error.message}</p>
      </div>

      <div className={styles.errorActions}>
        {isRetryable && onRetry && (
          <button
            onClick={handleRetry}
            className={styles.retryButton}
            aria-label="Retry request"
          >
            Retry
          </button>
        )}
        {onDismiss && (
          <button
            onClick={handleDismiss}
            className={styles.dismissButton}
            aria-label="Dismiss error"
          >
            Dismiss
          </button>
        )}
      </div>
    </div>
  );
}
