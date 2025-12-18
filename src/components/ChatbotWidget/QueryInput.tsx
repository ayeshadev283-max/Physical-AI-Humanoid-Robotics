/**
 * QueryInput Component
 * Text input field with submit button and character counter
 *
 * @feature 003-chatkit
 */

import React, { useState, useCallback, KeyboardEvent, ChangeEvent } from 'react';
import { VALIDATION } from '../../types/chatbot';
import styles from './styles.module.css';

export interface QueryInputProps {
  onSubmit: (query: string) => void;
  disabled?: boolean;
  placeholder?: string;
}

/**
 * Text input component for submitting chatbot queries
 * Supports keyboard navigation (Enter to submit) and character limit enforcement
 */
export function QueryInput({
  onSubmit,
  disabled = false,
  placeholder = 'Ask a question...',
}: QueryInputProps): JSX.Element {
  const [query, setQuery] = useState<string>('');

  const handleChange = useCallback((event: ChangeEvent<HTMLTextAreaElement>) => {
    const value = event.target.value;
    // Enforce max length
    if (value.length <= VALIDATION.MAX_QUESTION_LENGTH) {
      setQuery(value);
    }
  }, []);

  const handleSubmit = useCallback(() => {
    const trimmedQuery = query.trim();
    if (trimmedQuery.length > 0 && !disabled) {
      onSubmit(trimmedQuery);
      setQuery(''); // Clear input after submit
    }
  }, [query, disabled, onSubmit]);

  const handleKeyDown = useCallback(
    (event: KeyboardEvent<HTMLTextAreaElement>) => {
      // Submit on Enter (without Shift)
      if (event.key === 'Enter' && !event.shiftKey) {
        event.preventDefault();
        handleSubmit();
      }
    },
    [handleSubmit]
  );

  const isSubmitDisabled = disabled || query.trim().length === 0;
  const characterCount = query.length;
  const isNearLimit = characterCount >= VALIDATION.MAX_QUESTION_LENGTH * 0.9;

  return (
    <div className={styles.queryInputContainer}>
      <textarea
        value={query}
        onChange={handleChange}
        onKeyDown={handleKeyDown}
        placeholder={placeholder}
        disabled={disabled}
        maxLength={VALIDATION.MAX_QUESTION_LENGTH}
        rows={3}
        className={styles.queryTextarea}
        aria-label="Ask a question"
        aria-describedby="char-counter"
      />

      <div className={styles.queryInputFooter}>
        <span
          id="char-counter"
          className={`${styles.characterCounter} ${
            isNearLimit ? styles.characterCounterWarning : ''
          }`}
          aria-live="polite"
        >
          {characterCount}/{VALIDATION.MAX_QUESTION_LENGTH}
        </span>

        <button
          onClick={handleSubmit}
          disabled={isSubmitDisabled}
          className={styles.submitButton}
          aria-label="Submit question"
        >
          Send
        </button>
      </div>
    </div>
  );
}
