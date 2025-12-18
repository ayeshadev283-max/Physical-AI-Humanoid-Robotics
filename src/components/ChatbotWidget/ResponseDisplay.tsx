/**
 * ResponseDisplay Component
 * Renders chatbot responses with markdown formatting and source citations
 *
 * @feature 003-chatkit
 */

import React from 'react';
import ReactMarkdown from 'react-markdown';
import { ChatMessage } from '../../types/chatbot';
import styles from './styles.module.css';

export interface ResponseDisplayProps {
  message: ChatMessage;
  onFeedback?: (responseId: string, rating: 'helpful' | 'not_helpful') => void;
}

/**
 * Displays assistant responses with markdown rendering and source citations
 * Uses react-markdown for XSS-safe markdown rendering
 */
export function ResponseDisplay({
  message,
  onFeedback,
}: ResponseDisplayProps): JSX.Element {
  const handleFeedback = (rating: 'helpful' | 'not_helpful') => {
    if (message.responseId && onFeedback) {
      onFeedback(message.responseId, rating);
    }
  };

  return (
    <article className={styles.responseContainer} role="article">
      <div className={`${styles.responseContent} ${styles.responseMarkdown}`}>
        <ReactMarkdown>
          {message.content}
        </ReactMarkdown>
      </div>

      {message.sources && message.sources.length > 0 && (
        <div className={styles.sourcesContainer}>
          <h4 className={styles.sourcesHeading}>Sources:</h4>
          <ul className={styles.sourcesList}>
            {message.sources.map((source, index) => (
              <li key={index} className={styles.sourceItem}>
                <a
                  href={source.url}
                  target="_blank"
                  rel="noopener noreferrer"
                  className={styles.sourceLink}
                  aria-label={`Source ${index + 1}: ${source.title}`}
                >
                  {source.title}
                </a>
                {source.page_number && (
                  <span className={styles.sourcePageNumber}>
                    (Page {source.page_number})
                  </span>
                )}
              </li>
            ))}
          </ul>
        </div>
      )}

      {message.responseId && onFeedback && (
        <div className={styles.feedbackContainer}>
          <span className={styles.feedbackLabel}>Was this helpful?</span>
          <button
            onClick={() => handleFeedback('helpful')}
            className={styles.feedbackButton}
            aria-label="Mark as helpful"
            title="Helpful"
          >
            👍
          </button>
          <button
            onClick={() => handleFeedback('not_helpful')}
            className={styles.feedbackButton}
            aria-label="Mark as not helpful"
            title="Not helpful"
          >
            👎
          </button>
        </div>
      )}
    </article>
  );
}
