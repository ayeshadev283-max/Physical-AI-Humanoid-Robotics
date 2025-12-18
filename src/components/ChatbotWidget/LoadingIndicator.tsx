/**
 * LoadingIndicator Component
 * Displays loading state with animated spinner
 *
 * @feature 003-chatkit
 */

import React from 'react';
import styles from './styles.module.css';

export interface LoadingIndicatorProps {
  message?: string;
}

/**
 * Loading indicator with ARIA busy state for screen readers
 */
export function LoadingIndicator({
  message = 'Thinking...',
}: LoadingIndicatorProps): JSX.Element {
  return (
    <div className={styles.loadingContainer} role="status" aria-busy="true" aria-live="polite">
      <div className={styles.loadingSpinner} aria-hidden="true">
        <span className={styles.loadingDot}></span>
        <span className={styles.loadingDot}></span>
        <span className={styles.loadingDot}></span>
      </div>
      <span className={styles.loadingText}>{message}</span>
    </div>
  );
}
