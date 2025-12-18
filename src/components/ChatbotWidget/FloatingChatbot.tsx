/**
 * FloatingChatbot Component
 * Floating chat icon that opens/closes the ChatbotWidget
 *
 * @feature 003-chatkit
 */

import React, { useState, useCallback } from 'react';
import { ChatbotWidget } from './index';
import styles from './floating.module.css';

export interface FloatingChatbotProps {
  bookId: string;
  chapterNumber?: number;
  theme?: 'light' | 'dark';
  welcomeMessage?: string;
}

/**
 * Floating chatbot with toggle icon
 * Displays a chat icon button that opens the chatbot widget
 */
export function FloatingChatbot({
  bookId,
  chapterNumber = 1,
  theme = 'light',
  welcomeMessage,
}: FloatingChatbotProps): JSX.Element {
  const [isOpen, setIsOpen] = useState(false);

  const handleToggle = useCallback(() => {
    setIsOpen((prev) => !prev);
  }, []);

  const handleClose = useCallback(() => {
    setIsOpen(false);
  }, []);

  // Prevent body scroll when chatbot is open on mobile
  React.useEffect(() => {
    if (isOpen && window.innerWidth <= 768) {
      document.body.style.overflow = 'hidden';
    } else {
      document.body.style.overflow = '';
    }

    return () => {
      document.body.style.overflow = '';
    };
  }, [isOpen]);

  return (
    <div style={{ position: 'relative', zIndex: 9999 }}>
      {/* Backdrop for mobile */}
      {isOpen && (
        <div
          className={styles.backdrop}
          onClick={handleClose}
          aria-hidden="true"
        />
      )}

      {/* Floating Chat Button */}
      {!isOpen && (
        <button
          className={styles.floatingButton}
          onClick={handleToggle}
          aria-label="Open AI Assistant"
          title="Chat with AI Assistant"
        >
          {/* Modern Chat Bubble Icon */}
          <svg
            width="28"
            height="28"
            viewBox="0 0 24 24"
            fill="none"
            xmlns="http://www.w3.org/2000/svg"
            className={styles.chatIcon}
            aria-hidden="true"
          >
            {/* Chat Bubble Shape */}
            <path
              d="M20 2H4C2.9 2 2.01 2.9 2.01 4L2 22L6 18H20C21.1 18 22 17.1 22 16V4C22 2.9 21.1 2 20 2Z"
              fill="white"
              fillOpacity="0.95"
            />
            {/* Message Lines */}
            <path
              d="M7 8H17"
              stroke="#0f172a"
              strokeWidth="1.8"
              strokeLinecap="round"
            />
            <path
              d="M7 12H14"
              stroke="#0f172a"
              strokeWidth="1.8"
              strokeLinecap="round"
            />
          </svg>

          {/* AI Sparkle Indicator */}
          <svg
            width="14"
            height="14"
            viewBox="0 0 24 24"
            fill="none"
            xmlns="http://www.w3.org/2000/svg"
            className={styles.sparkle}
            aria-hidden="true"
          >
            <path
              d="M12 2L14.5 9.5L22 12L14.5 14.5L12 22L9.5 14.5L2 12L9.5 9.5L12 2Z"
              fill="#60a5fa"
            />
          </svg>

          {/* Pulse ring effect */}
          <span className={styles.pulseRing}></span>
        </button>
      )}

      {/* Chatbot Widget Popup */}
      {isOpen && (
        <div className={styles.popupContainer}>
          <div className={styles.popupHeader}>
            <h3 className={styles.popupTitle}>AI Assistant</h3>
            <button
              className={styles.closeButton}
              onClick={handleClose}
              aria-label="Close chatbot"
            >
              <svg
                width="20"
                height="20"
                viewBox="0 0 20 20"
                fill="none"
                xmlns="http://www.w3.org/2000/svg"
              >
                <path
                  d="M15 5L5 15M5 5L15 15"
                  stroke="currentColor"
                  strokeWidth="2"
                  strokeLinecap="round"
                />
              </svg>
            </button>
          </div>
          <ChatbotWidget
            bookId={bookId}
            chapterNumber={chapterNumber}
            theme={theme}
            position="bottom-right"
            welcomeMessage={welcomeMessage}
          />
        </div>
      )}
    </div>
  );
}

export default FloatingChatbot;
