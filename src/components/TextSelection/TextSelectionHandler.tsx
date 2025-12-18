import React, { useEffect, useState } from 'react';
import styles from './TextSelectionHandler.module.css';
import { useChatbot } from '../../hooks/useChatbot';

export default function TextSelectionHandler(): JSX.Element {
  const [selectedText, setSelectedText] = useState('');
  const [showPopup, setShowPopup] = useState(false);
  const [popupPosition, setPopupPosition] = useState({ x: 0, y: 0 });
  const [question, setQuestion] = useState('');
  const [showQuestionModal, setShowQuestionModal] = useState(false);
  const [isLoading, setIsLoading] = useState(false);
  const [answer, setAnswer] = useState('');
  const { submitQuery } = useChatbot();

  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      const text = selection?.toString().trim();

      if (text && text.length > 10) {
        setSelectedText(text);

        // Get selection position
        const range = selection?.getRangeAt(0);
        if (range) {
          const rect = range.getBoundingClientRect();
          setPopupPosition({
            x: rect.left + rect.width / 2,
            y: rect.top - 10,
          });
          setShowPopup(true);
        }
      } else {
        setShowPopup(false);
      }
    };

    document.addEventListener('mouseup', handleSelection);
    document.addEventListener('keyup', handleSelection);

    return () => {
      document.removeEventListener('mouseup', handleSelection);
      document.removeEventListener('keyup', handleSelection);
    };
  }, []);

  const handleAskAI = () => {
    setShowPopup(false);
    setShowQuestionModal(true);
    setAnswer('');
  };

  const handleSubmitQuestion = async (e: React.FormEvent) => {
    e.preventDefault();
    if (!question.trim() || isLoading) return;

    setIsLoading(true);
    try {
      const response = await submitQuery(question, selectedText);
      setAnswer(response.answer);
    } catch (error) {
      setAnswer('Sorry, I encountered an error. Please try again.');
    } finally {
      setIsLoading(false);
    }
  };

  const handleClose = () => {
    setShowQuestionModal(false);
    setQuestion('');
    setAnswer('');
    setSelectedText('');
  };

  return (
    <>
      {/* Floating "Ask AI" button */}
      {showPopup && (
        <button
          className={styles.askAiButton}
          style={{
            left: `${popupPosition.x}px`,
            top: `${popupPosition.y}px`,
          }}
          onClick={handleAskAI}
        >
          Ask AI about this
        </button>
      )}

      {/* Question modal */}
      {showQuestionModal && (
        <div className={styles.modalOverlay} onClick={handleClose}>
          <div className={styles.modal} onClick={(e) => e.stopPropagation()}>
            <div className={styles.modalHeader}>
              <h3>Ask about selected text</h3>
              <button className={styles.closeButton} onClick={handleClose}>
                ✕
              </button>
            </div>

            <div className={styles.selectedTextPreview}>
              <strong>Selected text:</strong>
              <p>{selectedText.substring(0, 200)}{selectedText.length > 200 ? '...' : ''}</p>
            </div>

            <form onSubmit={handleSubmitQuestion} className={styles.questionForm}>
              <input
                type="text"
                value={question}
                onChange={(e) => setQuestion(e.target.value)}
                placeholder="What do you want to know about this text?"
                disabled={isLoading}
                className={styles.questionInput}
                autoFocus
              />
              <button
                type="submit"
                disabled={isLoading || !question.trim()}
                className={styles.submitButton}
              >
                {isLoading ? 'Processing...' : 'Ask'}
              </button>
            </form>

            {answer && (
              <div className={styles.answer}>
                <strong>Answer:</strong>
                <p>{answer}</p>
              </div>
            )}
          </div>
        </div>
      )}
    </>
  );
}
