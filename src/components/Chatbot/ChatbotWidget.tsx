import React, { useState, useRef, useEffect } from 'react';
import styles from './ChatbotWidget.module.css';
import { useChatbot } from '../../hooks/useChatbot';

interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  sources?: SourceReference[];
  timestamp: Date;
}

interface SourceReference {
  chapter: string;
  section: string;
  page_url: string;
}

export default function ChatbotWidget(): JSX.Element {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const { submitQuery } = useChatbot();

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    if (!input.trim() || isLoading) return;

    const userMessage: Message = {
      id: Date.now().toString(),
      role: 'user',
      content: input,
      timestamp: new Date(),
    };

    setMessages(prev => [...prev, userMessage]);
    setInput('');
    setIsLoading(true);

    try {
      const response = await submitQuery(input);

      const assistantMessage: Message = {
        id: (Date.now() + 1).toString(),
        role: 'assistant',
        content: response.answer,
        sources: response.sources,
        timestamp: new Date(),
      };

      setMessages(prev => [...prev, assistantMessage]);
    } catch (error) {
      const errorMessage: Message = {
        id: (Date.now() + 1).toString(),
        role: 'assistant',
        content: 'Sorry, I encountered an error. Please try again.',
        timestamp: new Date(),
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <>
      {/* Chat Toggle Button */}
      <button
        className={styles.chatToggle}
        onClick={() => setIsOpen(!isOpen)}
        aria-label="Toggle chatbot"
      >
        {isOpen ? '✕' : '💬'}
      </button>

      {/* Chat Window */}
      {isOpen && (
        <div className={styles.chatWindow}>
          <div className={styles.chatHeader}>
            <h3>Ask about the book</h3>
            <button
              className={styles.closeButton}
              onClick={() => setIsOpen(false)}
              aria-label="Close chatbot"
            >
              ✕
            </button>
          </div>

          <div className={styles.chatMessages}>
            {messages.length === 0 && (
              <div className={styles.welcomeMessage}>
                <p>👋 Hi! I can answer questions about this book.</p>
                <p>Try asking:</p>
                <ul>
                  <li>"What is Physical AI?"</li>
                  <li>"Explain embodied intelligence"</li>
                  <li>"What topics are covered?"</li>
                </ul>
              </div>
            )}

            {messages.map(message => (
              <div
                key={message.id}
                className={`${styles.message} ${
                  message.role === 'user' ? styles.userMessage : styles.assistantMessage
                }`}
              >
                <div className={styles.messageContent}>{message.content}</div>
                {message.sources && message.sources.length > 0 && (
                  <div className={styles.sources}>
                    <strong>Sources:</strong>
                    {message.sources.map((source, idx) => (
                      <a
                        key={idx}
                        href={source.page_url}
                        className={styles.sourceLink}
                        target="_blank"
                        rel="noopener noreferrer"
                      >
                        {source.chapter} - {source.section}
                      </a>
                    ))}
                  </div>
                )}
              </div>
            ))}

            {isLoading && (
              <div className={styles.loadingIndicator}>
                <span className={styles.loadingDot}></span>
                <span className={styles.loadingDot}></span>
                <span className={styles.loadingDot}></span>
              </div>
            )}

            <div ref={messagesEndRef} />
          </div>

          <form onSubmit={handleSubmit} className={styles.chatInput}>
            <input
              type="text"
              value={input}
              onChange={(e) => setInput(e.target.value)}
              placeholder="Ask a question..."
              disabled={isLoading}
              className={styles.input}
            />
            <button
              type="submit"
              disabled={isLoading || !input.trim()}
              className={styles.sendButton}
            >
              Send
            </button>
          </form>
        </div>
      )}
    </>
  );
}
