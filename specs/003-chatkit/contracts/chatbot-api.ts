/**
 * ChatKit API Contracts
 *
 * TypeScript interface definitions for ChatKit component library.
 * These types define the contract between the frontend ChatbotWidget
 * and the backend RAG API (feature 002-rag-chatbot-education).
 *
 * @feature 003-chatkit
 * @date 2025-12-12
 */

// ============================================================================
// Component Props
// ============================================================================

/**
 * Props for the main ChatbotWidget component
 *
 * @example
 * ```tsx
 * <ChatbotWidget
 *   bookId="physical-ai-robotics"
 *   chapterNumber={1}
 *   theme="dark"
 *   position="bottom-right"
 *   welcomeMessage="Ask me anything about this chapter!"
 * />
 * ```
 */
export interface ChatbotProps {
  /** Unique identifier for the book (e.g., "physical-ai-robotics") */
  bookId: string;

  /** Chapter number (1-indexed) */
  chapterNumber: number;

  /** Visual theme (default: "light") */
  theme?: Theme;

  /** Position on screen (default: "bottom-right") */
  position?: Position;

  /** Custom welcome message (default: "Ask me anything about this chapter!") */
  welcomeMessage?: string;

  /** Optional callback when user submits a query (for analytics) */
  onQuerySubmit?: (query: string) => void;

  /** Optional callback when an error occurs (for error tracking) */
  onError?: (error: Error) => void;
}

// ============================================================================
// Chat Messages
// ============================================================================

/**
 * Represents a single message in the chat history
 */
export interface ChatMessage {
  /** Unique message ID (UUID v4) */
  id: string;

  /** Message type */
  type: MessageType;

  /** Message content (plain text for user, markdown for assistant) */
  content: string;

  /** Timestamp when message was created */
  timestamp: Date;

  /** Source citations (only present in assistant messages) */
  sources?: Source[];

  /** Error message (only present if API call failed) */
  error?: string;

  /** Backend response ID for feedback linking (only in assistant messages) */
  responseId?: string;
}

/**
 * Citation/source reference in an assistant message
 */
export interface Source {
  /** Chapter number where source is located */
  chapterNumber: number;

  /** Section title for citation */
  sectionTitle: string;

  /** Optional page number (for PDF/print versions) */
  pageNumber?: number;

  /** Link to book section (relative or absolute URL) */
  url: string;
}

// ============================================================================
// Component State
// ============================================================================

/**
 * Shape of React state in the ChatbotWidget component
 */
export interface ChatbotState {
  /** Chat history (chronological order, oldest first) */
  messages: ChatMessage[];

  /** Current input field value */
  inputValue: string;

  /** Is API request in progress? */
  loading: boolean;

  /** Current error message (null if no error) */
  error: string | null;

  /** Persistent user ID (from localStorage) */
  userId: string;
}

// ============================================================================
// Backend API Contracts (from 002-rag-chatbot-education)
// ============================================================================

/**
 * Request payload for POST /v1/query
 *
 * @example
 * ```json
 * {
 *   "book_id": "physical-ai-robotics",
 *   "chapter_number": 1,
 *   "question": "What is the difference between ROS1 and ROS2?",
 *   "user_id": "550e8400-e29b-41d4-a716-446655440000"
 * }
 * ```
 */
export interface QueryRequest {
  /** Book identifier */
  book_id: string;

  /** Chapter number */
  chapter_number: number;

  /** User's question (max 500 characters) */
  question: string;

  /** Persistent user UUID */
  user_id: string;
}

/**
 * Response from POST /v1/query
 *
 * @example
 * ```json
 * {
 *   "answer": "ROS2 improves upon ROS1 by using DDS for communication...",
 *   "sources": [
 *     {
 *       "chapter_number": 2,
 *       "section_title": "ROS2 Core Concepts",
 *       "page_number": 45
 *     }
 *   ],
 *   "confidence": 0.92,
 *   "response_id": "7c9e6679-7425-40de-944b-e07fc1f90ae7"
 * }
 * ```
 */
export interface QueryResponse {
  /** AI-generated answer (markdown format) */
  answer: string;

  /** Source citations */
  sources: BackendSource[];

  /** AI confidence score (0.0 to 1.0) */
  confidence: number;

  /** UUID for feedback tracking */
  response_id: string;
}

/**
 * Source citation from backend (snake_case to match API)
 */
export interface BackendSource {
  chapter_number: number;
  section_title: string;
  page_number?: number;
}

/**
 * Request payload for POST /v1/feedback
 *
 * @example
 * ```json
 * {
 *   "response_id": "7c9e6679-7425-40de-944b-e07fc1f90ae7",
 *   "rating": "helpful",
 *   "comment": "Clear and concise explanation!",
 *   "user_id": "550e8400-e29b-41d4-a716-446655440000"
 * }
 * ```
 */
export interface FeedbackRequest {
  /** Links to query_responses table */
  response_id: string;

  /** User rating */
  rating: FeedbackRating;

  /** Optional user comment (max 500 characters) */
  comment?: string;

  /** Persistent user UUID */
  user_id: string;
}

/**
 * Response from POST /v1/feedback
 */
export interface FeedbackResponse {
  /** Unique feedback ID */
  feedback_id: string;

  /** Status confirmation */
  status: 'recorded';
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * Chatbot-specific error class
 */
export class ChatbotError extends Error {
  constructor(
    message: string,
    public readonly code: ErrorCode,
    public readonly retryable: boolean = false
  ) {
    super(message);
    this.name = 'ChatbotError';
  }
}

/**
 * Error codes for different failure scenarios
 */
export enum ErrorCode {
  /** Network/connection error (retryable) */
  NETWORK_ERROR = 'NETWORK_ERROR',

  /** API returned 4xx error (not retryable) */
  CLIENT_ERROR = 'CLIENT_ERROR',

  /** API returned 5xx error (retryable) */
  SERVER_ERROR = 'SERVER_ERROR',

  /** Request timed out (retryable) */
  TIMEOUT = 'TIMEOUT',

  /** Invalid API response format */
  INVALID_RESPONSE = 'INVALID_RESPONSE',

  /** Backend returned "insufficient context" */
  INSUFFICIENT_CONTEXT = 'INSUFFICIENT_CONTEXT',
}

// ============================================================================
// Utility Types
// ============================================================================

/**
 * Message type discriminator
 */
export type MessageType = 'user' | 'assistant';

/**
 * Visual theme for chatbot widget
 */
export type Theme = 'light' | 'dark';

/**
 * Position of chatbot widget on screen
 */
export type Position =
  | 'bottom-right'
  | 'bottom-left'
  | 'top-right'
  | 'top-left';

/**
 * Feedback rating values
 */
export type FeedbackRating = 'helpful' | 'not_helpful';

// ============================================================================
// Service Interfaces
// ============================================================================

/**
 * Interface for ChatbotApiService (HTTP client)
 */
export interface IChatbotApiService {
  /**
   * Submit a query to the backend
   * @throws {ChatbotError} If request fails
   */
  submitQuery(request: QueryRequest): Promise<QueryResponse>;

  /**
   * Submit feedback for a response
   * @throws {ChatbotError} If request fails
   */
  submitFeedback(request: FeedbackRequest): Promise<FeedbackResponse>;
}

/**
 * Interface for caching service
 */
export interface ICacheService<T> {
  /**
   * Get cached value by key
   * @returns Cached value or null if not found/expired
   */
  get(key: string): T | null;

  /**
   * Set cached value with TTL
   */
  set(key: string, value: T, ttl?: number): void;

  /**
   * Clear all cached values
   */
  clear(): void;
}

// ============================================================================
// Hook Return Types
// ============================================================================

/**
 * Return type for useChatbot hook
 */
export interface UseChatbotReturn {
  /** Chat history */
  messages: ChatMessage[];

  /** Current input value */
  inputValue: string;

  /** Is API request in progress? */
  loading: boolean;

  /** Current error (null if none) */
  error: string | null;

  /** Send a query to the backend */
  sendQuery: (question: string) => Promise<void>;

  /** Retry the last failed query */
  retryLast: () => Promise<void>;

  /** Update input value */
  setInputValue: (value: string) => void;

  /** Clear error */
  clearError: () => void;
}

/**
 * Return type for useUserId hook
 */
export interface UseUserIdReturn {
  /** Persistent user ID (UUID) */
  userId: string;

  /** Is user ID loaded from storage? */
  loaded: boolean;
}

/**
 * Return type for useChatHistory hook
 */
export interface UseChatHistoryReturn {
  /** Chat history (max 50 messages) */
  messages: ChatMessage[];

  /** Add a message to history */
  addMessage: (message: ChatMessage) => void;

  /** Clear all messages */
  clearHistory: () => void;
}

// ============================================================================
// Validation Constants
// ============================================================================

/**
 * Validation constants from functional requirements
 */
export const VALIDATION = {
  /** Maximum question length (FR-006) */
  MAX_QUESTION_LENGTH: 500,

  /** Maximum messages in history (FR-036) */
  MAX_MESSAGES: 50,

  /** API request timeout in milliseconds (FR-017) */
  REQUEST_TIMEOUT: 10000,

  /** Cache TTL in milliseconds (FR-035) */
  CACHE_TTL: 5 * 60 * 1000,
} as const;

/**
 * Default prop values
 */
export const DEFAULTS = {
  THEME: 'light' as Theme,
  POSITION: 'bottom-right' as Position,
  WELCOME_MESSAGE: 'Ask me anything about this chapter!',
} as const;
