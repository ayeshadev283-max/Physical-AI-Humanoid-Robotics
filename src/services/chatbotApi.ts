/**
 * ChatbotApiService
 * HTTP client for RAG chatbot backend API
 *
 * @feature 003-chatkit
 */

import axios, { AxiosInstance, AxiosError } from 'axios';
import {
  QueryRequest,
  QueryResponse,
  FeedbackRequest,
  FeedbackResponse,
  IChatbotApiService,
  ChatbotError,
  ErrorCode,
  VALIDATION,
} from '../types/chatbot';

/**
 * Service for communicating with the RAG chatbot backend
 */
export class ChatbotApiService implements IChatbotApiService {
  private client: AxiosInstance;

  constructor(baseURL?: string) {
    // In browser environments, process.env may not be available
    // Use default localhost for development
    const apiUrl =
      baseURL ||
      (typeof process !== 'undefined' && process.env?.REACT_APP_API_URL) ||
      'http://localhost:8000';

    this.client = axios.create({
      baseURL: apiUrl,
      timeout: VALIDATION.REQUEST_TIMEOUT, // 10 seconds
      headers: {
        'Content-Type': 'application/json',
      },
    });
  }

  /**
   * Submit a query to the backend
   * @throws {ChatbotError} If request fails
   */
  async submitQuery(request: QueryRequest): Promise<QueryResponse> {
    try {
      const response = await this.client.post<QueryResponse>(
        '/v1/query',
        request
      );

      // Validate response structure
      if (!response.data || typeof response.data.answer !== 'string') {
        throw new ChatbotError(
          'Invalid response format from backend',
          ErrorCode.INVALID_RESPONSE,
          false
        );
      }

      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Submit feedback for a response
   * @throws {ChatbotError} If request fails
   */
  async submitFeedback(request: FeedbackRequest): Promise<FeedbackResponse> {
    try {
      const response = await this.client.post<FeedbackResponse>(
        '/v1/feedback',
        request
      );

      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  /**
   * Convert axios errors to ChatbotError
   */
  private handleError(error: unknown): ChatbotError {
    if (axios.isAxiosError(error)) {
      const axiosError = error as AxiosError;

      // Network error (no response received)
      if (!axiosError.response) {
        if (axiosError.code === 'ECONNABORTED') {
          return new ChatbotError(
            'Request timed out. Please try again.',
            ErrorCode.TIMEOUT,
            true
          );
        }

        return new ChatbotError(
          'Unable to connect to the server. Please check your internet connection and try again.',
          ErrorCode.NETWORK_ERROR,
          true
        );
      }

      // Server responded with error
      const status = axiosError.response.status;

      if (status >= 500) {
        return new ChatbotError(
          'Server error. Please try again later.',
          ErrorCode.SERVER_ERROR,
          true
        );
      }

      if (status === 429) {
        return new ChatbotError(
          'Too many requests. Please wait a moment and try again.',
          ErrorCode.CLIENT_ERROR,
          true
        );
      }

      if (status >= 400) {
        // Extract error message from response if available
        const errorMessage =
          (axiosError.response.data as any)?.detail ||
          'Invalid request. Please try again.';

        return new ChatbotError(
          errorMessage,
          ErrorCode.CLIENT_ERROR,
          false
        );
      }
    }

    // Unknown error
    return new ChatbotError(
      'An unexpected error occurred. Please try again.',
      ErrorCode.NETWORK_ERROR,
      true
    );
  }
}

// Export singleton instance for convenience
export const chatbotApi = new ChatbotApiService();
