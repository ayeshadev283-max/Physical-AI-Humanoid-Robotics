/**
 * Analytics API service - axios client for analytics endpoints
 * TypeScript types matching backend API schemas
 */

import axios, { AxiosInstance } from 'axios';

// Types matching backend models
export interface AnalyticsSummary {
  start_date: string;
  end_date: string;
  book_id: string | null;
  total_queries: number;
  unique_users: number;
  latency_p50: number;
  latency_p95: number;
  latency_p99: number;
  feedback_rate: number;
  positive_feedback_rate: number | null;
  average_confidence: number;
  teacher_time_saved_minutes: number;
  top_topics: TopicDistribution[];
}

export interface TopicDistribution {
  topic: string;
  count: number;
  percentage?: number;
}

export interface AnalyticsSummaryParams {
  start_date: string; // ISO 8601 format
  end_date: string;   // ISO 8601 format
  book_id?: string;   // Optional book filter
}

class AnalyticsAPI {
  private client: AxiosInstance;

  constructor(baseURL?: string) {
    // In browser environments, process.env may not be available
    const apiUrl = baseURL ||
      (typeof process !== 'undefined' && process.env?.REACT_APP_API_URL) ||
      'http://localhost:8000';

    this.client = axios.create({
      baseURL: apiUrl,
      timeout: 30000, // 30 second timeout (analytics queries can be slow)
      headers: {
        'Content-Type': 'application/json',
      },
    });
  }

  /**
   * Get analytics summary for a time period
   */
  async getAnalyticsSummary(params: AnalyticsSummaryParams): Promise<AnalyticsSummary> {
    try {
      const response = await this.client.get<AnalyticsSummary>('/v1/analytics/summary', {
        params: {
          start_date: params.start_date,
          end_date: params.end_date,
          book_id: params.book_id,
        },
      });
      return response.data;
    } catch (error) {
      if (axios.isAxiosError(error) && error.response) {
        const errorMessage = error.response.data?.detail || 'Failed to fetch analytics';
        throw new Error(errorMessage);
      }
      throw new Error('Network error. Please check your connection and try again.');
    }
  }

  /**
   * Format date to ISO 8601 string for API
   */
  formatDateForAPI(date: Date): string {
    return date.toISOString();
  }

  /**
   * Get default date range (last 7 days)
   */
  getDefaultDateRange(): { start_date: string; end_date: string } {
    const end = new Date();
    const start = new Date();
    start.setDate(start.getDate() - 7);

    return {
      start_date: this.formatDateForAPI(start),
      end_date: this.formatDateForAPI(end),
    };
  }

  /**
   * Get date range for last N days
   */
  getDateRangeForLastNDays(days: number): { start_date: string; end_date: string } {
    const end = new Date();
    const start = new Date();
    start.setDate(start.getDate() - days);

    return {
      start_date: this.formatDateForAPI(start),
      end_date: this.formatDateForAPI(end),
    };
  }
}

// Export singleton instance
export const analyticsApi = new AnalyticsAPI();
