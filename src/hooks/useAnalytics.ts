/**
 * useAnalytics - Custom React hook for analytics data management
 *
 * Features:
 * - Fetch analytics summary from backend
 * - Date range selection and validation
 * - Loading and error state management
 * - Client-side caching to reduce API calls
 */

import { useState, useEffect, useCallback } from 'react';
import { analyticsApi, AnalyticsSummary, AnalyticsSummaryParams } from '../services/analyticsApi';

interface UseAnalyticsOptions {
  initialDateRange?: { start_date: string; end_date: string };
  bookId?: string;
  autoFetch?: boolean;
}

interface UseAnalyticsReturn {
  summary: AnalyticsSummary | null;
  loading: boolean;
  error: string | null;
  dateRange: { start_date: string; end_date: string };
  setDateRange: (range: { start_date: string; end_date: string }) => void;
  refetch: () => Promise<void>;
  setBookId: (bookId: string | undefined) => void;
}

// Simple in-memory cache
const cache = new Map<string, { data: AnalyticsSummary; timestamp: number }>();
const CACHE_TTL_MS = 5 * 60 * 1000; // 5 minutes (matches backend cache)

function getCacheKey(params: AnalyticsSummaryParams): string {
  return `${params.start_date}|${params.end_date}|${params.book_id || 'all'}`;
}

function getCachedData(params: AnalyticsSummaryParams): AnalyticsSummary | null {
  const key = getCacheKey(params);
  const cached = cache.get(key);

  if (!cached) {
    return null;
  }

  const age = Date.now() - cached.timestamp;
  if (age > CACHE_TTL_MS) {
    cache.delete(key);
    return null;
  }

  return cached.data;
}

function setCachedData(params: AnalyticsSummaryParams, data: AnalyticsSummary): void {
  const key = getCacheKey(params);
  cache.set(key, { data, timestamp: Date.now() });

  // Simple LRU: keep only last 50 entries
  if (cache.size > 50) {
    const firstKey = cache.keys().next().value;
    cache.delete(firstKey);
  }
}

export function useAnalytics(options: UseAnalyticsOptions = {}): UseAnalyticsReturn {
  const {
    initialDateRange = analyticsApi.getDefaultDateRange(),
    bookId: initialBookId,
    autoFetch = true,
  } = options;

  const [summary, setSummary] = useState<AnalyticsSummary | null>(null);
  const [loading, setLoading] = useState<boolean>(false);
  const [error, setError] = useState<string | null>(null);
  const [dateRange, setDateRange] = useState(initialDateRange);
  const [bookId, setBookId] = useState<string | undefined>(initialBookId);

  const fetchAnalytics = useCallback(async () => {
    const params: AnalyticsSummaryParams = {
      start_date: dateRange.start_date,
      end_date: dateRange.end_date,
      book_id: bookId,
    };

    // Check cache first
    const cachedData = getCachedData(params);
    if (cachedData) {
      setSummary(cachedData);
      setError(null);
      return;
    }

    setLoading(true);
    setError(null);

    try {
      const data = await analyticsApi.getAnalyticsSummary(params);
      setSummary(data);
      setCachedData(params, data);
    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : 'Failed to fetch analytics';
      setError(errorMessage);
      setSummary(null);
    } finally {
      setLoading(false);
    }
  }, [dateRange, bookId]);

  // Auto-fetch on mount and when dependencies change
  useEffect(() => {
    if (autoFetch) {
      fetchAnalytics();
    }
  }, [fetchAnalytics, autoFetch]);

  return {
    summary,
    loading,
    error,
    dateRange,
    setDateRange,
    refetch: fetchAnalytics,
    setBookId,
  };
}
