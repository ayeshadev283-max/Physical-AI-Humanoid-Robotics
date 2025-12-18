/**
 * CacheService
 * In-memory cache with TTL for query responses
 *
 * @feature 003-chatkit
 */

import { ICacheService, VALIDATION } from '../types/chatbot';

/**
 * Cache entry with timestamp
 */
interface CacheEntry<T> {
  data: T;
  timestamp: number;
}

/**
 * Generic cache service with time-to-live (TTL) support
 */
export class CacheService<T> implements ICacheService<T> {
  private cache: Map<string, CacheEntry<T>>;
  private ttl: number;

  constructor(ttl: number = VALIDATION.CACHE_TTL) {
    this.cache = new Map();
    this.ttl = ttl; // Default: 5 minutes
  }

  /**
   * Get cached value by key
   * @param key Cache key
   * @returns Cached value or null if not found/expired
   */
  get(key: string): T | null {
    const entry = this.cache.get(key);

    if (!entry) {
      return null;
    }

    // Check if entry has expired
    const now = Date.now();
    if (now - entry.timestamp > this.ttl) {
      this.cache.delete(key);
      return null;
    }

    return entry.data;
  }

  /**
   * Set cached value with TTL
   * @param key Cache key
   * @param value Value to cache
   * @param ttl Optional TTL override in milliseconds
   */
  set(key: string, value: T, ttl?: number): void {
    const entry: CacheEntry<T> = {
      data: value,
      timestamp: Date.now(),
    };

    this.cache.set(key, entry);

    // If custom TTL provided, schedule deletion
    if (ttl) {
      setTimeout(() => {
        this.cache.delete(key);
      }, ttl);
    }
  }

  /**
   * Clear all cached values
   */
  clear(): void {
    this.cache.clear();
  }

  /**
   * Get current cache size
   */
  size(): number {
    return this.cache.size;
  }

  /**
   * Remove expired entries (garbage collection)
   */
  cleanup(): number {
    const now = Date.now();
    let removed = 0;

    for (const [key, entry] of this.cache.entries()) {
      if (now - entry.timestamp > this.ttl) {
        this.cache.delete(key);
        removed++;
      }
    }

    return removed;
  }
}

// Export singleton instance for QueryResponse caching
import { QueryResponse } from '../types/chatbot';
export const queryCache = new CacheService<QueryResponse>(VALIDATION.CACHE_TTL);
