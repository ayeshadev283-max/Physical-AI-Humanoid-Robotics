/**
 * useUserId Hook
 * Generates and persists a unique user ID using crypto.randomUUID and localStorage
 *
 * @feature 003-chatkit
 */

import { useState, useEffect } from 'react';
import { UseUserIdReturn } from '../types/chatbot';

const STORAGE_KEY = 'chatbot_user_id';

/**
 * Hook for managing persistent user ID
 * Uses localStorage with fallback to sessionStorage (Safari private mode)
 *
 * @returns {UseUserIdReturn} Object containing userId and loaded status
 */
export function useUserId(): UseUserIdReturn {
  const [userId, setUserId] = useState<string>('');
  const [loaded, setLoaded] = useState<boolean>(false);

  useEffect(() => {
    try {
      // Try localStorage first
      let id = localStorage.getItem(STORAGE_KEY);

      if (!id) {
        // Generate new UUID using Web Crypto API
        id = crypto.randomUUID();
        localStorage.setItem(STORAGE_KEY, id);
      }

      setUserId(id);
      setLoaded(true);
    } catch (error) {
      // Fallback to sessionStorage (Safari private mode, quota exceeded, etc.)
      console.warn('localStorage unavailable, using sessionStorage:', error);

      try {
        let id = sessionStorage.getItem(STORAGE_KEY);

        if (!id) {
          id = crypto.randomUUID();
          sessionStorage.setItem(STORAGE_KEY, id);
        }

        setUserId(id);
        setLoaded(true);
      } catch (sessionError) {
        // Last resort: generate UUID but don't persist
        console.error('All storage options failed:', sessionError);
        setUserId(crypto.randomUUID());
        setLoaded(true);
      }
    }
  }, []);

  return { userId, loaded };
}
