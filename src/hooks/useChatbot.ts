import { useState } from 'react';

interface QueryResponse {
  answer: string;
  sources: SourceReference[];
  confidence: number;
}

interface SourceReference {
  chapter: string;
  section: string;
  page_url: string;
}

interface UseChatbotReturn {
  submitQuery: (query: string, selectedText?: string) => Promise<QueryResponse>;
  isLoading: boolean;
  error: string | null;
}

const API_BASE_URL = process.env.NODE_ENV === 'production'
  ? 'https://your-backend-url.com'
  : 'http://localhost:8000';

export function useChatbot(): UseChatbotReturn {
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const submitQuery = async (
    query: string,
    selectedText?: string
  ): Promise<QueryResponse> => {
    setIsLoading(true);
    setError(null);

    try {
      const response = await fetch(`${API_BASE_URL}/v1/query`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query,
          book_context: {
            book_id: 'physical-ai-robotics',
            chapter_number: null,
            page_url: null,
          },
          selected_text: selectedText || null,
        }),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();

      return {
        answer: data.response_text,
        sources: data.source_references.map((source: any) => ({
          chapter: source.chapter || '',
          section: source.section || '',
          page_url: source.citation || '#',
        })),
        confidence: data.confidence_score || 0.8,
      };
    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : 'Unknown error occurred';
      setError(errorMessage);
      throw err;
    } finally {
      setIsLoading(false);
    }
  };

  return {
    submitQuery,
    isLoading,
    error,
  };
}
