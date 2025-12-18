/**
 * AnalyticsDashboard - Main analytics dashboard component
 *
 * Features:
 * - Date range selection
 * - Key metrics cards
 * - Latency statistics
 * - Top topics table
 * - Loading and error states
 */

import React from 'react';
import { useAnalytics } from '../../hooks/useAnalytics';
import { MetricsCard } from './MetricsCard';
import { TopicsTable } from './TopicsTable';
import { DateRangePicker } from './DateRangePicker';
import styles from './AnalyticsDashboard.module.css';

interface AnalyticsDashboardProps {
  bookId?: string;
}

export function AnalyticsDashboard({ bookId }: AnalyticsDashboardProps): JSX.Element {
  const { summary, loading, error, dateRange, setDateRange, refetch } = useAnalytics({
    bookId,
    autoFetch: true,
  });

  const handleDateRangeChange = (range: { start_date: string; end_date: string }) => {
    setDateRange(range);
  };

  if (error) {
    return (
      <div className={styles.container}>
        <div className={styles.error}>
          <h2>Error Loading Analytics</h2>
          <p>{error}</p>
          <button onClick={refetch} className={styles.retryButton}>
            Retry
          </button>
        </div>
      </div>
    );
  }

  return (
    <div className={styles.container}>
      <header className={styles.header}>
        <h1>Chatbot Usage Analytics</h1>
        <p className={styles.subtitle}>
          Monitor chatbot performance, user engagement, and teacher time savings
        </p>
      </header>

      <DateRangePicker
        startDate={dateRange.start_date}
        endDate={dateRange.end_date}
        onDateRangeChange={handleDateRangeChange}
      />

      {loading ? (
        <div className={styles.loading}>
          <div className={styles.spinner} />
          <p>Loading analytics...</p>
        </div>
      ) : summary ? (
        <>
          {/* Key Metrics Grid */}
          <section className={styles.section}>
            <h2 className={styles.sectionTitle}>Key Metrics</h2>
            <div className={styles.metricsGrid}>
              <MetricsCard
                title="Total Queries"
                value={summary.total_queries}
                format="number"
                icon="💬"
                description="Total number of student questions"
              />
              <MetricsCard
                title="Unique Users"
                value={summary.unique_users}
                format="number"
                icon="👥"
                description="Students who used the chatbot"
              />
              <MetricsCard
                title="Teacher Time Saved"
                value={summary.teacher_time_saved_minutes}
                format="time"
                icon="⏱️"
                description="Estimated time saved (2.5 min per query)"
              />
              <MetricsCard
                title="Feedback Rate"
                value={summary.feedback_rate}
                format="percentage"
                icon="📊"
                description="Percentage of queries with feedback"
              />
            </div>
          </section>

          {/* Quality Metrics */}
          <section className={styles.section}>
            <h2 className={styles.sectionTitle}>Quality Metrics</h2>
            <div className={styles.metricsGrid}>
              <MetricsCard
                title="Average Confidence"
                value={(summary.average_confidence * 100).toFixed(1)}
                format="percentage"
                icon="🎯"
                description="Average response confidence score"
              />
              {summary.positive_feedback_rate !== null && (
                <MetricsCard
                  title="Positive Feedback"
                  value={summary.positive_feedback_rate}
                  format="percentage"
                  icon="👍"
                  description="Percentage of helpful ratings"
                />
              )}
            </div>
          </section>

          {/* Latency Metrics */}
          <section className={styles.section}>
            <h2 className={styles.sectionTitle}>Response Latency</h2>
            <div className={styles.metricsGrid}>
              <MetricsCard
                title="Median (p50)"
                value={summary.latency_p50}
                format="latency"
                icon="📈"
                description="50% of responses faster than this"
              />
              <MetricsCard
                title="95th Percentile"
                value={summary.latency_p95}
                format="latency"
                icon="📈"
                description="95% of responses faster than this"
              />
              <MetricsCard
                title="99th Percentile"
                value={summary.latency_p99}
                format="latency"
                icon="📈"
                description="99% of responses faster than this"
              />
            </div>
          </section>

          {/* Top Topics */}
          <section className={styles.section}>
            <h2 className={styles.sectionTitle}>Top Question Topics</h2>
            <TopicsTable
              topics={summary.top_topics}
              totalQueries={summary.total_queries}
            />
          </section>

          {/* Footer with metadata */}
          <footer className={styles.footer}>
            <p className={styles.footerText}>
              Data period: {new Date(summary.start_date).toLocaleDateString()} -{' '}
              {new Date(summary.end_date).toLocaleDateString()}
            </p>
            {summary.book_id && (
              <p className={styles.footerText}>Book: {summary.book_id}</p>
            )}
          </footer>
        </>
      ) : (
        <div className={styles.emptyState}>
          <p>No data available for the selected period.</p>
        </div>
      )}
    </div>
  );
}
