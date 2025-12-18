/**
 * TopicsTable - Display top question topics with counts
 *
 * Features:
 * - Sortable by topic name or count
 * - Shows percentage of total queries
 * - Responsive design
 */

import React, { useState, useMemo } from 'react';
import styles from './TopicsTable.module.css';
import { TopicDistribution } from '../../services/analyticsApi';

interface TopicsTableProps {
  topics: TopicDistribution[];
  totalQueries: number;
}

type SortField = 'topic' | 'count';
type SortDirection = 'asc' | 'desc';

export function TopicsTable({ topics, totalQueries }: TopicsTableProps): JSX.Element {
  const [sortField, setSortField] = useState<SortField>('count');
  const [sortDirection, setSortDirection] = useState<SortDirection>('desc');

  const handleSort = (field: SortField) => {
    if (field === sortField) {
      // Toggle direction
      setSortDirection(sortDirection === 'asc' ? 'desc' : 'asc');
    } else {
      // New field, default to descending for count, ascending for topic
      setSortField(field);
      setSortDirection(field === 'count' ? 'desc' : 'asc');
    }
  };

  const sortedTopics = useMemo(() => {
    const sorted = [...topics].sort((a, b) => {
      let comparison = 0;

      if (sortField === 'topic') {
        comparison = a.topic.localeCompare(b.topic);
      } else {
        comparison = a.count - b.count;
      }

      return sortDirection === 'asc' ? comparison : -comparison;
    });

    return sorted;
  }, [topics, sortField, sortDirection]);

  const getSortIcon = (field: SortField): string => {
    if (field !== sortField) {
      return '⇅';
    }
    return sortDirection === 'asc' ? '↑' : '↓';
  };

  if (topics.length === 0) {
    return (
      <div className={styles.emptyState}>
        <p>No topic data available for this period.</p>
      </div>
    );
  }

  return (
    <div className={styles.container}>
      <table className={styles.table}>
        <thead>
          <tr>
            <th className={styles.rankColumn}>#</th>
            <th
              className={`${styles.topicColumn} ${styles.sortable}`}
              onClick={() => handleSort('topic')}
            >
              Topic {getSortIcon('topic')}
            </th>
            <th
              className={`${styles.countColumn} ${styles.sortable}`}
              onClick={() => handleSort('count')}
            >
              Count {getSortIcon('count')}
            </th>
            <th className={styles.percentColumn}>%</th>
          </tr>
        </thead>
        <tbody>
          {sortedTopics.map((topic, index) => {
            const percentage = totalQueries > 0
              ? ((topic.count / totalQueries) * 100).toFixed(1)
              : '0.0';

            return (
              <tr key={`${topic.topic}-${index}`}>
                <td className={styles.rankColumn}>{index + 1}</td>
                <td className={styles.topicColumn}>
                  <span className={styles.topicText}>{topic.topic}</span>
                </td>
                <td className={styles.countColumn}>
                  <span className={styles.countBadge}>{topic.count}</span>
                </td>
                <td className={styles.percentColumn}>
                  <span className={styles.percentText}>{percentage}%</span>
                </td>
              </tr>
            );
          })}
        </tbody>
      </table>
    </div>
  );
}
