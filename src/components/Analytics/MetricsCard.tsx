/**
 * MetricsCard - Display a single analytics metric
 *
 * Props:
 * - title: Metric display name
 * - value: Metric value (number or string)
 * - format: How to display the value ('number', 'time', 'percentage')
 * - icon: Optional icon/emoji to display
 * - description: Optional help text
 */

import React from 'react';
import styles from './MetricsCard.module.css';

interface MetricsCardProps {
  title: string;
  value: number | string;
  format?: 'number' | 'time' | 'percentage' | 'latency';
  icon?: string;
  description?: string;
  trend?: {
    value: number;
    isPositive: boolean;
  };
}

function formatValue(value: number | string, format?: string): string {
  if (typeof value === 'string') {
    return value;
  }

  switch (format) {
    case 'number':
      return value.toLocaleString();
    case 'time':
      // Convert minutes to hours and minutes
      const hours = Math.floor(value / 60);
      const minutes = value % 60;
      if (hours > 0) {
        return `${hours}h ${minutes}m`;
      }
      return `${minutes} min`;
    case 'percentage':
      return `${value.toFixed(1)}%`;
    case 'latency':
      // Latency in milliseconds
      if (value >= 1000) {
        return `${(value / 1000).toFixed(2)}s`;
      }
      return `${value}ms`;
    default:
      return String(value);
  }
}

export function MetricsCard({
  title,
  value,
  format = 'number',
  icon,
  description,
  trend,
}: MetricsCardProps): JSX.Element {
  const formattedValue = formatValue(value, format);

  return (
    <div className={styles.card}>
      <div className={styles.header}>
        {icon && <span className={styles.icon}>{icon}</span>}
        <h3 className={styles.title}>{title}</h3>
      </div>

      <div className={styles.body}>
        <div className={styles.value}>{formattedValue}</div>

        {trend && (
          <div className={`${styles.trend} ${trend.isPositive ? styles.trendPositive : styles.trendNegative}`}>
            <span className={styles.trendIcon}>{trend.isPositive ? '↑' : '↓'}</span>
            <span className={styles.trendValue}>{Math.abs(trend.value)}%</span>
          </div>
        )}
      </div>

      {description && <p className={styles.description}>{description}</p>}
    </div>
  );
}
