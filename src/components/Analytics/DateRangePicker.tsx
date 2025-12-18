/**
 * DateRangePicker - Allow users to select custom date ranges
 *
 * Features:
 * - Quick presets (Last 7 days, Last 30 days, etc.)
 * - Custom date range selection
 * - Validation (end date > start date)
 */

import React, { useState } from 'react';
import styles from './DateRangePicker.module.css';

interface DateRangePickerProps {
  startDate: string;
  endDate: string;
  onDateRangeChange: (range: { start_date: string; end_date: string }) => void;
}

type Preset = {
  label: string;
  days: number;
};

const PRESETS: Preset[] = [
  { label: 'Last 7 days', days: 7 },
  { label: 'Last 30 days', days: 30 },
  { label: 'Last 90 days', days: 90 },
];

export function DateRangePicker({
  startDate,
  endDate,
  onDateRangeChange,
}: DateRangePickerProps): JSX.Element {
  const [customStart, setCustomStart] = useState(toDateInputFormat(startDate));
  const [customEnd, setCustomEnd] = useState(toDateInputFormat(endDate));
  const [error, setError] = useState<string | null>(null);

  function toDateInputFormat(isoString: string): string {
    // Convert ISO 8601 to YYYY-MM-DD for input[type="date"]
    return isoString.split('T')[0];
  }

  function toISOString(dateString: string): string {
    // Convert YYYY-MM-DD to ISO 8601 timestamp (start of day UTC)
    const date = new Date(dateString + 'T00:00:00Z');
    return date.toISOString();
  }

  const handlePresetClick = (preset: Preset) => {
    const end = new Date();
    const start = new Date();
    start.setDate(start.getDate() - preset.days);

    const range = {
      start_date: start.toISOString(),
      end_date: end.toISOString(),
    };

    setCustomStart(toDateInputFormat(range.start_date));
    setCustomEnd(toDateInputFormat(range.end_date));
    setError(null);
    onDateRangeChange(range);
  };

  const handleCustomDateChange = () => {
    // Validate
    const start = new Date(customStart);
    const end = new Date(customEnd);

    if (isNaN(start.getTime()) || isNaN(end.getTime())) {
      setError('Invalid date format');
      return;
    }

    if (end <= start) {
      setError('End date must be after start date');
      return;
    }

    const now = new Date();
    if (start > now || end > now) {
      setError('Dates cannot be in the future');
      return;
    }

    setError(null);
    onDateRangeChange({
      start_date: toISOString(customStart),
      end_date: toISOString(customEnd),
    });
  };

  return (
    <div className={styles.container}>
      <div className={styles.presets}>
        <label className={styles.label}>Quick Select:</label>
        <div className={styles.presetsButtons}>
          {PRESETS.map((preset) => (
            <button
              key={preset.days}
              type="button"
              className={styles.presetButton}
              onClick={() => handlePresetClick(preset)}
            >
              {preset.label}
            </button>
          ))}
        </div>
      </div>

      <div className={styles.custom}>
        <label className={styles.label}>Custom Range:</label>
        <div className={styles.customInputs}>
          <div className={styles.inputGroup}>
            <label htmlFor="start-date" className={styles.inputLabel}>
              Start Date
            </label>
            <input
              id="start-date"
              type="date"
              value={customStart}
              onChange={(e) => setCustomStart(e.target.value)}
              className={styles.dateInput}
              max={toDateInputFormat(new Date().toISOString())}
            />
          </div>

          <div className={styles.inputGroup}>
            <label htmlFor="end-date" className={styles.inputLabel}>
              End Date
            </label>
            <input
              id="end-date"
              type="date"
              value={customEnd}
              onChange={(e) => setCustomEnd(e.target.value)}
              className={styles.dateInput}
              max={toDateInputFormat(new Date().toISOString())}
            />
          </div>

          <button
            type="button"
            className={styles.applyButton}
            onClick={handleCustomDateChange}
          >
            Apply
          </button>
        </div>

        {error && <p className={styles.error}>{error}</p>}
      </div>
    </div>
  );
}
