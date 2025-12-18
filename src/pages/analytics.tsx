/**
 * Analytics Dashboard Page
 *
 * Docusaurus page for teacher/admin analytics dashboard.
 * Accessible at /analytics
 */

import React from 'react';
import Layout from '@theme/Layout';
import { AnalyticsDashboard } from '../components/Analytics';

export default function AnalyticsPage(): JSX.Element {
  return (
    <Layout
      title="Analytics Dashboard"
      description="Chatbot usage analytics and teacher workload insights">
      <main>
        <AnalyticsDashboard bookId="physical-ai-robotics" />
      </main>
    </Layout>
  );
}
