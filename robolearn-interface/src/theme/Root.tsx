/**
 * Docusaurus Root Component
 *
 * This component wraps the entire site with:
 * 1. PyodideProvider - Enables direct Pyodide integration for interactive Python execution
 * 2. AnalyticsTracker - Tracks user interactions (page views, scroll depth, etc.)
 *
 * GA4 is configured via the GA4_MEASUREMENT_ID environment variable.
 * If not set, analytics will not load.
 */

import React from 'react';
import { PyodideProvider } from '@/contexts/PyodideContext';
import { AnalyticsTracker } from '@/components/AnalyticsTracker';

export default function Root({ children }: { children: React.ReactNode }) {
  return (
    <PyodideProvider>
      <AnalyticsTracker>
        {children}
      </AnalyticsTracker>
    </PyodideProvider>
  );
}
