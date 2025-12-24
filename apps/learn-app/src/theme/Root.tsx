/**
 * Docusaurus Root Component
 *
 * This component wraps the entire site with:
 * 1. AuthProvider - Enables authentication state across the site
 * 2. PyodideProvider - Enables direct Pyodide integration for interactive Python execution
 * 3. AnalyticsTracker - Tracks user interactions (page views, scroll depth, etc.)
 *
 * GA4 is configured via the GA4_MEASUREMENT_ID environment variable.
 * If not set, analytics will not load.
 */

import React from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { AuthProvider } from '@/contexts/AuthContext';
import { PyodideProvider } from '@/contexts/PyodideContext';
import { AnalyticsTracker } from '@/components/AnalyticsTracker';

export default function Root({ children }: { children: React.ReactNode }) {
  const { siteConfig } = useDocusaurusContext();
  const authUrl = (siteConfig.customFields?.authUrl as string) || 'http://localhost:3001';
  const oauthClientId = (siteConfig.customFields?.oauthClientId as string) || 'agent-factory-public-client';

  return (
    <AuthProvider authUrl={authUrl} oauthClientId={oauthClientId}>
      <PyodideProvider>
        <AnalyticsTracker>
          {children}
        </AnalyticsTracker>
      </PyodideProvider>
    </AuthProvider>
  );
}
