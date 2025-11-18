/**
 * Docusaurus Root Component
 *
 * This component wraps the entire site with the AnalyticsTracker,
 * enabling automatic tracking of user interactions (page views, scroll depth, etc.)
 *
 * GA4 is configured via the GA4_MEASUREMENT_ID environment variable.
 * If not set, analytics will not load.
 */

import React from 'react';
import { AnalyticsTracker } from '@/components/AnalyticsTracker';
import { BookmarkProvider } from '@/contexts/BookmarkContext';
import { NotesProvider } from '@/contexts/NotesContext';
import { SearchProvider } from '@/contexts/SearchContext';
import { SidebarProvider } from '@/contexts/SidebarContext';

export default function Root({ children }: { children: React.ReactNode }) {
  return (
    <SidebarProvider>
      <SearchProvider>
        <BookmarkProvider>
          <NotesProvider>
            <AnalyticsTracker>
              {children}
            </AnalyticsTracker>
          </NotesProvider>
        </BookmarkProvider>
      </SearchProvider>
    </SidebarProvider>
  );
}
