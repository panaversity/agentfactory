/**
 * Docusaurus Root Component
 * 
 * This component wraps the entire site with the AnalyticsTracker,
 * enabling automatic tracking of user interactions (page views, scroll depth, etc.)
 * 
 * GA4 is configured via the GA4_MEASUREMENT_ID environment variable.
 * If not set, analytics will not load.
 */

import React, { useState, useCallback } from 'react';
import { AnalyticsTracker } from '@/components/AnalyticsTracker';
import HighlightDetector from '@site/src/components/HighlightDetector';
import AIDialog from '@site/src/components/AIDialog';

export default function Root({ children }: { children: React.ReactNode }) {
  const [isAIDialogOpen, setIsAIDialogOpen] = useState(false);
  const [aiDialogContent, setAiDialogContent] = useState(null);
  const [aiDialogLoading, setAiDialogLoading] = useState(false);
  const [aiDialogError, setAiDialogError] = useState<string | null>(null);

  const handleHighlight = useCallback((selection: string) => {
    // You can use the selection text here if needed, e.g., to show a temporary indicator
    setIsAIDialogOpen(true);
  }, []);

  const handleAIDialogClose = useCallback(() => {
    setIsAIDialogOpen(false);
    setAiDialogContent(null);
    setAiDialogError(null);
  }, []);

  const handleAIDialogContent = useCallback((content: any | null) => {
    setAiDialogContent(content);
  }, []);

  const handleAIDialogLoading = useCallback((loading: boolean) => {
    setAiDialogLoading(loading);
  }, []);

  const handleAIDialogError = useCallback((error: string | null) => {
    setAiDialogError(error);
  }, []);

  return (
    <AnalyticsTracker>
      <HighlightDetector
        onHighlight={handleHighlight}
        onLoading={handleAIDialogLoading}
        onError={handleAIDialogError}
        onContent={handleAIDialogContent}
      />
      <AIDialog
        isOpen={isAIDialogOpen}
        onClose={handleAIDialogClose}
        content={aiDialogContent}
        isLoading={aiDialogLoading}
        error={aiDialogError}
      />
      {children}
    </AnalyticsTracker>
  );
}
