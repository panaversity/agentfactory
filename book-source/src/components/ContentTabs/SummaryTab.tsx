/**
 * SummaryTab Component - displays AI-generated summary with streaming support
 */
import React, { useState, useEffect, useRef } from 'react';
import { SummaryCacheEntry } from '../../types/contentTabs';
import * as authService from '../../services/authService';
import * as cacheService from '../../services/cacheService';
import * as summaryService from '../../services/summaryService';
import DummyLogin from './DummyLogin';
import styles from './styles.module.css';

interface SummaryTabProps {
  pageId: string;
  content: string;
}

export default function SummaryTab({ pageId, content }: SummaryTabProps): React.ReactElement {
  const [isAuthenticated, setIsAuthenticated] = useState(false);
  const [isLoading, setIsLoading] = useState(false);
  const [summary, setSummary] = useState<string>('');
  const [error, setError] = useState<string | null>(null);
  const [isGenerating, setIsGenerating] = useState(false);
  const [isCached, setIsCached] = useState(false);
  const summaryEndRef = useRef<HTMLDivElement>(null);
  const generatingRef = useRef(false);

  // Check authentication on mount
  useEffect(() => {
    const authenticated = authService.isAuthenticated();
    setIsAuthenticated(authenticated);

    if (authenticated) {
      checkCacheAndGenerate();
    }
  }, [pageId]);

  // Auto-scroll to latest content
  useEffect(() => {
    if (summary && summaryEndRef.current) {
      summaryEndRef.current.scrollIntoView({ behavior: 'smooth', block: 'end' });
    }
  }, [summary]);

  const checkCacheAndGenerate = async () => {
    // Check cache first
    const cacheKey = `summary_${pageId}`;
    const cached = cacheService.get<SummaryCacheEntry>(cacheKey);

    if (cached && cached.summary) {
      console.log(`✅ Cache hit for ${pageId} - displaying cached summary`);
      setSummary(cached.summary);
      setIsCached(true);
      return;
    }

    console.log(`❌ Cache miss for ${pageId} - generating new summary`);
    setIsCached(false);

    // Request deduplication: prevent multiple simultaneous requests
    if (generatingRef.current) {
      return;
    }

    // Generate new summary
    await generateSummary();
  };

  const generateSummary = async () => {
    const token = authService.getToken();
    if (!token) {
      setError('Authentication required');
      return;
    }

    // Request deduplication
    if (generatingRef.current) {
      return;
    }

    generatingRef.current = true;
    setIsGenerating(true);
    setIsLoading(true);
    setError(null);
    setSummary('');

    // Accumulator for final summary to cache
    let accumulatedSummary = '';

    try {
      await summaryService.fetchSummary(
        pageId,
        content,
        token,
        (chunk) => {
          // Progressive text append as chunks arrive
          accumulatedSummary += chunk;
          setSummary((prev) => prev + chunk);
        },
        () => {
          // On completion - cache the accumulated summary
          setIsLoading(false);
          setIsGenerating(false);
          generatingRef.current = false;

          // Cache the completed summary
          const cacheKey = `summary_${pageId}`;
          const cacheEntry: SummaryCacheEntry = {
            pageId,
            summary: accumulatedSummary,
            timestamp: Date.now(),
          };
          cacheService.set(cacheKey, cacheEntry);
        },
        (errorMessage) => {
          // On error
          setError(errorMessage);
          setIsLoading(false);
          setIsGenerating(false);
          generatingRef.current = false;
        }
      );
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to generate summary');
      setIsLoading(false);
      setIsGenerating(false);
      generatingRef.current = false;
    }
  };

  const handleLogin = () => {
    setIsAuthenticated(true);
    checkCacheAndGenerate();
  };

  const handleRetry = () => {
    setError(null);
    generateSummary();
  };

  // Show login if not authenticated
  if (!isAuthenticated) {
    return <DummyLogin onLogin={handleLogin} />;
  }

  // Show error with retry button
  if (error) {
    return (
      <div className={styles.error} role="alert">
        <div className={styles.errorTitle}>Error</div>
        <div className={styles.errorMessage}>{error}</div>
        <button className={styles.retryButton} onClick={handleRetry} type="button">
          Retry
        </button>
      </div>
    );
  }

  // Show loading spinner
  if (isLoading && !summary) {
    return (
      <div className={styles.loading}>
        <div className={styles.loadingSpinner} />
        <span>Generating summary...</span>
      </div>
    );
  }

  // Show summary content
  return (
    <div role="tabpanel" id="summary-panel" aria-labelledby="summary-tab">
      {summary && (
        <div className={styles.summaryContent}>
          {isCached && (
            <div className={styles.cacheIndicator} title="This summary was loaded from cache">
              💾 Cached
            </div>
          )}
          <p style={{ whiteSpace: 'pre-wrap' }}>{summary}</p>
          <div ref={summaryEndRef} />
        </div>
      )}
      {isGenerating && (
        <div className={styles.loading}>
          <div className={styles.loadingSpinner} />
          <span>Streaming...</span>
        </div>
      )}
    </div>
  );
}
