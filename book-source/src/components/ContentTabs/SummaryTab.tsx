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
import ReactMarkdown from "react-markdown";

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
  const [streamingText, setStreamingText] = useState<string>('');
  const summaryEndRef = useRef<HTMLDivElement>(null);
  const generatingRef = useRef(false);
  const summaryContainerRef = useRef<HTMLDivElement>(null);

  // Check authentication on mount
  useEffect(() => {
    const authenticated = authService.isAuthenticated();
    setIsAuthenticated(authenticated);

    if (authenticated) {
      checkCacheAndGenerate();
    }
  }, [pageId]);

  // Auto-scroll to latest content during streaming
  useEffect(() => {
    if (isGenerating && summaryContainerRef.current) {
      // Smooth scroll to bottom during streaming
      summaryContainerRef.current.scrollTop = summaryContainerRef.current.scrollHeight;
    }
  }, [streamingText, isGenerating]);

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
    setIsLoading(true); // Show loading screen initially
    setError(null);
    setSummary('');
    setStreamingText('');

    // Accumulator for final summary to cache
    let accumulatedSummary = '';
    let firstChunkReceived = false;

    try {
      await summaryService.fetchSummary(
        pageId,
        content,
        token,
        (chunk) => {
          // Hide loading screen and show streaming as soon as first chunk arrives
          if (!firstChunkReceived) {
            firstChunkReceived = true;
            setIsLoading(false);
          }
          
          // Progressive text append as chunks arrive
          accumulatedSummary += chunk;
          setStreamingText(accumulatedSummary);
        },
        () => {
          // On completion - cache the accumulated summary
          setIsLoading(false);
          setIsGenerating(false);
          setSummary(accumulatedSummary);
          setStreamingText('');
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
          setStreamingText('');
          generatingRef.current = false;
        }
      );
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to generate summary');
      setIsLoading(false);
      setIsGenerating(false);
      setStreamingText('');
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
      <div className={styles.loadingContainer}>
        <div className={styles.loadingContent}>
          <div className={styles.loadingSpinner} />
          <div className={styles.loadingText}>
            <div className={styles.loadingTitle}>Generating AI Summary</div>
            <div className={styles.loadingSubtitle}>This may take a few moments...</div>
          </div>
        </div>
      </div>
    );
  }

  // Show summary content
  return (
    <div role="tabpanel" id="summary-panel" aria-labelledby="summary-tab" className={styles.summaryPanel}>
      {(summary || streamingText) && (
        <>
          <div className={styles.summaryHeader}>
            <h3 className={styles.summaryTitle}>
              <span className={styles.summaryIcon}>✨</span>
              AI-Generated Summary
            </h3>
            {isCached && !isGenerating && (
              <div className={styles.cacheIndicator} title="Loaded from cache - no API call made">
                <span className={styles.cacheIcon}>💾</span> Cached
              </div>
            )}
            {isGenerating && (
              <div className={styles.streamingBadge}>
                <span className={styles.streamingDot}></span>
                Streaming...
              </div>
            )}
          </div>
          <div className={styles.summaryContent} ref={summaryContainerRef}>
            <div className={styles.summaryText}>
              <ReactMarkdown>{isGenerating ? streamingText : summary}</ReactMarkdown>
              {isGenerating && <span className={styles.cursor}>|</span>}
            </div>
            <div ref={summaryEndRef} />
          </div>
        </>
      )}
    </div>
  );
}
