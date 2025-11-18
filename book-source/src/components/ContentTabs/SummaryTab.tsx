/**
 * SummaryTab Component - displays AI-generated summary with streaming support
 */
import React, { useState, useEffect, useRef } from "react";
import { useHistory, useLocation } from "@docusaurus/router";
import { SummaryCacheEntry } from "../../types/contentTabs";
import * as authService from "../../services/authService";
import * as cacheService from "../../services/cacheService";
import * as summaryService from "../../services/summaryService";
import styles from "./styles.module.css";
import ReactMarkdown from "react-markdown";

interface SummaryTabProps {
  pageId: string;
  content: string;
}

export default function SummaryTab({
  pageId,
  content,
}: SummaryTabProps): React.ReactElement {
  const history = useHistory();
  const location = useLocation();
  
  // T031: Check authentication on mount
  const [isAuthenticated, setIsAuthenticated] = useState(false);
  const [isLoading, setIsLoading] = useState(false);
  const [summary, setSummary] = useState<string>("");
  const [error, setError] = useState<string | null>(null);
  const [isGenerating, setIsGenerating] = useState(false);
  const [isCached, setIsCached] = useState(false);
  const [streamingText, setStreamingText] = useState<string>("");
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
      summaryContainerRef.current.scrollTop =
        summaryContainerRef.current.scrollHeight;
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
      setError("Authentication required");
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
    setSummary("");
    setStreamingText("");

    // Accumulator for final summary to cache
    let accumulatedSummary = "";
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

          // Progressive text append and normalize whitespace
          accumulatedSummary += chunk;
          const normalized = accumulatedSummary
            .replace(/\n{2,}/g, '\n\n')      // Collapse any multiple newlines to exactly 2
            .replace(/^\s+/g, '')             // Remove leading whitespace
            .replace(/\s+$/g, '')             // Remove trailing whitespace
            .trim();
          
          setStreamingText(normalized);
        },
        () => {
          // On completion - normalize and cache the final summary
          const finalSummary = accumulatedSummary
            .replace(/\n{2,}/g, '\n\n')
            .trim();
          
          setIsLoading(false);
          setIsGenerating(false);
          setSummary(finalSummary);
          setStreamingText("");
          generatingRef.current = false;

          // Cache the completed summary
          const cacheKey = `summary_${pageId}`;
          const cacheEntry: SummaryCacheEntry = {
            pageId,
            summary: finalSummary,
            timestamp: Date.now(),
          };
          cacheService.set(cacheKey, cacheEntry);
        },
        (errorMessage) => {
          // On error
          setError(errorMessage);
          setIsLoading(false);
          setIsGenerating(false);
          setStreamingText("");
          generatingRef.current = false;
        }
      );
    } catch (err) {
      setError(
        err instanceof Error ? err.message : "Failed to generate summary"
      );
      setIsLoading(false);
      setIsGenerating(false);
      setStreamingText("");
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

  // T032: Show login button if not authenticated
  if (!isAuthenticated) {
    return (
      <div className={styles.loginButton}>
        <h3>Authentication Required</h3>
        <p>You need to be authenticated to view AI-generated summaries.</p>
        <button
          onClick={() => {
            // T035: Add navigation state with return URL
            const returnTo = `${location.pathname}${location.hash || '#summary'}`;
            history.push(`/login?returnTo=${encodeURIComponent(returnTo)}`);
          }}
          type="button"
        >
          Login to See Summary
        </button>
        <p style={{ fontSize: '0.85em', color: 'var(--ifm-color-emphasis-600)', marginTop: '1rem' }}>
          Note: This is a temporary dummy authentication for demonstration purposes.
        </p>
      </div>
    );
  }

  // Show error with retry button
  if (error) {
    return (
      <div className={styles.error} role="alert">
        <div className={styles.errorTitle}>Error</div>
        <div className={styles.errorMessage}>{error}</div>
        <button
          className={styles.retryButton}
          onClick={handleRetry}
          type="button"
        >
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
            <div className={styles.loadingSubtitle}>
              This may take a few moments...
            </div>
          </div>
        </div>
      </div>
    );
  }

  // Show summary content - clean format without header, badges, or regenerate button
  return (
    <div
      role="tabpanel"
      id="summary-panel"
      aria-labelledby="summary-tab"
    >
      {(summary || streamingText) && (
        <>
          <div className="markdown" ref={summaryContainerRef}>
            <ReactMarkdown>
              {isGenerating ? streamingText : summary}
            </ReactMarkdown>
            {isGenerating && <span className={styles.cursor}>|</span>}
          </div>
          <div ref={summaryEndRef} />
        </>
      )}
    </div>
  );
}
