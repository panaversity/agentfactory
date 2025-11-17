import React, { useEffect, useState } from 'react';
import styles from './styles.module.css';

interface CollapsibleSummaryProps {
  pagePath: string;
  pageContent: string;
}

const CollapsibleSummary: React.FC<CollapsibleSummaryProps> = ({ pagePath, pageContent }) => {
  const [isOpen, setIsOpen] = useState(false);
  const [summary, setSummary] = useState<string | null>(null);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [hasCheckedCache, setHasCheckedCache] = useState(false);

  const API_BASE = 'http://localhost:3001/api/summary';

  // Check if summary exists when component mounts
  useEffect(() => {
    const checkExistingSummary = async () => {
      try {
        const response = await fetch(
          `${API_BASE}/check?pagePath=${encodeURIComponent(pagePath)}`
        );
        const data = await response.json();

        if (data.exists && data.summary) {
          setSummary(data.summary);
        }
        setHasCheckedCache(true);
      } catch (err) {
        console.error('Error checking summary cache:', err);
        setHasCheckedCache(true);
      }
    };

    checkExistingSummary();
  }, [pagePath]);

  const handleToggle = async () => {
    const newIsOpen = !isOpen;
    setIsOpen(newIsOpen);

    // Only generate if opening for the first time and no summary exists
    // if (newIsOpen && !summary && !isLoading && hasCheckedCache) {
    //   await generateSummary();
    // }
  };

  const generateSummary = async () => {
    setIsLoading(true);
    setError(null);

    try {
      const response = await fetch(`${API_BASE}/generate`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          pagePath,
          pageContent,
        }),
      });

      if (!response.ok) {
        throw new Error('Failed to generate summary');
      }

      const data = await response.json();

      if (data.success && data.summary) {
        setSummary(data.summary);
      } else {
        throw new Error('Invalid response from server');
      }
    } catch (err) {
      console.error('Error generating summary:', err);
      setError(err instanceof Error ? err.message : 'Failed to generate summary');
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className={styles.summaryContainer}>
      <button
        className={styles.summaryToggle}
        onClick={handleToggle}
        aria-expanded={isOpen}
        aria-controls="page-summary"
      >
        <span className={styles.summaryIcon}>{isOpen ? '▼' : '▶'}</span>
        <span className={styles.summaryTitle}>Page Summary</span>
        {summary && !isOpen && (
          <span className={styles.cachedBadge}>Cached</span>
        )}
      </button>

      {isOpen && (
        <div
          id="page-summary"
          className={styles.summaryContent}
          role="region"
          aria-label="Page summary"
        >
          {isLoading && (
            <div className={styles.loadingState}>
              <div className={styles.spinner}></div>
              <p>Generating summary...</p>
            </div>
          )}

          {error && (
            <div className={styles.errorState}>
              <p className={styles.errorMessage}>⚠️ {error}</p>
              <button
                className={styles.retryButton}
                onClick={generateSummary}
              >
                Retry
              </button>
            </div>
          )}

          {summary && !isLoading && (
            <div
              className={styles.summaryText}
              dangerouslySetInnerHTML={{
                __html: renderMarkdown(summary),
              }}
            />
          )}
        </div>
      )}
    </div>
  );
};

// Simple markdown renderer for basic formatting
function renderMarkdown(text: string): string {
  let html = text;

  // Headers
  html = html.replace(/^### (.*$)/gim, '<h3>$1</h3>');
  html = html.replace(/^## (.*$)/gim, '<h2>$1</h2>');
  html = html.replace(/^# (.*$)/gim, '<h1>$1</h1>');

  // Bold
  html = html.replace(/\*\*(.*?)\*\*/g, '<strong>$1</strong>');

  // Italic
  html = html.replace(/\*(.*?)\*/g, '<em>$1</em>');

  // Code inline
  html = html.replace(/`(.*?)`/g, '<code>$1</code>');

  // Lists - unordered
  html = html.replace(/^\* (.*$)/gim, '<li>$1</li>');
  html = html.replace(/^- (.*$)/gim, '<li>$1</li>');

  // Wrap consecutive <li> in <ul>
  html = html.replace(/(<li>.*<\/li>)/s, (match) => {
    return '<ul>' + match + '</ul>';
  });

  // Line breaks
  html = html.replace(/\n\n/g, '</p><p>');
  html = '<p>' + html + '</p>';

  // Clean up empty paragraphs
  html = html.replace(/<p><\/p>/g, '');
  html = html.replace(/<p>(<h[1-3]>)/g, '$1');
  html = html.replace(/(<\/h[1-3]>)<\/p>/g, '$1');
  html = html.replace(/<p>(<ul>)/g, '$1');
  html = html.replace(/(<\/ul>)<\/p>/g, '$1');

  return html;
}

export default CollapsibleSummary;
