import React, { useEffect, useState } from 'react';
import styles from './styles.module.css';

interface CollapsibleSummaryProps {
  pagePath: string;
  pageTitle: string;
}

type SummarySize = 'short' | 'medium' | 'long';

const SIZE_SENTENCE_LIMITS = {
  short: '2 sentences',
  medium: '4 sentences',
  long: '6 sentences',
};

const CollapsibleSummary: React.FC<CollapsibleSummaryProps> = ({ pagePath, pageTitle }) => {
  const [isOpen, setIsOpen] = useState(false);
  const [summary, setSummary] = useState<string | null>(null);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [hasCheckedCache, setHasCheckedCache] = useState(false);
  const [selectedSize, setSelectedSize] = useState<SummarySize>('medium');

  const API_BASE = 'http://localhost:3001/api/summary';

  // Check if summary exists when component mounts or size changes
  useEffect(() => {
    const checkExistingSummary = async () => {
      try {
        const response = await fetch(
          `${API_BASE}/check?pagePath=${encodeURIComponent(pagePath)}&size=${selectedSize}`
        );
        const data = await response.json();

        if (data.exists && data.summary) {
          setSummary(data.summary);
        } else {
          setSummary(null);
        }
        setHasCheckedCache(true);
      } catch (err) {
        console.error('Error checking summary cache:', err);
        setHasCheckedCache(true);
      }
    };

    checkExistingSummary();
  }, [pagePath, selectedSize]);

  const handleToggle = async () => {
    const newIsOpen = !isOpen;
    setIsOpen(newIsOpen);

    // Only generate if opening for the first time and no summary exists
    if (newIsOpen && !summary && !isLoading && hasCheckedCache) {
      await generateSummary();
    }
  };

  const generateSummary = async () => {
    // Use the helper function with current selected size
    await generateSummaryWithSize(selectedSize);
  };

  const handleSizeChange = async (e: React.ChangeEvent<HTMLSelectElement>) => {
    const newSize = e.target.value as SummarySize;

    console.log('🔄 [Frontend] Size changed:', {
      oldSize: selectedSize,
      newSize: newSize,
      isOpen: isOpen,
    });

    // Update state first
    setSelectedSize(newSize);
    setError(null);
    setSummary(null); // Clear current summary when size changes

    // If panel is open, generate with new size directly
    // (Can't rely on state update since it's async)
    if (isOpen) {
      console.log('🔄 [Frontend] Panel is open, triggering regeneration with new size:', newSize);
      await generateSummaryWithSize(newSize);
    }
  };

  const generateSummaryWithSize = async (size: SummarySize) => {
    setIsLoading(true);
    setError(null);

    console.log('🔍 [Frontend] Generating summary with size:', size);

    try {
      const requestBody = {
        pagePath,
        pageTitle,
        size: size,
      };

      console.log('📤 [Frontend] Sending request body:', requestBody);

      const response = await fetch(`${API_BASE}/generate`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(requestBody),
      });

      if (!response.ok) {
        throw new Error('Failed to generate summary');
      }

      const data = await response.json();

      console.log('📥 [Frontend] Received response:', {
        success: data.success,
        size: data.size,
        summaryLength: data.summary?.length,
        summaryPreview: data.summary?.substring(0, 100) + '...',
      });

      if (data.success && data.summary) {
        setSummary(data.summary);
        console.log('✅ [Frontend] Summary set successfully');
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
      <div className={styles.summaryHeader}>
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

        <div className={styles.sizeSelector}>
          <label htmlFor="summary-size" className={styles.sizeLabel}>
            Size:
          </label>
          <select
            id="summary-size"
            value={selectedSize}
            onChange={handleSizeChange}
            className={styles.sizeDropdown}
            aria-label="Summary size"
          >
            <option value="short">Short (2 sentences)</option>
            <option value="medium">Medium (4 sentences)</option>
            <option value="long">Long (6 sentences)</option>
          </select>
        </div>
      </div>

      {isOpen && (
        <div
          id="page-summary"
          className={styles.summaryContent}
          role="region"
          aria-label="Page summary"
        >
          <div className={styles.summaryCard}>
            <div className={styles.summaryCardHeader}>
              <span className={styles.summaryIcon}>📝</span>
              <span className={styles.summaryCardTitle}>Summary</span>
            </div>

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
              <div className={styles.summaryText}>
                {summary}
              </div>
            )}
          </div>
        </div>
      )}
    </div>
  );
};

export default CollapsibleSummary;
