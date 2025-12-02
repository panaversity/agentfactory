import React, { useState, useEffect, useRef } from 'react';
import { useHistory } from '@docusaurus/router';
import { searchContent } from '@/lib/search-utils';
import styles from './styles.module.css';

/**
 * Industrial-Kinetic Search Bar
 * 
 * A distinctive search interface that matches the Industrial-Kinetic Futurism aesthetic.
 * Integrates with @easyops-cn/docusaurus-search-local plugin.
 */

/**
 * Industrial-Kinetic Search Bar
 * 
 * A distinctive search interface that matches the Industrial-Kinetic Futurism aesthetic.
 * Features:
 * - HUD/sensor-inspired design with cyan glow effects
 * - JetBrains Mono typography for technical feel
 * - Smooth animations and micro-interactions
 * - Keyboard shortcuts (Cmd/Ctrl + K)
 */
export function SearchBar(): React.ReactElement {
  const [isOpen, setIsOpen] = useState(false);
  const [query, setQuery] = useState('');
  const [results, setResults] = useState<any[]>([]);
  const [loading, setLoading] = useState(false);
  const inputRef = useRef<HTMLInputElement>(null);
  const containerRef = useRef<HTMLDivElement>(null);
  const history = useHistory();

  // Handle keyboard shortcut (Cmd/Ctrl + K)
  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      if ((e.metaKey || e.ctrlKey) && e.key === 'k') {
        e.preventDefault();
        setIsOpen(true);
        setTimeout(() => inputRef.current?.focus(), 100);
      }
      if (e.key === 'Escape' && isOpen) {
        setIsOpen(false);
        setQuery('');
      }
    };

    document.addEventListener('keydown', handleKeyDown);
    return () => document.removeEventListener('keydown', handleKeyDown);
  }, [isOpen]);

  // Close on outside click
  useEffect(() => {
    const handleClickOutside = (e: MouseEvent) => {
      if (containerRef.current && !containerRef.current.contains(e.target as Node)) {
        setIsOpen(false);
        setQuery('');
      }
    };

    if (isOpen) {
      document.addEventListener('mousedown', handleClickOutside);
      return () => document.removeEventListener('mousedown', handleClickOutside);
    }
  }, [isOpen]);

  // Perform search using the search utility
  useEffect(() => {
    if (!isOpen || !query.trim()) {
      setResults([]);
      return;
    }

    // Debounce search
    const timeoutId = setTimeout(async () => {
      setLoading(true);
      
      try {
        const searchResults = await searchContent(query);
        setResults(searchResults);
      } catch (error) {
        // Silently handle errors - search may not be available in dev mode
        setResults([]);
      } finally {
        setLoading(false);
      }
    }, 300);

    return () => clearTimeout(timeoutId);
  }, [query, isOpen]);

  const handleResultClick = (url: string) => {
    setIsOpen(false);
    setQuery('');
    history.push(url);
  };

  return (
    <div ref={containerRef} className={styles.searchContainer}>
      {/* Search Trigger Button */}
      <button
        className={styles.searchTrigger}
        onClick={() => {
          setIsOpen(!isOpen);
          if (!isOpen) {
            setTimeout(() => inputRef.current?.focus(), 100);
          }
        }}
        aria-label="Search"
        aria-expanded={isOpen}
      >
        <svg
          className={styles.searchIcon}
          width="20"
          height="20"
          viewBox="0 0 24 24"
          fill="none"
          stroke="currentColor"
          strokeWidth="2"
          strokeLinecap="round"
          strokeLinejoin="round"
        >
          <circle cx="11" cy="11" r="8" />
          <path d="m21 21-4.35-4.35" />
        </svg>
        <span className={styles.searchLabel}>Search</span>
        <kbd className={styles.searchShortcut}>
          <span className={styles.kbdKey}>⌘</span>
          <span className={styles.kbdKey}>K</span>
        </kbd>
      </button>

      {/* Search Modal/Overlay */}
      {isOpen && (
        <div className={styles.searchOverlay}>
          <div className={styles.searchModal}>
            {/* Search Input */}
            <div className={styles.searchInputWrapper}>
              <svg
                className={styles.searchInputIcon}
                width="20"
                height="20"
                viewBox="0 0 24 24"
                fill="none"
                stroke="currentColor"
                strokeWidth="2"
                strokeLinecap="round"
                strokeLinejoin="round"
              >
                <circle cx="11" cy="11" r="8" />
                <path d="m21 21-4.35-4.35" />
              </svg>
              <input
                ref={inputRef}
                type="text"
                className={styles.searchInput}
                placeholder="Search documentation..."
                value={query}
                onChange={(e) => setQuery(e.target.value)}
                autoFocus
              />
              {query && (
                <button
                  className={styles.clearButton}
                  onClick={() => setQuery('')}
                  aria-label="Clear search"
                >
                  <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                    <line x1="18" y1="6" x2="6" y2="18" />
                    <line x1="6" y1="6" x2="18" y2="18" />
                  </svg>
                </button>
              )}
            </div>

            {/* Search Results */}
            <div className={styles.searchResults}>
              {loading && query && (
                <div className={styles.searchLoading}>
                  <div className={styles.loadingSpinner} />
                  <span>Scanning knowledge base...</span>
                </div>
              )}

              {!loading && query && results.length === 0 && (
                <div className={styles.searchEmpty}>
                  <svg width="48" height="48" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5">
                    <circle cx="11" cy="11" r="8" />
                    <path d="m21 21-4.35-4.35" />
                  </svg>
                  <p>No results found</p>
                  <span className={styles.searchEmptyHint}>Try different keywords</span>
                </div>
              )}

              {!loading && query && results.length > 0 && (
                <>
                  <div className={styles.searchResultsHeader}>
                    <span className={styles.resultsCount}>
                      {results.length} {results.length === 1 ? 'result' : 'results'}
                    </span>
                  </div>
                  <div className={styles.resultsList}>
                        {results.map((result: any, index: number) => (
                      <button
                        key={index}
                        className={styles.resultItem}
                        onClick={() => handleResultClick(result.url || result.href || '')}
                      >
                        <div className={styles.resultHeader}>
                          <span className={styles.resultTitle}>{result.title || result.content}</span>
                          <span className={styles.resultType}>{result.type || 'doc'}</span>
                        </div>
                        {result.text && (
                          <p className={styles.resultSnippet}>{result.text}</p>
                        )}
                        {result.url && (
                          <div className={styles.resultPath}>
                            {result.url.replace(/^\//, '').replace(/\/$/, '')}
                          </div>
                        )}
                      </button>
                    ))}
                  </div>
                </>
              )}

              {!query && (
                <div className={styles.searchEmpty}>
                  <div className={styles.searchHint}>
                    <kbd className={styles.hintKbd}>⌘</kbd>
                    <span>+</span>
                    <kbd className={styles.hintKbd}>K</kbd>
                    <span>to search</span>
                  </div>
                  <p className={styles.searchPlaceholder}>Start typing to search documentation...</p>
                </div>
              )}
            </div>
          </div>
        </div>
      )}
    </div>
  );
}

