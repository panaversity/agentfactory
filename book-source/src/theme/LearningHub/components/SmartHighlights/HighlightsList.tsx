/**
 * HighlightsList Component
 * Displays saved highlights in sidebar "Smart Highlights" tab with delete buttons
 */

import React from 'react';
import type { Highlight } from '../../types';
import styles from './HighlightsList.module.css';

interface HighlightsListProps {
  highlights: Highlight[];
  onDelete: (id: string) => void;
  onHighlightClick: (highlight: Highlight) => void;
}

export function HighlightsList({
  highlights,
  onDelete,
  onHighlightClick,
}: HighlightsListProps) {
  if (highlights.length === 0) {
    return (
      <div className={styles.emptyState}>
        <div className={styles.emptyIcon}>📝</div>
        <h3>No highlights yet</h3>
        <p>Select text on this page and click "Save" to create your first highlight!</p>
      </div>
    );
  }

  return (
    <div className={styles.highlightsList}>
      <div className={styles.header}>
        <h3>Your Highlights ({highlights.length})</h3>
      </div>

      <div className={styles.items}>
        {highlights.map((highlight) => (
          <div key={highlight.id} className={styles.highlightItem}>
            <div className={styles.highlightText}>
              <span className={styles.highlightMarker}>"{highlight.text}"</span>
            </div>

            {highlight.explanation && (
              <div className={styles.explanation}>
                <strong>💡 Explanation:</strong>
                <p>{highlight.explanation}</p>
              </div>
            )}

            <div className={styles.footer}>
              <span className={styles.timestamp}>
                {new Date(highlight.timestamp).toLocaleDateString()}
              </span>
              <button
                className={styles.deleteButton}
                onClick={() => onDelete(highlight.id)}
                title="Delete this highlight"
              >
                🗑️ Delete
              </button>
            </div>
          </div>
        ))}
      </div>
    </div>
  );
}
