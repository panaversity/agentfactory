/**
 * ConceptsList Component
 * Displays extracted key concepts with loading state
 */

import React, { type JSX } from 'react';
import type { KeyConcept } from '../../types';
import ConceptItem from './ConceptItem';
import styles from './ConceptsList.module.css';

interface ConceptsListProps {
  concepts: KeyConcept[];
  isLoading: boolean;
  error: string | null;
  onConceptClick: (concept: KeyConcept) => void;
  onRetry?: () => void;
}

export default function ConceptsList({
  concepts,
  isLoading,
  error,
  onConceptClick,
  onRetry,
}: ConceptsListProps): JSX.Element {
  // Loading state
  if (isLoading) {
    return (
      <div className={styles.container}>
        <div className={styles.loading}>
          <div className={styles.spinner}></div>
          <p>Extracting key concepts...</p>
        </div>
      </div>
    );
  }

  // Error state
  if (error) {
    return (
      <div className={styles.container}>
        <div className={styles.error}>
          <h3>⚠️ Failed to Extract Concepts</h3>
          <p>{error}</p>
          {onRetry && (
            <button onClick={onRetry} className={styles.retryButton}>
              Try Again
            </button>
          )}
        </div>
      </div>
    );
  }

  // Empty state
  if (concepts.length === 0) {
    return (
      <div className={styles.container}>
        <div className={styles.emptyState}>
          <h3>💡 Key Concepts</h3>
          <p>No concepts extracted yet. Try a chapter with more content.</p>
        </div>
      </div>
    );
  }

  // Concepts list
  return (
    <div className={styles.container}>
      <div className={styles.header}>
        <h3>💡 Key Concepts</h3>
        <span className={styles.count}>{concepts.length} concepts</span>
      </div>

      <div className={styles.conceptsList}>
        {concepts.map((concept) => (
          <ConceptItem
            key={concept.id}
            concept={concept}
            onClick={() => onConceptClick(concept)}
          />
        ))}
      </div>

      <div className={styles.footer}>
        <p className={styles.note}>
          Click any concept to jump to its section in the chapter
        </p>
      </div>
    </div>
  );
}
