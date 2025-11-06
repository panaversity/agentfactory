/**
 * ConceptItem Component
 * Displays a single concept with title, description, importance indicator
 */

import React, { type JSX } from 'react';
import type { KeyConcept } from '../../types';
import styles from './ConceptItem.module.css';

interface ConceptItemProps {
  concept: KeyConcept;
  onClick: () => void;
}

export default function ConceptItem({
  concept,
  onClick,
}: ConceptItemProps): JSX.Element {
  // Determine importance level for styling
  const getImportanceLevel = (importance: number): 'high' | 'medium' | 'low' => {
    if (importance >= 8) return 'high';
    if (importance >= 5) return 'medium';
    return 'low';
  };

  const importanceLevel = getImportanceLevel(concept.importance);

  return (
    <div
      className={`${styles.conceptItem} ${styles[importanceLevel]}`}
      onClick={onClick}
      role="button"
      tabIndex={0}
      onKeyDown={(e) => {
        if (e.key === 'Enter' || e.key === ' ') {
          e.preventDefault();
          onClick();
        }
      }}
    >
      <div className={styles.header}>
        <div className={styles.importanceIndicator}>
          {importanceLevel === 'high' && '🔥'}
          {importanceLevel === 'medium' && '⭐'}
          {importanceLevel === 'low' && '💡'}
        </div>
        <h4 className={styles.title}>{concept.title}</h4>
      </div>
      
      <p className={styles.description}>{concept.description}</p>

      {concept.sectionId && (
        <div className={styles.linkIndicator}>
          <span className={styles.linkIcon}>→</span>
          <span className={styles.linkText}>Jump to section</span>
        </div>
      )}
    </div>
  );
}
