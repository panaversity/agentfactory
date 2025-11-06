/**
 * HighlightMarker Component
 * Renders saved highlights with yellow background, clickable to view explanation
 */

import React from 'react';
import type { Highlight } from '../../types';
import styles from './HighlightMarker.module.css';

interface HighlightMarkerProps {
  highlight: Highlight;
  onClick: (highlight: Highlight) => void;
}

export function HighlightMarker({ highlight, onClick }: HighlightMarkerProps) {
  const handleClick = (e: React.MouseEvent) => {
    e.preventDefault();
    e.stopPropagation();
    onClick(highlight);
  };

  return (
    <mark
      className={styles.highlightMarker}
      onClick={handleClick}
      title="Click to view explanation"
      data-highlight-id={highlight.id}
    >
      {highlight.text}
    </mark>
  );
}
