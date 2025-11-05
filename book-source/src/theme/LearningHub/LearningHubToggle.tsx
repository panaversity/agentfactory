/**
 * Learning Hub Toggle Button
 * Fixed positioning (bottom-right), smooth animation
 */

import React from 'react';
import styles from './LearningHubToggle.module.css';

interface LearningHubToggleProps {
  isOpen: boolean;
  onClick: () => void;
}

export function LearningHubToggle({ isOpen, onClick }: LearningHubToggleProps) {
  return (
    <button
      className={`${styles.toggleButton} ${isOpen ? styles.active : ''}`}
      onClick={onClick}
      aria-label={isOpen ? 'Close Learning Hub' : 'Open Learning Hub'}
      aria-expanded={isOpen}
      title={isOpen ? 'Close Learning Hub' : 'Open Learning Hub'}
    >
      {isOpen ? '✕' : '🎓'}
    </button>
  );
}
