/**
 * ProgressChart Component
 * Visual progress indicators with CSS progress bars
 */

import React from 'react';
import styles from './ProgressChart.module.css';

interface ProgressChartProps {
  completionPercentage: number;
  totalChaptersVisited: number;
  totalChapters: number;
  label?: string;
}

export function ProgressChart({
  completionPercentage,
  totalChaptersVisited,
  totalChapters,
  label = 'Overall Progress',
}: ProgressChartProps) {
  return (
    <div className={styles.chart}>
      <div className={styles.header}>
        <span className={styles.label}>{label}</span>
        <span className={styles.stats}>
          {totalChaptersVisited} / {totalChapters} chapters
        </span>
      </div>
      
      <div className={styles.progressBar}>
        <div 
          className={styles.progressFill} 
          style={{ width: `${completionPercentage}%` }}
          aria-valuenow={completionPercentage}
          aria-valuemin={0}
          aria-valuemax={100}
          role="progressbar"
        >
          {completionPercentage > 10 && (
            <span className={styles.percentage}>{completionPercentage}%</span>
          )}
        </div>
      </div>
      
      {completionPercentage <= 10 && (
        <div className={styles.percentageExternal}>
          {completionPercentage}%
        </div>
      )}
    </div>
  );
}
