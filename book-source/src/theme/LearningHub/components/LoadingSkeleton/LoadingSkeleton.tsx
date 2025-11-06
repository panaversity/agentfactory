/**
 * Loading Skeleton Component
 * Generic skeleton loader for AI operations
 */

import React from 'react';
import styles from './LoadingSkeleton.module.css';

interface LoadingSkeletonProps {
  type?: 'chat' | 'highlights' | 'quiz' | 'concepts' | 'progress';
  count?: number;
}

export function LoadingSkeleton({ type = 'chat', count = 1 }: LoadingSkeletonProps) {
  const renderSkeleton = () => {
    switch (type) {
      case 'chat':
        return (
          <div className={styles.chatSkeleton}>
            <div className={styles.skeletonMessage}>
              <div className={styles.skeletonAvatar}></div>
              <div className={styles.skeletonContent}>
                <div className={`${styles.skeletonLine} ${styles.short}`}></div>
                <div className={`${styles.skeletonLine} ${styles.medium}`}></div>
                <div className={`${styles.skeletonLine} ${styles.long}`}></div>
              </div>
            </div>
          </div>
        );

      case 'highlights':
        return (
          <div className={styles.highlightSkeleton}>
            <div className={styles.skeletonHighlightCard}>
              <div className={`${styles.skeletonLine} ${styles.medium}`}></div>
              <div className={`${styles.skeletonLine} ${styles.short}`}></div>
            </div>
          </div>
        );

      case 'quiz':
        return (
          <div className={styles.quizSkeleton}>
            <div className={styles.skeletonQuizCard}>
              <div className={`${styles.skeletonLine} ${styles.long}`}></div>
              <div className={styles.skeletonOptions}>
                {[1, 2, 3, 4].map(i => (
                  <div key={i} className={styles.skeletonOption}></div>
                ))}
              </div>
            </div>
          </div>
        );

      case 'concepts':
        return (
          <div className={styles.conceptSkeleton}>
            <div className={styles.skeletonConceptCard}>
              <div className={`${styles.skeletonLine} ${styles.short}`}></div>
              <div className={`${styles.skeletonLine} ${styles.medium}`}></div>
            </div>
          </div>
        );

      case 'progress':
        return (
          <div className={styles.progressSkeleton}>
            <div className={styles.skeletonStatsGrid}>
              {[1, 2, 3, 4].map(i => (
                <div key={i} className={styles.skeletonStatCard}>
                  <div className={`${styles.skeletonLine} ${styles.short}`}></div>
                  <div className={`${styles.skeletonLine} ${styles.veryShort}`}></div>
                </div>
              ))}
            </div>
          </div>
        );

      default:
        return null;
    }
  };

  return (
    <div className={styles.skeletonContainer}>
      {Array.from({ length: count }).map((_, index) => (
        <React.Fragment key={index}>
          {renderSkeleton()}
        </React.Fragment>
      ))}
    </div>
  );
}
