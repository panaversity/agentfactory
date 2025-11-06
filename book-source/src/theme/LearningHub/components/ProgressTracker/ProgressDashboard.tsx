/**
 * ProgressDashboard Component
 * Displays reader engagement statistics and learning progress
 */

import React from 'react';
import styles from './ProgressDashboard.module.css';

interface ProgressRecord {
  url: string;
  title: string;
  visitedAt: string;
  readDuration: number;
  lastVisitedAt: string;
  visitCount: number;
}

interface ProgressStats {
  totalChaptersVisited: number;
  totalReadTime: number; // seconds
  totalHighlights: number;
  completionPercentage: number;
  records: ProgressRecord[];
}

interface ProgressDashboardProps {
  progress: ProgressStats;
  elapsedTime: number; // current page elapsed time in seconds
  isCurrentPageVisited: boolean;
  onClearProgress: () => void;
  recentRecords: ProgressRecord[];
}

/**
 * Format seconds to human-readable duration
 */
function formatDuration(seconds: number): string {
  if (seconds < 60) {
    return `${seconds}s`;
  }
  
  const minutes = Math.floor(seconds / 60);
  if (minutes < 60) {
    return `${minutes}m ${seconds % 60}s`;
  }
  
  const hours = Math.floor(minutes / 60);
  const remainingMinutes = minutes % 60;
  return `${hours}h ${remainingMinutes}m`;
}

/**
 * Format ISO timestamp to relative time
 */
function formatRelativeTime(isoString: string): string {
  const now = Date.now();
  const then = new Date(isoString).getTime();
  const diffMs = now - then;
  const diffMinutes = Math.floor(diffMs / (1000 * 60));
  
  if (diffMinutes < 1) return 'just now';
  if (diffMinutes < 60) return `${diffMinutes}m ago`;
  
  const diffHours = Math.floor(diffMinutes / 60);
  if (diffHours < 24) return `${diffHours}h ago`;
  
  const diffDays = Math.floor(diffHours / 24);
  if (diffDays < 7) return `${diffDays}d ago`;
  
  const diffWeeks = Math.floor(diffDays / 7);
  return `${diffWeeks}w ago`;
}

export function ProgressDashboard({
  progress,
  elapsedTime,
  isCurrentPageVisited,
  onClearProgress,
  recentRecords,
}: ProgressDashboardProps) {
  const [showClearConfirm, setShowClearConfirm] = React.useState(false);

  const handleClearClick = () => {
    setShowClearConfirm(true);
  };

  const handleConfirmClear = () => {
    onClearProgress();
    setShowClearConfirm(false);
  };

  const handleCancelClear = () => {
    setShowClearConfirm(false);
  };

  return (
    <div className={styles.dashboard}>
      {/* Stats Overview */}
      <div className={styles.statsGrid}>
        <div className={styles.statCard}>
          <div className={styles.statValue}>{progress.totalChaptersVisited}</div>
          <div className={styles.statLabel}>Chapters Visited</div>
        </div>
        
        <div className={styles.statCard}>
          <div className={styles.statValue}>{formatDuration(progress.totalReadTime)}</div>
          <div className={styles.statLabel}>Total Reading Time</div>
        </div>
        
        <div className={styles.statCard}>
          <div className={styles.statValue}>{progress.totalHighlights}</div>
          <div className={styles.statLabel}>Highlights Created</div>
        </div>
        
        <div className={styles.statCard}>
          <div className={styles.statValue}>{progress.completionPercentage}%</div>
          <div className={styles.statLabel}>Completion</div>
        </div>
      </div>

      {/* Current Page Status */}
      <div className={styles.currentPage}>
        <h3 className={styles.sectionTitle}>Current Page</h3>
        <div className={styles.currentPageInfo}>
          <div className={styles.timeDisplay}>
            <strong>{formatDuration(elapsedTime)}</strong> on this page
          </div>
          {!isCurrentPageVisited && (
            <div className={styles.visitHint}>
              📖 Read for {30 - elapsedTime}s more to mark as visited
            </div>
          )}
          {isCurrentPageVisited && (
            <div className={styles.visitBadge}>
              ✓ Visited
            </div>
          )}
        </div>
      </div>

      {/* Progress Bar */}
      <div className={styles.progressSection}>
        <h3 className={styles.sectionTitle}>Overall Progress</h3>
        <div className={styles.progressBar}>
          <div 
            className={styles.progressFill} 
            style={{ width: `${progress.completionPercentage}%` }}
          />
        </div>
        <div className={styles.progressLabel}>
          {progress.completionPercentage}% Complete • {progress.totalChaptersVisited} of ~50 chapters explored
        </div>
        <div className={styles.progressNote}>
          Based on chapters you've spent 30+ seconds reading
        </div>
      </div>

      {/* Recent Activity */}
      <div className={styles.recentActivity}>
        <h3 className={styles.sectionTitle}>Recent Activity</h3>
        {recentRecords.length === 0 ? (
          <div className={styles.emptyState}>
            No chapters visited yet. Start exploring to track your progress!
          </div>
        ) : (
          <div className={styles.recordsList}>
            {recentRecords.map((record, index) => (
              <div key={`${record.url}-${index}`} className={styles.recordItem}>
                <div className={styles.recordTitle}>
                  {record.title || record.url}
                </div>
                <div className={styles.recordMeta}>
                  <span className={styles.recordDuration}>
                    {formatDuration(record.readDuration)}
                  </span>
                  <span className={styles.recordSeparator}>•</span>
                  <span className={styles.recordTime}>
                    {formatRelativeTime(record.lastVisitedAt)}
                  </span>
                  {record.visitCount > 1 && (
                    <>
                      <span className={styles.recordSeparator}>•</span>
                      <span className={styles.recordVisits}>
                        {record.visitCount}× visited
                      </span>
                    </>
                  )}
                </div>
              </div>
            ))}
          </div>
        )}
      </div>

      {/* Clear Progress Button */}
      {progress.records.length > 0 && (
        <div className={styles.actions}>
          {!showClearConfirm ? (
            <button 
              className={styles.clearButton}
              onClick={handleClearClick}
            >
              Clear All Progress
            </button>
          ) : (
            <div className={styles.confirmDialog}>
              <p className={styles.confirmMessage}>
                Are you sure? This will delete all tracking data.
              </p>
              <div className={styles.confirmButtons}>
                <button 
                  className={styles.confirmYes}
                  onClick={handleConfirmClear}
                >
                  Yes, Clear All
                </button>
                <button 
                  className={styles.confirmNo}
                  onClick={handleCancelClear}
                >
                  Cancel
                </button>
              </div>
            </div>
          )}
        </div>
      )}
    </div>
  );
}
