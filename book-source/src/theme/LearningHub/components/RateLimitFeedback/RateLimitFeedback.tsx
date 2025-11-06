/**
 * RateLimitFeedback Component
 * Shows API request rate limit status in sidebar footer
 */

import React, { useState, useEffect } from 'react';
import styles from './RateLimitFeedback.module.css';

interface RateLimitInfo {
  remaining: number;
  total: number;
  resetTime: Date | null;
}

export function RateLimitFeedback() {
  const [rateLimitInfo, setRateLimitInfo] = useState<RateLimitInfo>({
    remaining: 60,
    total: 60,
    resetTime: null,
  });

  useEffect(() => {
    // Check localStorage for rate limit info
    const checkRateLimit = () => {
      try {
        const stored = localStorage.getItem('learningHub_rateLimit');
        if (stored) {
          const data = JSON.parse(stored);
          setRateLimitInfo({
            remaining: data.remaining || 60,
            total: data.total || 60,
            resetTime: data.resetTime ? new Date(data.resetTime) : null,
          });
        }
      } catch (error) {
        console.error('[RateLimitFeedback] Error reading rate limit:', error);
      }
    };

    // Check immediately
    checkRateLimit();

    // Check every 5 seconds
    const intervalId = setInterval(checkRateLimit, 5000);

    return () => clearInterval(intervalId);
  }, []);

  // Calculate percentage
  const percentage = (rateLimitInfo.remaining / rateLimitInfo.total) * 100;
  
  // Determine status color
  let statusClass = styles.statusHigh;
  if (percentage < 20) {
    statusClass = styles.statusLow;
  } else if (percentage < 50) {
    statusClass = styles.statusMedium;
  }

  // Format reset time
  const formatResetTime = () => {
    if (!rateLimitInfo.resetTime) return null;
    
    const now = new Date();
    const diff = rateLimitInfo.resetTime.getTime() - now.getTime();
    
    if (diff < 0) return 'Resetting...';
    
    const seconds = Math.floor(diff / 1000);
    if (seconds < 60) return `${seconds}s`;
    
    const minutes = Math.floor(seconds / 60);
    return `${minutes}m`;
  };

  const resetTimeStr = formatResetTime();

  return (
    <div className={styles.rateLimitFeedback}>
      <div className={styles.limitHeader}>
        <span className={styles.limitLabel}>API Requests</span>
        {resetTimeStr && (
          <span className={styles.resetTime}>Reset in {resetTimeStr}</span>
        )}
      </div>
      <div className={styles.limitBar}>
        <div 
          className={`${styles.limitProgress} ${statusClass}`}
          style={{ width: `${percentage}%` }}
        />
      </div>
      <div className={styles.limitText}>
        <span className={statusClass}>
          {rateLimitInfo.remaining} of {rateLimitInfo.total} remaining
        </span>
      </div>
    </div>
  );
}
