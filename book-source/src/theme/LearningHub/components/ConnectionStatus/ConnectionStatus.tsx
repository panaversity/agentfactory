/**
 * ConnectionStatus Component
 * Shows online/offline status and reconnection indicator
 */

import React from 'react';
import { useOnlineStatus } from '../../hooks/useOnlineStatus';
import styles from './ConnectionStatus.module.css';

export function ConnectionStatus() {
  const { isOnline, wasOffline } = useOnlineStatus();

  // Don't show anything if online and never was offline
  if (isOnline && !wasOffline) {
    return null;
  }

  return (
    <div className={`${styles.connectionStatus} ${isOnline ? styles.online : styles.offline}`}>
      {isOnline ? (
        <>
          <span className={styles.statusIcon}>✅</span>
          <span className={styles.statusText}>Back online</span>
        </>
      ) : (
        <>
          <span className={styles.statusIcon}>⚠️</span>
          <span className={styles.statusText}>Offline - AI features unavailable</span>
        </>
      )}
    </div>
  );
}
