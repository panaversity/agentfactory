/**
 * useOnlineStatus Hook
 * Detects online/offline state and provides retry queue functionality
 */

import { useState, useEffect } from 'react';

interface OnlineStatusResult {
  isOnline: boolean;
  wasOffline: boolean; // True if just came back online
}

export function useOnlineStatus(): OnlineStatusResult {
  const [isOnline, setIsOnline] = useState(() => 
    typeof navigator !== 'undefined' ? navigator.onLine : true
  );
  const [wasOffline, setWasOffline] = useState(false);

  useEffect(() => {
    // Only run on client side
    if (typeof window === 'undefined') return;

    const handleOnline = () => {
      console.log('[useOnlineStatus] 🌐 Connection restored');
      setIsOnline(true);
      setWasOffline(true);
      
      // Clear wasOffline flag after 3 seconds
      setTimeout(() => {
        setWasOffline(false);
      }, 3000);
    };

    const handleOffline = () => {
      console.log('[useOnlineStatus] ⚠️ Connection lost');
      setIsOnline(false);
      setWasOffline(false);
    };

    // Set up event listeners
    window.addEventListener('online', handleOnline);
    window.addEventListener('offline', handleOffline);

    // Initial check
    setIsOnline(navigator.onLine);

    return () => {
      window.removeEventListener('online', handleOnline);
      window.removeEventListener('offline', handleOffline);
    };
  }, []);

  return { isOnline, wasOffline };
}
