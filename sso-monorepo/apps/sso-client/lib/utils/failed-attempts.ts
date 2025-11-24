/**
 * Failed Login Attempts Tracking
 * Client-side tracking for CAPTCHA trigger (server enforces actual rate limiting)
 */

export interface FailedAttempts {
  count: number;
  lastAttempt: number; // timestamp
  resetAt: number; // timestamp
}

const STORAGE_KEY = 'auth:failed-attempts';
const RESET_WINDOW_MS = 15 * 60 * 1000; // 15 minutes
const CAPTCHA_THRESHOLD = 5; // Show CAPTCHA after 5 failed attempts

/**
 * Get Failed Attempts from localStorage
 * Returns current failed attempts or initializes new tracking
 */
export function getFailedAttempts(): FailedAttempts {
  if (typeof window === 'undefined') {
    return { count: 0, lastAttempt: 0, resetAt: Date.now() + RESET_WINDOW_MS };
  }
  
  try {
    const stored = localStorage.getItem(STORAGE_KEY);
    if (!stored) {
      return { count: 0, lastAttempt: 0, resetAt: Date.now() + RESET_WINDOW_MS };
    }
    
    const attempts: FailedAttempts = JSON.parse(stored);
    
    // Reset if window expired
    if (Date.now() > attempts.resetAt) {
      return { count: 0, lastAttempt: 0, resetAt: Date.now() + RESET_WINDOW_MS };
    }
    
    return attempts;
  } catch {
    return { count: 0, lastAttempt: 0, resetAt: Date.now() + RESET_WINDOW_MS };
  }
}

/**
 * Increment Failed Attempts Counter
 * Adds one to the failure count and updates timestamp
 */
export function incrementFailedAttempts(): FailedAttempts {
  if (typeof window === 'undefined') {
    return { count: 0, lastAttempt: 0, resetAt: Date.now() + RESET_WINDOW_MS };
  }
  
  const attempts = getFailedAttempts();
  const updated: FailedAttempts = {
    count: attempts.count + 1,
    lastAttempt: Date.now(),
    resetAt: attempts.resetAt,
  };
  
  try {
    localStorage.setItem(STORAGE_KEY, JSON.stringify(updated));
  } catch {
    // Ignore localStorage errors
  }
  
  return updated;
}

/**
 * Clear Failed Attempts
 * Resets the counter (called after successful login)
 */
export function clearFailedAttempts(): void {
  if (typeof window === 'undefined') return;
  
  try {
    localStorage.removeItem(STORAGE_KEY);
  } catch {
    // Ignore localStorage errors
  }
}

/**
 * Should Show CAPTCHA
 * Returns true if failed attempt count is >= threshold
 */
export function shouldShowCaptcha(): boolean {
  const attempts = getFailedAttempts();
  return attempts.count >= CAPTCHA_THRESHOLD;
}
