/**
 * Rate Limiter Service
 * Enforces 15 RPM (requests per minute) shared across all AI features
 * Based on sliding window algorithm
 */

import type { RateLimitInfo } from '../types';

export class RateLimiter {
  private readonly maxRequests: number;
  private readonly windowMs: number;
  private requestTimestamps: number[] = [];

  constructor(maxRequestsPerMinute: number = 15) {
    this.maxRequests = maxRequestsPerMinute;
    this.windowMs = 60 * 1000; // 1 minute in milliseconds
  }

  /**
   * Check if a request can be made and record it if allowed
   * @returns true if request allowed, false if rate limited
   */
  public async tryRequest(): Promise<boolean> {
    const now = Date.now();
    this.cleanOldTimestamps(now);

    if (this.requestTimestamps.length >= this.maxRequests) {
      return false; // Rate limited
    }

    this.requestTimestamps.push(now);
    return true;
  }

  /**
   * Get current rate limit status
   */
  public getStatus(): RateLimitInfo {
    const now = Date.now();
    this.cleanOldTimestamps(now);

    const remaining = Math.max(0, this.maxRequests - this.requestTimestamps.length);
    const oldestTimestamp = this.requestTimestamps[0];
    const resetAt = oldestTimestamp ? oldestTimestamp + this.windowMs : now;

    return {
      remaining,
      resetAt,
      limited: remaining === 0,
    };
  }

  /**
   * Wait until a request slot is available
   * @param maxWaitMs Maximum time to wait in milliseconds (default: 60000)
   * @returns true if slot became available, false if timeout
   */
  public async waitForSlot(maxWaitMs: number = 60000): Promise<boolean> {
    const startTime = Date.now();

    while (Date.now() - startTime < maxWaitMs) {
      if (await this.tryRequest()) {
        return true;
      }

      const status = this.getStatus();
      const waitTime = Math.min(status.resetAt - Date.now(), maxWaitMs - (Date.now() - startTime));

      if (waitTime > 0) {
        await this.sleep(waitTime);
      }
    }

    return false; // Timeout
  }

  /**
   * Reset the rate limiter (clear all timestamps)
   * Useful for testing or manual reset
   */
  public reset(): void {
    this.requestTimestamps = [];
  }

  /**
   * Remove timestamps older than the sliding window
   */
  private cleanOldTimestamps(now: number): void {
    const cutoff = now - this.windowMs;
    this.requestTimestamps = this.requestTimestamps.filter(ts => ts > cutoff);
  }

  /**
   * Sleep for specified milliseconds
   */
  private sleep(ms: number): Promise<void> {
    return new Promise(resolve => setTimeout(resolve, ms));
  }
}

// Singleton instance shared across all AI features
export const globalRateLimiter = new RateLimiter(15);
