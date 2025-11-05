/**
 * Error Logger Service
 * Console + localStorage ring buffer logging (last 50 errors)
 * Privacy-first: no external tracking
 */

import type { ErrorLogEntry } from '../types';
import { STORAGE_KEYS, LIMITS } from '../types';
import { storage } from './storageService';

export class ErrorLogger {
  private readonly maxEntries: number;

  constructor(maxEntries: number = LIMITS.MAX_ERROR_LOG_ENTRIES) {
    this.maxEntries = maxEntries;
  }

  /**
   * Log an error with context
   */
  public logError(
    error: Error | string,
    context?: Record<string, any>
  ): void {
    const entry: ErrorLogEntry = {
      id: this.generateId(),
      timestamp: Date.now(),
      message: typeof error === 'string' ? error : error.message,
      stack: typeof error === 'object' ? error.stack : undefined,
      context,
    };

    // Log to console
    console.error('[LearningHub Error]', entry.message, {
      timestamp: new Date(entry.timestamp).toISOString(),
      stack: entry.stack,
      context: entry.context,
    });

    // Save to localStorage ring buffer
    this.saveToBuffer(entry);
  }

  /**
   * Get all logged errors
   */
  public getErrors(): ErrorLogEntry[] {
    return storage.get<ErrorLogEntry[]>(STORAGE_KEYS.ERROR_LOG, { default: [] }) || [];
  }

  /**
   * Clear all logged errors
   */
  public clearErrors(): void {
    storage.remove(STORAGE_KEYS.ERROR_LOG);
  }

  /**
   * Get recent errors (last N)
   */
  public getRecentErrors(count: number = 10): ErrorLogEntry[] {
    const errors = this.getErrors();
    return errors.slice(-count);
  }

  /**
   * Check if there are any critical errors (useful for debugging)
   */
  public hasCriticalErrors(): boolean {
    const errors = this.getErrors();
    return errors.length > 0;
  }

  /**
   * Save error to ring buffer (FIFO when limit reached)
   */
  private saveToBuffer(entry: ErrorLogEntry): void {
    try {
      const errors = this.getErrors();

      // Add new entry
      errors.push(entry);

      // Maintain ring buffer size (FIFO)
      if (errors.length > this.maxEntries) {
        errors.splice(0, errors.length - this.maxEntries);
      }

      storage.set(STORAGE_KEYS.ERROR_LOG, errors);
    } catch (error) {
      // If we can't save to storage, at least log to console
      console.error('[ErrorLogger] Failed to save error to storage:', error);
    }
  }

  /**
   * Generate a simple unique ID
   */
  private generateId(): string {
    return `err_${Date.now()}_${Math.random().toString(36).substring(2, 9)}`;
  }
}

// Singleton instance
export const errorLogger = new ErrorLogger();

/**
 * Helper function to wrap async functions with error logging
 */
export function withErrorLog<T extends (...args: any[]) => Promise<any>>(
  fn: T,
  context?: Record<string, any>
): T {
  return (async (...args: Parameters<T>): Promise<ReturnType<T>> => {
    try {
      return await fn(...args);
    } catch (error) {
      errorLogger.logError(
        error as Error,
        { ...context, functionName: fn.name, args }
      );
      throw error;
    }
  }) as T;
}
