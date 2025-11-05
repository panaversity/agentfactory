/**
 * Storage Service
 * localStorage abstraction with namespaced keys, quota management, and schema versioning
 */

import type { StorageGetOptions, StorageSetOptions, StorageQuotaInfo } from '../types';
import { STORAGE_KEYS } from '../types';

export class StorageService {
  private readonly namespace: string;

  constructor(namespace: string = 'learningHub') {
    this.namespace = namespace;
  }

  /**
   * Get item from localStorage with optional validation
   */
  public get<T>(key: string, options: StorageGetOptions = {}): T | null {
    try {
      const fullKey = this.getFullKey(key);
      const raw = localStorage.getItem(fullKey);

      if (raw === null) {
        return options.default ?? null;
      }

      const parsed = JSON.parse(raw);

      // Validate if validator provided
      if (options.validate && !options.validate(parsed)) {
        console.warn(`[StorageService] Validation failed for key: ${key}`);
        return options.default ?? null;
      }

      return parsed as T;
    } catch (error) {
      console.error(`[StorageService] Error reading key: ${key}`, error);
      return options.default ?? null;
    }
  }

  /**
   * Set item in localStorage with optional expiration
   */
  public set<T>(key: string, value: T, options: StorageSetOptions = {}): boolean {
    try {
      const fullKey = this.getFullKey(key);

      // Check quota before writing
      const quotaInfo = this.getQuotaInfo();
      if (quotaInfo.exceeded) {
        console.warn('[StorageService] localStorage quota exceeded, attempting cleanup');
        this.cleanupExpired();
      }

      let dataToStore = value;

      // Merge with existing data if requested
      if (options.merge) {
        const existing = this.get<any>(key);
        if (existing && typeof existing === 'object' && typeof value === 'object') {
          dataToStore = { ...existing, ...value } as T;
        }
      }

      // Wrap with expiration if provided
      const payload = options.expires
        ? { data: dataToStore, expiresAt: options.expires }
        : dataToStore;

      localStorage.setItem(fullKey, JSON.stringify(payload));
      return true;
    } catch (error) {
      if ((error as any).name === 'QuotaExceededError') {
        console.error('[StorageService] localStorage quota exceeded');
        this.handleQuotaExceeded(key);
      } else {
        console.error(`[StorageService] Error writing key: ${key}`, error);
      }
      return false;
    }
  }

  /**
   * Remove item from localStorage
   */
  public remove(key: string): void {
    try {
      const fullKey = this.getFullKey(key);
      localStorage.removeItem(fullKey);
    } catch (error) {
      console.error(`[StorageService] Error removing key: ${key}`, error);
    }
  }

  /**
   * Check if key exists
   */
  public has(key: string): boolean {
    const fullKey = this.getFullKey(key);
    return localStorage.getItem(fullKey) !== null;
  }

  /**
   * Clear all items with this namespace
   */
  public clearNamespace(): void {
    try {
      const keysToRemove: string[] = [];
      for (let i = 0; i < localStorage.length; i++) {
        const key = localStorage.key(i);
        if (key && key.startsWith(`${this.namespace}_`)) {
          keysToRemove.push(key);
        }
      }
      keysToRemove.forEach(key => localStorage.removeItem(key));
    } catch (error) {
      console.error('[StorageService] Error clearing namespace', error);
    }
  }

  /**
   * Get storage quota information
   */
  public getQuotaInfo(): StorageQuotaInfo {
    try {
      let used = 0;
      for (let i = 0; i < localStorage.length; i++) {
        const key = localStorage.key(i);
        if (key) {
          const value = localStorage.getItem(key);
          if (value) {
            used += key.length + value.length;
          }
        }
      }

      // Typical localStorage quota is 5-10MB, use 5MB as conservative estimate
      const available = 5 * 1024 * 1024; // 5MB in bytes
      const percentage = (used / available) * 100;

      return {
        used,
        available: available - used,
        percentage,
        exceeded: percentage > 95, // Consider 95% as exceeded
      };
    } catch (error) {
      console.error('[StorageService] Error calculating quota', error);
      return {
        used: 0,
        available: 0,
        percentage: 0,
        exceeded: false,
      };
    }
  }

  /**
   * Clean up expired entries
   */
  private cleanupExpired(): void {
    try {
      const now = Date.now();
      const keysToRemove: string[] = [];

      for (let i = 0; i < localStorage.length; i++) {
        const key = localStorage.key(i);
        if (!key || !key.startsWith(`${this.namespace}_`)) continue;

        const raw = localStorage.getItem(key);
        if (!raw) continue;

        try {
          const parsed = JSON.parse(raw);
          if (parsed.expiresAt && parsed.expiresAt < now) {
            keysToRemove.push(key);
          }
        } catch {
          // Not a wrapped object, skip
        }
      }

      keysToRemove.forEach(key => localStorage.removeItem(key));
      console.log(`[StorageService] Cleaned up ${keysToRemove.length} expired entries`);
    } catch (error) {
      console.error('[StorageService] Error during cleanup', error);
    }
  }

  /**
   * Handle quota exceeded error by removing oldest entries
   */
  private handleQuotaExceeded(attemptedKey: string): void {
    console.warn('[StorageService] Attempting to free space by removing old entries');

    // Strategy: Remove highlights and cache entries (less critical than progress)
    const entriesToRemove = [
      STORAGE_KEYS.HIGHLIGHTS,
      STORAGE_KEYS.KEY_CONCEPTS,
      STORAGE_KEYS.RELATED_TOPICS,
    ];

    for (const key of entriesToRemove) {
      if (this.has(key)) {
        this.remove(key);
        console.log(`[StorageService] Removed ${key} to free space`);
        return; // Try one at a time
      }
    }
  }

  /**
   * Get full namespaced key
   */
  private getFullKey(key: string): string {
    return `${this.namespace}_${key}`;
  }
}

// Singleton instance
export const storage = new StorageService('learningHub');
