/**
 * Cache Service
 * MD5 content hashing + 7-day TTL for AI responses
 * Content-based invalidation for key concepts and related topics
 */

import type { CacheEntry, CacheOptions } from '../types';
import { LIMITS } from '../types';
import { storage } from './storageService';
import { computeHash } from '../utils/hash';

export class CacheService {
  private readonly defaultTtlMs: number;

  constructor(ttlDays: number = LIMITS.CACHE_TTL_DAYS) {
    this.defaultTtlMs = ttlDays * 24 * 60 * 60 * 1000;
  }

  /**
   * Get cached data if valid (not expired, content hash matches)
   */
  public get<T>(key: string, contentHash?: string): T | null {
    try {
      const entry = storage.get<CacheEntry<T>>(key);

      if (!entry) {
        return null; // Cache miss
      }

      const now = Date.now();

      // Check expiration
      if (entry.expiresAt < now) {
        this.delete(key); // Clean up expired entry
        return null; // Cache expired
      }

      // Check content hash if provided
      if (contentHash && entry.contentHash && entry.contentHash !== contentHash) {
        this.delete(key); // Content changed, invalidate
        return null; // Cache invalidated
      }

      return entry.data;
    } catch (error) {
      console.error(`[CacheService] Error reading cache for key: ${key}`, error);
      return null;
    }
  }

  /**
   * Set cache entry with optional TTL and content hash
   */
  public set<T>(key: string, data: T, options: CacheOptions = {}): boolean {
    try {
      const now = Date.now();
      const ttl = options.ttl ?? this.defaultTtlMs;
      const expiresAt = now + ttl;

      const entry: CacheEntry<T> = {
        data,
        timestamp: now,
        expiresAt,
        contentHash: options.contentHash,
      };

      return storage.set(key, entry);
    } catch (error) {
      console.error(`[CacheService] Error writing cache for key: ${key}`, error);
      return false;
    }
  }

  /**
   * Delete cache entry
   */
  public delete(key: string): void {
    storage.remove(key);
  }

  /**
   * Check if cache entry exists and is valid
   */
  public has(key: string, contentHash?: string): boolean {
    return this.get(key, contentHash) !== null;
  }

  /**
   * Generate cache key from page URL and operation
   */
  public generateKey(pageUrl: string, operation: string): string {
    return `cache_${operation}_${computeHash(pageUrl)}`;
  }

  /**
   * Generate cache key with content hash for invalidation
   */
  public generateKeyWithContent(
    pageUrl: string,
    operation: string,
    content: string
  ): { key: string; contentHash: string } {
    const contentHash = computeHash(content);
    const key = `cache_${operation}_${computeHash(pageUrl)}`;
    return { key, contentHash };
  }

  /**
   * Clear all cache entries (for testing or manual reset)
   */
  public clearAll(): void {
    try {
      // Find and remove all cache_ prefixed keys
      const keysToRemove: string[] = [];
      for (let i = 0; i < localStorage.length; i++) {
        const key = localStorage.key(i);
        if (key && key.includes('cache_')) {
          keysToRemove.push(key);
        }
      }
      keysToRemove.forEach(key => localStorage.removeItem(key));
      console.log(`[CacheService] Cleared ${keysToRemove.length} cache entries`);
    } catch (error) {
      console.error('[CacheService] Error clearing cache', error);
    }
  }

  /**
   * Get cache statistics
   */
  public getStats(): {
    totalEntries: number;
    expiredEntries: number;
    validEntries: number;
  } {
    let total = 0;
    let expired = 0;
    const now = Date.now();

    try {
      for (let i = 0; i < localStorage.length; i++) {
        const key = localStorage.key(i);
        if (!key || !key.includes('cache_')) continue;

        total++;

        const raw = localStorage.getItem(key);
        if (!raw) continue;

        try {
          const entry = JSON.parse(raw);
          if (entry.expiresAt && entry.expiresAt < now) {
            expired++;
          }
        } catch {
          // Skip invalid entries
        }
      }
    } catch (error) {
      console.error('[CacheService] Error calculating stats', error);
    }

    return {
      totalEntries: total,
      expiredEntries: expired,
      validEntries: total - expired,
    };
  }

  /**
   * Clean up all expired cache entries
   */
  public cleanup(): number {
    let cleaned = 0;
    const now = Date.now();

    try {
      const keysToRemove: string[] = [];

      for (let i = 0; i < localStorage.length; i++) {
        const key = localStorage.key(i);
        if (!key || !key.includes('cache_')) continue;

        const raw = localStorage.getItem(key);
        if (!raw) continue;

        try {
          const entry = JSON.parse(raw);
          if (entry.expiresAt && entry.expiresAt < now) {
            keysToRemove.push(key);
          }
        } catch {
          // Remove invalid entries too
          keysToRemove.push(key);
        }
      }

      keysToRemove.forEach(key => localStorage.removeItem(key));
      cleaned = keysToRemove.length;

      if (cleaned > 0) {
        console.log(`[CacheService] Cleaned up ${cleaned} expired entries`);
      }
    } catch (error) {
      console.error('[CacheService] Error during cleanup', error);
    }

    return cleaned;
  }
}

// Singleton instance
export const cacheService = new CacheService();
