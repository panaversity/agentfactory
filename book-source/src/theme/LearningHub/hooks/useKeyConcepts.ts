/**
 * useKeyConcepts Hook
 * Manages key concepts extraction with 7-day cache and MD5 content hash validation
 */

import { useState, useCallback, useEffect } from 'react';
import type { KeyConcept, CachedKeyConcepts } from '../types';
import { geminiService } from '../services/geminiService';
import { storage } from '../services/storageService';
import { errorLogger } from '../services/errorLogger';
import { computeHash } from '../utils/hash';

const CACHE_KEY_PREFIX = 'learningHub_concepts_v1';
const CACHE_TTL = 7 * 24 * 60 * 60 * 1000; // 7 days in milliseconds

interface UseKeyConceptsResult {
  concepts: KeyConcept[];
  isLoading: boolean;
  error: string | null;
  extractConcepts: (pageContext: { url: string; title: string; content: string }) => Promise<void>;
  clearCache: () => void;
}

export function useKeyConcepts(pageUrl: string): UseKeyConceptsResult {
  const [concepts, setConcepts] = useState<KeyConcept[]>([]);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  /**
   * Get cache key for current page
   */
  const getCacheKey = useCallback((url: string) => {
    return `${CACHE_KEY_PREFIX}_${url}`;
  }, []);

  /**
   * Check if cached concepts are still valid
   */
  const isCacheValid = useCallback((cached: CachedKeyConcepts, currentContentHash: string): boolean => {
    const now = Date.now();
    const age = now - cached.cachedAt;
    
    // Check if cache expired (>7 days)
    if (age > CACHE_TTL) {
      console.log('[useKeyConcepts] Cache expired:', { age, ttl: CACHE_TTL });
      return false;
    }

    // Check if content changed (hash mismatch)
    if (cached.contentHash !== currentContentHash) {
      console.log('[useKeyConcepts] Content hash mismatch:', {
        cached: cached.contentHash,
        current: currentContentHash,
      });
      return false;
    }

    return true;
  }, []);

  /**
   * Extract concepts from page content with caching
   */
  const extractConcepts = useCallback(async (pageContext: {
    url: string;
    title: string;
    content: string;
  }) => {
    try {
      setIsLoading(true);
      setError(null);

      console.log('[useKeyConcepts] Extracting concepts for:', pageContext.title);

      // Compute content hash for cache validation
      const contentHash = await computeHash(pageContext.content);
      const cacheKey = getCacheKey(pageContext.url);

      // Check cache first
      const cached = storage.get<CachedKeyConcepts>(cacheKey);
      
      if (cached && isCacheValid(cached, contentHash)) {
        console.log('[useKeyConcepts] Cache hit - using cached concepts');
        setConcepts(cached.concepts);
        setIsLoading(false);
        return;
      }

      console.log('[useKeyConcepts] Cache miss - extracting new concepts');

      // Determine target count based on content length
      const wordCount = pageContext.content.split(/\s+/).length;
      let targetCount = 6;
      
      if (wordCount < 500) {
        targetCount = 3; // Minimum for short content
      } else if (wordCount > 3000) {
        targetCount = 7; // More for long content
      }

      // Extract concepts from API
      const rawConcepts = await geminiService.extractConcepts(pageContext, targetCount);

      // Transform to full KeyConcept objects
      const fullConcepts: KeyConcept[] = rawConcepts.map((c) => ({
        id: crypto.randomUUID(),
        pageUrl: pageContext.url,
        title: c.title,
        description: c.description,
        sectionId: c.sectionId,
        importance: c.importance,
        contentHash,
        cachedAt: Date.now(),
      }));

      // Cache the results
      const cacheData: CachedKeyConcepts = {
        contentHash,
        cachedAt: Date.now(),
        expiresAt: Date.now() + CACHE_TTL,
        concepts: fullConcepts,
      };

      storage.set(cacheKey, cacheData);

      console.log('[useKeyConcepts] Concepts extracted and cached:', fullConcepts.length);
      setConcepts(fullConcepts);
    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : 'Failed to extract concepts';
      console.error('[useKeyConcepts] Error extracting concepts:', err);
      errorLogger.logError(err as Error, { context: 'useKeyConcepts.extractConcepts' });
      setError(errorMessage);
    } finally {
      setIsLoading(false);
    }
  }, [getCacheKey, isCacheValid]);

  /**
   * Clear cached concepts for current page
   */
  const clearCache = useCallback(() => {
    const cacheKey = getCacheKey(pageUrl);
    storage.remove(cacheKey);
    setConcepts([]);
    console.log('[useKeyConcepts] Cache cleared for:', pageUrl);
  }, [getCacheKey, pageUrl]);

  /**
   * Clear concepts when page changes
   */
  useEffect(() => {
    setConcepts([]);
    setError(null);
  }, [pageUrl]);

  return {
    concepts,
    isLoading,
    error,
    extractConcepts,
    clearCache,
  };
}
