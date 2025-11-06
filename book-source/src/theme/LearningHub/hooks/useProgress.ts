/**
 * useProgress Hook
 * Tracks reader engagement: page visits, read duration, learning progress
 */

import { useState, useEffect, useCallback, useRef } from 'react';
import { storage } from '../services/storageService';
import { errorLogger } from '../services/errorLogger';

interface ProgressRecord {
  url: string;
  title: string;
  visitedAt: string; // ISO timestamp
  readDuration: number; // seconds
  lastVisitedAt: string; // ISO timestamp of most recent visit
  visitCount: number;
}

interface ProgressStats {
  totalChaptersVisited: number;
  totalReadTime: number; // seconds
  totalHighlights: number;
  completionPercentage: number;
  records: ProgressRecord[];
}

const STORAGE_KEY = 'learningHub_progress_v1';
const MIN_READ_DURATION = 30; // seconds - minimum time to mark page as "visited"
const UPDATE_INTERVAL = 10000; // 10 seconds - how often to persist duration

/**
 * Hook to track and manage reader progress
 */
export function useProgress(currentUrl: string, currentTitle: string, totalChapters: number = 50) {
  const [progress, setProgress] = useState<ProgressStats>({
    totalChaptersVisited: 0,
    totalReadTime: 0,
    totalHighlights: 0,
    completionPercentage: 0,
    records: [],
  });
  const [startTime, setStartTime] = useState<number>(Date.now());
  const [elapsedTime, setElapsedTime] = useState<number>(0);
  const baselineDuration = useRef<number>(0); // Duration that existed BEFORE this session
  const isNewVisit = useRef<boolean>(true); // Is this a new visit to increment count?

  // Load progress from localStorage on mount and recalculate stats
  const recalculateStats = useCallback(() => {
    try {
      const stored = storage.get<{ records: ProgressRecord[]; totalHighlights?: number }>(STORAGE_KEY);
      if (stored && stored.records) {
        const records = stored.records;
        const totalTime = records.reduce((sum, r) => sum + r.readDuration, 0);
        const uniquePages = new Set(records.filter(r => r.readDuration >= MIN_READ_DURATION).map(r => r.url)).size;
        const completion = totalChapters > 0 ? (uniquePages / totalChapters) * 100 : 0;

        setProgress({
          totalChaptersVisited: uniquePages,
          totalReadTime: totalTime,
          totalHighlights: stored.totalHighlights || 0,
          completionPercentage: Math.round(completion),
          records: records,
        });
      }
    } catch (error) {
      errorLogger.logError('Failed to load progress from localStorage', error as Error);
    }
  }, [totalChapters]);

  // Load on mount
  useEffect(() => {
    recalculateStats();
  }, [recalculateStats]);

  // Update elapsed time periodically
  useEffect(() => {
    const interval = setInterval(() => {
      const elapsed = Math.floor((Date.now() - startTime) / 1000);
      setElapsedTime(elapsed);
    }, 1000);

    return () => clearInterval(interval);
  }, [startTime]);

  // Initialize baseline duration when page changes
  useEffect(() => {
    // Reset timer for new page
    setStartTime(Date.now());
    setElapsedTime(0);
    
    // Load existing duration for this page
    const stored = storage.get<{ records: ProgressRecord[] }>(STORAGE_KEY);
    const existing = stored?.records?.find(r => r.url === currentUrl);
    
    if (existing) {
      baselineDuration.current = existing.readDuration;
      isNewVisit.current = true; // Still count as new visit
    } else {
      baselineDuration.current = 0;
      isNewVisit.current = true;
    }
  }, [currentUrl]);

  // Persist progress periodically and on unmount
  useEffect(() => {
    const persistProgress = () => {
      try {
        const currentElapsed = Math.floor((Date.now() - startTime) / 1000);
        if (currentElapsed < 3) return; // Don't record very short visits

        // Load FRESH data from storage
        const stored = storage.get<{ records: ProgressRecord[]; totalHighlights?: number }>(STORAGE_KEY);
        const existingRecords = stored?.records || [];

        // Find existing record for current page
        const existingIndex = existingRecords.findIndex(r => r.url === currentUrl);
        let updatedRecords: ProgressRecord[];

        if (existingIndex >= 0) {
          // Page exists - SET absolute duration (baseline + current session)
          updatedRecords = [...existingRecords];
          const existingRecord = updatedRecords[existingIndex];
          
          updatedRecords[existingIndex] = {
            ...existingRecord,
            readDuration: baselineDuration.current + currentElapsed, // ABSOLUTE duration
            lastVisitedAt: new Date().toISOString(),
            visitCount: isNewVisit.current ? existingRecord.visitCount + 1 : existingRecord.visitCount,
          };
          
          isNewVisit.current = false; // Only increment once
        } else {
          // New page - CREATE record
          const newRecord: ProgressRecord = {
            url: currentUrl,
            title: currentTitle,
            visitedAt: new Date().toISOString(),
            readDuration: currentElapsed,
            lastVisitedAt: new Date().toISOString(),
            visitCount: 1,
          };
          updatedRecords = [...existingRecords, newRecord];
          isNewVisit.current = false;
        }

        // Persist to localStorage
        storage.set(STORAGE_KEY, {
          records: updatedRecords,
          totalHighlights: stored?.totalHighlights || 0,
          lastUpdated: new Date().toISOString(),
        });

        // Recalculate stats from fresh data
        recalculateStats();
        
      } catch (error) {
        errorLogger.logError('Failed to persist progress', error as Error);
      }
    };

    // Persist every UPDATE_INTERVAL seconds
    const interval = setInterval(persistProgress, UPDATE_INTERVAL);

    // Persist on unmount (cleanup)
    return () => {
      clearInterval(interval);
      persistProgress();
    };
  }, [currentUrl, currentTitle, startTime, recalculateStats]);

  // Clear all progress data
  const clearProgress = useCallback(() => {
    try {
      storage.remove(STORAGE_KEY);
      setProgress({
        totalChaptersVisited: 0,
        totalReadTime: 0,
        totalHighlights: 0,
        completionPercentage: 0,
        records: [],
      });
    } catch (error) {
      errorLogger.logError('Failed to clear progress', error as Error);
      throw error;
    }
  }, []);

  // Update highlight count (called from external hook)
  const updateHighlightCount = useCallback((count: number) => {
    setProgress(prev => ({
      ...prev,
      totalHighlights: count,
    }));

    // Also persist to localStorage
    try {
      const stored = storage.get<{ records: ProgressRecord[]; totalHighlights?: number }>(STORAGE_KEY) || { records: [] };
      storage.set(STORAGE_KEY, {
        ...stored,
        totalHighlights: count,
      });
    } catch (error) {
      errorLogger.logError('Failed to update highlight count', error as Error);
    }
  }, []);

  // Get records sorted by most recent visit
  const getRecentRecords = useCallback((limit: number = 10): ProgressRecord[] => {
    return [...progress.records]
      .sort((a, b) => new Date(b.lastVisitedAt).getTime() - new Date(a.lastVisitedAt).getTime())
      .slice(0, limit);
  }, [progress.records]);

  // Check if current page is marked as "visited" (>30 seconds)
  const isCurrentPageVisited = elapsedTime >= MIN_READ_DURATION;

  return {
    progress,
    elapsedTime,
    isCurrentPageVisited,
    clearProgress,
    updateHighlightCount,
    getRecentRecords,
  };
}
